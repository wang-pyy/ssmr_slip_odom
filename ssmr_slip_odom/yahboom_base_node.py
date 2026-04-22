from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float32MultiArray, Int32MultiArray

try:
    import serial
except ImportError:  # pragma: no cover - hardware dependency
    serial = None


class YahboomBaseNode(Node):
    """USB-serial base driver for Yahboom 4-channel motor controller."""

    def __init__(self):
        super().__init__('yahboom_base_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_base', 0.23)
        self.declare_parameter('max_linear_speed', 0.6)
        self.declare_parameter('max_angular_speed', 3.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('channel_fl', 1)
        self.declare_parameter('channel_fr', 2)
        self.declare_parameter('channel_rl', 3)
        self.declare_parameter('channel_rr', 4)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('publish_feedback', True)
        self.declare_parameter('apply_board_config', False)
        self.declare_parameter('motor_type', 0)
        self.declare_parameter('deadzone', 0)
        self.declare_parameter('encoder_lines', 0)
        self.declare_parameter('motor_phase', 0)
        self.declare_parameter('wheel_diameter_mm', 100)
        self.declare_parameter('upload_mall', 0)
        self.declare_parameter('upload_mtep', 1)
        self.declare_parameter('upload_mspd', 1)

        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.max_linear = float(self.get_parameter('max_linear_speed').value)
        self.max_angular = float(self.get_parameter('max_angular_speed').value)
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self.publish_feedback = bool(self.get_parameter('publish_feedback').value)
        self.control_frequency = float(self.get_parameter('control_frequency').value)
        self.invert_left = bool(self.get_parameter('invert_left').value)
        self.invert_right = bool(self.get_parameter('invert_right').value)

        self.channel_fl = int(self.get_parameter('channel_fl').value)
        self.channel_fr = int(self.get_parameter('channel_fr').value)
        self.channel_rl = int(self.get_parameter('channel_rl').value)
        self.channel_rr = int(self.get_parameter('channel_rr').value)

        self._serial: Optional['serial.Serial'] = None
        self._rx_buffer = ''
        self._last_cmd_stamp = self.get_clock().now()
        self._target_vr = 0.0
        self._target_vl = 0.0
        self._last_speed_mps = [0.0, 0.0, 0.0, 0.0]
        self._last_encoder = [0, 0, 0, 0]

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.wheel_speed_pub = self.create_publisher(TwistStamped, '/wheel_speeds', 10)
        self.motor_speed_pub = self.create_publisher(Float32MultiArray, '/driver/motor_speeds', 10)
        self.motor_encoder_pub = self.create_publisher(Int32MultiArray, '/driver/motor_encoders', 10)

        self._open_serial()
        self._apply_board_config()

        period = 1.0 / max(self.control_frequency, 1.0)
        self.create_timer(period, self._update)

        self.get_logger().info(
            f'Yahboom base ready port={self.serial_port} baud={self.baudrate} '
            f'wheel_base={self.wheel_base:.3f}'
        )

    def _open_serial(self):
        if serial is None:
            self.get_logger().error('pyserial not installed, cannot open Yahboom USB driver.')
            return

        try:
            self._serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=0.0,
                write_timeout=0.1,
            )
        except Exception as exc:  # pragma: no cover - hardware dependency
            self._serial = None
            self.get_logger().error(f'Open serial failed: {exc}')

    def _apply_board_config(self):
        if self._serial is None:
            return
        if not bool(self.get_parameter('apply_board_config').value):
            self._send_upload_flags()
            return

        self._send_optional_scalar('mtype', int(self.get_parameter('motor_type').value))
        self._send_optional_scalar('deadzone', int(self.get_parameter('deadzone').value))
        self._send_optional_scalar('mline', int(self.get_parameter('encoder_lines').value))
        self._send_optional_scalar('mphase', int(self.get_parameter('motor_phase').value))
        self._send_optional_scalar('wdiameter', int(self.get_parameter('wheel_diameter_mm').value))
        self._send_upload_flags()

    def _send_optional_scalar(self, tag: str, value: int):
        if value > 0:
            self._send_frame(f'{tag}:{value}')

    def _send_upload_flags(self):
        mall = int(self.get_parameter('upload_mall').value)
        mtep = int(self.get_parameter('upload_mtep').value)
        mspd = int(self.get_parameter('upload_mspd').value)
        self._send_frame(f'upload:{mall},{mtep},{mspd}')

    def _cmd_cb(self, msg: Twist):
        linear = max(min(msg.linear.x, self.max_linear), -self.max_linear)
        angular = max(min(msg.angular.z, self.max_angular), -self.max_angular)

        half_track = self.wheel_base * 0.5
        self._target_vr = linear + angular * half_track
        self._target_vl = linear - angular * half_track
        self._last_cmd_stamp = self.get_clock().now()

    def _update(self):
        self._poll_feedback()

        age = (self.get_clock().now() - self._last_cmd_stamp).nanoseconds * 1e-9
        if age > self.cmd_vel_timeout:
            vr = 0.0
            vl = 0.0
        else:
            vr = self._target_vr
            vl = self._target_vl

        self._send_wheel_speeds(vr, vl)

    def _send_wheel_speeds(self, vr_mps: float, vl_mps: float):
        right_mm_s = int(round(vr_mps * 1000.0))
        left_mm_s = int(round(vl_mps * 1000.0))

        speeds = [0, 0, 0, 0]
        speeds[self.channel_fl - 1] = -left_mm_s if self.invert_left else left_mm_s
        speeds[self.channel_rl - 1] = -left_mm_s if self.invert_left else left_mm_s
        speeds[self.channel_fr - 1] = -right_mm_s if self.invert_right else right_mm_s
        speeds[self.channel_rr - 1] = -right_mm_s if self.invert_right else right_mm_s
        self._send_frame(f'spd:{speeds[0]},{speeds[1]},{speeds[2]},{speeds[3]}')

    def _send_frame(self, body: str):
        if self._serial is None:
            return
        try:
            self._serial.write(f'${body}#'.encode('ascii'))
        except Exception as exc:  # pragma: no cover - hardware dependency
            self.get_logger().error(f'Serial write failed: {exc}')

    def _poll_feedback(self):
        if self._serial is None:
            return
        try:
            waiting = self._serial.in_waiting
            if waiting <= 0:
                return
            data = self._serial.read(waiting).decode('ascii', errors='ignore')
        except Exception as exc:  # pragma: no cover - hardware dependency
            self.get_logger().error(f'Serial read failed: {exc}')
            return

        self._rx_buffer += data
        while True:
            start = self._rx_buffer.find('$')
            end = self._rx_buffer.find('#', start + 1)
            if start < 0 or end < 0:
                if len(self._rx_buffer) > 256:
                    self._rx_buffer = self._rx_buffer[-256:]
                return
            body = self._rx_buffer[start + 1:end]
            self._rx_buffer = self._rx_buffer[end + 1:]
            self._parse_frame(body)

    def _parse_frame(self, body: str):
        if ':' not in body:
            return
        tag, payload = body.split(':', 1)
        parts = payload.split(',')

        if tag == 'MSPD' and len(parts) >= 4:
            try:
                mm_s = [float(parts[i]) for i in range(4)]
            except ValueError:
                return
            self._last_speed_mps = [v / 1000.0 for v in mm_s]
            self._publish_speed_feedback()
        elif tag == 'MTEP' and len(parts) >= 4:
            try:
                self._last_encoder = [int(parts[i]) for i in range(4)]
            except ValueError:
                return
            self.motor_encoder_pub.publish(Int32MultiArray(data=self._last_encoder))
        elif tag == 'MAll' and len(parts) >= 8:
            try:
                self._last_encoder = [int(parts[i]) for i in range(4)]
                self._last_speed_mps = [float(parts[i + 4]) / 1000.0 for i in range(4)]
            except ValueError:
                return
            self.motor_encoder_pub.publish(Int32MultiArray(data=self._last_encoder))
            self._publish_speed_feedback()

    def _publish_speed_feedback(self):
        self.motor_speed_pub.publish(Float32MultiArray(data=self._last_speed_mps))
        if not self.publish_feedback:
            return

        left = self._signed_channel_speed(self.channel_fl, self.invert_left)
        left_rear = self._signed_channel_speed(self.channel_rl, self.invert_left)
        right = self._signed_channel_speed(self.channel_fr, self.invert_right)
        right_rear = self._signed_channel_speed(self.channel_rr, self.invert_right)

        v_l = 0.5 * (left + left_rear)
        v_r = 0.5 * (right + right_rear)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = v_r
        msg.twist.linear.y = v_l
        msg.twist.angular.z = (v_r - v_l) / self.wheel_base if self.wheel_base > 1e-6 else 0.0
        self.wheel_speed_pub.publish(msg)

    def _signed_channel_speed(self, channel: int, invert: bool) -> float:
        if channel < 1 or channel > 4:
            return 0.0
        speed = self._last_speed_mps[channel - 1]
        return -speed if invert else speed

    def destroy_node(self):
        try:
            self._send_frame('spd:0,0,0,0')
        finally:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YahboomBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
