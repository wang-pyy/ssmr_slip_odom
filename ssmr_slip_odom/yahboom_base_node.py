from typing import Optional

import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float32MultiArray, Int32MultiArray

try:
    import serial
except ImportError:  # pragma: no cover - hardware dependency
    serial = None


class YahboomBaseNode(Node):
    """USB-serial base driver for Yahboom 4-channel motor controller.

    修改重点：
    1. 保留 /cmd_vel -> 左右轮速度的逻辑；
    2. 增加直行左右轮速度闭环修正；
    3. 停车时连续发送多次 0 速度，减少残余运动；
    4. 保留 /wheel_speeds 反馈发布，供 slip_odom_node 使用。
    """

    def __init__(self):
        super().__init__('yahboom_base_node')

        # ------------------------------------------------------------------
        # 基础参数
        # ------------------------------------------------------------------
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_base', 0.23)
        self.declare_parameter('max_linear_speed', 0.6)
        self.declare_parameter('max_angular_speed', 3.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('control_frequency', 50.0)

        # 电机通道映射
        self.declare_parameter('channel_fl', 1)
        self.declare_parameter('channel_fr', 2)
        self.declare_parameter('channel_rl', 3)
        self.declare_parameter('channel_rr', 4)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)

        # 反馈与驱动板配置
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

        # ------------------------------------------------------------------
        # 新增：直行闭环修正参数
        # ------------------------------------------------------------------
        self.declare_parameter('straight_balance_enable', True)

        # 比例系数：越大，左右轮速度差修正越强；太大会抖
        self.declare_parameter('straight_balance_kp', 0.35)

        # 左右速度差小于这个值时不修正，避免抖动
        self.declare_parameter('straight_balance_deadband', 0.01)  # m/s

        # 单次最大修正量，防止修正过猛
        self.declare_parameter('straight_balance_max_correction', 0.06)  # m/s

        # 线速度太小时不做闭环，避免低速编码器噪声影响
        self.declare_parameter('straight_balance_linear_min', 0.03)  # m/s

        # 只有 angular 接近 0 时才认为是直行
        self.declare_parameter('straight_balance_angular_threshold', 0.05)  # rad/s

        # 反馈超时，超过这个时间没有新速度反馈就不闭环
        self.declare_parameter('feedback_timeout', 0.25)  # s

        # 静态微调，默认不改
        self.declare_parameter('left_trim', 1.0)
        self.declare_parameter('right_trim', 1.0)

        # 停车时重复发送 0 速度
        self.declare_parameter('stop_burst_count', 5)
        self.declare_parameter('stop_burst_interval', 0.02)

        # ------------------------------------------------------------------
        # 读取参数
        # ------------------------------------------------------------------
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.max_linear = float(self.get_parameter('max_linear_speed').value)
        self.max_angular = float(self.get_parameter('max_angular_speed').value)
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self.control_frequency = float(self.get_parameter('control_frequency').value)

        self.channel_fl = int(self.get_parameter('channel_fl').value)
        self.channel_fr = int(self.get_parameter('channel_fr').value)
        self.channel_rl = int(self.get_parameter('channel_rl').value)
        self.channel_rr = int(self.get_parameter('channel_rr').value)

        self.invert_left = bool(self.get_parameter('invert_left').value)
        self.invert_right = bool(self.get_parameter('invert_right').value)
        self.publish_feedback = bool(self.get_parameter('publish_feedback').value)

        self.straight_balance_enable = bool(
            self.get_parameter('straight_balance_enable').value
        )
        self.straight_balance_kp = float(
            self.get_parameter('straight_balance_kp').value
        )
        self.straight_balance_deadband = float(
            self.get_parameter('straight_balance_deadband').value
        )
        self.straight_balance_max_correction = float(
            self.get_parameter('straight_balance_max_correction').value
        )
        self.straight_balance_linear_min = float(
            self.get_parameter('straight_balance_linear_min').value
        )
        self.straight_balance_angular_threshold = float(
            self.get_parameter('straight_balance_angular_threshold').value
        )
        self.feedback_timeout = float(self.get_parameter('feedback_timeout').value)

        self.left_trim = float(self.get_parameter('left_trim').value)
        self.right_trim = float(self.get_parameter('right_trim').value)

        self.stop_burst_count = int(self.get_parameter('stop_burst_count').value)
        self.stop_burst_interval = float(
            self.get_parameter('stop_burst_interval').value
        )

        # 安全限制
        self.wheel_base = max(self.wheel_base, 1e-6)
        self.control_frequency = max(self.control_frequency, 1.0)
        self.cmd_vel_timeout = max(self.cmd_vel_timeout, 0.05)

        self.straight_balance_kp = max(self.straight_balance_kp, 0.0)
        self.straight_balance_deadband = max(self.straight_balance_deadband, 0.0)
        self.straight_balance_max_correction = max(
            self.straight_balance_max_correction, 0.0
        )
        self.feedback_timeout = max(self.feedback_timeout, 0.02)

        self.left_trim = max(self.left_trim, 0.0)
        self.right_trim = max(self.right_trim, 0.0)

        self.stop_burst_count = max(self.stop_burst_count, 1)
        self.stop_burst_interval = max(self.stop_burst_interval, 0.0)

        half_track = self.wheel_base * 0.5
        self.max_wheel_speed = self.max_linear + self.max_angular * half_track

        # ------------------------------------------------------------------
        # 运行状态
        # ------------------------------------------------------------------
        self._serial: Optional['serial.Serial'] = None
        self._rx_buffer = ''

        self._last_cmd_stamp = self.get_clock().now()

        self._target_linear = 0.0
        self._target_angular = 0.0
        self._target_vr = 0.0
        self._target_vl = 0.0

        # 原始通道速度反馈，单位 m/s，顺序为 [1, 2, 3, 4]
        self._last_speed_mps = [0.0, 0.0, 0.0, 0.0]
        self._last_encoder = [0, 0, 0, 0]
        self._last_feedback_stamp = None

        self._was_moving = False

        # ------------------------------------------------------------------
        # ROS 接口
        # ------------------------------------------------------------------
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_cb,
            10
        )

        self.wheel_speed_pub = self.create_publisher(
            TwistStamped,
            '/wheel_speeds',
            10
        )
        self.motor_speed_pub = self.create_publisher(
            Float32MultiArray,
            '/driver/motor_speeds',
            10
        )
        self.motor_encoder_pub = self.create_publisher(
            Int32MultiArray,
            '/driver/motor_encoders',
            10
        )

        self._open_serial()
        self._apply_board_config()

        period = 1.0 / self.control_frequency
        self.create_timer(period, self._update)

        self.get_logger().info(
            f'Yahboom base ready port={self.serial_port} baud={self.baudrate} '
            f'wheel_base={self.wheel_base:.3f} '
            f'straight_balance_enable={self.straight_balance_enable} '
            f'straight_balance_kp={self.straight_balance_kp:.3f}'
        )

    # ------------------------------------------------------------------
    # 串口与驱动板配置
    # ------------------------------------------------------------------

    def _open_serial(self):
        if serial is None:
            self.get_logger().error(
                'pyserial not installed, cannot open Yahboom USB driver.'
            )
            return

        try:
            self._serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=0.0,
                write_timeout=0.1,
            )
        except Exception as exc:  # pragma: no cover
            self._serial = None
            self.get_logger().error(f'Open serial failed: {exc}')

    def _apply_board_config(self):
        if self._serial is None:
            return

        if not bool(self.get_parameter('apply_board_config').value):
            self._send_upload_flags()
            return

        self._send_optional_scalar(
            'mtype',
            int(self.get_parameter('motor_type').value)
        )
        self._send_optional_scalar(
            'deadzone',
            int(self.get_parameter('deadzone').value)
        )
        self._send_optional_scalar(
            'mline',
            int(self.get_parameter('encoder_lines').value)
        )
        self._send_optional_scalar(
            'mphase',
            int(self.get_parameter('motor_phase').value)
        )
        self._send_optional_scalar(
            'wdiameter',
            int(self.get_parameter('wheel_diameter_mm').value)
        )
        self._send_upload_flags()

    def _send_optional_scalar(self, tag: str, value: int):
        if value > 0:
            self._send_frame(f'{tag}:{value}')

    def _send_upload_flags(self):
        mall = int(self.get_parameter('upload_mall').value)
        mtep = int(self.get_parameter('upload_mtep').value)
        mspd = int(self.get_parameter('upload_mspd').value)
        self._send_frame(f'upload:{mall},{mtep},{mspd}')

    # ------------------------------------------------------------------
    # cmd_vel 处理
    # ------------------------------------------------------------------

    def _cmd_cb(self, msg: Twist):
        linear = self._clamp(
            float(msg.linear.x),
            -self.max_linear,
            self.max_linear
        )
        angular = self._clamp(
            float(msg.angular.z),
            -self.max_angular,
            self.max_angular
        )

        half_track = self.wheel_base * 0.5

        self._target_linear = linear
        self._target_angular = angular

        self._target_vr = linear + angular * half_track
        self._target_vl = linear - angular * half_track

        self._target_vr = self._clamp(
            self._target_vr,
            -self.max_wheel_speed,
            self.max_wheel_speed
        )
        self._target_vl = self._clamp(
            self._target_vl,
            -self.max_wheel_speed,
            self.max_wheel_speed
        )

        self._last_cmd_stamp = self.get_clock().now()

    def _update(self):
        self._poll_feedback()

        now = self.get_clock().now()
        age = (now - self._last_cmd_stamp).nanoseconds * 1e-9

        if age > self.cmd_vel_timeout:
            linear = 0.0
            angular = 0.0
            vr = 0.0
            vl = 0.0
        else:
            linear = self._target_linear
            angular = self._target_angular
            vr = self._target_vr
            vl = self._target_vl

        moving = abs(vr) > 1e-4 or abs(vl) > 1e-4

        if not moving:
            if self._was_moving:
                self._send_stop_burst()
            else:
                self._send_wheel_speeds(0.0, 0.0)

            self._was_moving = False
            return

        vr_cmd, vl_cmd = self._apply_straight_balance(
            vr_cmd=vr,
            vl_cmd=vl,
            linear=linear,
            angular=angular,
            now=now
        )

        # 静态微调，默认 left_trim/right_trim 都是 1.0
        vr_cmd *= self.right_trim
        vl_cmd *= self.left_trim

        vr_cmd = self._clamp(
            vr_cmd,
            -self.max_wheel_speed,
            self.max_wheel_speed
        )
        vl_cmd = self._clamp(
            vl_cmd,
            -self.max_wheel_speed,
            self.max_wheel_speed
        )

        self._send_wheel_speeds(vr_cmd, vl_cmd)
        self._was_moving = True

    def _apply_straight_balance(
        self,
        vr_cmd: float,
        vl_cmd: float,
        linear: float,
        angular: float,
        now
    ):
        """直行闭环修正。

        只有满足以下条件才启用：
        1. straight_balance_enable 为 true；
        2. 当前命令接近纯直行；
        3. 有近期轮速反馈；
        4. 线速度不是特别低。
        """

        if not self.straight_balance_enable:
            return vr_cmd, vl_cmd

        if abs(linear) < self.straight_balance_linear_min:
            return vr_cmd, vl_cmd

        if abs(angular) > self.straight_balance_angular_threshold:
            return vr_cmd, vl_cmd

        if not self._has_recent_feedback(now):
            return vr_cmd, vl_cmd

        v_l_actual, v_r_actual = self._get_side_speeds()

        # 正值表示右侧实际速度大于左侧
        speed_error = v_r_actual - v_l_actual

        if abs(speed_error) < self.straight_balance_deadband:
            return vr_cmd, vl_cmd

        correction = self.straight_balance_kp * speed_error
        correction = self._clamp(
            correction,
            -self.straight_balance_max_correction,
            self.straight_balance_max_correction
        )

        # 如果右侧更快，就降低右侧命令、提高左侧命令
        vr_cmd -= correction
        vl_cmd += correction

        return vr_cmd, vl_cmd

    # ------------------------------------------------------------------
    # 电机速度下发
    # ------------------------------------------------------------------

    def _send_wheel_speeds(self, vr_mps: float, vl_mps: float):
        right_mm_s = int(round(vr_mps * 1000.0))
        left_mm_s = int(round(vl_mps * 1000.0))

        speeds = [0, 0, 0, 0]

        speeds[self.channel_fl - 1] = (
            -left_mm_s if self.invert_left else left_mm_s
        )
        speeds[self.channel_rl - 1] = (
            -left_mm_s if self.invert_left else left_mm_s
        )
        speeds[self.channel_fr - 1] = (
            -right_mm_s if self.invert_right else right_mm_s
        )
        speeds[self.channel_rr - 1] = (
            -right_mm_s if self.invert_right else right_mm_s
        )

        self._send_frame(
            f'spd:{speeds[0]},{speeds[1]},{speeds[2]},{speeds[3]}'
        )

    def _send_stop_burst(self):
        for idx in range(self.stop_burst_count):
            self._send_wheel_speeds(0.0, 0.0)

            if idx + 1 < self.stop_burst_count and self.stop_burst_interval > 0.0:
                time.sleep(self.stop_burst_interval)

    def _send_frame(self, body: str):
        if self._serial is None:
            return

        try:
            self._serial.write(f'${body}#'.encode('ascii'))
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f'Serial write failed: {exc}')

    # ------------------------------------------------------------------
    # 反馈解析
    # ------------------------------------------------------------------

    def _poll_feedback(self):
        if self._serial is None:
            return

        try:
            waiting = self._serial.in_waiting
            if waiting <= 0:
                return

            data = self._serial.read(waiting).decode(
                'ascii',
                errors='ignore'
            )
        except Exception as exc:  # pragma: no cover
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
            self._last_feedback_stamp = self.get_clock().now()
            self._publish_speed_feedback()

        elif tag == 'MTEP' and len(parts) >= 4:
            try:
                self._last_encoder = [int(parts[i]) for i in range(4)]
            except ValueError:
                return

            self.motor_encoder_pub.publish(
                Int32MultiArray(data=self._last_encoder)
            )

        elif tag == 'MAll' and len(parts) >= 8:
            try:
                self._last_encoder = [int(parts[i]) for i in range(4)]
                self._last_speed_mps = [
                    float(parts[i + 4]) / 1000.0 for i in range(4)
                ]
            except ValueError:
                return

            self._last_feedback_stamp = self.get_clock().now()

            self.motor_encoder_pub.publish(
                Int32MultiArray(data=self._last_encoder)
            )
            self._publish_speed_feedback()

    def _publish_speed_feedback(self):
        self.motor_speed_pub.publish(
            Float32MultiArray(data=self._last_speed_mps)
        )

        if not self.publish_feedback:
            return

        v_l, v_r = self._get_side_speeds()

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # 约定：linear.x = 右侧速度，linear.y = 左侧速度
        msg.twist.linear.x = v_r
        msg.twist.linear.y = v_l
        msg.twist.angular.z = (
            (v_r - v_l) / self.wheel_base
            if self.wheel_base > 1e-6
            else 0.0
        )

        self.wheel_speed_pub.publish(msg)

    def _get_side_speeds(self):
        left_front = self._signed_channel_speed(
            self.channel_fl,
            self.invert_left
        )
        left_rear = self._signed_channel_speed(
            self.channel_rl,
            self.invert_left
        )
        right_front = self._signed_channel_speed(
            self.channel_fr,
            self.invert_right
        )
        right_rear = self._signed_channel_speed(
            self.channel_rr,
            self.invert_right
        )

        v_l = 0.5 * (left_front + left_rear)
        v_r = 0.5 * (right_front + right_rear)

        return v_l, v_r

    def _signed_channel_speed(self, channel: int, invert: bool) -> float:
        if channel < 1 or channel > 4:
            return 0.0

        speed = self._last_speed_mps[channel - 1]
        return -speed if invert else speed

    def _has_recent_feedback(self, now) -> bool:
        if self._last_feedback_stamp is None:
            return False

        age = (now - self._last_feedback_stamp).nanoseconds * 1e-9
        return age <= self.feedback_timeout

    # ------------------------------------------------------------------
    # 工具与退出
    # ------------------------------------------------------------------

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(min(value, high), low)

    def destroy_node(self):
        try:
            self._send_stop_burst()
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
