"""
Slip-aware odometry node for skid-steer mobile robots.

Fuses wheel encoder and IMU data using a sigmoid-weighted slip estimator
to produce robust odometry in the presence of wheel slip.
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped, Vector3Stamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import tf2_ros


class SlipOdomNode(Node):

    def __init__(self):
        super().__init__('slip_odom_node')

        # -- Declare parameters --
        self.declare_parameter('wheel_base', 0.3)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('encoder_mode', 'A')
        self.declare_parameter('window_size', 20)
        self.declare_parameter('imu_bias_calib_seconds', 3.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('use_sigmoid_params_fixed', True)
        self.declare_parameter('sigmoid_s0', 0.175)
        self.declare_parameter('sigmoid_k', 8.79)

        # -- Read parameters --
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.encoder_mode = self.get_parameter('encoder_mode').value.upper()
        self.window_size = self.get_parameter('window_size').value
        self.calib_seconds = self.get_parameter('imu_bias_calib_seconds').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.use_fixed_sigmoid = self.get_parameter('use_sigmoid_params_fixed').value
        self.sigmoid_s0 = self.get_parameter('sigmoid_s0').value
        self.sigmoid_k = self.get_parameter('sigmoid_k').value

        # -- State variables --
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.v_enc = 0.0
        self.omega_enc = 0.0
        self.omega_imu_raw = 0.0

        self.last_enc_time = None   # Time object
        self.last_imu_time = None
        self.last_fuse_stamp = None  # rclpy.time.Time for dt calculation
        self.enc_received = False
        self.imu_received = False

        # IMU bias calibration
        self.imu_bias = 0.0
        self.bias_samples = []
        self.bias_calibrated = False
        self.calib_start_time = None

        # Sliding window for s_bar
        self.s_window = deque(maxlen=self.window_size)

        # -- Publishers --
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.pub_s_raw = self.create_publisher(Float32, '/slip/s_raw', 10)
        self.pub_s_bar = self.create_publisher(Float32, '/slip/s_bar', 10)
        self.pub_lambda = self.create_publisher(Float32, '/slip/lambda', 10)
        self.pub_w_enc = self.create_publisher(Float32, '/slip/w_enc', 10)
        self.pub_w_imu = self.create_publisher(Float32, '/slip/w_imu', 10)
        self.pub_w_fused = self.create_publisher(Float32, '/slip/w_fused', 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # -- Subscribers --
        self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)

        if self.encoder_mode == 'A':
            self.create_subscription(
                TwistStamped, '/wheel_speeds', self._wheel_speeds_cb, 10)
            self.get_logger().info('Encoder mode A: subscribing to /wheel_speeds (TwistStamped)')
        elif self.encoder_mode == 'B':
            self.create_subscription(
                Vector3Stamped, '/wheel_omega', self._wheel_omega_cb, 10)
            self.get_logger().info('Encoder mode B: subscribing to /wheel_omega (Vector3Stamped)')
        else:
            self.get_logger().error(
                f'Unknown encoder_mode "{self.encoder_mode}". Use "A" or "B".')

        # Timer for odometry update at 50 Hz
        self.create_timer(0.02, self._update)

        self.get_logger().info(
            f'SlipOdomNode started  wheel_base={self.wheel_base}  '
            f'wheel_radius={self.wheel_radius}  encoder_mode={self.encoder_mode}')

    # ------------------------------------------------------------------ #
    #  Callbacks                                                          #
    # ------------------------------------------------------------------ #

    def _imu_cb(self, msg: Imu):
        self.omega_imu_raw = msg.angular_velocity.z
        self.imu_received = True
        self.last_imu_time = Time.from_msg(msg.header.stamp)

        # Bias calibration: collect samples during first N seconds
        if not self.bias_calibrated:
            if self.calib_start_time is None:
                self.calib_start_time = self.last_imu_time
            elapsed = (self.last_imu_time - self.calib_start_time).nanoseconds * 1e-9
            if elapsed < self.calib_seconds:
                self.bias_samples.append(self.omega_imu_raw)
            else:
                if len(self.bias_samples) > 0:
                    self.imu_bias = sum(self.bias_samples) / len(self.bias_samples)
                self.bias_calibrated = True
                self.get_logger().info(
                    f'IMU bias calibrated: {self.imu_bias:.6f} rad/s  '
                    f'({len(self.bias_samples)} samples)')

    def _wheel_speeds_cb(self, msg: TwistStamped):
        """Mode A: v_r, v_l already in m/s."""
        v_r = msg.twist.linear.x
        v_l = msg.twist.linear.y
        self._process_encoder(v_r, v_l, Time.from_msg(msg.header.stamp))

    def _wheel_omega_cb(self, msg: Vector3Stamped):
        """Mode B: omega_r, omega_l in rad/s -> convert via wheel_radius."""
        v_r = msg.vector.x * self.wheel_radius
        v_l = msg.vector.y * self.wheel_radius
        self._process_encoder(v_r, v_l, Time.from_msg(msg.header.stamp))

    def _process_encoder(self, v_r: float, v_l: float, stamp: Time):
        self.v_enc = (v_r + v_l) / 2.0
        self.omega_enc = (v_r - v_l) / self.wheel_base
        self.enc_received = True
        self.last_enc_time = stamp

    # ------------------------------------------------------------------ #
    #  Main update loop                                                   #
    # ------------------------------------------------------------------ #

    def _update(self):
        # Don't integrate until both sources have been received and bias is done
        if not (self.enc_received and self.imu_received and self.bias_calibrated):
            return

        now = self.get_clock().now()

        # Compute dt from previous update
        if self.last_fuse_stamp is None:
            self.last_fuse_stamp = now
            return
        dt = (now - self.last_fuse_stamp).nanoseconds * 1e-9
        self.last_fuse_stamp = now

        if dt <= 0.0 or dt > 1.0:
            return  # skip unreasonable dt

        # Corrected IMU angular velocity
        omega_imu_c = self.omega_imu_raw - self.imu_bias

        # Slip metric
        s_raw = abs(omega_imu_c - self.omega_enc)
        self.s_window.append(s_raw)
        s_bar = sum(self.s_window) / len(self.s_window)

        # Sigmoid weight
        k = self.sigmoid_k if not self.use_fixed_sigmoid else 8.79
        s0 = self.sigmoid_s0 if not self.use_fixed_sigmoid else 0.175
        exponent = -k * (s_bar - s0)
        # clamp exponent to avoid overflow
        exponent = max(min(exponent, 500.0), -500.0)
        lam = 1.0 / (1.0 + math.exp(exponent))

        # Fused angular velocity
        omega_fused = (1.0 - lam) * self.omega_enc + lam * omega_imu_c

        # Integrate pose
        self.theta += omega_fused * dt
        # Wrap theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        self.x += self.v_enc * math.cos(self.theta) * dt
        self.y += self.v_enc * math.sin(self.theta) * dt

        # -- Build Odometry message --
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Quaternion from yaw
        cy = math.cos(self.theta * 0.5)
        sy = math.sin(self.theta * 0.5)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy

        odom.twist.twist.linear.x = self.v_enc
        odom.twist.twist.angular.z = omega_fused

        # Pose covariance (6x6, row-major, indices: x=0, y=1, z=2, roll=3, pitch=4, yaw=5)
        s_bar2 = s_bar * s_bar
        sigma_x2 = 4e-4 + 0.1067 * s_bar2
        sigma_y2 = 4e-4 + 0.1067 * s_bar2
        sigma_yaw2 = 3.73e-5 + 0.0842 * s_bar2

        cov = [0.0] * 36
        cov[0] = sigma_x2        # (0,0) x
        cov[7] = sigma_y2        # (1,1) y
        cov[14] = 1e3            # (2,2) z
        cov[21] = 1e3            # (3,3) roll
        cov[28] = 1e3            # (4,4) pitch
        cov[35] = sigma_yaw2     # (5,5) yaw
        odom.pose.covariance = cov

        self.odom_pub.publish(odom)

        # -- TF broadcast --
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

        # -- Debug topics --
        self.pub_s_raw.publish(Float32(data=s_raw))
        self.pub_s_bar.publish(Float32(data=s_bar))
        self.pub_lambda.publish(Float32(data=lam))
        self.pub_w_enc.publish(Float32(data=self.omega_enc))
        self.pub_w_imu.publish(Float32(data=omega_imu_c))
        self.pub_w_fused.publish(Float32(data=omega_fused))


def main(args=None):
    rclpy.init(args=args)
    node = SlipOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
