"""
Slip-aware odometry node for skid-steer mobile robots.

Fuses wheel encoder and IMU data using a sigmoid-weighted slip estimator
to produce odometry in the presence of wheel slip.

Modified version:
- Adds encoder_yaw_scale for yaw calibration.
- Adds lambda_min/lambda_max to avoid high lambda at zero slip.
- Adds slip_deadband and w_deadband to suppress straight-line noise.
- Replaces sliding-window s_bar with asymmetric low-pass filtering.
- Uses exact integration for turning motion.
"""

import math

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

        # ------------------------------------------------------------------
        # Declare parameters
        # ------------------------------------------------------------------
        self.declare_parameter('wheel_base', 0.3)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('encoder_mode', 'A')
        self.declare_parameter('window_size', 20)
        self.declare_parameter('imu_bias_calib_seconds', 3.0)
        self.declare_parameter('publish_tf', True)

        # Kept for compatibility with old YAML files.
        # This version uses sigmoid_s0 and sigmoid_k from YAML directly.
        self.declare_parameter('use_sigmoid_params_fixed', False)

        # Sigmoid parameters
        self.declare_parameter('sigmoid_s0', 0.25)
        self.declare_parameter('sigmoid_k', 12.0)

        # New tuning parameters
        self.declare_parameter('encoder_yaw_scale', 1.0)
        self.declare_parameter('lambda_min', 0.05)
        self.declare_parameter('lambda_max', 0.90)
        self.declare_parameter('slip_deadband', 0.04)
        self.declare_parameter('w_deadband', 0.015)
        self.declare_parameter('alpha_up', 0.10)
        self.declare_parameter('alpha_down', 0.25)

        # Optional linear velocity correction.
        # Default 0.0 means disabled.
        # Kept for backward compatibility with old YAML files.
        # The main velocity correction model is now the nonlinear one below.
        self.declare_parameter('linear_slip_gain', 0.0)

        # ------------------------------------------------------------------
        # New: nonlinear velocity correction parameters
        # v_eff = v_enc * exp(-s_bar^2 / (2*sigma_s^2))
        #              * max(0, 1 - gamma * |omega_imu|)
        # ------------------------------------------------------------------
        self.declare_parameter('use_nonlinear_velocity_correction', True)
        self.declare_parameter('slip_sigma_s', 0.35)
        self.declare_parameter('turn_gamma', 0.20)

        # ------------------------------------------------------------------
        # New: ICR / effective track width on-line estimation
        # B_eff is updated only when both omega_imu and delta_v are large
        # enough; otherwise b_eff keeps its previous value.
        # b_eff_init / beff_update_w_min default to wheel_base / w_deadband.
        # ------------------------------------------------------------------
        self.declare_parameter('use_icr_beff', True)
        self.declare_parameter('beff_alpha', 0.08)
        self.declare_parameter('beff_min', 0.15)
        self.declare_parameter('beff_max', 1.20)
        self.declare_parameter(
            'beff_init',
            float(self.get_parameter('wheel_base').value)
        )
        self.declare_parameter(
            'beff_update_w_min',
            float(self.get_parameter('w_deadband').value)
        )
        self.declare_parameter('beff_update_dv_min', 0.01)

        # ------------------------------------------------------------------
        # Read parameters
        # ------------------------------------------------------------------
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.encoder_mode = str(self.get_parameter('encoder_mode').value).upper()
        self.window_size = int(self.get_parameter('window_size').value)
        self.calib_seconds = float(self.get_parameter('imu_bias_calib_seconds').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.use_fixed_sigmoid = bool(self.get_parameter('use_sigmoid_params_fixed').value)
        self.sigmoid_s0 = float(self.get_parameter('sigmoid_s0').value)
        self.sigmoid_k = float(self.get_parameter('sigmoid_k').value)

        self.encoder_yaw_scale = float(self.get_parameter('encoder_yaw_scale').value)
        self.lambda_min = float(self.get_parameter('lambda_min').value)
        self.lambda_max = float(self.get_parameter('lambda_max').value)
        self.slip_deadband = float(self.get_parameter('slip_deadband').value)
        self.w_deadband = float(self.get_parameter('w_deadband').value)
        self.alpha_up = float(self.get_parameter('alpha_up').value)
        self.alpha_down = float(self.get_parameter('alpha_down').value)
        self.linear_slip_gain = float(self.get_parameter('linear_slip_gain').value)

        self.use_nonlinear_velocity_correction = bool(
            self.get_parameter('use_nonlinear_velocity_correction').value
        )
        self.slip_sigma_s = float(self.get_parameter('slip_sigma_s').value)
        self.turn_gamma = float(self.get_parameter('turn_gamma').value)

        self.use_icr_beff = bool(self.get_parameter('use_icr_beff').value)
        self.beff_alpha = float(self.get_parameter('beff_alpha').value)
        self.beff_min = float(self.get_parameter('beff_min').value)
        self.beff_max = float(self.get_parameter('beff_max').value)
        self.beff_init = float(self.get_parameter('beff_init').value)
        self.beff_update_w_min = float(
            self.get_parameter('beff_update_w_min').value
        )
        self.beff_update_dv_min = float(
            self.get_parameter('beff_update_dv_min').value
        )

        # Safety checks
        if self.wheel_base <= 0.0:
            self.get_logger().warn('wheel_base <= 0, reset to 0.3')
            self.wheel_base = 0.3

        if self.wheel_radius <= 0.0:
            self.get_logger().warn('wheel_radius <= 0, reset to 0.05')
            self.wheel_radius = 0.05

        if self.lambda_min < 0.0:
            self.lambda_min = 0.0

        if self.lambda_max > 1.0:
            self.lambda_max = 1.0

        if self.lambda_min > self.lambda_max:
            self.get_logger().warn('lambda_min > lambda_max, swapping them')
            self.lambda_min, self.lambda_max = self.lambda_max, self.lambda_min

        self.alpha_up = max(0.0, min(1.0, self.alpha_up))
        self.alpha_down = max(0.0, min(1.0, self.alpha_down))

        # Nonlinear velocity correction safety checks.
        if self.slip_sigma_s < 1e-6:
            self.get_logger().warn(
                'slip_sigma_s too small, reset to 1e-6'
            )
            self.slip_sigma_s = 1e-6

        if self.turn_gamma < 0.0:
            self.get_logger().warn('turn_gamma < 0, reset to 0')
            self.turn_gamma = 0.0

        # ICR / B_eff safety checks.
        self.beff_alpha = max(0.0, min(1.0, self.beff_alpha))

        if self.beff_min <= 0.0:
            self.get_logger().warn('beff_min <= 0, reset to 0.05')
            self.beff_min = 0.05

        if self.beff_max <= self.beff_min:
            self.get_logger().warn(
                'beff_max <= beff_min, reset beff_max = 2 * beff_min'
            )
            self.beff_max = self.beff_min * 2.0

        if self.beff_init <= 0.0:
            self.get_logger().warn(
                'beff_init <= 0, reset to wheel_base'
            )
            self.beff_init = self.wheel_base

        # Clip b_eff_init into [beff_min, beff_max].
        self.beff_init = max(
            self.beff_min, min(self.beff_max, self.beff_init)
        )

        if self.beff_update_w_min < 0.0:
            self.beff_update_w_min = 0.0

        if self.beff_update_dv_min < 0.0:
            self.beff_update_dv_min = 0.0

        # ------------------------------------------------------------------
        # State variables
        # ------------------------------------------------------------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.v_enc = 0.0
        self.omega_enc = 0.0
        self.omega_imu_raw = 0.0

        # Raw left/right wheel speeds (from encoder callback).
        self.v_r = 0.0
        self.v_l = 0.0
        self.delta_v = 0.0

        # Traditional fixed-wheel-base encoder yaw-rate, kept for debug only.
        self.omega_enc_nominal = 0.0

        # Online-estimated effective track width (ICR model).
        self.b_eff = self.beff_init
        self.last_b_eff_raw = self.beff_init

        self.last_enc_time = None
        self.last_imu_time = None
        self.last_fuse_stamp = None

        self.enc_received = False
        self.imu_received = False

        # IMU bias calibration
        self.imu_bias = 0.0
        self.bias_samples = []
        self.bias_calibrated = False
        self.calib_start_time = None

        # Filtered slip indicator
        self.s_bar = 0.0

        # Last debug values
        self.last_s_raw = 0.0
        self.last_lambda = self.lambda_min
        self.last_omega_imu_used = 0.0
        self.last_omega_enc_used = 0.0
        self.last_omega_fused = 0.0

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.pub_s_raw = self.create_publisher(Float32, '/slip/s_raw', 10)
        self.pub_s_bar = self.create_publisher(Float32, '/slip/s_bar', 10)
        self.pub_lambda = self.create_publisher(Float32, '/slip/lambda', 10)

        self.pub_w_enc = self.create_publisher(Float32, '/slip/w_enc', 10)
        self.pub_w_imu = self.create_publisher(Float32, '/slip/w_imu', 10)
        self.pub_w_fused = self.create_publisher(Float32, '/slip/w_fused', 10)

        # New debug publishers (ICR B_eff + nonlinear v correction).
        self.pub_b_eff = self.create_publisher(Float32, '/slip/b_eff', 10)
        self.pub_b_eff_raw = self.create_publisher(Float32, '/slip/b_eff_raw', 10)
        self.pub_w_enc_nominal = self.create_publisher(
            Float32, '/slip/w_enc_nominal', 10
        )
        self.pub_slip_decay = self.create_publisher(Float32, '/slip/slip_decay', 10)
        self.pub_turn_decay = self.create_publisher(Float32, '/slip/turn_decay', 10)
        self.pub_v_eff = self.create_publisher(Float32, '/slip/v_eff', 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)

        if self.encoder_mode == 'A':
            self.create_subscription(
                TwistStamped,
                '/wheel_speeds',
                self._wheel_speeds_cb,
                10
            )
            self.get_logger().info(
                'Encoder mode A: subscribing to /wheel_speeds '
                '(TwistStamped, linear.x=v_r, linear.y=v_l)'
            )

        elif self.encoder_mode == 'B':
            self.create_subscription(
                Vector3Stamped,
                '/wheel_omega',
                self._wheel_omega_cb,
                10
            )
            self.get_logger().info(
                'Encoder mode B: subscribing to /wheel_omega '
                '(Vector3Stamped, vector.x=omega_r, vector.y=omega_l)'
            )

        else:
            self.get_logger().error(
                f'Unknown encoder_mode "{self.encoder_mode}". Use "A" or "B".'
            )

        # Timer for odometry update at 50 Hz
        self.create_timer(0.02, self._update)

        self.get_logger().info(
            'SlipOdomNode started: '
            f'wheel_base={self.wheel_base:.4f}, '
            f'wheel_radius={self.wheel_radius:.4f}, '
            f'encoder_mode={self.encoder_mode}, '
            f'encoder_yaw_scale={self.encoder_yaw_scale:.3f}, '
            f'lambda_min={self.lambda_min:.3f}, '
            f'lambda_max={self.lambda_max:.3f}, '
            f'slip_deadband={self.slip_deadband:.3f}, '
            f'w_deadband={self.w_deadband:.3f}, '
            f'sigmoid_s0={self.sigmoid_s0:.3f}, '
            f'sigmoid_k={self.sigmoid_k:.3f}, '
            f'alpha_up={self.alpha_up:.3f}, '
            f'alpha_down={self.alpha_down:.3f}, '
            f'use_nonlinear_velocity_correction='
            f'{self.use_nonlinear_velocity_correction}, '
            f'slip_sigma_s={self.slip_sigma_s:.3f}, '
            f'turn_gamma={self.turn_gamma:.3f}, '
            f'use_icr_beff={self.use_icr_beff}, '
            f'beff_alpha={self.beff_alpha:.3f}, '
            f'beff_min={self.beff_min:.3f}, '
            f'beff_max={self.beff_max:.3f}, '
            f'beff_init={self.beff_init:.4f}'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _imu_cb(self, msg: Imu):
        self.omega_imu_raw = float(msg.angular_velocity.z)
        self.imu_received = True
        self.last_imu_time = Time.from_msg(msg.header.stamp)

        # Bias calibration: collect samples during first N seconds.
        # Important: the robot should remain stationary during this period.
        if not self.bias_calibrated:
            if self.calib_start_time is None:
                self.calib_start_time = self.last_imu_time

            elapsed = (self.last_imu_time - self.calib_start_time).nanoseconds * 1e-9

            if elapsed < self.calib_seconds:
                self.bias_samples.append(self.omega_imu_raw)
            else:
                if len(self.bias_samples) > 0:
                    self.imu_bias = sum(self.bias_samples) / len(self.bias_samples)
                else:
                    self.imu_bias = 0.0

                self.bias_calibrated = True
                self.get_logger().info(
                    f'IMU bias calibrated: {self.imu_bias:.6f} rad/s '
                    f'({len(self.bias_samples)} samples)'
                )

    def _wheel_speeds_cb(self, msg: TwistStamped):
        """
        Mode A:
        v_r and v_l are already in m/s.

        Convention:
        msg.twist.linear.x = right-side velocity
        msg.twist.linear.y = left-side velocity
        """
        v_r = float(msg.twist.linear.x)
        v_l = float(msg.twist.linear.y)
        self._process_encoder(v_r, v_l, Time.from_msg(msg.header.stamp))

    def _wheel_omega_cb(self, msg: Vector3Stamped):
        """
        Mode B:
        omega_r and omega_l are in rad/s, converted to m/s by wheel_radius.

        Convention:
        msg.vector.x = right wheel angular velocity
        msg.vector.y = left wheel angular velocity
        """
        v_r = float(msg.vector.x) * self.wheel_radius
        v_l = float(msg.vector.y) * self.wheel_radius
        self._process_encoder(v_r, v_l, Time.from_msg(msg.header.stamp))

    def _process_encoder(self, v_r: float, v_l: float, stamp: Time):
        """
        Process encoder-derived left/right velocities.

        Note: the final fused omega_enc is no longer computed here.
        It is recomputed in _update() based on the online-estimated
        effective track width self.b_eff (ICR model).
        Here we only store raw wheel speeds and the nominal
        (fixed-wheel_base) omega for debug purposes.
        """
        self.v_r = v_r
        self.v_l = v_l
        self.delta_v = v_r - v_l

        self.v_enc = (v_r + v_l) / 2.0

        self.omega_enc_nominal = (
            self.encoder_yaw_scale * self.delta_v / self.wheel_base
        )

        self.enc_received = True
        self.last_enc_time = stamp

    # ------------------------------------------------------------------
    # Helper functions
    # ------------------------------------------------------------------

    @staticmethod
    def _deadband(value: float, threshold: float) -> float:
        if abs(value) < threshold:
            return 0.0
        return value

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _yaw_to_quaternion(yaw: float):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return 0.0, 0.0, sy, cy

    # ------------------------------------------------------------------
    # Main update loop
    # ------------------------------------------------------------------

    def _update(self):
        # Do not integrate until both sources have been received and IMU bias is done.
        if not (self.enc_received and self.imu_received and self.bias_calibrated):
            return

        now = self.get_clock().now()

        # Compute dt from previous update.
        if self.last_fuse_stamp is None:
            self.last_fuse_stamp = now
            return

        dt = (now - self.last_fuse_stamp).nanoseconds * 1e-9
        self.last_fuse_stamp = now

        if dt <= 0.0 or dt > 1.0:
            return

        # --------------------------------------------------------------
        # 1. Correct IMU angular velocity
        # --------------------------------------------------------------
        omega_imu_c = self.omega_imu_raw - self.imu_bias

        # Apply small angular velocity deadband.
        # This suppresses tiny straight-line jitter.
        omega_imu_use = self._deadband(omega_imu_c, self.w_deadband)

        # --------------------------------------------------------------
        # 2. Online ICR / effective track width estimation
        # --------------------------------------------------------------
        # B_eff is updated only when:
        #   - ICR mode is enabled,
        #   - |omega_imu| is large enough to trust as ground-truth yaw,
        #   - |delta_v| is large enough to give a meaningful divisor,
        #   - the resulting b_eff_raw is in [beff_min, beff_max] AND POSITIVE.
        #
        # If b_eff_raw <= 0, the encoder yaw direction disagrees with the
        # IMU yaw direction. That points at a coordinate-frame / motor-sign
        # issue and we deliberately do NOT use abs() to hide it; we just
        # skip the update and keep the previous b_eff.
        #
        # encoder_yaw_scale is folded into b_eff_raw so that the downstream
        # omega_enc_use = encoder_yaw_scale * delta_v / b_eff converges to
        # omega_imu_use in steady state. Without this factor, b_eff would
        # under-/over-shoot by exactly encoder_yaw_scale and the fusion
        # would carry a systematic yaw bias.
        if self.use_icr_beff:
            if (abs(omega_imu_use) > self.beff_update_w_min
                    and abs(self.delta_v) > self.beff_update_dv_min):
                b_eff_raw = (
                    self.encoder_yaw_scale * self.delta_v / omega_imu_use
                )
                self.last_b_eff_raw = b_eff_raw

                if self.beff_min <= b_eff_raw <= self.beff_max:
                    self.b_eff = (
                        (1.0 - self.beff_alpha) * self.b_eff
                        + self.beff_alpha * b_eff_raw
                    )
        else:
            self.b_eff = self.wheel_base

        # Clamp b_eff defensively so downstream division is always safe.
        if self.b_eff < self.beff_min:
            self.b_eff = self.beff_min
        elif self.b_eff > self.beff_max:
            self.b_eff = self.beff_max

        # --------------------------------------------------------------
        # 3. Encoder yaw-rate from ICR model
        # --------------------------------------------------------------
        omega_icr_enc = self.delta_v / self.b_eff
        omega_enc_use = self._deadband(
            self.encoder_yaw_scale * omega_icr_enc, self.w_deadband
        )

        # Expose the ICR-corrected encoder yaw-rate as the canonical
        # self.omega_enc as well, so any external consumer sees the
        # correct value.
        self.omega_enc = self.encoder_yaw_scale * omega_icr_enc

        # --------------------------------------------------------------
        # 4. Slip metric with deadband
        # --------------------------------------------------------------
        # Important design decision:
        # s_raw must be computed against the NOMINAL fixed-wheel_base
        # encoder model, NOT against the ICR-corrected omega_enc_use.
        # Reason: B_eff is designed to absorb the slip discrepancy into
        # the angular kinematics, so by construction
        # omega_enc_use - omega_imu_use -> 0 once B_eff converges. If we
        # used omega_enc_use here, s_bar would collapse to zero exactly
        # when slip is most severe, killing both the v_eff decay and the
        # pose covariance growth.
        #
        # Causal chain after this change:
        #   omega_enc_nominal vs omega_imu  -> s_bar (slip severity)
        #   B_eff                           -> corrects omega integration
        #   s_bar, omega_imu                -> corrects v_eff
        #
        # Prerequisite: encoder_yaw_scale must be statically calibrated
        # so that omega_enc_nominal == omega_imu under no-slip conditions.
        # Otherwise s_bar will be permanently nonzero on straight lines.
        omega_nominal_use = self._deadband(
            self.omega_enc_nominal, self.w_deadband
        )
        diff = abs(omega_imu_use - omega_nominal_use)

        if diff < self.slip_deadband:
            s_raw = 0.0
        else:
            s_raw = diff - self.slip_deadband

        # --------------------------------------------------------------
        # 5. Asymmetric low-pass filter for s_bar
        # --------------------------------------------------------------
        # When s_raw increases, rise slowly to avoid reacting to encoder spikes.
        # When s_raw decreases, fall faster so lambda recovers after turning.
        if s_raw > self.s_bar:
            alpha = self.alpha_up
        else:
            alpha = self.alpha_down

        self.s_bar = self.s_bar + alpha * (s_raw - self.s_bar)
        s_bar = self.s_bar

        # --------------------------------------------------------------
        # 6. Sigmoid lambda
        # --------------------------------------------------------------
        # Use YAML parameters directly.
        k = self.sigmoid_k
        s0 = self.sigmoid_s0

        exponent = -k * (s_bar - s0)
        exponent = max(min(exponent, 500.0), -500.0)

        lambda_raw = 1.0 / (1.0 + math.exp(exponent))

        lam = self.lambda_min + (self.lambda_max - self.lambda_min) * lambda_raw
        lam = max(self.lambda_min, min(self.lambda_max, lam))

        # --------------------------------------------------------------
        # 7. Fuse angular velocity
        # --------------------------------------------------------------
        omega_fused = (1.0 - lam) * omega_enc_use + lam * omega_imu_use

        # --------------------------------------------------------------
        # 8. Nonlinear linear velocity correction
        # --------------------------------------------------------------
        # Paper model:
        #   v_actual = v_enc * exp(-s_bar^2 / (2 * sigma_s^2))
        #                    * max(0, 1 - gamma * |omega_imu|)
        # When use_nonlinear_velocity_correction is False we fall back
        # to v_eff = v_enc (no correction). linear_slip_gain is kept
        # only for backward compatibility and intentionally not used here.
        if self.use_nonlinear_velocity_correction:
            slip_decay = math.exp(
                -(s_bar * s_bar)
                / (2.0 * self.slip_sigma_s * self.slip_sigma_s)
            )
            turn_decay = max(
                0.0, 1.0 - self.turn_gamma * abs(omega_imu_use)
            )
            v_eff = self.v_enc * slip_decay * turn_decay
        else:
            slip_decay = 1.0
            turn_decay = 1.0
            v_eff = self.v_enc

        # Avoid reversing linear velocity due to correction.
        if self.v_enc >= 0.0:
            v_eff = max(0.0, v_eff)
        else:
            v_eff = min(0.0, v_eff)

        # --------------------------------------------------------------
        # 9. Exact integration for differential/skid-steer motion
        # --------------------------------------------------------------
        theta_old = self.theta
        dtheta = omega_fused * dt

        if abs(omega_fused) < 1e-6:
            dx = v_eff * math.cos(theta_old) * dt
            dy = v_eff * math.sin(theta_old) * dt
        else:
            dx = (v_eff / omega_fused) * (
                math.sin(theta_old + dtheta) - math.sin(theta_old)
            )
            dy = -(v_eff / omega_fused) * (
                math.cos(theta_old + dtheta) - math.cos(theta_old)
            )

        self.x += dx
        self.y += dy
        self.theta = self._wrap_angle(theta_old + dtheta)

        # Store debug values
        self.last_s_raw = s_raw
        self.last_lambda = lam
        self.last_omega_imu_used = omega_imu_use
        self.last_omega_enc_used = omega_enc_use
        self.last_omega_fused = omega_fused

        # --------------------------------------------------------------
        # 10. Build Odometry message
        # --------------------------------------------------------------
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        qx, qy, qz, qw = self._yaw_to_quaternion(self.theta)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v_eff
        odom.twist.twist.angular.z = omega_fused

        # Pose covariance.
        # Larger s_bar means less confidence in odometry.
        s_bar2 = s_bar * s_bar

        sigma_x2 = 4e-4 + 0.1067 * s_bar2
        sigma_y2 = 4e-4 + 0.1067 * s_bar2
        sigma_yaw2 = 3.73e-5 + 0.0842 * s_bar2

        cov = [0.0] * 36
        cov[0] = sigma_x2       # x
        cov[7] = sigma_y2       # y
        cov[14] = 1e3           # z
        cov[21] = 1e3           # roll
        cov[28] = 1e3           # pitch
        cov[35] = sigma_yaw2    # yaw
        odom.pose.covariance = cov

        self.odom_pub.publish(odom)

        # --------------------------------------------------------------
        # 11. TF broadcast
        # --------------------------------------------------------------
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

        # --------------------------------------------------------------
        # 12. Debug topics
        # --------------------------------------------------------------
        self.pub_s_raw.publish(Float32(data=float(s_raw)))
        self.pub_s_bar.publish(Float32(data=float(s_bar)))
        self.pub_lambda.publish(Float32(data=float(lam)))

        self.pub_w_enc.publish(Float32(data=float(omega_enc_use)))
        self.pub_w_imu.publish(Float32(data=float(omega_imu_use)))
        self.pub_w_fused.publish(Float32(data=float(omega_fused)))

        # ICR / B_eff debug
        self.pub_b_eff.publish(Float32(data=float(self.b_eff)))
        self.pub_b_eff_raw.publish(Float32(data=float(self.last_b_eff_raw)))
        self.pub_w_enc_nominal.publish(
            Float32(data=float(self.omega_enc_nominal))
        )

        # Nonlinear velocity correction debug
        self.pub_slip_decay.publish(Float32(data=float(slip_decay)))
        self.pub_turn_decay.publish(Float32(data=float(turn_decay)))
        self.pub_v_eff.publish(Float32(data=float(v_eff)))


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
