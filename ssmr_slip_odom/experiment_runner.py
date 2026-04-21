"""
Experiment runner node: publishes /cmd_vel for predefined motion patterns.

Supported modes: figure8, spin, turn90
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ExperimentRunner(Node):

    def __init__(self):
        super().__init__('experiment_runner')

        self.declare_parameter('mode', 'figure8')
        self.declare_parameter('duration', 30.0)
        self.declare_parameter('v', 0.3)
        self.declare_parameter('w', 0.8)
        self.declare_parameter('repeat', 1)
        self.declare_parameter('stop_at_end', True)

        self.mode = self.get_parameter('mode').value
        self.duration = self.get_parameter('duration').value
        self.v_param = self.get_parameter('v').value
        self.w_param = self.get_parameter('w').value
        self.repeat = self.get_parameter('repeat').value
        self.stop_at_end = self.get_parameter('stop_at_end').value

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.t0 = self.get_clock().now()
        self.elapsed = 0.0
        self.current_repeat = 0
        self.finished = False

        # turn90 sub-state
        self.turn90_phase = 'straight1'
        self.phase_start = 0.0

        self.dt = 0.02  # 50 Hz
        self.timer = self.create_timer(self.dt, self._tick)

        self.get_logger().info(
            f'ExperimentRunner  mode={self.mode}  duration={self.duration}  '
            f'v={self.v_param}  w={self.w_param}  repeat={self.repeat}')

    def _tick(self):
        if self.finished:
            return

        now = self.get_clock().now()
        self.elapsed = (now - self.t0).nanoseconds * 1e-9

        # Total time across all repeats
        total_duration = self.duration * self.repeat
        if self.elapsed >= total_duration:
            if self.stop_at_end:
                self._publish(0.0, 0.0)
            self.finished = True
            self.get_logger().info('Experiment finished.')
            return

        # Time within current repeat cycle
        t_cycle = self.elapsed % self.duration

        cmd = Twist()

        if self.mode == 'figure8':
            cmd = self._figure8(t_cycle)
        elif self.mode == 'spin':
            cmd = self._spin()
        elif self.mode == 'turn90':
            cmd = self._turn90(t_cycle)
        else:
            self.get_logger().error(f'Unknown mode: {self.mode}')
            self.finished = True
            return

        self.cmd_pub.publish(cmd)

    def _figure8(self, t: float) -> Twist:
        """Figure-8: constant linear speed, sinusoidal angular velocity."""
        period = self.duration
        omega = self.w_param * math.sin(2.0 * math.pi * t / period)
        cmd = Twist()
        cmd.linear.x = self.v_param
        cmd.angular.z = omega
        return cmd

    def _spin(self) -> Twist:
        """Spin in place: zero linear, constant angular."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = self.w_param
        return cmd

    def _turn90(self, t: float) -> Twist:
        """Straight -> 90-deg turn in place -> straight.

        Segments split the duration into thirds:
          [0, dur/3)          straight
          [dur/3, 2*dur/3)    rotate 90 deg
          [2*dur/3, dur)      straight
        """
        seg = self.duration / 3.0
        cmd = Twist()

        if t < seg:
            # Straight ahead
            cmd.linear.x = self.v_param
            cmd.angular.z = 0.0
        elif t < 2.0 * seg:
            # Turn 90 degrees in place: omega = (pi/2) / seg
            cmd.linear.x = 0.0
            cmd.angular.z = (math.pi / 2.0) / seg
        else:
            # Straight ahead again
            cmd.linear.x = self.v_param
            cmd.angular.z = 0.0

        return cmd

    def _publish(self, linear: float, angular: float):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
