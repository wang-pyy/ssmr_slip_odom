import select
import sys
import termios
import threading
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class WasdTeleopNode(Node):
    def __init__(self):
        super().__init__('wasd_teleop_node')

        self.declare_parameter('linear_speed', 0.20)
        self.declare_parameter('angular_speed', 1.20)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('key_timeout', 0.15)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.key_timeout = float(self.get_parameter('key_timeout').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._lock = threading.Lock()
        self._last_key_time = 0.0
        self._target_linear = 0.0
        self._target_angular = 0.0

        period = 1.0 / max(self.publish_rate, 1.0)
        self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'wasd_teleop_node started: '
            f'linear_speed={self.linear_speed:.3f}, '
            f'angular_speed={self.angular_speed:.3f}, '
            f'publish_rate={self.publish_rate:.1f}, '
            f'key_timeout={self.key_timeout:.2f}'
        )

    def set_cmd(self, linear_x: float, angular_z: float):
        with self._lock:
            self._target_linear = linear_x
            self._target_angular = angular_z
            self._last_key_time = time.monotonic()

    def stop_now(self):
        with self._lock:
            self._target_linear = 0.0
            self._target_angular = 0.0
            self._last_key_time = 0.0

        msg = Twist()
        self.cmd_pub.publish(msg)

    def _on_timer(self):
        now = time.monotonic()
        msg = Twist()

        with self._lock:
            if now - self._last_key_time <= self.key_timeout:
                msg.linear.x = self._target_linear
                msg.angular.z = self._target_angular
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0

        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WasdTeleopNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    print('\n=== WASD Teleop ===')
    print('w: 前进')
    print('s: 后退')
    print('a: 左转')
    print('d: 右转')
    print('space: 立即停车')
    print('q: 退出')
    print('松手后会自动停车\n')

    try:
        tty.setraw(fd)

        while rclpy.ok():
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not rlist:
                continue

            ch = sys.stdin.read(1)

            if ch in ('w', 'W'):
                node.set_cmd(node.linear_speed, 0.0)
            elif ch in ('s', 'S'):
                node.set_cmd(-node.linear_speed, 0.0)
            elif ch in ('a', 'A'):
                node.set_cmd(0.0, node.angular_speed)
            elif ch in ('d', 'D'):
                node.set_cmd(0.0, -node.angular_speed)
            elif ch == ' ':
                node.stop_now()
            elif ch in ('q', 'Q', '\x03'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.stop_now()
        time.sleep(0.1)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
