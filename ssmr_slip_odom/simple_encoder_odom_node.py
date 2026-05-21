"""
Baseline encoder-only differential / skid-steer odometry node.

Subscribes:
    /wheel_speeds  (geometry_msgs/TwistStamped)
        linear.x = right wheel linear speed v_r (m/s)
        linear.y = left  wheel linear speed v_l (m/s)

Publishes:
    /odom          (nav_msgs/Odometry, default; can be remapped)
    /tf            (odom -> base_link, if publish_tf=True)

This is the BASELINE: no IMU fusion, no slip awareness. It is intended
as an apples-to-apples comparison against slip_odom_node when running
slam_toolbox in long-corridor scenarios.

NOTE: Only one node may publish odom -> base_link at a time. Set
publish_tf=False on this node if slip_odom_node is also active.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros


class SimpleEncoderOdomNode(Node):

    def __init__(self):
        super().__init__('simple_encoder_odom_node')

        self.declare_parameter('wheel_base', 0.13)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('wheel_speeds_topic', '/wheel_speeds')

        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        wheel_topic = str(self.get_parameter('wheel_speeds_topic').value)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_stamp = None

        self.odom_pub = self.create_publisher(Odometry, odom_topic, 30)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(
            TwistStamped, wheel_topic, self._on_wheel, 50)

        self.get_logger().info(
            f'simple_encoder_odom_node up: wheel_base={self.wheel_base}, '
            f'publish_tf={self.publish_tf}, odom_topic={odom_topic}')

    def _on_wheel(self, msg: TwistStamped):
        stamp = Time.from_msg(msg.header.stamp)
        if self.last_stamp is None:
            self.last_stamp = stamp
            return

        dt = (stamp - self.last_stamp).nanoseconds * 1e-9
        self.last_stamp = stamp
        if dt <= 0.0 or dt > 1.0:
            return

        v_r = float(msg.twist.linear.x)
        v_l = float(msg.twist.linear.y)
        v = 0.5 * (v_r + v_l)
        w = (v_r - v_l) / self.wheel_base

        # Exact integration when |w| is non-trivial; small-angle otherwise.
        if abs(w) > 1e-5:
            dtheta = w * dt
            theta_mid = self.theta + 0.5 * dtheta
            self.x += v * math.cos(theta_mid) * dt
            self.y += v * math.sin(theta_mid) * dt
            self.theta += dtheta
        else:
            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt

        self._publish(stamp, v, w)

    def _publish(self, stamp: Time, v: float, w: float):
        qz = math.sin(self.theta * 0.5)
        qw = math.cos(self.theta * 0.5)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[35] = 0.05
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleEncoderOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
