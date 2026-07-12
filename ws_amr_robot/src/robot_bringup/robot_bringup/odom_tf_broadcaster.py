#!/usr/bin/env python3
"""
odom_tf_broadcaster.py
----------------------
Publishes odom->base_link TF from /odom_raw Odometry messages.

Used by localization_test.launch.py in modes A and B, where the EKF is
not running and therefore nothing else publishes this transform. AMCL
(mode B) needs odom->base_link to compute its motion model.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class OdomTFBroadcaster(Node):

    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        self.br = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom_raw', self._cb, 10)
        self.get_logger().info('odom_tf_broadcaster: publishing odom->base_link from /odom_raw')

    def _cb(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
