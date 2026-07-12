#!/usr/bin/env python3
"""
send_nav_goal.py  --  Send exact Nav2 goals from WAREHOUSE_WAYPOINTS
---------------------------------------------------------------------
Sends NavigateToPose action goals with precise coordinates from the
single source of truth (amr_test_utils.WAREHOUSE_WAYPOINTS).

Usage:
  python3 send_nav_goal.py A1          # send goal to A1
  python3 send_nav_goal.py A1 --yaw 0  # with specific heading (rad)
  python3 send_nav_goal.py --list      # list all waypoints

For Test G, the ground truth marker script handles success/fail input.
This script just sends the goal and exits (non-blocking).
"""

import argparse
import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

import amr_test_utils as U


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class GoalSender(Node):
    def __init__(self):
        super().__init__('nav_goal_sender')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, yaw=0.0):
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose action server not available!')
            return False

        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(f'Sending goal: ({x:.3f}, {y:.3f}, yaw={math.degrees(yaw):.1f}°)')
        self._client.send_goal_async(goal)
        return True


def main():
    ap = argparse.ArgumentParser(description='Send exact Nav2 goal from WAREHOUSE_WAYPOINTS')
    ap.add_argument('waypoint', nargs='?', help='Waypoint name (e.g. A1, B4, Home)')
    ap.add_argument('--yaw', type=float, default=0.0,
                    help='Goal heading in radians (default: 0.0 = facing +X)')
    ap.add_argument('--list', action='store_true', help='List all waypoints and exit')
    args = ap.parse_args()

    if args.list:
        print("Available waypoints:")
        for name, (x, y) in U.WAREHOUSE_WAYPOINTS.items():
            print(f"  {name:12s}  ({x:+6.2f}, {y:+6.2f})")
        return

    if not args.waypoint:
        ap.print_help()
        sys.exit(1)

    wp = args.waypoint
    if wp not in U.WAREHOUSE_WAYPOINTS:
        print(f"ERROR: '{wp}' not in WAREHOUSE_WAYPOINTS.")
        print(f"Available: {', '.join(U.WAREHOUSE_WAYPOINTS.keys())}")
        sys.exit(1)

    x, y = U.WAREHOUSE_WAYPOINTS[wp]
    print(f"Sending Nav2 goal to {wp} ({x}, {y})")

    rclpy.init()
    node = GoalSender()
    success = node.send_goal(x, y, yaw=args.yaw)
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

    if success:
        print(f"Goal sent. Robot navigating to {wp}.")
    else:
        print("Failed to send goal.")
        sys.exit(1)


if __name__ == '__main__':
    main()
