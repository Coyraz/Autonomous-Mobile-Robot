#!/usr/bin/env python3
"""
drive_10s_test.py
-----------------
Physical ground-truth speed test.

Commands the robot at a fixed speed for exactly 10 seconds, then stops.
Measure the distance traveled with a tape measure to get the robot's
TRUE physical speed, independent of what /odom_raw reports.

HOW TO USE:
  Terminal 1: ~/kill_robot.sh && ros2 launch robot_bringup hardware_launch.py
  Terminal 2:
    source ~/Autonomous-Mobile-Robot/ws_amr_robot/install/setup.bash
    python3 ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts/characterization/drive_10s_test.py

WHAT TO COMPARE:
  - If measured distance is about 1.48m: robot's real speed is ~148mm/s
    (physically fast, /odom_raw is UNDER-REPORTING, software bug in dt)
  - If measured distance is about 1.17m: robot's real speed is ~117mm/s
    (genuine undershoot, /odom_raw is now reporting correctly)
  - Report the exact measured number so we can determine which case it is.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

TEST_SPEED_MPS = 0.150   # 150 mm/s
TEST_DURATION  = 10.0    # seconds


class DriveTest(Node):
    def __init__(self):
        super().__init__('drive_10s_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def send(self, v):
        msg = Twist()
        msg.linear.x = v
        self.pub.publish(msg)

    def stop(self):
        for _ in range(5):
            self.send(0.0)
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    rclpy.init()
    node = DriveTest()

    print()
    print('=' * 60)
    print('  PHYSICAL DISTANCE TEST - 10 SECONDS AT 150mm/s')
    print('=' * 60)
    print()
    print('  BEFORE STARTING:')
    print('  1. Put robot on flat floor, at least 1.8m clear space ahead')
    print('  2. Mark the FRONT of the robot with tape (start position)')
    print()
    input('  Press ENTER when ready to drive...')

    print()
    print('  Driving at 150mm/s for 10 seconds...')
    print()

    start = time.time()
    last_print = 10
    while True:
        elapsed = time.time() - start
        remaining = TEST_DURATION - elapsed
        if remaining <= 0:
            break

        node.send(TEST_SPEED_MPS)
        rclpy.spin_once(node, timeout_sec=0.05)

        secs_left = int(remaining)
        if secs_left != last_print and secs_left > 0:
            print(f'  {secs_left}s remaining...')
            last_print = secs_left

    node.stop()
    print()
    print('  STOPPED.')
    print()
    print('  NOW:')
    print('  1. Mark the FRONT of the robot with tape (end position)')
    print('  2. Measure the distance between the two tape marks')
    print()
    print('  +------------------------------------------------------+')
    print('  |  What your measurement means:                        |')
    print('  |                                                      |')
    print('  |  ~1.48m  -> real speed ~148mm/s                      |')
    print('  |             /odom_raw is UNDER-REPORTING             |')
    print('  |             (software bug in dt calculation)         |')
    print('  |                                                      |')
    print('  |  ~1.17m  -> real speed ~117mm/s                      |')
    print('  |             /odom_raw is NOW CORRECT                 |')
    print('  |             (old method was over-reporting)          |')
    print('  |                                                      |')
    print('  |  Report the exact centimeter reading here.           |')
    print('  +------------------------------------------------------+')
    print()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
