#!/usr/bin/env python3
"""
nav2_oscillation_recorder.py
-----------------------------
Records Nav2 navigation data to CSV for oscillation diagnosis.

Subscribes to:
  /cmd_vel          (Twist)       - what Nav2 commands to the robot
  /odom             (Odometry)    - EKF fused output (best estimate)
  /odom_raw         (Odometry)    - encoder odometry only
  /odom_rf2o        (Odometry)    - laser odometry only
  /imu/data_raw     (Imu)         - IMU angular velocity

HOW TO USE:
  Step 1: Start your navigation stack normally.
  Step 2: Set a goal pose in Foxglove so robot starts moving.
  Step 3: In a separate terminal, run:
            python3 ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts/debug/nav2_oscillation_recorder.py
  Step 4: Let the robot navigate to the goal (or oscillate).
  Step 5: Press Ctrl+C to stop recording.
  Step 6: The CSV file is saved to ~/nav2_oscillation_data.csv
  Step 7: Send the CSV file to be analyzed.

WHAT EACH COLUMN MEANS:
  time_s            - time in seconds since recording started
  cmd_vel_vx        - linear speed commanded by Nav2 (m/s)
  cmd_vel_wz        - angular speed commanded by Nav2 (rad/s) <-- WATCH THIS
  odom_vx           - EKF estimated linear speed (m/s)
  odom_wz           - EKF estimated angular speed (rad/s)
  odom_x            - EKF estimated X position (m)
  odom_y            - EKF estimated Y position (m)
  odom_yaw          - EKF estimated heading (rad)
  odom_raw_vx       - encoder linear speed (m/s)
  odom_raw_wz       - encoder angular speed (rad/s)
  odom_rf2o_vx      - laser odometry linear speed (m/s)
  odom_rf2o_wz      - laser odometry angular speed (rad/s)
  imu_wz            - IMU yaw rate (rad/s)  <-- WATCH THIS TOO
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import csv
import math
import os
import time
from threading import Lock


def quaternion_to_yaw(q):
    """Convert quaternion to yaw angle in radians."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class OscillationRecorder(Node):

    def __init__(self):
        super().__init__('oscillation_recorder')

        self.lock = Lock()
        self.start_time = None
        self.row_count = 0

        # Latest values from each topic
        self.cmd_vx  = 0.0
        self.cmd_wz  = 0.0

        self.odom_vx  = 0.0
        self.odom_wz  = 0.0
        self.odom_x   = 0.0
        self.odom_y   = 0.0
        self.odom_yaw = 0.0

        self.odom_raw_vx = 0.0
        self.odom_raw_wz = 0.0

        self.odom_rf2o_vx = 0.0
        self.odom_rf2o_wz = 0.0

        self.imu_wz = 0.0

        # CSV output file
        self.csv_path = os.path.expanduser('~/nav2_oscillation_data.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'time_s',
            'cmd_vel_vx', 'cmd_vel_wz',
            'odom_vx', 'odom_wz',
            'odom_x', 'odom_y', 'odom_yaw',
            'odom_raw_vx', 'odom_raw_wz',
            'odom_rf2o_vx', 'odom_rf2o_wz',
            'imu_wz'
        ])

        # Subscribers
        self.create_subscription(
            Twist, '/cmd_vel',
            self.cb_cmd_vel, 10)

        self.create_subscription(
            Odometry, '/odom',
            self.cb_odom, 10)

        self.create_subscription(
            Odometry, '/odom_raw',
            self.cb_odom_raw, 10)

        self.create_subscription(
            Odometry, '/odom_rf2o',
            self.cb_odom_rf2o, 10)

        self.create_subscription(
            Imu, '/imu/data_raw',
            self.cb_imu,
            qos_profile_sensor_data)

        # Recording timer at 20Hz (same as controller frequency)
        self.create_timer(0.05, self.record_row)

        self.get_logger().info('='*55)
        self.get_logger().info('  Oscillation Recorder ACTIVE')
        self.get_logger().info(f'  Saving to: {self.csv_path}')
        self.get_logger().info('  Set a goal in Foxglove and let robot move.')
        self.get_logger().info('  Press Ctrl+C to stop and save.')
        self.get_logger().info('='*55)

    # ------------------------------------------------------------------ #
    #  CALLBACKS: just store the latest value, do not write to CSV here   #
    # ------------------------------------------------------------------ #

    def cb_cmd_vel(self, msg):
        with self.lock:
            self.cmd_vx = msg.linear.x
            self.cmd_wz = msg.angular.z

    def cb_odom(self, msg):
        with self.lock:
            self.odom_vx  = msg.twist.twist.linear.x
            self.odom_wz  = msg.twist.twist.angular.z
            self.odom_x   = msg.pose.pose.position.x
            self.odom_y   = msg.pose.pose.position.y
            self.odom_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def cb_odom_raw(self, msg):
        with self.lock:
            self.odom_raw_vx = msg.twist.twist.linear.x
            self.odom_raw_wz = msg.twist.twist.angular.z

    def cb_odom_rf2o(self, msg):
        with self.lock:
            self.odom_rf2o_vx = msg.twist.twist.linear.x
            self.odom_rf2o_wz = msg.twist.twist.angular.z

    def cb_imu(self, msg):
        with self.lock:
            self.imu_wz = msg.angular_velocity.z

    # ------------------------------------------------------------------ #
    #  TIMER: write one row every 50ms                                    #
    # ------------------------------------------------------------------ #

    def record_row(self):
        now = self.get_clock().now().nanoseconds / 1e9

        if self.start_time is None:
            self.start_time = now

        elapsed = now - self.start_time

        with self.lock:
            self.writer.writerow([
                f'{elapsed:.4f}',
                f'{self.cmd_vx:.4f}',  f'{self.cmd_wz:.4f}',
                f'{self.odom_vx:.4f}', f'{self.odom_wz:.4f}',
                f'{self.odom_x:.4f}',  f'{self.odom_y:.4f}',
                f'{self.odom_yaw:.4f}',
                f'{self.odom_raw_vx:.4f}', f'{self.odom_raw_wz:.4f}',
                f'{self.odom_rf2o_vx:.4f}', f'{self.odom_rf2o_wz:.4f}',
                f'{self.imu_wz:.4f}'
            ])
            self.row_count += 1

        # Print a status line every 2 seconds (every 40 rows at 20Hz)
        if self.row_count % 40 == 0:
            with self.lock:
                print(
                    f'  t={elapsed:6.1f}s | '
                    f'cmd_wz={self.cmd_wz:+.3f} | '
                    f'odom_wz={self.odom_wz:+.3f} | '
                    f'imu_wz={self.imu_wz:+.3f} | '
                    f'rows={self.row_count}'
                )

    def shutdown(self):
        self.csv_file.flush()
        self.csv_file.close()
        self.get_logger().info(f'Saved {self.row_count} rows to {self.csv_path}')


def main(args=None):
    rclpy.init(args=args)
    node = OscillationRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        print(f'\nDone. File saved to ~/nav2_oscillation_data.csv')
        print('Send that file here for analysis.')


if __name__ == '__main__':
    main()
