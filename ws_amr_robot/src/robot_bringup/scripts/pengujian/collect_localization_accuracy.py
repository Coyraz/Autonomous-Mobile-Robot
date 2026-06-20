#!/usr/bin/env python3
"""
collect_localization_accuracy.py
----------------------
Data collection script for Pengujian 3: Localization Accuracy.

Records robot position from three sources simultaneously:
  /odom_raw   - wheel encoder odometry only
  /odom_rf2o  - laser odometry (RF2O) only
  /odom       - EKF fused output (best estimate)

HOW TO USE:
  Step 1: Launch navigation stack normally.
  Step 2: Drive robot manually to a reference point (tape mark on floor).
  Step 3: Stop the robot exactly on the mark.
  Step 4: Run this script in a new terminal:
            source ~/Autonomous-Mobile-Robot/ws_amr_robot/install/setup.bash
            python3 ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts/pengujian/collect_localization_accuracy.py
  Step 5: Follow the prompts.
  Step 6: Repeat for each trial and each reference point.

OUTPUT FILE: ~/thesis_data/pengujian_3/localization_data.csv
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import math
import os
from datetime import datetime

# Ground truth coordinates measured from /amcl_pose after successful localization.
# Format: 'Name': (x_m, y_m, yaw_rad)
# C1 verified 2026-06-15 from /amcl_pose after robot parked at rack.
TITIK_KOORDINAT = {
    'Home':  ( 0.294,  0.018,  0.02),
    'Stage': ( 3.760,  0.286,  1.54),
    'A1':    ( 4.087, -8.052, -1.81),
    'A2':    ( 4.261, -6.606, -1.37),
    'A3':    ( 4.251, -4.724, -1.33),
    'A4':    ( 4.135, -3.011, -1.36),
    'B1':    ( 1.642, -8.256, -1.15),
    'B2':    ( 1.749, -7.193, -1.69),
    'B3':    ( 1.639, -4.818,  1.59),
    'B4':    ( 1.748, -3.528,  1.46),
    'C1':    (-0.830, -8.531, -1.33),
    'C2':    (-0.972, -7.018, -1.51),
    'C3':    (-1.002, -4.472, -1.53),
    'C4':    (-0.944, -2.656, -1.37),
}


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class Pengujian3Collector(Node):

    def __init__(self):
        super().__init__('pengujian3_collector')

        self.enc_x = self.enc_y = self.enc_yaw = None
        self.rf2o_x = self.rf2o_y = self.rf2o_yaw = None
        self.ekf_x = self.ekf_y = self.ekf_yaw = None

        self.create_subscription(Odometry, '/odom_raw',  self.cb_enc,  10)
        self.create_subscription(Odometry, '/odom_rf2o', self.cb_rf2o, 10)
        self.create_subscription(Odometry, '/odom',      self.cb_ekf,  10)

        self.output_dir = os.path.expanduser('~/thesis_data/pengujian_3')
        os.makedirs(self.output_dir, exist_ok=True)
        self.output_path = os.path.join(self.output_dir, 'localization_data.csv')

        file_exists = os.path.exists(self.output_path)
        self.csv_file = open(self.output_path, 'a', newline='')
        self.writer = csv.writer(self.csv_file)

        if not file_exists:
            self.writer.writerow([
                'timestamp', 'point_name', 'trial',
                'gt_x_m', 'gt_y_m',
                'enc_x_m', 'enc_y_m', 'enc_yaw_rad', 'enc_error_m',
                'rf2o_x_m', 'rf2o_y_m', 'rf2o_yaw_rad', 'rf2o_error_m',
                'ekf_x_m', 'ekf_y_m', 'ekf_yaw_rad', 'ekf_error_m'
            ])
            self.csv_file.flush()
            print(f"  Created new file: {self.output_path}")
        else:
            print(f"  Appending to existing file: {self.output_path}")

    def cb_enc(self, msg):
        self.enc_x   = msg.pose.pose.position.x
        self.enc_y   = msg.pose.pose.position.y
        self.enc_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def cb_rf2o(self, msg):
        self.rf2o_x   = msg.pose.pose.position.x
        self.rf2o_y   = msg.pose.pose.position.y
        self.rf2o_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def cb_ekf(self, msg):
        self.ekf_x   = msg.pose.pose.position.x
        self.ekf_y   = msg.pose.pose.position.y
        self.ekf_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def wait_for_data(self, timeout_sec=8.0):
        import time
        start = time.time()
        while time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all([self.enc_x is not None,
                    self.rf2o_x is not None,
                    self.ekf_x is not None]):
                return True
        return False

    def record_snapshot(self, point_name, trial, gt_x, gt_y):
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        if None in [self.enc_x, self.rf2o_x, self.ekf_x]:
            print("  ERROR: Not all topics are publishing.")
            return False

        enc_error  = math.sqrt((self.enc_x  - gt_x)**2 + (self.enc_y  - gt_y)**2)
        rf2o_error = math.sqrt((self.rf2o_x - gt_x)**2 + (self.rf2o_y - gt_y)**2)
        ekf_error  = math.sqrt((self.ekf_x  - gt_x)**2 + (self.ekf_y  - gt_y)**2)

        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        self.writer.writerow([
            timestamp, point_name, trial,
            f'{gt_x:.4f}', f'{gt_y:.4f}',
            f'{self.enc_x:.4f}',  f'{self.enc_y:.4f}',  f'{self.enc_yaw:.4f}',  f'{enc_error:.4f}',
            f'{self.rf2o_x:.4f}', f'{self.rf2o_y:.4f}', f'{self.rf2o_yaw:.4f}', f'{rf2o_error:.4f}',
            f'{self.ekf_x:.4f}',  f'{self.ekf_y:.4f}',  f'{self.ekf_yaw:.4f}',  f'{ekf_error:.4f}'
        ])
        self.csv_file.flush()

        print()
        print(f"  {'='*55}")
        print(f"  RECORDED: {point_name}, Trial {trial}")
        print(f"  Ground Truth:  x={gt_x:.4f}m  y={gt_y:.4f}m")
        print(f"  {'='*55}")
        print(f"  Encoder:  x={self.enc_x:.4f}  y={self.enc_y:.4f}  err={enc_error*100:.1f}cm")
        print(f"  RF2O:     x={self.rf2o_x:.4f}  y={self.rf2o_y:.4f}  err={rf2o_error*100:.1f}cm")
        print(f"  EKF:      x={self.ekf_x:.4f}  y={self.ekf_y:.4f}  err={ekf_error*100:.1f}cm")
        print(f"  {'='*55}")
        errors = [('Encoder', enc_error), ('RF2O', rf2o_error), ('EKF', ekf_error)]
        best = min(errors, key=lambda x: x[1])
        print(f"  Best sensor this trial: {best[0]} ({best[1]*100:.1f}cm error)")
        print()
        return True

    def shutdown(self):
        self.csv_file.close()


def get_float_input(prompt):
    while True:
        try:
            return float(input(prompt))
        except ValueError:
            print("  Please enter a number (e.g. 1.50 or -0.30)")


def get_int_input(prompt, min_val=1, max_val=10):
    while True:
        try:
            val = int(input(prompt))
            if min_val <= val <= max_val:
                return val
            print(f"  Please enter a number between {min_val} and {max_val}")
        except ValueError:
            print("  Please enter a whole number")


def main(args=None):
    rclpy.init(args=args)
    node = Pengujian3Collector()

    print()
    print("=" * 60)
    print("  PENGUJIAN 3: LOCALIZATION ACCURACY DATA COLLECTOR")
    print("=" * 60)
    print()
    print("  Instructions:")
    print("  1. Drive robot to the reference point (tape mark)")
    print("  2. Stop the robot exactly on the mark")
    print("  3. Answer the prompts below")
    print()

    print("  Waiting for odometry topics...")
    ready = node.wait_for_data(timeout_sec=8.0)
    if not ready:
        print("  WARNING: Some topics not yet available.")
        print("  Make sure navigation stack is running.")
    else:
        print("  All three odometry topics ready.")
    print()

    try:
        while True:
            print("-" * 60)
            print("  Enter measurement details (or Ctrl+C to quit):")
            print()

            point_name = input("  Reference point name (e.g. Titik_A): ").strip()
            if not point_name:
                point_name = "Unknown"

            trial = get_int_input("  Trial number (1-10): ", 1, 10)

            print()
            print("  Reference coordinates (from TITIK_KOORDINAT):")
            for name, (x, y, _) in TITIK_KOORDINAT.items():
                print(f"    {name:6s}  x={x:+.3f}  y={y:+.3f}")
            print()
            gt_x = get_float_input("  Ground truth X (meters): ")
            gt_y = get_float_input("  Ground truth Y (meters): ")

            print()
            input("  Press ENTER when robot is stopped exactly on the mark...")

            success = node.record_snapshot(point_name, trial, gt_x, gt_y)

            if success:
                print(f"  Saved to: {node.output_path}")
            else:
                print("  Recording failed. Check that all nodes are running.")

            print()
            again = input("  Record another measurement? (y/n): ").strip().lower()
            if again != 'y':
                break

    except KeyboardInterrupt:
        print("\n  Stopped by user.")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        print(f"\n  Data saved to: ~/thesis_data/pengujian_3/localization_data.csv")
        print("  Done.")


if __name__ == '__main__':
    main()
