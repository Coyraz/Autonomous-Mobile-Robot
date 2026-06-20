#!/usr/bin/env python3
"""
collect_navigation_pointtopoint.py
----------------------
Data collection script for Pengujian 4: Point-to-Point Navigation.

Records each navigation attempt: start position, goal position,
arrival position, travel time, path length, and success/fail.

HOW TO USE:
  Step 1: Launch navigation stack normally.
  Step 2: Run this script:
            source ~/Autonomous-Mobile-Robot/ws_amr_robot/install/setup.bash
            python3 ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts/pengujian/collect_navigation_pointtopoint.py
  Step 3: Enter goal details when prompted.
  Step 4: Send the goal pose from Foxglove.
  Step 5: When robot arrives (or fails), press ENTER to record result.

OUTPUT FILE: ~/thesis_data/pengujian_4/navigation_data.csv
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import csv
import math
import os
import time
from datetime import datetime

# Goal coordinates for each rack point (map frame, from /amcl_pose).
# Format: 'Name': (x_m, y_m, yaw_rad)
# C1 verified 2026-06-15 from /amcl_pose after robot parked at rack.
RACK_GOALS = {
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

# Routes for Pengujian 4
ROUTES = {
    'Route_Short':  [('A4', RACK_GOALS['A4']),  ('Home', RACK_GOALS['Home'])],
    'Route_Medium': [('B2', RACK_GOALS['B2']),  ('Home', RACK_GOALS['Home'])],
    'Route_Long':   [('C1', RACK_GOALS['C1']),  ('Home', RACK_GOALS['Home'])],
    'Route_Order':  [('Stage', RACK_GOALS['Stage']), ('A1', RACK_GOALS['A1']),
                     ('B3', RACK_GOALS['B3']),  ('Home', RACK_GOALS['Home'])],
}


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class Pengujian4Collector(Node):

    def __init__(self):
        super().__init__('pengujian4_collector')

        self.ekf_x = self.ekf_y = self.ekf_yaw = None
        self.path_distance = 0.0
        self.last_x = None
        self.last_y = None

        self.create_subscription(Odometry, '/odom', self.cb_ekf, 10)

        self.output_dir = os.path.expanduser('~/thesis_data/pengujian_4')
        os.makedirs(self.output_dir, exist_ok=True)
        self.output_path = os.path.join(self.output_dir, 'navigation_data.csv')

        file_exists = os.path.exists(self.output_path)
        self.csv_file = open(self.output_path, 'a', newline='')
        self.writer = csv.writer(self.csv_file)

        if not file_exists:
            self.writer.writerow([
                'timestamp', 'route_name', 'trial',
                'start_x_m', 'start_y_m',
                'goal_x_m', 'goal_y_m',
                'arrive_x_m', 'arrive_y_m',
                'goal_distance_m',
                'arrival_error_m',
                'travel_time_s',
                'path_length_m',
                'path_efficiency_pct',
                'success'
            ])
            self.csv_file.flush()
            print(f"  Created new file: {self.output_path}")
        else:
            print(f"  Appending to existing file: {self.output_path}")

    def cb_ekf(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Accumulate path distance while recording
        if self.last_x is not None and self._recording:
            dist = math.sqrt((x - self.last_x)**2 + (y - self.last_y)**2)
            # Only count movement above 1mm to filter noise
            if dist > 0.001:
                self.path_distance += dist

        self.ekf_x   = x
        self.ekf_y   = y
        self.ekf_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.last_x  = x
        self.last_y  = y

    def wait_for_data(self):
        start = time.time()
        while time.time() - start < 8.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.ekf_x is not None:
                return True
        return False

    def spin_for(self, seconds):
        start = time.time()
        while time.time() - start < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)

    def record_trial(self, route_name, trial, goal_x, goal_y, success):

        # Get final position
        self.spin_for(0.5)

        if self.ekf_x is None:
            print("  ERROR: No EKF data available.")
            return False

        arrival_error = math.sqrt(
            (self.ekf_x - goal_x)**2 + (self.ekf_y - goal_y)**2
        )
        goal_distance = math.sqrt(
            (goal_x - self.start_x)**2 + (goal_y - self.start_y)**2
        )
        efficiency = (goal_distance / self.path_distance * 100) if self.path_distance > 0 else 0.0
        travel_time = time.time() - self.start_time

        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        self.writer.writerow([
            timestamp, route_name, trial,
            f'{self.start_x:.4f}', f'{self.start_y:.4f}',
            f'{goal_x:.4f}', f'{goal_y:.4f}',
            f'{self.ekf_x:.4f}', f'{self.ekf_y:.4f}',
            f'{goal_distance:.4f}',
            f'{arrival_error:.4f}',
            f'{travel_time:.1f}',
            f'{self.path_distance:.4f}',
            f'{efficiency:.1f}',
            'YES' if success else 'NO'
        ])
        self.csv_file.flush()
        self._recording = False

        print()
        print(f"  {'='*55}")
        print(f"  RECORDED: {route_name}, Trial {trial}")
        print(f"  {'='*55}")
        print(f"  Start:         x={self.start_x:.3f}  y={self.start_y:.3f}")
        print(f"  Goal:          x={goal_x:.3f}  y={goal_y:.3f}")
        print(f"  Arrived at:    x={self.ekf_x:.3f}  y={self.ekf_y:.3f}")
        print(f"  Goal distance: {goal_distance*100:.1f}cm")
        print(f"  Arrival error: {arrival_error*100:.1f}cm")
        print(f"  Travel time:   {travel_time:.1f}s")
        print(f"  Path length:   {self.path_distance:.3f}m")
        print(f"  Efficiency:    {efficiency:.1f}%")
        print(f"  Result:        {'SUCCESS' if success else 'FAILED'}")
        print(f"  {'='*55}")
        print()
        return True

    def start_trial(self):
        self.spin_for(0.5)
        self.start_x = self.ekf_x
        self.start_y = self.ekf_y
        self.path_distance = 0.0
        self.last_x = self.ekf_x
        self.last_y = self.ekf_y
        self.start_time = time.time()
        self._recording = True
        print(f"  Trial started. Start position: x={self.start_x:.3f}  y={self.start_y:.3f}")

    def shutdown(self):
        self.csv_file.close()


def get_float_input(prompt):
    while True:
        try:
            return float(input(prompt))
        except ValueError:
            print("  Please enter a number")


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
    node = Pengujian4Collector()
    node._recording = False

    print()
    print("=" * 60)
    print("  PENGUJIAN 4: POINT-TO-POINT NAVIGATION DATA COLLECTOR")
    print("=" * 60)
    print()

    print("  Waiting for EKF odometry topic...")
    ready = node.wait_for_data()
    if not ready:
        print("  WARNING: EKF topic not available. Check navigation stack.")
    else:
        print("  EKF topic ready.")
    print()

    try:
        while True:
            print("-" * 60)
            print("  Enter trial details (or Ctrl+C to quit):")
            print()

            route_name = input("  Route name (e.g. Titik_A_ke_B): ").strip()
            if not route_name:
                route_name = "Unknown_Route"

            trial = get_int_input("  Trial number (1-10): ", 1, 10)

            print()
            print("  Available goal points (RACK_GOALS):")
            for name, (x, y, _) in RACK_GOALS.items():
                print(f"    {name:6s}  x={x:+.3f}  y={y:+.3f}")
            print()
            goal_x = get_float_input("  Goal X (meters): ")
            goal_y = get_float_input("  Goal Y (meters): ")

            print()
            input("  Position robot at START point, then press ENTER to begin...")
            node.start_trial()

            print()
            print("  NOW: Send goal pose from Foxglove.")
            print("  Wait for robot to navigate to goal.")
            input("  When robot STOPS (success or fail), press ENTER...")

            print()
            result_str = input("  Did robot reach goal within 30cm? (y/n): ").strip().lower()
            success = result_str == 'y'

            node.record_trial(route_name, trial, goal_x, goal_y, success)
            print(f"  Saved to: {node.output_path}")

            print()
            again = input("  Record another trial? (y/n): ").strip().lower()
            if again != 'y':
                break

    except KeyboardInterrupt:
        print("\n  Stopped by user.")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        print(f"\n  Data saved to: ~/thesis_data/pengujian_4/navigation_data.csv")
        print("  Done.")


if __name__ == '__main__':
    main()
