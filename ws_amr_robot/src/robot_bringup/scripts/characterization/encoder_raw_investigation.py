#!/usr/bin/env python3
"""
ENCODER RAW DATA INVESTIGATION COLLECTOR (v2 - CORRECTED)
============================================================
Thesis: Kendali dan Navigasi AMR Berbasis LiDAR dan IMU di Warehouse
Author: Reinald Mariel - Universitas Telkom

PURPOSE (Professor's request, Session 5):
  Professor asked to check the encoder INPUT signal: is it a clean
  continuous reading, or does it sometimes show anomalies (drops,
  repeats, zeros) at the ~4.2Hz dip frequency found earlier?

WHAT CHANGED FROM v1:
  v1 assumed a non-existent /encoder_raw_debug topic. After reading
  stm32_bridge.py directly, we found it ALREADY publishes raw cumulative
  tick counts on /wheel_encoders (Int32MultiArray: [total_left_ticks,
  total_right_ticks]). This script uses THAT topic directly. No patch
  to stm32_bridge.py needed.

HYPOTHESIS BEING TESTED (Session 5 finding):
  stm32_bridge.py runs a 20Hz timer (create_timer(0.05, ...)) that BOTH
  sends cmd_vel AND drains+reads the serial buffer, keeping only the
  LATEST telemetry line and discarding older ones if multiple arrived.
  The STM32 ALSO sends telemetry at 20Hz (every 5 cycles of its 100Hz
  loop). These are two INDEPENDENT 20Hz clocks. If they drift out of
  phase, some Pi-side read cycles will find ZERO new STM32 messages
  (latest_line stays None, /wheel_encoders does NOT get a fresh publish
  that cycle), causing odometry_node to compute a near-zero delta for
  that interval. This could be THE CAUSE of the 4.2Hz dip, and it would
  NOT scale with wheel speed since it's a TIMING/beat-frequency issue,
  not a motion-dependent issue. This matches what was observed.

WHAT THIS SCRIPT RECORDS:
  - time_s: timestamp since recording start
  - cmd_vel_mmps: command sent by this script
  - wheel_encoders_left, wheel_encoders_right: RAW cumulative tick
    totals from /wheel_encoders (published by stm32_bridge.py)
  - wheel_encoders_received: 1 if a NEW message arrived this sample
    cycle, 0 if the value is UNCHANGED from last sample (meaning
    /wheel_encoders did not publish anything new - this directly
    tests the hypothesis above)
  - odom_raw_speed_mmps: Pi-calculated speed from /odom_raw, for
    comparison against the raw ticks

HOW TO USE:
  Terminal 1:
    ~/kill_robot.sh && ros2 launch robot_bringup hardware_launch.py

  Terminal 2:
    source ~/Autonomous-Mobile-Robot/ws_amr_robot/install/setup.bash
    python3 ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts/characterization/encoder_raw_investigation.py

  Run at 150 mm/s (default) to compare directly against the previous
  ripple_raw_150mmps data.

ANALYSIS THIS SCRIPT PRINTS AT THE END:
  - How many sample cycles had wheel_encoders_received = 0 (no new data)
  - The frequency/period of these "missed" cycles
  - Whether this frequency matches the ~4.2Hz dip (period ~0.238s)
  - This DIRECTLY answers your professor's question: "is the encoder
    input sometimes zero/missing, and at what frequency"

OUTPUT CSV:
  ~/thesis_data/ripple_analysis/encoder_raw_investigation_<speed>mmps_<timestamp>.csv
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
import csv
import os
import time
import datetime

# ============================================================
# SETTINGS
# ============================================================
TEST_SPEED_MMPS   = 150       # match this with your previous ripple test for comparison
RECORD_DURATION   = 20.0
WARMUP_DURATION   = 3.0

OUTPUT_DIR = os.path.expanduser('~/thesis_data/ripple_analysis')


class EncoderInvestigator(Node):
    def __init__(self):
        super().__init__('encoder_investigator')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.latest_odom_speed = 0.0
        self.latest_enc = (None, None)
        self.last_enc_seen = (None, None)
        self.recording = False
        self.record_start = None
        self.history = []

        self.sub_odom = self.create_subscription(
            Odometry, '/odom_raw', self._cb_odom, 10)
        self.sub_enc = self.create_subscription(
            Int32MultiArray, '/wheel_encoders', self._cb_enc, 10)

        self.get_logger().info('Encoder Investigator v2 ready. Subscribed to /wheel_encoders and /odom_raw.')

    def _cb_odom(self, msg):
        self.latest_odom_speed = msg.twist.twist.linear.x * 1000.0

    def _cb_enc(self, msg):
        if len(msg.data) >= 2:
            self.latest_enc = (msg.data[0], msg.data[1])

    def send_speed(self, speed_mmps):
        msg = Twist()
        msg.linear.x = speed_mmps / 1000.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def stop(self):
        self.send_speed(0)

    def sample(self):
        """Take one sample. Returns whether /wheel_encoders had NEW data."""
        current_enc = self.latest_enc
        received_new = 0
        if current_enc != self.last_enc_seen and current_enc != (None, None):
            received_new = 1
        self.last_enc_seen = current_enc

        if self.recording:
            t = time.time() - self.record_start
            self.history.append((
                t,
                self.latest_odom_speed,
                current_enc[0], current_enc[1],
                received_new
            ))


def analyze_results(history, sample_rate_hz):
    print('\n' + '=' * 60)
    print('ANALYSIS: Encoder Input Signal Check')
    print('=' * 60)

    total = len(history)
    missed = sum(1 for row in history if row[4] == 0)
    received = total - missed

    print(f'\nTotal samples: {total}')
    print(f'Samples with NEW /wheel_encoders data: {received} ({received/total*100:.1f}%)')
    print(f'Samples with NO new data (repeated/missed): {missed} ({missed/total*100:.1f}%)')

    if missed == 0:
        print('\n>>> RESULT: /wheel_encoders published NEW data on every sample.')
        print('>>> The encoder input signal appears CONTINUOUS, not intermittent.')
        print('>>> The 4.2Hz dip is likely NOT caused by missed /wheel_encoders updates.')
        print('>>> Next step: investigate odometry_node.py math/timing itself.')
        return

    # Find the pattern of missed samples - look for periodicity
    missed_times = [row[0] for row in history if row[4] == 0]
    if len(missed_times) > 2:
        intervals = [missed_times[i+1] - missed_times[i] for i in range(len(missed_times)-1)]
        # Filter to "gap" intervals (not consecutive missed samples)
        gap_intervals = [iv for iv in intervals if iv > 1.5/sample_rate_hz]
        if gap_intervals:
            avg_gap = sum(gap_intervals) / len(gap_intervals)
            freq = 1.0 / avg_gap if avg_gap > 0 else 0
            print(f'\n>>> Missed-sample pattern: average interval = {avg_gap:.3f}s')
            print(f'>>> Implied frequency = {freq:.2f} Hz')
            print(f'>>> Previously found dip frequency = ~4.2 Hz (period ~0.238s)')
            if abs(freq - 4.2) < 1.0:
                print(f'>>> MATCH! This strongly suggests the 4.2Hz dip is caused by')
                print(f'>>> /wheel_encoders not receiving new data at this rate.')
                print(f'>>> ROOT CAUSE: timing mismatch between STM32 20Hz send timer')
                print(f'>>> and Pi 20Hz stm32_bridge.py read timer (beat frequency).')
            else:
                print(f'>>> Does not closely match 4.2Hz. May be a different issue,')
                print(f'>>> or the dip is introduced later in odometry_node.py.')

    # Show calculated speed during missed vs received samples
    speeds_missed = [row[1] for row in history if row[4] == 0]
    speeds_received = [row[1] for row in history if row[4] == 1]
    if speeds_missed and speeds_received:
        avg_missed = sum(speeds_missed) / len(speeds_missed)
        avg_received = sum(speeds_received) / len(speeds_received)
        print(f'\nAverage odom_raw speed when /wheel_encoders had NEW data   : {avg_received:.1f} mm/s')
        print(f'Average odom_raw speed when /wheel_encoders had NO new data: {avg_missed:.1f} mm/s')
        if avg_missed < avg_received * 0.7:
            print('>>> CONFIRMED: speed drops significantly during "no new data" cycles.')
            print('>>> This directly explains the dip pattern.')


def main():
    rclpy.init()
    node = EncoderInvestigator()

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    out_csv = os.path.join(OUTPUT_DIR, f'encoder_raw_investigation_{TEST_SPEED_MMPS}mmps_{ts}.csv')

    print('=' * 60)
    print('ENCODER RAW DATA INVESTIGATION v2')
    print('=' * 60)
    print(f'Test speed: {TEST_SPEED_MMPS} mm/s')
    print(f'Record duration: {RECORD_DURATION} s')
    print()
    print('Using REAL existing topic /wheel_encoders (raw cumulative ticks)')
    print('published by stm32_bridge.py - no patches needed.')
    print()
    print('MAKE SURE:')
    print('  - hardware_launch.py is running')
    print('  - Robot has clear space ahead')
    print()
    input('Press ENTER to start...')

    # Check topic exists
    topic_names = [name for name, _ in node.get_topic_names_and_types()]
    if '/wheel_encoders' not in topic_names:
        print('[ERROR] /wheel_encoders topic not found!')
        print('Is stm32_bridge.py running? Check: ros2 topic list')
        node.destroy_node()
        rclpy.shutdown()
        return
    print('[OK] /wheel_encoders topic found.\n')

    # Warmup
    print(f'[1/3] Warming up at {TEST_SPEED_MMPS} mm/s for {WARMUP_DURATION}s...')
    warmup_end = time.time() + WARMUP_DURATION
    sample_period = 0.01  # sample at 100Hz to catch every possible update
    while time.time() < warmup_end:
        node.send_speed(TEST_SPEED_MMPS)
        rclpy.spin_once(node, timeout_sec=0.001)
        node.sample()
        time.sleep(sample_period)

    # Record
    print(f'[2/3] Recording for {RECORD_DURATION}s...')
    node.recording = True
    node.record_start = time.time()
    node.history = []
    record_end = time.time() + RECORD_DURATION
    while time.time() < record_end:
        node.send_speed(TEST_SPEED_MMPS)
        rclpy.spin_once(node, timeout_sec=0.001)
        node.sample()
        time.sleep(sample_period)

    node.recording = False
    node.stop()

    stop_end = time.time() + 1.0
    while time.time() < stop_end:
        rclpy.spin_once(node, timeout_sec=0.01)

    print(f'[3/3] Collected {len(node.history)} samples at ~{1/sample_period:.0f}Hz sampling.')

    with open(out_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time_s', 'odom_raw_speed_mmps', 'wheel_enc_left', 'wheel_enc_right', 'wheel_enc_new_data'])
        for row in node.history:
            writer.writerow([round(row[0], 4), round(row[1], 3), row[2], row[3], row[4]])

    print(f'\nSaved: {out_csv}')

    analyze_results(node.history, 1/sample_period)

    print('\n' + '=' * 60)
    print('COPY THIS CSV TO YOUR PC AND SHARE THE ANALYSIS OUTPUT ABOVE')
    print('WITH YOUR PROFESSOR. This directly answers his question about')
    print('whether the encoder input signal is continuous or intermittent,')
    print('and at what frequency any gaps occur.')
    print('=' * 60)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()