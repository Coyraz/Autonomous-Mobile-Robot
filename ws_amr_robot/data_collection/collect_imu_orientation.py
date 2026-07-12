#!/usr/bin/env python3
"""
collect_imu_orientation.py  --  BAB IV, Stage 1, TEST A (IMU heading validation)
-----------------------------------------------------------------------------------
Validate IMU heading at 4 physically-marked references (0, 90, 180, 270 deg).

WHY THIS SCRIPT WORKS THE WAY IT DOES (important, read before the defense):
  This IMU publishes NO absolute orientation. We verified on hardware that
  /imu/data_raw has orientation = (0,0,0,1) and orientation_covariance[0] = -1.0,
  which is the ROS convention for "orientation not provided". The only usable
  heading signal is the gyro YAW RATE (angular_velocity.z, rad/s). The EKF also
  uses exactly this (ekf.yaml fuses Vyaw only). So the meaningful IMU heading test
  is: integrate the gyro rate over a known physical rotation and compare.

  Two real-world effects are handled explicitly:
    1. GYRO BIAS. While still the gyro reads ~ -0.9 deg/s on this unit. We measure
       the bias over a still window at the start and SUBTRACT it before integrating.
       Without this, integrated heading drifts several degrees in seconds.
    2. INTEGRATION DRIFT. Any residual bias still accumulates. That residual error
       is what this test reports. We do not hide it.

METHOD (per rep):
  - Robot starts at the 0-deg mark, held still. We (re)calibrate bias here, then
    set integrated heading = 0.
  - You rotate the robot IN PLACE, CCW (to the left), to the 90 mark, hold still,
    press Enter. Repeat to 180, then 270. Heading is integrated CONTINUOUSLY in a
    background thread during the rotations.
  - measured = cumulative integrated heading (deg, CCW positive)
    error    = wrapTo180(measured - reference)

Output CSV: rep, reference_deg, measured_cum_deg, error_deg, bias_deg_s,
            still_rate_std_deg_s
Then a per-heading summary: reference | mean measured | mean error | max|err|.

Usage:
  python3 collect_imu_orientation.py --reps 5 --bias-window 6 \
      --topic /imu/data_raw --label lab
"""

import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

import amr_test_utils as U

REFERENCE_HEADINGS = [0.0, 90.0, 180.0, 270.0]   # CCW positive, cumulative


class GyroIntegrator(Node):
    """Continuously integrates bias-corrected yaw rate into a heading (deg)."""

    def __init__(self, topic):
        super().__init__('imu_heading_collector')
        self._lock = threading.Lock()
        self._last_rate = None          # rad/s, latest sample
        self._last_stamp = None         # float seconds
        self.bias = 0.0                 # rad/s, subtracted before integrating
        self.heading_deg = 0.0          # integrated, cumulative
        self._integrating = False
        self.create_subscription(Imu, topic, self._cb, qos_profile_sensor_data)

    def _cb(self, msg: Imu):
        rate = msg.angular_velocity.z
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self._last_rate = rate
            if self._integrating and self._last_stamp is not None:
                dt = stamp - self._last_stamp
                # guard against bad/duplicate stamps
                if 0.0 < dt < 0.5:
                    corrected = rate - self.bias
                    self.heading_deg += math.degrees(corrected * dt)
            self._last_stamp = stamp

    def latest_rate(self):
        with self._lock:
            return self._last_rate

    def start_integration(self):
        with self._lock:
            self.heading_deg = 0.0
            self._last_stamp = None     # first sample just seeds the clock
            self._integrating = True

    def stop_integration(self):
        with self._lock:
            self._integrating = False

    def read_heading(self):
        with self._lock:
            return self.heading_deg


def spin_thread(node, stop_event):
    while rclpy.ok() and not stop_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.05)


def calibrate_bias(node, seconds):
    """Average yaw rate while still -> bias (rad/s). Returns (bias, std_deg_s)."""
    print(f"  Calibrating gyro bias: hold robot PERFECTLY STILL for {seconds:.0f}s...")
    samples = []
    t_end = time.time() + seconds
    while time.time() < t_end and rclpy.ok():
        r = node.latest_rate()
        if r is not None:
            samples.append(r)
        time.sleep(0.05)
        remain = t_end - time.time()
        print(f"\r    {remain:4.1f}s left, {len(samples)} samples   ",
              end='', flush=True)
    print()
    if not samples:
        return 0.0, float('nan')
    bias = sum(samples) / len(samples)
    var = sum((s - bias) ** 2 for s in samples) / len(samples)
    return bias, math.degrees(var ** 0.5)


def main():
    ap = argparse.ArgumentParser(description="TEST A: IMU heading validation (gyro)")
    ap.add_argument('--reps', type=int, default=5,
                    help="repetitions of the full 0->90->180->270 sweep (min 5)")
    ap.add_argument('--bias-window', type=float, default=6.0,
                    help="still seconds used to estimate gyro bias each rep")
    ap.add_argument('--topic', default='/imu/data_raw')
    ap.add_argument('--label', default=None)
    ap.add_argument('--out-dir', default=None)
    args = ap.parse_args()

    rclpy.init()
    node = GyroIntegrator(args.topic)
    stop_event = threading.Event()
    spinner = threading.Thread(target=spin_thread, args=(node, stop_event),
                               daemon=True)
    spinner.start()

    print("Waiting for first IMU message...")
    t0 = time.time()
    while node.latest_rate() is None and rclpy.ok():
        time.sleep(0.1)
        if time.time() - t0 > 10.0:
            print(f"ERROR: no data on {args.topic} after 10s. Is hardware.launch up?")
            stop_event.set(); node.destroy_node(); rclpy.shutdown(); sys.exit(1)
    print("IMU yaw-rate is publishing. Good.\n")

    print("=" * 66)
    print(" TEST A  -  IMU HEADING VALIDATION (bias-corrected gyro integration)")
    print(f" reps={args.reps}  bias_window={args.bias_window}s  topic={args.topic}")
    print(" Rotate IN PLACE, CCW (left), through 90 -> 180 -> 270 each rep.")
    print("=" * 66)

    rows = []
    try:
        for rep in range(1, args.reps + 1):
            print(f"\n----- REP {rep} / {args.reps} -----")
            input("  Place robot on the 0-deg mark, hold still, press Enter to calibrate...")
            bias, std_deg_s = calibrate_bias(node, args.bias_window)
            node.bias = bias
            print(f"    bias = {math.degrees(bias):+.3f} deg/s "
                  f"(still-rate std {std_deg_s:.3f} deg/s)")
            node.start_integration()
            print("    integration started, heading zeroed at 0 deg.")

            for ref in REFERENCE_HEADINGS:
                if ref == 0.0:
                    measured = node.read_heading()  # ~0 by construction
                else:
                    input(f"  Rotate CCW to {ref:.0f} deg mark, hold still, press Enter...")
                    measured = node.read_heading()
                error = U.wrap_to_180(measured - ref)
                print(f"    ref={ref:6.1f}  measured={measured:8.2f}  "
                      f"error={error:7.2f} deg")
                rows.append({
                    'rep': rep,
                    'reference_deg': round(ref, 2),
                    'measured_cum_deg': round(measured, 3),
                    'error_deg': round(error, 3),
                    'bias_deg_s': round(math.degrees(bias), 4),
                    'still_rate_std_deg_s': round(std_deg_s, 4),
                })
            node.stop_integration()
    except KeyboardInterrupt:
        print("\nInterrupted by user. Saving whatever was collected...")

    stop_event.set()
    node.destroy_node()
    rclpy.shutdown()

    if not rows:
        print("\nNo data collected. Nothing saved.")
        return

    path = U.timestamped_path('imu_heading', out_dir=args.out_dir, label=args.label)
    U.save_csv(path, ['rep', 'reference_deg', 'measured_cum_deg', 'error_deg',
                      'bias_deg_s', 'still_rate_std_deg_s'], rows)

    print("\n" + "=" * 66)
    print(" SUMMARY  (error = wrapTo180(measured - reference), degrees)")
    print("=" * 66)
    print(f" {'ref':>6} | {'mean meas':>10} | {'mean err':>9} | "
          f"{'max|err|':>9} | {'n':>3}")
    print(" " + "-" * 58)
    for ref in REFERENCE_HEADINGS:
        errs = [r['error_deg'] for r in rows if r['reference_deg'] == round(ref, 2)]
        meas = [r['measured_cum_deg'] for r in rows
                if r['reference_deg'] == round(ref, 2)]
        if not errs:
            continue
        print(f" {ref:6.1f} | {sum(meas)/len(meas):10.2f} | "
              f"{sum(errs)/len(errs):9.2f} | {max(abs(e) for e in errs):9.2f} | "
              f"{len(errs):3d}")
    nonzero = [abs(r['error_deg']) for r in rows if r['reference_deg'] != 0.0]
    if nonzero:
        print(" " + "-" * 58)
        print(f" overall mean|err|={sum(nonzero)/len(nonzero):.2f} deg   "
              f"worst|err|={max(nonzero):.2f} deg  (excludes the 0-deg start point)")
    print(f"\nSaved: {path}")
    print("Note: NO-MAGNETOMETER IMU. This validates gyro-integration heading after")
    print("bias removal -- exactly the Vyaw signal the EKF fuses -- not absolute compass.")


if __name__ == '__main__':
    main()
