#!/usr/bin/env python3
"""
collect_localization_test.py  --  BAB IV, TEST E (3-mode localization accuracy)
-------------------------------------------------------------------------------
Compare localization accuracy across 3 modes at known reference points.

  Mode A: Encoder odometry only (/odom_raw). No AMCL, no EKF.
  Mode B: AMCL only (map->base_link via raw encoder odom TF).
  Mode C: EKF fusion (RF2O + encoder Vx + IMU Vyaw) + AMCL.

PROCEDURE (per mode):
  1. Launch localization_test.launch.py mode:=X in a separate terminal.
  2. Run this script with --mode X.
  3. Drive robot to Home (0,0), press Enter to set baseline.
  4. For each rep, drive the robot to each reference point in order.
     At each point, align to tape marks and press Enter.
  5. Script samples position over ~2s and records the mean estimate.
  6. Repeat for N reps (default 5).

GROUND TRUTH: Physical tape-measured positions relative to Home = (0,0).
These match the warehouse prototype layout from the map geometry test.

OUTPUT: CSV + summary with per-point error and overall RMSE per mode.

Usage:
  python3 collect_localization_test.py --mode A --reps 5
  python3 collect_localization_test.py --mode B --reps 5
  python3 collect_localization_test.py --mode C --reps 5
"""

import argparse
import math
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
import tf2_ros

import amr_test_utils as U

# Ground truth reference points: (name, x_m, y_m) relative to Home.
# Selected for good spatial coverage across 3 aisles.
# Source: WAREHOUSE_WAYPOINTS in amr_test_utils.py (tape-measured).
W = U.WAREHOUSE_WAYPOINTS
REFERENCE_POINTS = [
    ('Home', *W['Home']),
    ('A1',   *W['A1']),
    ('A4',   *W['A4']),
    ('B2',   *W['B2']),
    ('B4',   *W['B4']),
    ('C3',   *W['C3']),
]

SAMPLE_DURATION = 2.0   # seconds to average position at each point
SAMPLE_RATE_HZ = 20.0   # how fast to poll position


class LocalizationCollector(Node):
    """Collects position estimates from the active localization mode."""

    def __init__(self, mode):
        super().__init__('localization_collector')
        self.mode = mode.upper()
        self._lock = threading.Lock()

        # For mode A: subscribe to /odom_raw directly
        self._latest_odom_x = None
        self._latest_odom_y = None
        self._latest_odom_yaw = None
        self.create_subscription(
            Odometry, '/odom_raw', self._odom_cb, 10)

        # For modes B and C: TF lookup map->base_link
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = U.yaw_from_quaternion(q.x, q.y, q.z, q.w)
        with self._lock:
            self._latest_odom_x = msg.pose.pose.position.x
            self._latest_odom_y = msg.pose.pose.position.y
            self._latest_odom_yaw = yaw

    def get_position(self):
        """Return (x, y, yaw) from the active mode, or None."""
        if self.mode == 'A':
            with self._lock:
                if self._latest_odom_x is None:
                    return None
                return (self._latest_odom_x, self._latest_odom_y,
                        self._latest_odom_yaw)
        else:
            try:
                t = self.tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time())
                x = t.transform.translation.x
                y = t.transform.translation.y
                q = t.transform.rotation
                yaw = U.yaw_from_quaternion(q.x, q.y, q.z, q.w)
                return (x, y, yaw)
            except Exception:
                return None

    def sample_position(self, duration=SAMPLE_DURATION):
        """Collect position samples over `duration` seconds, return mean."""
        samples_x = []
        samples_y = []
        samples_yaw = []
        t_end = time.time() + duration
        interval = 1.0 / SAMPLE_RATE_HZ
        while time.time() < t_end and rclpy.ok():
            pos = self.get_position()
            if pos is not None:
                samples_x.append(pos[0])
                samples_y.append(pos[1])
                samples_yaw.append(pos[2])
            time.sleep(interval)
        if not samples_x:
            return None
        mean_x = sum(samples_x) / len(samples_x)
        mean_y = sum(samples_y) / len(samples_y)
        mean_yaw = math.atan2(
            sum(math.sin(y) for y in samples_yaw) / len(samples_yaw),
            sum(math.cos(y) for y in samples_yaw) / len(samples_yaw))
        return (mean_x, mean_y, mean_yaw, len(samples_x))


def spin_thread(node, stop_event):
    while rclpy.ok() and not stop_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.05)


def wait_for_data(node, mode, timeout=15.0):
    """Block until the first position reading arrives."""
    t0 = time.time()
    while rclpy.ok():
        pos = node.get_position()
        if pos is not None:
            return True
        if time.time() - t0 > timeout:
            return False
        time.sleep(0.2)
    return False


MODE_NAMES = {
    'A': 'Encoder Odometry Only',
    'B': 'AMCL Only (no EKF)',
    'C': 'EKF Fusion (encoder+RF2O+IMU) + AMCL',
}


def main():
    ap = argparse.ArgumentParser(
        description='TEST E: 3-mode localization accuracy comparison')
    ap.add_argument('--mode', required=True, choices=['A', 'B', 'C'],
                    help='Localization mode (must match launch file)')
    ap.add_argument('--reps', type=int, default=5,
                    help='Number of full circuits through all reference points')
    ap.add_argument('--sample-time', type=float, default=SAMPLE_DURATION,
                    help='Seconds to average position at each point')
    ap.add_argument('--out-dir', default=None,
                    help='Output directory (default: ~/thesis_data/localization_test)')
    args = ap.parse_args()

    mode = args.mode.upper()
    out_dir = args.out_dir or os.path.expanduser(
        '~/thesis_data/localization_test')

    rclpy.init()
    node = LocalizationCollector(mode)
    stop_event = threading.Event()
    spinner = threading.Thread(target=spin_thread, args=(node, stop_event),
                               daemon=True)
    spinner.start()

    # Wait for data
    src = '/odom_raw' if mode == 'A' else 'TF map->base_link'
    print(f"Waiting for position data from {src}...")
    if not wait_for_data(node, mode):
        print(f"ERROR: no position data after 15s. "
              f"Is localization_test.launch.py mode:={mode} running?")
        stop_event.set()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    print("Position data received. Good.\n")

    print("=" * 70)
    print(f" TEST E  -  LOCALIZATION ACCURACY: Mode {mode} ({MODE_NAMES[mode]})")
    print(f" reps={args.reps}  sample_time={args.sample_time}s  "
          f"points={len(REFERENCE_POINTS)}")
    print("=" * 70)
    print("\nReference points (ground truth, relative to Home):")
    for name, gx, gy in REFERENCE_POINTS:
        print(f"  {name:6s}  ({gx:+6.1f}, {gy:+6.1f})")
    print()

    if mode == 'A':
        print("MODE A: Position from encoder odometry (/odom_raw).")
        print("  Odom starts at (0,0). Place robot at Home before starting.\n")
    elif mode == 'B':
        print("MODE B: Position from AMCL (map->base_link TF).")
        print("  Make sure AMCL is localized. Drive around briefly if needed.\n")
    else:
        print("MODE C: Position from EKF+AMCL (map->base_link TF).")
        print("  Full fusion active. Drive around briefly to let EKF converge.\n")

    rows = []
    try:
        for rep in range(1, args.reps + 1):
            print(f"\n{'='*50}")
            print(f" REP {rep} / {args.reps}")
            print(f"{'='*50}")

            for pt_name, gt_x, gt_y in REFERENCE_POINTS:
                input(f"  Drive to {pt_name} ({gt_x:+.1f}, {gt_y:+.1f}), "
                      f"align to tape mark, press Enter...")

                print(f"    Sampling for {args.sample_time:.1f}s...", end='',
                      flush=True)
                result = node.sample_position(args.sample_time)

                if result is None:
                    print(" FAILED (no data)")
                    continue

                est_x, est_y, est_yaw, n_samples = result
                err = U.position_error(est_x, est_y, gt_x, gt_y)

                print(f" est=({est_x:+.3f}, {est_y:+.3f})  "
                      f"err={err*100:.1f}cm  [{n_samples} samples]")

                rows.append({
                    'mode': mode,
                    'rep': rep,
                    'point': pt_name,
                    'gt_x': round(gt_x, 3),
                    'gt_y': round(gt_y, 3),
                    'est_x': round(est_x, 4),
                    'est_y': round(est_y, 4),
                    'est_yaw_rad': round(est_yaw, 4),
                    'error_m': round(err, 4),
                    'error_cm': round(err * 100, 1),
                    'n_samples': n_samples,
                })

    except KeyboardInterrupt:
        print("\nInterrupted. Saving collected data...")

    stop_event.set()
    node.destroy_node()
    rclpy.shutdown()

    if not rows:
        print("\nNo data collected. Nothing saved.")
        return

    # Save CSV
    fieldnames = ['mode', 'rep', 'point', 'gt_x', 'gt_y',
                  'est_x', 'est_y', 'est_yaw_rad',
                  'error_m', 'error_cm', 'n_samples']
    path = U.timestamped_path(
        f'localization_mode_{mode}', out_dir=out_dir, label=None)
    U.save_csv(path, fieldnames, rows)

    # Print summary
    print("\n" + "=" * 70)
    print(f" SUMMARY  -  Mode {mode} ({MODE_NAMES[mode]})")
    print("=" * 70)
    print(f" {'Point':>6} | {'Mean Error':>10} | {'Std':>7} | "
          f"{'Min':>7} | {'Max':>7} | {'N':>3}")
    print(" " + "-" * 58)

    all_errors = []
    for pt_name, _, _ in REFERENCE_POINTS:
        errs_cm = [r['error_cm'] for r in rows if r['point'] == pt_name]
        if not errs_cm:
            continue
        s = U.summarize(errs_cm)
        all_errors.extend(errs_cm)
        print(f" {pt_name:>6} | {s['mean']:8.1f} cm | {s['std']:5.1f} | "
              f"{s['min']:5.1f} | {s['max']:5.1f} | {s['n']:3d}")

    if all_errors:
        errors_m = [e / 100.0 for e in all_errors]
        rmse_val = U.rmse(errors_m)
        mean_err = sum(all_errors) / len(all_errors)
        print(" " + "-" * 58)
        print(f" Overall: mean={mean_err:.1f} cm  "
              f"RMSE={rmse_val*100:.1f} cm  "
              f"worst={max(all_errors):.1f} cm  "
              f"n={len(all_errors)}")

    print(f"\nSaved: {path}")
    print(f"\nTo compare all modes, run with --mode A, --mode B, --mode C")
    print(f"and check ~/thesis_data/localization_test/ for all CSVs.")



if __name__ == '__main__':
    main()
