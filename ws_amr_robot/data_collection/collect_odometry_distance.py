#!/usr/bin/env python3
"""
collect_odometry_distance.py  --  BAB IV, Stage 1, TEST B (Encoder validation)
--------------------------------------------------------------------------------
Validate wheel-encoder odometry (/odom_raw) against tape-measured ground truth
for both STRAIGHT distances and IN-PLACE rotations.

Method:
  STRAIGHT tests (1m, 3m):
    1. Place robot at a start line. Press Enter to zero.
    2. Drive robot forward to the end line (tape-measured distance).
    3. Press Enter to capture. Script reads cumulative distance from /odom_raw.
    4. Error = odom_distance - tape_distance.

  ROTATION tests (90, 180, 360 deg):
    1. Align robot to a heading mark. Press Enter to zero.
    2. Rotate robot in place to the target angle (use protractor/wall marks).
    3. Press Enter to capture. Script reads cumulative yaw change from /odom_raw.
    4. Error = wrapTo180(odom_yaw_change - target).

  Repeat each target 5x (configurable).

Output CSV: test_type, target, target_unit, odom_value, error, error_pct, rep
Then a summary table: target | mean odom | mean error | mean error% | n.

If error is SYSTEMATIC (consistent sign, >2%), the script suggests a correction
factor for wheel_diameter or wheelbase. You can then update amr_test_utils.py
constants and re-test once.

Usage:
  python3 collect_odometry_distance.py --reps 5 --label warehouse
"""

import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry

import amr_test_utils as U

STRAIGHT_TARGETS_M = [1.0, 3.0]
ROTATION_TARGETS_DEG = [90.0, 180.0, 360.0]


class OdomTracker(Node):
    """Tracks cumulative distance and yaw from /odom_raw."""

    def __init__(self, topic):
        super().__init__('odom_distance_collector')
        self._lock = threading.Lock()
        self._prev_x = None
        self._prev_y = None
        self._start_yaw = None
        self.cum_dist = 0.0       # meters, monotonically increasing
        self.cum_yaw_deg = 0.0    # degrees, cumulative (handles wrapping)
        self._prev_yaw = None
        self._has_data = False
        self.create_subscription(Odometry, topic, self._cb, qos_profile_sensor_data)

    def _cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = U.yaw_from_quaternion(q.x, q.y, q.z, q.w)
        yaw_deg = math.degrees(yaw)

        with self._lock:
            self._has_data = True
            if self._prev_x is not None:
                dx = x - self._prev_x
                dy = y - self._prev_y
                self.cum_dist += math.hypot(dx, dy)
            if self._prev_yaw is not None:
                dyaw = U.wrap_to_180(yaw_deg - self._prev_yaw)
                self.cum_yaw_deg += dyaw
            self._prev_x = x
            self._prev_y = y
            self._prev_yaw = yaw_deg

    @property
    def has_data(self):
        with self._lock:
            return self._has_data

    def zero(self):
        with self._lock:
            self.cum_dist = 0.0
            self.cum_yaw_deg = 0.0

    def read(self):
        with self._lock:
            return self.cum_dist, self.cum_yaw_deg


def spin_thread(node, stop_event):
    while rclpy.ok() and not stop_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.05)


def main():
    ap = argparse.ArgumentParser(description="TEST B: Encoder distance/rotation validation")
    ap.add_argument('--reps', type=int, default=5)
    ap.add_argument('--topic', default='/odom_raw')
    ap.add_argument('--label', default=None)
    ap.add_argument('--out-dir', default=None)
    ap.add_argument('--straight-only', action='store_true',
                    help="skip rotation tests")
    ap.add_argument('--rotation-only', action='store_true',
                    help="skip straight tests")
    args = ap.parse_args()

    rclpy.init()
    tracker = OdomTracker(args.topic)
    stop_event = threading.Event()
    spinner = threading.Thread(target=spin_thread, args=(tracker, stop_event),
                               daemon=True)
    spinner.start()

    print("Waiting for first odom message...")
    t0 = time.time()
    while not tracker.has_data and rclpy.ok():
        time.sleep(0.1)
        if time.time() - t0 > 10.0:
            print(f"ERROR: no data on {args.topic} after 10s.")
            stop_event.set(); tracker.destroy_node(); rclpy.shutdown(); sys.exit(1)
    print(f"Receiving {args.topic}. Good.\n")

    print("=" * 66)
    print(" TEST B  -  ENCODER DISTANCE & ROTATION VALIDATION")
    print(f" reps={args.reps}  topic={args.topic}")
    print(f" straight targets: {STRAIGHT_TARGETS_M} m")
    print(f" rotation targets: {ROTATION_TARGETS_DEG} deg")
    print("=" * 66)

    rows = []

    # ---------- STRAIGHT ----------
    if not args.rotation_only:
        print("\n>>> STRAIGHT DISTANCE TESTS <<<")
        for target_m in STRAIGHT_TARGETS_M:
            for rep in range(1, args.reps + 1):
                input(f"\n  [{target_m}m] Rep {rep}/{args.reps}: "
                      f"place at start line, press Enter to zero...")
                tracker.zero()
                input(f"  Drive robot forward {target_m}m to end line, "
                      f"stop, press Enter to capture...")
                dist, _ = tracker.read()
                err = dist - target_m
                err_pct = (err / target_m) * 100.0 if target_m else 0.0
                print(f"    odom={dist:.4f}m  tape={target_m}m  "
                      f"err={err:+.4f}m ({err_pct:+.2f}%)")
                rows.append({
                    'test_type': 'straight',
                    'target': target_m,
                    'target_unit': 'm',
                    'odom_value': round(dist, 4),
                    'error': round(err, 4),
                    'error_pct': round(err_pct, 2),
                    'rep': rep,
                })

    # ---------- ROTATION ----------
    if not args.straight_only:
        print("\n>>> IN-PLACE ROTATION TESTS <<<")
        print("  Rotate CCW (left) = positive angle.")
        for target_deg in ROTATION_TARGETS_DEG:
            for rep in range(1, args.reps + 1):
                input(f"\n  [{target_deg}deg] Rep {rep}/{args.reps}: "
                      f"align to start heading, press Enter to zero...")
                tracker.zero()
                input(f"  Rotate robot {target_deg} deg CCW in place, "
                      f"stop, press Enter to capture...")
                _, yaw = tracker.read()
                measured = abs(yaw)
                err = measured - target_deg
                err_pct = (err / target_deg) * 100.0 if target_deg else 0.0
                print(f"    odom={measured:.2f}deg  target={target_deg}deg  "
                      f"err={err:+.2f}deg ({err_pct:+.2f}%)")
                rows.append({
                    'test_type': 'rotation',
                    'target': target_deg,
                    'target_unit': 'deg',
                    'odom_value': round(measured, 3),
                    'error': round(err, 3),
                    'error_pct': round(err_pct, 2),
                    'rep': rep,
                })

    stop_event.set()
    tracker.destroy_node()
    rclpy.shutdown()

    if not rows:
        print("\nNo data collected.")
        return

    path = U.timestamped_path('encoder_validation', out_dir=args.out_dir,
                              label=args.label)
    U.save_csv(path, ['test_type', 'target', 'target_unit', 'odom_value',
                      'error', 'error_pct', 'rep'], rows)

    # ---------- SUMMARY ----------
    print("\n" + "=" * 66)
    print(" SUMMARY")
    print("=" * 66)
    print(f" {'type':>8} {'target':>8} | {'mean odom':>10} | {'mean err':>10} | "
          f"{'mean err%':>9} | {'n':>3}")
    print(" " + "-" * 62)

    straight_errs_pct = []
    rotation_errs_pct = []

    for ttype in ['straight', 'rotation']:
        targets = STRAIGHT_TARGETS_M if ttype == 'straight' else ROTATION_TARGETS_DEG
        unit = 'm' if ttype == 'straight' else 'deg'
        for t in targets:
            subset = [r for r in rows
                      if r['test_type'] == ttype and r['target'] == t]
            if not subset:
                continue
            ovals = [r['odom_value'] for r in subset]
            errs = [r['error'] for r in subset]
            epcts = [r['error_pct'] for r in subset]
            mo = sum(ovals) / len(ovals)
            me = sum(errs) / len(errs)
            mp = sum(epcts) / len(epcts)
            print(f" {ttype:>8} {t:>6g}{unit:>2} | {mo:10.3f} | {me:+10.3f} | "
                  f"{mp:+9.2f}% | {len(subset):3d}")
            if ttype == 'straight':
                straight_errs_pct.extend(epcts)
            else:
                rotation_errs_pct.extend(epcts)

    # ---------- CORRECTION SUGGESTION ----------
    print("\n--- SYSTEMATIC ERROR CHECK ---")
    if straight_errs_pct:
        mean_s = sum(straight_errs_pct) / len(straight_errs_pct)
        if abs(mean_s) > 2.0:
            factor = 1.0 / (1.0 + mean_s / 100.0)
            new_diam = U.WHEEL_DIAMETER * factor
            print(f"  STRAIGHT: mean error = {mean_s:+.2f}% -> SYSTEMATIC.")
            print(f"    Current WHEEL_DIAMETER = {U.WHEEL_DIAMETER:.4f} m")
            print(f"    Suggested correction:    {new_diam:.5f} m  (factor {factor:.4f})")
            print(f"    Update amr_test_utils.py WHEEL_DIAMETER, rebuild, and re-test once.")
        else:
            print(f"  STRAIGHT: mean error = {mean_s:+.2f}% -> within 2%, OK.")

    if rotation_errs_pct:
        mean_r = sum(rotation_errs_pct) / len(rotation_errs_pct)
        if abs(mean_r) > 2.0:
            # Rotation: d_theta = (dR-dL)/wb. Overshoot (+err) means wb too SMALL -> increase.
            factor = 1.0 + mean_r / 100.0
            new_wb = U.WHEEL_BASE * factor
            print(f"  ROTATION: mean error = {mean_r:+.2f}% -> SYSTEMATIC.")
            print(f"    Current WHEEL_BASE = {U.WHEEL_BASE:.4f} m")
            print(f"    Suggested correction:  {new_wb:.5f} m  (factor {factor:.4f})")
            print(f"    Update amr_test_utils.py WHEEL_BASE, rebuild, and re-test once.")
        else:
            print(f"  ROTATION: mean error = {mean_r:+.2f}% -> within 2%, OK.")

    print(f"\nSaved: {path}")


if __name__ == '__main__':
    main()
