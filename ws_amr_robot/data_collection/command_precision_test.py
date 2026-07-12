#!/usr/bin/env python3
"""
command_precision_test.py  --  BAB IV, Test 12: Multi-Source Odometry / Command Precision
-----------------------------------------------------------------------------------------
Gap this closes: Test B (collect_odometry_distance.py) and Test F (trajectory scenarios)
both validate odometry/localization ACCURACY while a HUMAN manually drives/rotates the
robot and judges when to stop at a tape/protractor mark, and only ONE odometry source
(/odom_raw) was ever logged. Test G (navigate_to_pose via Nav2) is the only truly
autonomously-commanded test, but that's full pose-navigation through the planner, not an
isolated straight-line/rotation actuation check.

This script has TWO modes, selectable via --mode:

  manual (default) -- read-back check.
    Operator drives/rotates the robot manually (teleop) to a tape/protractor mark, EXACTLY
    like Test B. The mark itself is the ground truth (no separate measurement needed).
    Script is purely passive: subscribes to and accumulates cumulative distance/yaw from
    THREE odometry sources at once (/odom fused, /odom_raw encoder, /odom_rf2o laser) and
    compares each against the target. Answers: "does the system correctly SENSE that it
    moved 1m / turned 90deg?"

  auto -- commanded-motion precision check.
    Script itself COMMANDS the robot via /cmd_vel_teleop (same twist_mux input as
    teleop_keyboard.py) using a simple P-controller fed by the EKF-fused /odom (the same
    source real navigation trusts), and stops autonomously once the target distance/angle
    is reached -- no human judgement involved in the stop decision. The operator then
    tape/protractor-measures the ACTUAL physical result as ground truth (since the
    physical stop point is what's being validated, not just internal self-consistency).
    Also logs /odom_raw and /odom_rf2o at the same stop instant for comparison. Answers:
    "does the system correctly EXECUTE a commanded 1m move / 90deg turn?" -- i.e. proves
    calibration (wheelbase, wheel diameter, EKF tuning) is correct end-to-end, the same
    way Test G proves it for full pose-navigation but isolated to straight-line/rotation.

Both modes also double as empirical justification for the EKF's "complementary" sensor
design in ekf.yaml (encoder contributes Vx only, RF2O contributes X/Y/yaw only): if
raw-encoder error grows faster than rf2o/fused error as target size increases, that is
the wheel-slip-accumulation signature.

Targets: straight 1m/3m; rotation 90/180/270/360 deg (270 added 2026-07-10 to cover
all four quadrant turns, not just 90/180/360).

Output CSV: mode, test_type, target, target_unit, rep, ground_truth,
            odom_fused, odom_raw, odom_rf2o, err_fused, err_raw, err_rf2o,
            err_pct_fused, err_pct_raw, err_pct_rf2o, elapsed_s (auto mode only)
Then a summary table comparing all three sources per target.

Usage:
  python3 command_precision_test.py --mode manual --reps 5 --label warehouse
  python3 command_precision_test.py --mode auto --reps 5 --label warehouse
  python3 command_precision_test.py --mode auto --reps 3 --straight-only
  python3 command_precision_test.py --mode manual --reps 3 --rotation-only
"""

import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import amr_test_utils as U

STRAIGHT_TARGETS_M = [1.0, 3.0]
ROTATION_TARGETS_DEG = [90.0, 180.0, 270.0, 360.0]

CMD_VEL_TOPIC = '/cmd_vel_teleop'   # same twist_mux input as teleop_keyboard.py (auto mode only)
CONTROL_HZ = 20.0

# auto mode only -- must stay inside the hardware limits declared in custom_controller.yaml
MAX_LINEAR = 0.20     # m/s, L298N hardware cap
MIN_LINEAR = 0.05     # m/s
MAX_ANGULAR = 0.60    # rad/s
MIN_ANGULAR = 0.55    # rad/s, motor rotation deadband
KP_LINEAR = 0.6
KP_ANGULAR = 1.2
TOL_LINEAR_M = 0.01
TOL_ANGULAR_DEG = 1.0
SETTLE_S = 1.0          # wait after autonomous stop for residual motion to die out
TIMEOUT_DISTANCE_S = 90.0
TIMEOUT_ROTATION_S = 60.0


class OdomCumulative:
    """Tracks NET straight-line displacement (start pose -> latest pose) and
    cumulative yaw (deg) from any Odometry topic, independent of the others.
    Three of these run at once (fused/raw/rf2o).

    Distance uses NET displacement (hypot of start->current), NOT a running sum of
    hypot() between consecutive samples. 2026-07-12 finding: summing hypot(dx,dy)
    (a norm, always >= 0) at every sample is a biased "path length" estimator for any
    noisy/jittery signal -- jitter never cancels, it only ever ADDS extra apparent
    path length, however small the jitter. On a high-rate (30Hz), frequently-corrected
    signal like the EKF-fused /odom this inflated straight-line "distance traveled" by
    ~13% even when the true net displacement matched the encoder reference to <1%. Net
    displacement is also what a tape measurement on the real robot actually captures
    (start line -> stop point), so it is the metrologically correct comparison anyway.

    Yaw does NOT have this problem and is intentionally left as an incremental SIGNED
    sum (wrap_to_180 of consecutive deltas): summing signed deltas lets zero-mean
    jitter cancel out (unlike a norm), and incremental summing is also required to
    correctly track rotations > 180 deg (a single start/end wrap_to_180 would fold a
    270 or 360 deg turn back into a misleadingly small angle)."""

    def __init__(self, node, topic):
        self._lock = threading.Lock()
        self._start_x = None
        self._start_y = None
        self._last_x = None
        self._last_y = None
        self._prev_yaw = None
        self.cum_yaw_deg = 0.0
        self._has_data = False
        node.create_subscription(Odometry, topic, self._cb, qos_profile_sensor_data)

    def _cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw_deg = math.degrees(U.yaw_from_quaternion(q.x, q.y, q.z, q.w))
        with self._lock:
            self._has_data = True
            if self._start_x is None:
                self._start_x, self._start_y = x, y
            self._last_x, self._last_y = x, y
            if self._prev_yaw is not None:
                self.cum_yaw_deg += U.wrap_to_180(yaw_deg - self._prev_yaw)
            self._prev_yaw = yaw_deg

    @property
    def has_data(self):
        with self._lock:
            return self._has_data

    def zero(self):
        with self._lock:
            self._start_x = None
            self._start_y = None
            self._last_x = None
            self._last_y = None
            self.cum_yaw_deg = 0.0

    def read(self):
        """Returns (net_displacement_m, cumulative_yaw_deg)."""
        with self._lock:
            if self._start_x is None or self._last_x is None:
                dist = 0.0
            else:
                dist = math.hypot(self._last_x - self._start_x, self._last_y - self._start_y)
            return dist, self.cum_yaw_deg


class CommandPrecisionNode(Node):
    def __init__(self, fused_topic, raw_topic, rf2o_topic, need_cmd_pub):
        super().__init__('command_precision_test')
        self.fused = OdomCumulative(self, fused_topic)
        self.raw = OdomCumulative(self, raw_topic)
        self.rf2o = OdomCumulative(self, rf2o_topic)
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10) if need_cmd_pub else None

    def wait_for_data(self, timeout_s=10.0):
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout_s:
            if self.fused.has_data and self.raw.has_data and self.rf2o.has_data:
                return True
            time.sleep(0.1)
        return False

    def zero_all(self):
        self.fused.zero()
        self.raw.zero()
        self.rf2o.zero()

    def stop(self):
        if self.cmd_pub is not None:
            self.cmd_pub.publish(Twist())

    # ---------------------------------------------------------- auto mode only
    def drive_distance(self, target_m):
        """P-controller straight-line drive using FUSED odom as feedback.
        Returns (elapsed_s, timed_out)."""
        period = 1.0 / CONTROL_HZ
        t0 = time.time()
        try:
            while rclpy.ok():
                traveled, _ = self.fused.read()
                remaining = target_m - traveled
                if remaining <= TOL_LINEAR_M:
                    break
                if time.time() - t0 > TIMEOUT_DISTANCE_S:
                    return time.time() - t0, True
                v = max(MIN_LINEAR, min(MAX_LINEAR, KP_LINEAR * remaining))
                cmd = Twist()
                cmd.linear.x = v
                self.cmd_pub.publish(cmd)
                time.sleep(period)
        finally:
            self.stop()
        return time.time() - t0, False

    def drive_rotation(self, target_deg):
        """P-controller in-place CCW rotation using FUSED odom as feedback.
        Returns (elapsed_s, timed_out)."""
        period = 1.0 / CONTROL_HZ
        t0 = time.time()
        try:
            while rclpy.ok():
                _, rotated = self.fused.read()
                remaining = target_deg - rotated
                if remaining <= TOL_ANGULAR_DEG:
                    break
                if time.time() - t0 > TIMEOUT_ROTATION_S:
                    return time.time() - t0, True
                w = max(MIN_ANGULAR, min(MAX_ANGULAR, KP_ANGULAR * math.radians(remaining)))
                cmd = Twist()
                cmd.angular.z = w
                self.cmd_pub.publish(cmd)
                time.sleep(period)
        finally:
            self.stop()
        return time.time() - t0, False


def spin_thread(node, stop_event):
    while rclpy.ok() and not stop_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.05)


def err_pct(err, target):
    return (err / target) * 100.0 if target else 0.0


def main():
    ap = argparse.ArgumentParser(description="Test 12: Odometry read-back / commanded "
                                              "motion precision (manual or auto mode)")
    ap.add_argument('--mode', choices=['manual', 'auto'], default='manual',
                    help="manual = operator drives to a tape mark, script only logs "
                         "(read-back check). auto = script commands+auto-stops the "
                         "robot, operator tape-measures the physical result "
                         "(commanded-motion precision check).")
    ap.add_argument('--reps', type=int, default=5)
    ap.add_argument('--fused-topic', default='/odom')
    ap.add_argument('--raw-topic', default='/odom_raw')
    ap.add_argument('--rf2o-topic', default='/odom_rf2o')
    ap.add_argument('--label', default=None)
    ap.add_argument('--out-dir', default=None)
    ap.add_argument('--straight-only', action='store_true', help="skip rotation tests")
    ap.add_argument('--rotation-only', action='store_true', help="skip straight tests")
    args = ap.parse_args()

    auto = (args.mode == 'auto')

    rclpy.init()
    node = CommandPrecisionNode(args.fused_topic, args.raw_topic, args.rf2o_topic,
                                need_cmd_pub=auto)
    stop_event = threading.Event()
    spinner = threading.Thread(target=spin_thread, args=(node, stop_event), daemon=True)
    spinner.start()

    print("Waiting for /odom (fused), /odom_raw, /odom_rf2o ...")
    if not node.wait_for_data(timeout_s=10.0):
        print("ERROR: did not receive data on all three odometry topics within 10s.")
        print("Check hardware.launch.py + ekf_filter_node + rf2o_laser_odometry are running.")
        print("If /odom_rf2o is the one missing: check the LiDAR's USB connection "
              "(lsusb / /dev/serial/by-id/) -- sllidar_node dies silently if unplugged.")
        stop_event.set(); node.destroy_node(); rclpy.shutdown(); sys.exit(1)
    print("Receiving all three odometry sources. Good.\n")

    mode_desc = ("AUTO -- script commands the robot, autonomous stop, operator "
                 "tape-measures the result" if auto else
                 "MANUAL -- operator drives to a tape/protractor mark, script only logs")
    print("=" * 74)
    print(f" TEST 12  -  mode={args.mode.upper()}  ({mode_desc})")
    print(f" reps={args.reps}  straight targets: {STRAIGHT_TARGETS_M} m   "
          f"rotation targets: {ROTATION_TARGETS_DEG} deg")
    print("=" * 74)

    rows = []

    if not args.rotation_only:
        verb = "Script will drive itself" if auto else "Drive robot manually"
        print(f"\n>>> STRAIGHT DISTANCE TESTS ({verb}) <<<")
        for target_m in STRAIGHT_TARGETS_M:
            for rep in range(1, args.reps + 1):
                if auto:
                    input(f"\n  [{target_m}m] Rep {rep}/{args.reps}: place at start line, "
                          f"clear path ahead ({target_m}m+), press Enter to start...")
                    node.zero_all()
                    elapsed, timed_out = node.drive_distance(target_m)
                    time.sleep(SETTLE_S)
                    if timed_out:
                        print(f"    WARNING: timed out after {elapsed:.1f}s -- check for "
                              f"a stall/deadband issue.")
                    ground_truth = float(input("    Robot stopped. Tape-measure ACTUAL "
                                               "distance traveled (m): "))
                else:
                    input(f"\n  [{target_m}m] Rep {rep}/{args.reps}: place at start line, "
                          f"press Enter to zero...")
                    node.zero_all()
                    input(f"  Drive robot forward {target_m}m to end line, stop, "
                          f"press Enter to capture...")
                    elapsed, ground_truth = None, target_m

                fused_d, _ = node.fused.read()
                raw_d, _ = node.raw.read()
                rf2o_d, _ = node.rf2o.read()
                e_f, e_r, e_o = fused_d - ground_truth, raw_d - ground_truth, rf2o_d - ground_truth
                print(f"    ground_truth={ground_truth:.4f}m  fused={fused_d:.4f}m({e_f:+.4f})  "
                      f"raw={raw_d:.4f}m({e_r:+.4f})  rf2o={rf2o_d:.4f}m({e_o:+.4f})")
                rows.append({
                    'mode': args.mode, 'test_type': 'straight', 'target': target_m,
                    'target_unit': 'm', 'rep': rep, 'ground_truth': round(ground_truth, 4),
                    'odom_fused': round(fused_d, 4), 'odom_raw': round(raw_d, 4),
                    'odom_rf2o': round(rf2o_d, 4),
                    'err_fused': round(e_f, 4), 'err_raw': round(e_r, 4), 'err_rf2o': round(e_o, 4),
                    'err_pct_fused': round(err_pct(e_f, ground_truth), 2),
                    'err_pct_raw': round(err_pct(e_r, ground_truth), 2),
                    'err_pct_rf2o': round(err_pct(e_o, ground_truth), 2),
                    'elapsed_s': round(elapsed, 2) if elapsed is not None else '',
                })

    if not args.straight_only:
        verb = "Script will rotate itself" if auto else "Rotate robot manually"
        print(f"\n>>> IN-PLACE ROTATION TESTS ({verb}, CCW = positive) <<<")
        for target_deg in ROTATION_TARGETS_DEG:
            for rep in range(1, args.reps + 1):
                if auto:
                    input(f"\n  [{target_deg}deg] Rep {rep}/{args.reps}: align to start "
                          f"heading, press Enter to start...")
                    node.zero_all()
                    elapsed, timed_out = node.drive_rotation(target_deg)
                    time.sleep(SETTLE_S)
                    if timed_out:
                        print(f"    WARNING: timed out after {elapsed:.1f}s -- check for "
                              f"a stall/deadband issue.")
                    ground_truth = float(input("    Robot stopped. Read ACTUAL heading "
                                               "change off protractor/wall mark (deg): "))
                else:
                    input(f"\n  [{target_deg}deg] Rep {rep}/{args.reps}: align to start "
                          f"heading, press Enter to zero...")
                    node.zero_all()
                    input(f"  Rotate robot {target_deg} deg CCW in place, stop, "
                          f"press Enter to capture...")
                    elapsed, ground_truth = None, target_deg

                fused_d, fused_y = node.fused.read()
                raw_d, raw_y = node.raw.read()
                rf2o_d, rf2o_y = node.rf2o.read()
                if not auto:
                    fused_y, raw_y, rf2o_y = abs(fused_y), abs(raw_y), abs(rf2o_y)
                e_f, e_r, e_o = fused_y - ground_truth, raw_y - ground_truth, rf2o_y - ground_truth
                print(f"    ground_truth={ground_truth:.2f}deg  fused={fused_y:.2f}deg({e_f:+.2f})  "
                      f"raw={raw_y:.2f}deg({e_r:+.2f})  rf2o={rf2o_y:.2f}deg({e_o:+.2f})")
                rows.append({
                    'mode': args.mode, 'test_type': 'rotation', 'target': target_deg,
                    'target_unit': 'deg', 'rep': rep, 'ground_truth': round(ground_truth, 3),
                    'odom_fused': round(fused_y, 3), 'odom_raw': round(raw_y, 3),
                    'odom_rf2o': round(rf2o_y, 3),
                    'err_fused': round(e_f, 3), 'err_raw': round(e_r, 3), 'err_rf2o': round(e_o, 3),
                    'err_pct_fused': round(err_pct(e_f, ground_truth), 2),
                    'err_pct_raw': round(err_pct(e_r, ground_truth), 2),
                    'err_pct_rf2o': round(err_pct(e_o, ground_truth), 2),
                    'elapsed_s': round(elapsed, 2) if elapsed is not None else '',
                })

    stop_event.set()
    node.destroy_node()
    rclpy.shutdown()

    if not rows:
        print("\nNo data collected.")
        return

    path = U.timestamped_path('command_precision', out_dir=args.out_dir,
                              label=args.label or args.mode)
    U.save_csv(path, ['mode', 'test_type', 'target', 'target_unit', 'rep', 'ground_truth',
                      'odom_fused', 'odom_raw', 'odom_rf2o',
                      'err_fused', 'err_raw', 'err_rf2o',
                      'err_pct_fused', 'err_pct_raw', 'err_pct_rf2o', 'elapsed_s'], rows)

    print("\n" + "=" * 78)
    print(f" SUMMARY  (mode={args.mode}, mean error % vs ground truth, by source)")
    print("=" * 78)
    print(f" {'type':>8} {'target':>8} | {'fused%':>8} | {'raw%':>8} | {'rf2o%':>8} | {'n':>3}")
    print(" " + "-" * 74)
    for ttype in ['straight', 'rotation']:
        targets = STRAIGHT_TARGETS_M if ttype == 'straight' else ROTATION_TARGETS_DEG
        unit = 'm' if ttype == 'straight' else 'deg'
        for t in targets:
            subset = [r for r in rows if r['test_type'] == ttype and r['target'] == t]
            if not subset:
                continue
            mf = sum(r['err_pct_fused'] for r in subset) / len(subset)
            mr = sum(r['err_pct_raw'] for r in subset) / len(subset)
            mo = sum(r['err_pct_rf2o'] for r in subset) / len(subset)
            print(f" {ttype:>8} {t:>6g}{unit:>2} | {mf:+8.2f} | {mr:+8.2f} | {mo:+8.2f} | {len(subset):3d}")

    print(f"\nSaved: {path}")
    print("\nNote: 'raw' (encoder-only) vs 'fused'/'rf2o' error trend across targets is the "
          "evidence for the ekf.yaml complementary-fusion design (encoder position excluded, "
          "RF2O owns position) -- see test12.md.")


if __name__ == '__main__':
    main()
