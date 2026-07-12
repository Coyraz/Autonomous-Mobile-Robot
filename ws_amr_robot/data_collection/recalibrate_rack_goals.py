#!/usr/bin/env python3
"""
recalibrate_rack_goals.py  --  Re-verify RACK_GOALS for ils_gui.py from live /amcl_pose

Drive/teleop the robot to each rack in turn (e.g. via ils_gui's D-pad, or a
joystick), press Enter when it's parked, and this records the current
/amcl_pose (x, y, yaw) for that rack. At the end it prints a ready-to-paste
Python dict in the exact format ils_gui.py's RACK_GOALS expects.

Usage:
  python3 recalibrate_rack_goals.py                # all racks + Home + Stage
  python3 recalibrate_rack_goals.py A1 A2 B3        # only specific ones
  python3 recalibrate_rack_goals.py --list          # show current ils_gui values
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

# Kept in sync manually with ils_gui.py's RACK_GOALS -- only used here to
# decide default ordering / what "--list" shows.
CURRENT_RACK_GOALS = {
    'Home':  (0.294,  0.018,  0.02),
    'Stage': (3.760,  0.286,  1.54),
    'A1':    (4.087, -8.052, -1.81),
    'A2':    (4.261, -6.606, -1.37),
    'A3':    (4.251, -4.724, -1.33),
    'A4':    (4.135, -3.011, -1.36),
    'B1':    (1.642, -8.256, -1.15),
    'B2':    (1.749, -7.193, -1.69),
    'B3':    (1.639, -4.818,  1.59),
    'B4':    (1.748, -3.528,  1.46),
    'C1':    (-0.830, -8.531, -1.33),
    'C2':    (-0.972, -7.018, -1.51),
    'C3':    (-1.002, -4.472, -1.53),
    'C4':    (-0.944, -2.656, -1.37),
}

DEFAULT_ORDER = list(CURRENT_RACK_GOALS.keys())


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class PoseReader(Node):
    def __init__(self):
        super().__init__('recalibrate_rack_goals')
        self.pose = None
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._cb, 5)

    def _cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = (p.x, p.y, yaw_from_quat(q.x, q.y, q.z, q.w))

    def wait_for_pose(self, timeout_sec=5.0):
        """Spin until a fresh /amcl_pose arrives (or timeout)."""
        import time
        self.pose = None
        deadline = time.time() + timeout_sec
        while rclpy.ok() and self.pose is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.pose


def main():
    ap = argparse.ArgumentParser(description='Re-verify RACK_GOALS from live /amcl_pose')
    ap.add_argument('racks', nargs='*', help='Rack names to redo (default: all)')
    ap.add_argument('--list', action='store_true', help='Show current ils_gui RACK_GOALS and exit')
    args = ap.parse_args()

    if args.list:
        print("Current ils_gui.py RACK_GOALS:")
        for name, (x, y, th) in CURRENT_RACK_GOALS.items():
            print(f"  {name:8s} ({x:+.3f}, {y:+.3f}, {th:+.2f})")
        return

    racks = args.racks if args.racks else DEFAULT_ORDER
    unknown = [r for r in racks if r not in CURRENT_RACK_GOALS]
    if unknown:
        print(f"ERROR: unknown rack(s): {unknown}")
        print(f"Known: {', '.join(CURRENT_RACK_GOALS.keys())}")
        sys.exit(1)

    rclpy.init()
    node = PoseReader()

    print("=" * 60)
    print(" RACK_GOALS RE-VERIFICATION")
    print(" Drive/teleop the robot to each rack (e.g. via ils_gui's")
    print(" D-pad), park it exactly as you would for a real pick, then")
    print(" press Enter. Ctrl+C any time to stop early (partial results")
    print(" are still printed).")
    print("=" * 60)

    results = {}
    try:
        for name in racks:
            old = CURRENT_RACK_GOALS[name]
            input(f"\n  Park robot at {name} (old: {old[0]:+.3f}, {old[1]:+.3f}, "
                  f"{old[2]:+.2f}), then press Enter...")
            pose = node.wait_for_pose()
            if pose is None:
                print(f"    WARNING: no /amcl_pose received for {name} -- skipped "
                      f"(is the nav stack / AMCL running?)")
                continue
            x, y, yaw = pose
            results[name] = (round(x, 3), round(y, 3), round(yaw, 2))
            print(f"    Recorded {name}: ({x:+.3f}, {y:+.3f}, {yaw:+.2f})  "
                  f"[old: ({old[0]:+.3f}, {old[1]:+.3f}, {old[2]:+.2f})]")
    except KeyboardInterrupt:
        print("\n  Stopped early.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not results:
        print("\nNo results recorded.")
        return

    print("\n" + "=" * 60)
    print(" RESULT -- paste into ils_gui.py's RACK_GOALS")
    print("=" * 60)
    print("RACK_GOALS = {")
    for name in DEFAULT_ORDER:
        if name in results:
            x, y, th = results[name]
        elif name in CURRENT_RACK_GOALS:
            x, y, th = CURRENT_RACK_GOALS[name]
        else:
            continue
        tag = '  # updated' if name in results else ''
        print(f"    '{name}': ({x:+.3f}, {y:+.3f}, {th:+.2f}),{tag}")
    print("}")


if __name__ == '__main__':
    main()
