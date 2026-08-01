#!/usr/bin/env python3
"""
test_h_detection_probe.py  --  Live obstacle-detection sensitivity probe
--------------------------------------------------------------------------
NOT the full Test H run (that's rosbag_ground_truth.py --test obstacle,
which measures stop/resume behavior during real navigation). This is a
narrower, standalone characterization tool: while the robot sits still (or
you drive it manually), it prints a live DETECTED / CLEAR line so you can
walk a box toward the robot and read off, by eye, the exact moment the
software "sees" it -- to compare against your own tape measurement.

It reports two numbers, because they answer different questions:

  scan_dist_cm      Raw LiDAR reading: nearest return within a forward cone
                     (default +/-15 deg around 0 rad in the laser frame).
                     This is "how far is the nearest physical point" --
                     updates the instant ANY beam touches the obstacle.

  stop_margin_cm    Replicates custom_path_controller's actual collision
                     check (_arc_blocked in custom_path_controller.py): scans
                     the global costmap along a straight line in front of the
                     robot's current heading, out to collision_horizon(1.2s)
                     * max_linear_speed(0.20 m/s) = ~24 cm, looking for the
                     first cell at cost >= collision_cost(99). This is what
                     actually makes the robot stop -- it lags scan_dist_cm
                     because the costmap needs a fresh scan + inflation to
                     mark the cell lethal, and because it only "fires" once
                     the obstacle is within the software's real look-ahead
                     margin (~24cm at full speed), not merely LiDAR range.

DETECTED means stop_margin_cm has a lethal cell inside the look-ahead arc
right now, i.e. the robot's controller would hold still if it were driving
straight at max speed at this instant.

Two test protocols this is meant to support (pick per rep, log which one):
  gradual  - start obstacle >=100cm out, walk it toward the robot slowly.
             Repeat ~5-10x, record scan_dist_cm / stop_margin_cm at the
             DETECTED transition each time -> mean detection range.
  sudden   - obstacle placed at ~20-25cm (inside the ~24cm computed stop
             margin) in one motion while the robot is driving toward it.
             Tests the worst case: can it actually halt before contact.

Usage:
  python3 test_h_detection_probe.py
  python3 test_h_detection_probe.py --cone-deg 20 --log-csv
"""

import argparse
import csv
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

import tf2_ros

# Must match custom_controller.yaml / custom_path_controller.py defaults.
# If you've tuned these away from default, pass the matching --v-max /
# --collision-horizon / --collision-cost flags so stop_margin_cm stays
# faithful to what the real controller does.
DEFAULT_V_MAX = 0.20
DEFAULT_COLLISION_HORIZON = 1.2
DEFAULT_COLLISION_COST = 99


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class DetectionProbe(Node):

    def __init__(self, cone_deg, v_max, collision_horizon, collision_cost, log_path):
        super().__init__('test_h_detection_probe')
        self.cone_rad = math.radians(cone_deg)
        self.v_max = v_max
        self.collision_horizon = collision_horizon
        self.collision_cost = collision_cost
        self.probe_dist = v_max * collision_horizon  # meters, straight-line look-ahead

        self._scan_dist_m = None
        self._costmap = None
        self._prev_detected = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            LaserScan, '/scan', self._cb_scan, qos_profile_sensor_data)
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self._cb_costmap, 1)

        self.log_path = log_path
        self._log_file = None
        self._log_writer = None
        if log_path:
            os.makedirs(os.path.dirname(log_path), exist_ok=True)
            self._log_file = open(log_path, 'w', newline='')
            self._log_writer = csv.writer(self._log_file)
            self._log_writer.writerow(
                ['t_iso', 'event', 'scan_dist_cm', 'stop_margin_cm', 'detected'])

        self.get_logger().info(
            f'Probe ready. Forward cone +/-{cone_deg:.0f} deg. '
            f'Look-ahead {self.probe_dist*100:.1f} cm '
            f'(v_max={v_max} m/s x horizon={collision_horizon}s).')

    # ---------------------------------------------------------- callbacks
    def _cb_scan(self, msg):
        n = len(msg.ranges)
        best = None
        for i, r in enumerate(msg.ranges):
            ang = msg.angle_min + i * msg.angle_increment
            if abs(ang) > self.cone_rad:
                continue
            if r < msg.range_min or r > msg.range_max:
                continue
            if math.isnan(r) or math.isinf(r):
                continue
            if best is None or r < best:
                best = r
        self._scan_dist_m = best

    def _cb_costmap(self, msg):
        self._costmap = msg

    # -------------------------------------------------------------- costmap lookup
    def _cost_at(self, wx, wy):
        cm = self._costmap
        if cm is None:
            return None
        info = cm.info
        mx = int((wx - info.origin.position.x) / info.resolution)
        my = int((wy - info.origin.position.y) / info.resolution)
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None
        return cm.data[my * info.width + mx]

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        return (t.transform.translation.x, t.transform.translation.y,
                yaw_from_quat(t.transform.rotation))

    def stop_margin_cm(self, pose):
        """Straight line ahead of the robot's current heading, out to
        probe_dist. Returns distance (cm) to the first lethal cell, or None
        if the arc is clear (no cell >= collision_cost within range)."""
        if self._costmap is None:
            return None
        rx, ry, ryaw = pose
        dt = 0.02
        steps = int(self.probe_dist / (self.v_max * dt)) if self.v_max > 0 else 0
        for i in range(steps):
            d = self.v_max * dt * (i + 1)
            x = rx + d * math.cos(ryaw)
            y = ry + d * math.sin(ryaw)
            c = self._cost_at(x, y)
            if c is not None and c >= self.collision_cost:
                return d * 100.0
        return None

    # -------------------------------------------------------------------- log
    def _log(self, event, scan_cm, margin_cm, detected):
        if self._log_writer is None:
            return
        self._log_writer.writerow([
            datetime.now().isoformat(timespec='milliseconds'),
            event,
            f'{scan_cm:.1f}' if scan_cm is not None else '',
            f'{margin_cm:.1f}' if margin_cm is not None else '',
            int(detected)])
        self._log_file.flush()

    def close(self):
        if self._log_file is not None:
            self._log_file.close()

    # ------------------------------------------------------------------- loop
    def tick(self):
        pose = self.get_robot_pose()
        scan_cm = self._scan_dist_m * 100.0 if self._scan_dist_m is not None else None
        margin_cm = self.stop_margin_cm(pose) if pose is not None else None
        detected = margin_cm is not None

        scan_str = f'{scan_cm:6.1f} cm' if scan_cm is not None else '   n/a   '
        margin_str = f'{margin_cm:6.1f} cm' if margin_cm is not None else '   n/a   '
        state = 'DETECTED (would stop)' if detected else 'clear                '
        print(f'\r  scan_dist={scan_str}  stop_margin={margin_str}  [{state}]',
              end='', flush=True)

        if detected != self._prev_detected:
            event = 'DETECTED' if detected else 'CLEARED'
            print(f'\n  >>> {event} at t={datetime.now().strftime("%H:%M:%S.%f")[:-3]} '
                  f'  scan_dist={scan_str}  stop_margin={margin_str}')
            self._log(event, scan_cm, margin_cm, detected)
            self._prev_detected = detected


def main():
    ap = argparse.ArgumentParser(description='Live obstacle-detection sensitivity probe')
    ap.add_argument('--cone-deg', type=float, default=15.0,
                     help='Forward LiDAR cone half-angle for scan_dist_cm (default 15)')
    ap.add_argument('--v-max', type=float, default=DEFAULT_V_MAX,
                     help='Must match custom_controller.yaml max_linear_speed (default 0.20)')
    ap.add_argument('--collision-horizon', type=float, default=DEFAULT_COLLISION_HORIZON,
                     help='Must match custom_controller.yaml collision_horizon (default 1.2)')
    ap.add_argument('--collision-cost', type=int, default=DEFAULT_COLLISION_COST,
                     help='Must match custom_controller.yaml collision_cost (default 99)')
    ap.add_argument('--log-csv', action='store_true',
                     help='Log DETECTED/CLEARED transitions to ~/thesis_data/obstacle_test/')
    args = ap.parse_args()

    log_path = None
    if args.log_csv:
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_path = os.path.expanduser(f'~/thesis_data/obstacle_test/detection_probe_{ts}.csv')

    rclpy.init()
    node = DetectionProbe(args.cone_deg, args.v_max, args.collision_horizon,
                           args.collision_cost, log_path)
    print('Walk an obstacle toward the robot\'s front and watch the state flip.\n'
          'Ctrl+C to stop.\n')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            node.tick()
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
