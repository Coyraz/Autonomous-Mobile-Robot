#!/usr/bin/env python3
"""
analyze_navigation_bag.py  --  Post-process rosbag for Test G point-to-point nav
--------------------------------------------------------------------------------
Reads a bag recorded during `rosbag_ground_truth.py --test navigation`.
For each goal, pairs the `nav_goal_sent` and `nav_goal_result` events and
computes the navigation-quality metrics.

METRICS (per goal + aggregate):
  success            result == 'success' (1/0)                 -> success rate
  travel_time_s      wall time from goal_sent to goal_result   -> per goal & mean
  stop_error_cm      distance robot's final map pose -> tape ground truth (gt_x,gt_y)
  tape_error_cm      operator's live tape-measurement of stop error (cm), if entered
                     during collection -- a physical cross-check independent of AMCL/map
  final_yaw_deg      robot heading at stop (no yaw GT; reported for reference)
  path_len_m         integrated path length map->base_link during the goal
  straight_line_m    straight-line distance start -> goal
  path_efficiency    path_len / straight_line   (1.0 = perfect straight path)

Robot map-frame pose = TF chain  map->odom (AMCL) . odom->base_link (EKF),
fallback to /amcl_pose. Ground truth = tape coordinates carried in the events.

NOTE: with the STM32 motor at ~23% steady-state error the robot drives slower,
so `travel_time_s` is inflated (~+30%). success / stop_error / path_efficiency
are robust to a symmetric wheel slowdown and remain valid.

Usage:
  python3 analyze_navigation_bag.py path/to/test_g/
  python3 analyze_navigation_bag.py path/to/test_g/ --out-dir ~/thesis_data/navigation_test
"""

import argparse
import csv
import json
import math
import os
from datetime import datetime

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import amr_test_utils as U


def read_bag(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                            output_serialization_format='cdr'))
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic in type_map:
            yield topic, deserialize_message(data, get_message(type_map[topic])), ts


def compose_map_base(map_odom, odom_base):
    """Compose map->odom and odom->base_link (2D) into map->base_link (x,y,yaw)."""
    mx, my, myaw = map_odom
    ox, oy, oyaw = odom_base
    c, s = math.cos(myaw), math.sin(myaw)
    x = mx + c * ox - s * oy
    y = my + s * ox + c * oy
    return (x, y, U.wrap_to_pi(myaw + oyaw))


def analyze_bag(bag_path, out_dir):
    print(f"Reading bag: {bag_path}")
    latest_map_odom = None
    latest_odom_base = None
    latest_amcl = None

    open_goal = None       # dict for the currently in-flight goal
    path_pts = []          # map-frame (x,y) samples during the current goal
    records = []
    mode = 'C'

    def cur_pose():
        if latest_map_odom is not None and latest_odom_base is not None:
            return compose_map_base(latest_map_odom, latest_odom_base)
        return latest_amcl   # (x,y,yaw) or None

    for topic, msg, ts_ns in read_bag(bag_path):
        t_s = ts_ns * 1e-9

        if topic == '/tf':
            for tf in msg.transforms:
                fid, cid = tf.header.frame_id, tf.child_frame_id
                tr, q = tf.transform.translation, tf.transform.rotation
                yaw = U.yaw_from_quaternion(q.x, q.y, q.z, q.w)
                if (fid, cid) == ('map', 'odom'):
                    latest_map_odom = (tr.x, tr.y, yaw)
                elif (fid, cid) == ('odom', 'base_link'):
                    latest_odom_base = (tr.x, tr.y, yaw)

        elif topic == '/amcl_pose':
            p = msg.pose.pose
            q = p.orientation
            latest_amcl = (p.position.x, p.position.y,
                           U.yaw_from_quaternion(q.x, q.y, q.z, q.w))

        elif topic == '/ground_truth_event':
            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                continue
            ev = data.get('event')
            mode = data.get('mode', mode)

            if ev == 'nav_goal_sent':
                pose = cur_pose()
                open_goal = {
                    'rep': data.get('rep'), 'target': data.get('target'),
                    'gt_x': data.get('gt_x'), 'gt_y': data.get('gt_y'),
                    't_sent': t_s,
                    'start': pose,
                }
                path_pts = [pose[:2]] if pose else []

            elif ev == 'nav_goal_result' and open_goal is not None:
                pose = cur_pose()
                if pose:
                    path_pts.append(pose[:2])
                result = data.get('result', 'unknown')
                # tolerate operator typos in the live s/f/t prompt (e.g. 'ss'
                # instead of 's') -- 's...'=success, 'f...'=fail, 't...'=timeout
                # never share a first letter, so this can't misclassify a
                # real failure/timeout as a success.
                success = 1 if result.lower().startswith('s') else 0

                gt_x, gt_y = open_goal['gt_x'], open_goal['gt_y']
                travel_time = t_s - open_goal['t_sent']
                tape_error_cm = data.get('tape_error_cm')

                # stop accuracy vs tape ground truth
                if pose and gt_x is not None:
                    stop_err = math.hypot(pose[0] - gt_x, pose[1] - gt_y)
                    final_yaw = math.degrees(pose[2])
                else:
                    stop_err, final_yaw = float('nan'), float('nan')

                # path length + efficiency
                path_len = sum(math.hypot(path_pts[i+1][0]-path_pts[i][0],
                                          path_pts[i+1][1]-path_pts[i][1])
                               for i in range(len(path_pts)-1)) if len(path_pts) > 1 else 0.0
                if open_goal['start'] and gt_x is not None:
                    straight = math.hypot(gt_x - open_goal['start'][0],
                                          gt_y - open_goal['start'][1])
                else:
                    straight = float('nan')
                eff = (path_len / straight) if straight and straight > 0.05 else float('nan')

                records.append({
                    'mode': mode, 'rep': open_goal['rep'], 'target': open_goal['target'],
                    'result': result, 'success': success,
                    'gt_x': gt_x, 'gt_y': gt_y,
                    'stop_x': round(pose[0], 4) if pose else float('nan'),
                    'stop_y': round(pose[1], 4) if pose else float('nan'),
                    'final_yaw_deg': round(final_yaw, 2),
                    'travel_time_s': round(travel_time, 2),
                    'stop_error_cm': round(stop_err * 100, 1) if not math.isnan(stop_err) else float('nan'),
                    'tape_error_cm': tape_error_cm if tape_error_cm is not None else float('nan'),
                    'path_len_m': round(path_len, 3),
                    'straight_line_m': round(straight, 3) if not math.isnan(straight) else float('nan'),
                    'path_efficiency': round(eff, 3) if not math.isnan(eff) else float('nan'),
                })
                open_goal = None
                path_pts = []
            continue

        # accumulate path samples while a goal is in flight
        if open_goal is not None:
            p = cur_pose()
            if p:
                if not path_pts or math.hypot(p[0]-path_pts[-1][0], p[1]-path_pts[-1][1]) > 0.02:
                    path_pts.append(p[:2])

    if not records:
        print("ERROR: no nav goals found. Did you run rosbag_ground_truth.py --test navigation?")
        return

    # ---- summary ----
    n = len(records)
    n_ok = sum(r['success'] for r in records)
    print(f"\n{'='*64}")
    print(f"  TEST G  NAVIGATION  (mode {mode})   goals={n}")
    print(f"{'='*64}")
    print(f"  Success rate      : {n_ok}/{n} = {100.0*n_ok/n:.1f}%")

    ok = [r for r in records if r['success']]
    def meanf(key, rows):
        vals = [r[key] for r in rows if not math.isnan(r[key])]
        return sum(vals)/len(vals) if vals else float('nan')
    if ok:
        print(f"  Travel time (ok)  : mean {meanf('travel_time_s', ok):.1f} s  "
              f"(inflated ~+30% by 23% motor SS error)")
        print(f"  Stop error (ok)   : mean {meanf('stop_error_cm', ok):.1f} cm  "
              f"worst {max(r['stop_error_cm'] for r in ok if not math.isnan(r['stop_error_cm'])):.1f} cm")
        print(f"  Path efficiency   : mean {meanf('path_efficiency', ok):.3f} "
              f"(1.0 = perfectly straight)")

    print(f"\n  {'target':10} {'rep':>3} {'result':>8} {'time_s':>7} "
          f"{'stop_cm':>8} {'eff':>6}")
    for r in records:
        print(f"  {r['target']:10} {r['rep']:>3} {r['result']:>8} "
              f"{r['travel_time_s']:>7.1f} {r['stop_error_cm']:>8} {r['path_efficiency']:>6}")

    # ---- CSV ----
    os.makedirs(out_dir, exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(out_dir, f'navigation_mode_{mode}_{ts}.csv')
    fields = ['mode', 'rep', 'target', 'result', 'success', 'gt_x', 'gt_y',
              'stop_x', 'stop_y', 'final_yaw_deg', 'travel_time_s',
              'stop_error_cm', 'tape_error_cm', 'path_len_m', 'straight_line_m',
              'path_efficiency']
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        w.writeheader()
        w.writerows(records)
    print(f"\nCSV saved: {csv_path}")


def main():
    ap = argparse.ArgumentParser(description='Analyze Test G navigation rosbag')
    ap.add_argument('bag_path', help='Path to the test_g bag directory')
    ap.add_argument('--out-dir', default=None,
                    help='Output dir (default ~/thesis_data/navigation_test)')
    args = ap.parse_args()
    out_dir = args.out_dir or os.path.expanduser('~/thesis_data/navigation_test')
    analyze_bag(args.bag_path, out_dir)


if __name__ == '__main__':
    main()
