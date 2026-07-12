#!/usr/bin/env python3
"""
analyze_trajectory_bag.py  --  Post-process rosbag for Test F trajectory drift
-------------------------------------------------------------------------------
Reads a rosbag recorded during trajectory_test (rosbag_ground_truth.py --test trajectory).
For each scenario, extracts robot pose at trajectory_start and trajectory_end,
then computes drift vs expected ground truth.

SCENARIOS and what "drift" means for each:
  stationary    : displacement from start while robot is still (should be ~0)
  straight_3m   : lateral drift (perpendicular to travel) + forward error vs 3m
  rotation_180  : heading error vs pi rad
  rotation_360  : heading error vs 2*pi rad (residual after full circle)
  return_origin : Euclidean distance from start when back at origin (should be ~0)

Usage:
  python3 analyze_trajectory_bag.py path/to/bag_directory/
  python3 analyze_trajectory_bag.py path/to/bag_directory/ --out-dir ~/thesis_data/trajectory_test
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


def yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def wrap_to_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def extract_pose(tf_odom_base, odom_raw):
    """Return (x, y, yaw) from odom->base_link TF, fallback to /odom_raw."""
    if tf_odom_base is not None:
        return tf_odom_base
    return odom_raw


def read_bag(bag_path):
    reader = rosbag2_py.SequentialReader()
    storage_opts = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_opts = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    reader.open(storage_opts, converter_opts)
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic in type_map:
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            yield topic, msg, timestamp


# Ground truth for each scenario
SCENARIO_GT = {
    'stationary':    {'type': 'position', 'expected_dx': 0.0, 'expected_dy': 0.0},
    'straight_3m':   {'type': 'straight', 'expected_dist': 3.0},
    'rotation_180':  {'type': 'rotation', 'expected_deg': 180.0},
    'rotation_360':  {'type': 'rotation', 'expected_deg': 360.0},
    'return_origin': {'type': 'position', 'expected_dx': 0.0, 'expected_dy': 0.0},
}


def compute_drift(scenario, start_pose, end_pose):
    """Compute drift for a scenario given start and end (x, y, yaw)."""
    sx, sy, syaw = start_pose
    ex, ey, eyaw = end_pose
    gt = SCENARIO_GT.get(scenario, {})
    result = {}

    if gt.get('type') == 'position':
        # Expected: robot is back at (or still at) start position
        dx = ex - sx
        dy = ey - sy
        drift_m = math.hypot(dx, dy)
        result = {
            'drift_x_m': round(dx, 4),
            'drift_y_m': round(dy, 4),
            'drift_total_m': round(drift_m, 4),
            'drift_total_cm': round(drift_m * 100, 1),
            'drift_yaw_deg': round(math.degrees(wrap_to_pi(eyaw - syaw)), 2),
        }

    elif gt.get('type') == 'straight':
        # Robot drove straight; measure displacement in travel direction vs 3m,
        # and lateral (sideways) drift.
        dx = ex - sx
        dy = ey - sy
        dist = math.hypot(dx, dy)
        expected = gt['expected_dist']
        # Forward error: how close to expected distance
        forward_error = dist - expected
        # Lateral drift: component perpendicular to the travel vector
        # We estimate travel direction from start->end vector
        if dist > 0.01:
            travel_yaw = math.atan2(dy, dx)
            lateral = abs(math.sin(travel_yaw - syaw) * dist)
        else:
            lateral = 0.0
        result = {
            'expected_dist_m': expected,
            'actual_dist_m': round(dist, 4),
            'forward_error_m': round(forward_error, 4),
            'forward_error_cm': round(forward_error * 100, 1),
            'lateral_drift_m': round(lateral, 4),
            'lateral_drift_cm': round(lateral * 100, 1),
        }

    elif gt.get('type') == 'rotation':
        expected_deg = gt['expected_deg']
        actual_change = math.degrees(wrap_to_pi(eyaw - syaw))
        # For 360, wrap_to_pi gives ~0; heading error = residual from full circle
        if expected_deg == 360.0:
            # Accumulate: we expect to be back to original heading
            heading_error = abs(actual_change)
        else:
            heading_error = abs(abs(actual_change) - expected_deg)
        result = {
            'expected_deg': expected_deg,
            'actual_change_deg': round(actual_change, 2),
            'heading_error_deg': round(heading_error, 2),
        }

    return result


def analyze_bag(bag_path, out_dir):
    print(f"Reading bag: {bag_path}")

    latest_odom_base = None
    latest_odom_raw = None

    # Collect (scenario, rep, start_pose, end_pose) pairs
    open_starts = {}   # key=(scenario, rep) -> start_pose
    records = []

    mode = 'C'

    for topic, msg, ts_ns in read_bag(bag_path):
        if topic == '/tf':
            for tf in msg.transforms:
                if (tf.header.frame_id, tf.child_frame_id) == ('odom', 'base_link'):
                    x = tf.transform.translation.x
                    y = tf.transform.translation.y
                    q = tf.transform.rotation
                    latest_odom_base = (x, y, yaw_from_quat(q.x, q.y, q.z, q.w))

        elif topic == '/odom_raw':
            p = msg.pose.pose
            q = p.orientation
            latest_odom_raw = (p.position.x, p.position.y,
                               yaw_from_quat(q.x, q.y, q.z, q.w))

        elif topic == '/ground_truth_event':
            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                continue

            event = data.get('event')
            scenario = data.get('scenario', '')
            rep = data.get('rep', 0)
            mode = data.get('mode', 'C')
            key = (scenario, rep)

            pose = extract_pose(latest_odom_base, latest_odom_raw)
            if pose is None:
                continue

            if event == 'trajectory_start':
                open_starts[key] = pose

            elif event == 'trajectory_end':
                if key in open_starts:
                    start_pose = open_starts.pop(key)
                    drift = compute_drift(scenario, start_pose, pose)
                    records.append({
                        'mode': mode,
                        'scenario': scenario,
                        'rep': rep,
                        'start_x': round(start_pose[0], 4),
                        'start_y': round(start_pose[1], 4),
                        'start_yaw_deg': round(math.degrees(start_pose[2]), 2),
                        'end_x': round(pose[0], 4),
                        'end_y': round(pose[1], 4),
                        'end_yaw_deg': round(math.degrees(pose[2]), 2),
                        **drift,
                    })

    if not records:
        print("ERROR: No trajectory records found. Did you run rosbag_ground_truth.py --test trajectory?")
        return

    print(f"\nFound {len(records)} trajectory measurements")
    print(f"Mode: {mode}\n")

    # Print per-scenario summary
    scenarios_seen = list(dict.fromkeys(r['scenario'] for r in records))
    summary_rows = []

    for sc in scenarios_seen:
        recs = [r for r in records if r['scenario'] == sc]
        gt = SCENARIO_GT.get(sc, {})
        print(f"{'='*60}")
        print(f"  {sc.upper()} (n={len(recs)})")
        print(f"{'='*60}")

        if gt.get('type') == 'position':
            drifts = [r['drift_total_cm'] for r in recs]
            mean = sum(drifts) / len(drifts)
            worst = max(drifts)
            print(f"  Position drift from start:")
            for r in recs:
                print(f"    rep {r['rep']}: dx={r['drift_x_m']:+.3f}m  dy={r['drift_y_m']:+.3f}m  "
                      f"total={r['drift_total_cm']:.1f}cm  yaw={r['drift_yaw_deg']:+.1f}°")
            print(f"  Mean drift: {mean:.1f} cm  |  Worst: {worst:.1f} cm")
            summary_rows.append({'scenario': sc, 'metric': 'drift_cm',
                                  'mean': round(mean, 1), 'worst': round(worst, 1)})

        elif gt.get('type') == 'straight':
            fwd_errs = [abs(r['forward_error_cm']) for r in recs]
            lat_errs = [r['lateral_drift_cm'] for r in recs]
            print(f"  Expected distance: {recs[0].get('expected_dist_m', 3.0)} m")
            for r in recs:
                print(f"    rep {r['rep']}: actual={r['actual_dist_m']:.3f}m  "
                      f"fwd_err={r['forward_error_cm']:+.1f}cm  lateral={r['lateral_drift_cm']:.1f}cm")
            print(f"  Mean forward error: {sum(fwd_errs)/len(fwd_errs):.1f} cm  "
                  f"|  Mean lateral drift: {sum(lat_errs)/len(lat_errs):.1f} cm")
            summary_rows.append({'scenario': sc, 'metric': 'fwd_err_cm + lateral_cm',
                                  'mean': round(sum(fwd_errs)/len(fwd_errs), 1),
                                  'worst': round(max(fwd_errs), 1)})

        elif gt.get('type') == 'rotation':
            errs = [r['heading_error_deg'] for r in recs]
            mean = sum(errs) / len(errs)
            print(f"  Expected rotation: {recs[0].get('expected_deg', '?')}°")
            for r in recs:
                print(f"    rep {r['rep']}: actual={r['actual_change_deg']:+.1f}°  "
                      f"error={r['heading_error_deg']:.1f}°")
            print(f"  Mean heading error: {mean:.1f}°  |  Worst: {max(errs):.1f}°")
            summary_rows.append({'scenario': sc, 'metric': 'heading_error_deg',
                                  'mean': round(mean, 1), 'worst': round(max(errs), 1)})

    # Save CSV
    os.makedirs(out_dir, exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(out_dir, f'trajectory_mode_{mode}_{ts}.csv')

    all_keys = set()
    for r in records:
        all_keys.update(r.keys())
    fieldnames = ['mode', 'scenario', 'rep',
                  'start_x', 'start_y', 'start_yaw_deg',
                  'end_x', 'end_y', 'end_yaw_deg'] + \
                 sorted(k for k in all_keys
                        if k not in ('mode', 'scenario', 'rep',
                                     'start_x', 'start_y', 'start_yaw_deg',
                                     'end_x', 'end_y', 'end_yaw_deg'))

    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
        w.writeheader()
        w.writerows(records)

    print(f"\nCSV saved: {csv_path}")


def main():
    ap = argparse.ArgumentParser(description='Analyze trajectory drift rosbag for Test F')
    ap.add_argument('bag_path', help='Path to rosbag directory')
    ap.add_argument('--out-dir', default=None,
                    help='Output directory (default: ~/thesis_data/trajectory_test)')
    args = ap.parse_args()

    out_dir = args.out_dir or os.path.expanduser('~/thesis_data/trajectory_test')
    analyze_bag(args.bag_path, out_dir)


if __name__ == '__main__':
    main()
