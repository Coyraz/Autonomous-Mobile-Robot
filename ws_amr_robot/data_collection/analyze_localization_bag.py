#!/usr/bin/env python3
"""
analyze_localization_bag.py  --  Post-process rosbag for Test E localization accuracy
-------------------------------------------------------------------------------------
Reads a rosbag recorded during a localization test, finds ground truth markers,
extracts the robot's estimated position at each marker timestamp from /tf,
and computes per-point error and overall RMSE.

HOW IT WORKS:
  1. Reads all /tf messages to track odom->base_link and map->odom transforms.
  2. Reads /ground_truth_event messages to find when the user marked reference points.
  3. At each marker timestamp, chains the latest transforms to get the robot's
     position in the appropriate frame:
       Mode A: odom->base_link (odom frame ≈ map frame when starting at Home)
       Mode B/C: map->odom + odom->base_link (full map frame position)
  4. Compares estimated position to ground truth, computes error.
  5. Outputs CSV and prints RMSE summary.

Also reads /odom_raw as a fallback position source for Mode A.

Usage:
  python3 analyze_localization_bag.py path/to/bag_directory/
  python3 analyze_localization_bag.py path/to/bag_directory/ --out-dir ~/thesis_data/localization_test
"""

import argparse
import csv
import json
import math
import os
import sys
from datetime import datetime

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def chain_2d(parent_tf, child_tf):
    """Chain two 2D transforms: parent * child -> position in parent's parent frame.
    Each tf is (x, y, yaw).
    Returns (x, y, yaw) of child origin in grandparent frame."""
    px, py, pyaw = parent_tf
    cx, cy, cyaw = child_tf
    x = px + math.cos(pyaw) * cx - math.sin(pyaw) * cy
    y = py + math.sin(pyaw) * cx + math.cos(pyaw) * cy
    yaw = pyaw + cyaw
    return (x, y, yaw)


def extract_2d_from_transform(tf_msg):
    """Extract (x, y, yaw) from a geometry_msgs/TransformStamped."""
    t = tf_msg.transform
    x = t.translation.x
    y = t.translation.y
    q = t.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return (x, y, yaw)


def read_bag(bag_path):
    """Read all messages from a rosbag. Yields (topic, msg, timestamp_ns)."""
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


def analyze_bag(bag_path, out_dir):
    """Main analysis: read bag, match markers to TF, compute errors."""

    print(f"Reading bag: {bag_path}")
    print("Pass 1: scanning all messages...")

    # State: latest transforms
    latest_odom_base = None    # (x, y, yaw) from odom->base_link
    latest_map_odom = None     # (x, y, yaw) from map->odom
    latest_odom_raw = None     # (x, y, yaw) from /odom_raw topic

    # Collected ground truth events
    markers = []

    # Track which transforms we've seen for diagnostics
    tf_frames_seen = set()

    msg_count = 0
    for topic, msg, ts_ns in read_bag(bag_path):
        msg_count += 1

        if topic == '/tf':
            for tf in msg.transforms:
                pair = (tf.header.frame_id, tf.child_frame_id)
                tf_frames_seen.add(pair)
                if pair == ('odom', 'base_link'):
                    latest_odom_base = extract_2d_from_transform(tf)
                elif pair == ('map', 'odom'):
                    latest_map_odom = extract_2d_from_transform(tf)

        elif topic == '/odom_raw':
            p = msg.pose.pose
            q = p.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            latest_odom_raw = (p.position.x, p.position.y, yaw)

        elif topic == '/ground_truth_event':
            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                continue
            if data.get('event') != 'point_reached':
                continue

            mode = data.get('mode', 'C')
            est_pos = None

            if mode == 'A':
                # Mode A: use odom->base_link (or /odom_raw fallback)
                if latest_odom_base is not None:
                    est_pos = latest_odom_base
                elif latest_odom_raw is not None:
                    est_pos = latest_odom_raw
            else:
                # Modes B, C: chain map->odom + odom->base_link
                if latest_map_odom is not None and latest_odom_base is not None:
                    est_pos = chain_2d(latest_map_odom, latest_odom_base)

            markers.append({
                'mode': mode,
                'rep': data.get('rep', 0),
                'point': data.get('point', '?'),
                'gt_x': data.get('gt_x', 0.0),
                'gt_y': data.get('gt_y', 0.0),
                'est_pos': est_pos,
                'ts_ns': ts_ns,
            })

    print(f"  Total messages: {msg_count}")
    print(f"  TF frame pairs seen: {tf_frames_seen}")
    print(f"  Ground truth markers: {len(markers)}")

    if not markers:
        print("\nERROR: No ground truth markers found in this bag.")
        print("Did you run rosbag_ground_truth.py during recording?")
        return

    # Compute errors
    rows = []
    for m in markers:
        if m['est_pos'] is None:
            print(f"  WARNING: no position data at marker {m['point']} rep {m['rep']}")
            continue
        est_x, est_y, est_yaw = m['est_pos']
        gt_x, gt_y = m['gt_x'], m['gt_y']
        err = math.hypot(est_x - gt_x, est_y - gt_y)
        rows.append({
            'mode': m['mode'],
            'rep': m['rep'],
            'point': m['point'],
            'gt_x': round(gt_x, 3),
            'gt_y': round(gt_y, 3),
            'est_x': round(est_x, 4),
            'est_y': round(est_y, 4),
            'est_yaw_rad': round(est_yaw, 4),
            'error_m': round(err, 4),
            'error_cm': round(err * 100, 1),
        })

    if not rows:
        print("\nERROR: Could not compute any position estimates.")
        return

    # Save CSV
    os.makedirs(out_dir, exist_ok=True)
    mode_label = rows[0]['mode']
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(out_dir, f'localization_mode_{mode_label}_{ts}.csv')
    fieldnames = ['mode', 'rep', 'point', 'gt_x', 'gt_y',
                  'est_x', 'est_y', 'est_yaw_rad', 'error_m', 'error_cm']
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)

    # Print summary
    print("\n" + "=" * 70)
    print(f" RESULTS  -  Mode {mode_label} ({len(rows)} measurements)")
    print("=" * 70)
    print(f" {'Point':>6} | {'Mean Err':>9} | {'Std':>7} | "
          f"{'Min':>7} | {'Max':>7} | {'N':>3}")
    print(" " + "-" * 58)

    points = sorted(set(r['point'] for r in rows),
                    key=lambda p: [r['point'] for r in rows].index(p))
    all_errors = []
    for pt in points:
        errs = [r['error_cm'] for r in rows if r['point'] == pt]
        all_errors.extend(errs)
        n = len(errs)
        mean = sum(errs) / n
        std = (sum((e - mean)**2 for e in errs) / n) ** 0.5 if n > 1 else 0.0
        print(f" {pt:>6} | {mean:7.1f} cm | {std:5.1f} | "
              f"{min(errs):5.1f} | {max(errs):5.1f} | {n:3d}")

    if all_errors:
        n = len(all_errors)
        mean = sum(all_errors) / n
        rmse = (sum(e**2 for e in all_errors) / n) ** 0.5
        print(" " + "-" * 58)
        print(f" Overall: mean={mean:.1f} cm  RMSE={rmse:.1f} cm  "
              f"worst={max(all_errors):.1f} cm  n={n}")

    print(f"\nCSV saved: {csv_path}")
    return csv_path


def main():
    ap = argparse.ArgumentParser(
        description='Analyze localization rosbag for Test E')
    ap.add_argument('bag_path', help='Path to rosbag directory')
    ap.add_argument('--out-dir', default=None,
                    help='Output directory (default: ~/thesis_data/localization_test)')
    args = ap.parse_args()

    out_dir = args.out_dir or os.path.expanduser(
        '~/thesis_data/localization_test')
    analyze_bag(args.bag_path, out_dir)


if __name__ == '__main__':
    main()
