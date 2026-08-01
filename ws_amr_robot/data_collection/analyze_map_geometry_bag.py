#!/usr/bin/env python3
"""
analyze_map_geometry_bag.py  --  Post-process rosbag for Test C map geometry validation
------------------------------------------------------------------------------------------
Reads a rosbag recorded via `rosbag_ground_truth.py --test map_geometry`, finds
the 14 ground truth markers, extracts the robot's map-frame position (AMCL,
composed with TF) at each marker timestamp, and reproduces the same two
outputs as the original (2026-06-22) measure_map_geometry.py:
  - per-point position error (AMCL vs tape)
  - pairwise distance error for the same rack/aisle pairs

Position source: map->odom (AMCL) chained with odom->base_link (from /tf),
same convention as analyze_localization_bag.py / analyze_navigation_bag.py.

Usage:
  python3 analyze_map_geometry_bag.py path/to/test_c_map_geometry/
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

# Same 14-point tape ground truth as rosbag_ground_truth.py's
# MAP_GEOMETRY_POINTS / test3.md's TITIK_KOORDINAT. Do not delete/regenerate.
REAL = {
    'Home':  (0.0,   0.0),
    'Stage': (3.5,   0.5),
    'A1':    (4.0,  -8.5),
    'A2':    (4.0,  -7.0),
    'A3':    (4.0,  -4.5),
    'A4':    (4.0,  -3.0),
    'B1':    (1.5,  -8.5),
    'B2':    (1.5,  -7.0),
    'B3':    (1.5,  -4.5),
    'B4':    (1.5,  -3.0),
    'C1':    (-1.0, -8.5),
    'C2':    (-1.0, -7.0),
    'C3':    (-1.0, -4.5),
    'C4':    (-1.0, -3.0),
}

# Same meaningful pairs as the original measure_map_geometry.py
PAIRS = [
    ('A1', 'A2', 'Rack A: A1-A2'), ('A2', 'A3', 'Rack A: A2-A3'),
    ('A3', 'A4', 'Rack A: A3-A4'), ('A1', 'A4', 'Rack A full: A1-A4'),
    ('B1', 'B2', 'Rack B: B1-B2'), ('B2', 'B3', 'Rack B: B2-B3'),
    ('B3', 'B4', 'Rack B: B3-B4'), ('B1', 'B4', 'Rack B full: B1-B4'),
    ('C1', 'C2', 'Rack C: C1-C2'), ('C2', 'C3', 'Rack C: C2-C3'),
    ('C3', 'C4', 'Rack C: C3-C4'), ('C1', 'C4', 'Rack C full: C1-C4'),
    ('A1', 'B1', 'Aisle A-B at row 1'), ('A2', 'B2', 'Aisle A-B at row 2'),
    ('A3', 'B3', 'Aisle A-B at row 3'), ('A4', 'B4', 'Aisle A-B at row 4'),
    ('B1', 'C1', 'Aisle B-C at row 1'), ('B2', 'C2', 'Aisle B-C at row 2'),
    ('B3', 'C3', 'Aisle B-C at row 3'), ('B4', 'C4', 'Aisle B-C at row 4'),
    ('A1', 'C1', 'Full width A-C at row 1'), ('A4', 'C4', 'Full width A-C at row 4'),
    ('Home', 'Stage', 'Home to Stage'),
]


def dist(p1, p2):
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


def chain_2d(parent_tf, child_tf):
    px, py, pyaw = parent_tf
    cx, cy, cyaw = child_tf
    x = px + math.cos(pyaw) * cx - math.sin(pyaw) * cy
    y = py + math.sin(pyaw) * cx + math.cos(pyaw) * cy
    return (x, y, pyaw + cyaw)


def extract_2d_from_transform(tf_msg):
    t = tf_msg.transform
    q = t.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return (t.translation.x, t.translation.y, yaw)


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


def analyze_bag(bag_path, out_dir):
    print(f"Reading bag: {bag_path}")

    latest_odom_base = None
    latest_map_odom = None
    markers = []

    msg_count = 0
    for topic, msg, ts_ns in read_bag(bag_path):
        msg_count += 1
        if topic == '/tf':
            for tf in msg.transforms:
                pair = (tf.header.frame_id, tf.child_frame_id)
                if pair == ('odom', 'base_link'):
                    latest_odom_base = extract_2d_from_transform(tf)
                elif pair == ('map', 'odom'):
                    latest_map_odom = extract_2d_from_transform(tf)
        elif topic == '/ground_truth_event':
            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                continue
            if data.get('event') != 'point_reached' or data.get('test') != 'map_geometry':
                continue
            est_pos = None
            if latest_map_odom is not None and latest_odom_base is not None:
                est_pos = chain_2d(latest_map_odom, latest_odom_base)
            markers.append({
                'rep': data.get('rep', 1),
                'point': data.get('point', '?'),
                'gt_x': data.get('gt_x', 0.0),
                'gt_y': data.get('gt_y', 0.0),
                'est_pos': est_pos,
                'manual_offset_cm': data.get('manual_offset_cm'),
            })

    print(f"  Total messages: {msg_count}")
    print(f"  Ground truth markers: {len(markers)}")
    if not markers:
        print("\nERROR: No map_geometry ground truth markers found in this bag.")
        print("Did you run rosbag_ground_truth.py --test map_geometry during recording?")
        return

    # Use the LAST rep's readings if multiple reps were recorded (most settled)
    # Group all valid readings by point (across ALL reps, not just the last)
    by_point = {}
    for m in markers:
        if m['est_pos'] is None:
            continue
        by_point.setdefault(m['point'], []).append(
            (m['rep'], m['est_pos'][0], m['est_pos'][1], m.get('manual_offset_cm')))

    missing = [name for name in REAL if name not in by_point]
    if missing:
        print(f"  WARNING: missing AMCL reads for: {missing} (excluded from pairs that need them)")

    n_reps = max((r for m in markers for r in [m['rep']]), default=1)
    print(f"  Reps found: {n_reps}")

    # Raw per-rep rows (for reprocessing/plotting later, and repeatability checks)
    raw_rows = []
    for name, (rx, ry) in REAL.items():
        for rep, ax, ay, manual_cm in by_point.get(name, []):
            pe = math.hypot(ax - rx, ay - ry)
            raw_rows.append({'rep': rep, 'point': name, 'real_x': rx, 'real_y': ry,
                              'amcl_x': round(ax, 4), 'amcl_y': round(ay, 4),
                              'error_cm': round(pe * 100, 1),
                              'manual_offset_cm': manual_cm})

    # Per-point aggregate (mean/std/min/max/N across reps) -- mean AMCL
    # position is also what feeds the pairwise-distance calc below.
    point_rows = []
    mean_amcl = {}
    for name, (rx, ry) in REAL.items():
        readings = by_point.get(name)
        if not readings:
            continue
        errs = [math.hypot(ax - rx, ay - ry) * 100 for (_, ax, ay, _) in readings]
        mean_x = sum(ax for (_, ax, ay, _) in readings) / len(readings)
        mean_y = sum(ay for (_, ax, ay, _) in readings) / len(readings)
        mean_amcl[name] = (mean_x, mean_y)
        n = len(errs)
        mean_e = sum(errs) / n
        std_e = (sum((e - mean_e) ** 2 for e in errs) / n) ** 0.5 if n > 1 else 0.0
        point_rows.append({'point': name, 'real_x': rx, 'real_y': ry,
                            'amcl_x': round(mean_x, 3), 'amcl_y': round(mean_y, 3),
                            'mean_error_cm': round(mean_e, 1), 'std_error_cm': round(std_e, 1),
                            'min_error_cm': round(min(errs), 1), 'max_error_cm': round(max(errs), 1),
                            'n': n})

    # Pairwise distances, computed from the MEAN AMCL position per point
    dist_rows = []
    for pa, pb, desc in PAIRS:
        if pa not in mean_amcl or pb not in mean_amcl:
            continue
        real_d = dist(REAL[pa], REAL[pb])
        amcl_d = dist(mean_amcl[pa], mean_amcl[pb])
        err_cm = (amcl_d - real_d) * 100
        err_pct = (err_cm / 100 / real_d * 100) if real_d > 0 else 0.0
        dist_rows.append({'pair': f"{pa}-{pb}", 'description': desc,
                           'tape_m': round(real_d, 3), 'map_m': round(amcl_d, 3),
                           'error_cm': round(err_cm, 1), 'error_pct': round(err_pct, 1)})

    # Print
    print(f"\n{'':>2} {'Description':>25} | {'tape(m)':>8} | {'map(m)':>8} | {'err(cm)':>8} | {'err%':>7}")
    print(" " + "-" * 68)
    for r in dist_rows:
        print(f"  {r['description']:>25} | {r['tape_m']:8.3f} | {r['map_m']:8.3f} | "
              f"{r['error_cm']:+8.1f} | {r['error_pct']:+7.1f}%")
    if dist_rows:
        abs_errs = [abs(r['error_cm']) for r in dist_rows]
        abs_pcts = [abs(r['error_pct']) for r in dist_rows]
        print(" " + "-" * 68)
        print(f"  mean |error|: {sum(abs_errs)/len(abs_errs):.1f} cm ({sum(abs_pcts)/len(abs_pcts):.1f}%)")
        print(f"  max  |error|: {max(abs_errs):.1f} cm ({max(abs_pcts):.1f}%)")

    print(f"\n{'':>2} PER-POINT POSITION ERROR (AMCL vs tape, mean over {n_reps} reps)")
    print(f"{'':>2} {'Point':>8} | {'Mean':>7} | {'Std':>6} | {'Min':>6} | {'Max':>6} | {'N':>3}")
    print(" " + "-" * 56)
    for p in point_rows:
        print(f"  {p['point']:>8} | {p['mean_error_cm']:5.1f}cm | {p['std_error_cm']:4.1f} | "
              f"{p['min_error_cm']:4.1f} | {p['max_error_cm']:4.1f} | {p['n']:3d}")
    if point_rows:
        pt_errs = [p['mean_error_cm'] for p in point_rows]
        rmse = (sum((e/100)**2 for e in pt_errs) / len(pt_errs)) ** 0.5 * 100
        print(" " + "-" * 56)
        print(f"  mean: {sum(pt_errs)/len(pt_errs):.1f} cm   max: {max(pt_errs):.1f} cm   RMSE: {rmse:.1f} cm")

    # Save CSVs, same naming convention as the original measure_map_geometry.py
    # (+ a new raw per-rep CSV for repeatability/reprocessing)
    os.makedirs(out_dir, exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv1 = os.path.join(out_dir, f"map_geometry_distances_{ts}.csv")
    with open(csv1, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['pair', 'description', 'tape_m', 'map_m', 'error_cm', 'error_pct'])
        w.writeheader()
        w.writerows(dist_rows)
    csv2 = os.path.join(out_dir, f"map_geometry_points_{ts}.csv")
    with open(csv2, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['point', 'real_x', 'real_y', 'amcl_x', 'amcl_y',
                                           'mean_error_cm', 'std_error_cm', 'min_error_cm',
                                           'max_error_cm', 'n'])
        w.writeheader()
        w.writerows(point_rows)
    csv3 = os.path.join(out_dir, f"map_geometry_raw_{ts}.csv")
    with open(csv3, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['rep', 'point', 'real_x', 'real_y', 'amcl_x', 'amcl_y',
                                           'error_cm', 'manual_offset_cm'])
        w.writeheader()
        w.writerows(raw_rows)

    print(f"\n  Distance CSV: {csv1}")
    print(f"  Point CSV:    {csv2}")
    print(f"  Raw CSV:      {csv3}")


def main():
    ap = argparse.ArgumentParser(description='Analyze map geometry rosbag for Test C')
    ap.add_argument('bag_path', help='Path to test_c_map_geometry bag directory')
    ap.add_argument('--out-dir', default=None,
                    help='Output directory (default: ~/thesis_data/map_geometry)')
    args = ap.parse_args()
    out_dir = args.out_dir or os.path.expanduser('~/thesis_data/map_geometry')
    analyze_bag(args.bag_path, out_dir)


if __name__ == '__main__':
    main()
