#!/usr/bin/env python3
"""
plot_test_e_localization.py  --  3-mode localization XY overlay (Test E)
--------------------------------------------------------------------------
Reads the three Test E rosbags (recorded via localization_test.launch.py
mode:=A/B/C + rosbag_ground_truth.py --test localization) and plots all
three position traces overlaid in one map-frame XY figure, against the tape
ground-truth reference points.

Per mode, position source differs (see localization_test.launch.py):
  Mode A  encoder odometry only, NO map frame -- /odom_raw directly (its
          own odom frame origin coincides with Home/map (0,0), same
          convention as every other test)
  Mode B  AMCL correcting RAW encoder odom -- TF map->odom (AMCL) composed
          with odom->base_link (raw encoder)
  Mode C  AMCL correcting EKF fused odom -- TF map->odom (AMCL) composed
          with odom->base_link (EKF)

Usage:
  python3 plot_test_e_localization.py \\
      ~/thesis_data/rosbags/test_e_mode_a/test_e_mode_a \\
      ~/thesis_data/rosbags/test_e_mode_b/test_e_mode_b \\
      ~/thesis_data/rosbags/test_e_mode_c/test_e_mode_c
"""

import argparse
import math
import os

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import amr_test_utils as U

# Test E's 6 reference points (test7.md) -- same values as TITIK_KOORDINAT's
# corresponding entries (confirmed 2026-07-14 as the real tape ground truth).
REFERENCE_POINTS = {
    'Home': (0.0, 0.0),
    'A1':   (4.0, -8.5),
    'A4':   (4.0, -3.0),
    'B2':   (1.5, -7.0),
    'B4':   (1.5, -3.0),
    'C3':  (-1.0, -4.5),
}

# dataviz skill palette (references/palette.md) -- 3 well-separated
# categorical slots, one per mode.
COLOR_MODE_A = '#008300'   # slot 4 green  -- encoder-only (expected worst, big drift)
COLOR_MODE_B = '#2a78d6'   # slot 1 blue   -- AMCL + raw encoder
COLOR_MODE_C = '#e34948'   # slot 6 red    -- AMCL + EKF fused (expected best)
COLOR_GT     = '#0b0b0b'   # tape ground truth markers -- primary ink


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
    mx, my, myaw = map_odom
    ox, oy, oyaw = odom_base
    c, s = math.cos(myaw), math.sin(myaw)
    x = mx + c * ox - s * oy
    y = my + s * ox + c * oy
    return (x, y)


def extract_mode_a(bag_paths):
    """Encoder-only: no map frame, use /odom_raw directly. bag_paths: list,
    read in order and concatenated (dual-mode session split across
    test_e_mode_ab_v2 + v3 -- see test7.md provenance)."""
    trace = []
    for bag_path in bag_paths:
        for topic, msg, ts_ns in read_bag(bag_path):
            if topic == '/odom_raw':
                p = msg.pose.pose.position
                trace.append((p.x, p.y))
    return trace


def extract_mode_bc(bag_paths):
    """AMCL-corrected: compose map->odom (AMCL) with odom->base_link.
    bag_paths: list, read in order and concatenated."""
    trace = []
    for bag_path in bag_paths:
        latest_map_odom = None
        for topic, msg, ts_ns in read_bag(bag_path):
            if topic == '/tf':
                for tf in msg.transforms:
                    fid, cid = tf.header.frame_id, tf.child_frame_id
                    tr, q = tf.transform.translation, tf.transform.rotation
                    yaw = U.yaw_from_quaternion(q.x, q.y, q.z, q.w)
                    if (fid, cid) == ('map', 'odom'):
                        latest_map_odom = (tr.x, tr.y, yaw)
                    elif (fid, cid) == ('odom', 'base_link'):
                        if latest_map_odom is not None:
                            trace.append(compose_map_base(latest_map_odom, (tr.x, tr.y, yaw)))
    return trace


def plot(trace_a, trace_b, trace_c, out_path, title, map_yaml=None):
    fig, ax = plt.subplots(figsize=(9, 11), dpi=150)
    fig.patch.set_facecolor('#fcfcfb')
    ax.set_facecolor('#fcfcfb')

    map_extent = None
    if map_yaml:
        map_img, map_extent = U.load_map_image(map_yaml)
        ax.imshow(map_img, extent=map_extent, origin='upper', cmap='gray', zorder=0)

    if trace_a:
        xs, ys = zip(*trace_a)
        ax.plot(xs, ys, color=COLOR_MODE_A, linewidth=1.4, alpha=0.85,
                 zorder=2, label='Mode A - Encoder only')
    if trace_b:
        xs, ys = zip(*trace_b)
        ax.plot(xs, ys, color=COLOR_MODE_B, linewidth=1.4, alpha=0.85,
                 zorder=3, label='Mode B - AMCL + encoder')
    if trace_c:
        xs, ys = zip(*trace_c)
        ax.plot(xs, ys, color=COLOR_MODE_C, linewidth=1.4, alpha=0.85,
                 zorder=4, label='Mode C - AMCL + EKF fused')

    for name, (gx, gy) in REFERENCE_POINTS.items():
        ax.scatter([gx], [gy], marker='*', s=200, color=COLOR_GT,
                    edgecolors='white', linewidths=0.8, zorder=5)
        ax.annotate(name, (gx, gy), textcoords='offset points',
                     xytext=(7, 5), fontsize=10, color='#0b0b0b', zorder=6)

    ax.scatter([], [], marker='*', s=200, color=COLOR_GT, edgecolors='white',
                linewidths=0.8, label='Titik referensi meteran (ground truth)')

    ax.set_xlabel('X (m)', color='#0b0b0b')
    ax.set_ylabel('Y (m)', color='#0b0b0b')
    ax.set_title(title, color='#0b0b0b', fontsize=13)
    ax.set_aspect('equal')
    if map_extent:
        ax.set_xlim(map_extent[0], map_extent[1])
        ax.set_ylim(map_extent[2], map_extent[3])
    ax.grid(True, color='#e3e2dc', linewidth=0.6, zorder=0)
    for spine in ax.spines.values():
        spine.set_color('#c3c2b7')
    ax.tick_params(colors='#52514e')
    ax.legend(loc='upper left', frameon=True, facecolor='#fcfcfb',
              edgecolor='#c3c2b7', fontsize=9)

    fig.tight_layout()
    fig.savefig(out_path, facecolor=fig.get_facecolor())
    print(f"Saved: {out_path}")
    print(f"  Mode A samples: {len(trace_a)}")
    print(f"  Mode B samples: {len(trace_b)}")
    print(f"  Mode C samples: {len(trace_c)}")


def main():
    ap = argparse.ArgumentParser(description='Plot Test E 3-mode localization XY overlay')
    ap.add_argument('--bag-a', nargs='+', required=True, help='Path(s) to Mode A bag(s), read in order')
    ap.add_argument('--bag-b', nargs='+', required=True, help='Path(s) to Mode B bag(s), read in order')
    ap.add_argument('--bag-c', nargs='+', required=True, help='Path(s) to Mode C bag(s), read in order')
    ap.add_argument('--map', default=None, help='Path to map yaml, drawn as background (full extent, not zoomed)')
    ap.add_argument('--out', default=None, help='Output PNG path')
    args = ap.parse_args()

    out_path = args.out or os.path.expanduser(
        '~/thesis_data/localization_test/test_e_localization_xy.png')
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    print(f"Reading Mode A: {args.bag_a}")
    trace_a = extract_mode_a(args.bag_a)
    print(f"Reading Mode B: {args.bag_b}")
    trace_b = extract_mode_bc(args.bag_b)
    print(f"Reading Mode C: {args.bag_c}")
    trace_c = extract_mode_bc(args.bag_c)

    plot(trace_a, trace_b, trace_c, out_path,
         title='Test E: Perbandingan Posisi XY 3 Mode Lokalisasi',
         map_yaml=args.map)


if __name__ == '__main__':
    main()
