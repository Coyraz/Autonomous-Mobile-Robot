#!/usr/bin/env python3
"""
plot_test_f_trajectory.py  --  Drift-scenario XY overlay (Test F)
--------------------------------------------------------------------------
Reads the Test F rosbag (mode C: EKF + AMCL, 5 scenarios x 3 reps, one
continuous session -- see test8.md) and plots two traces over the WHOLE
session in one map-frame XY figure:

  RAW    /odom_raw directly (encoder dead-reckoning, own "odom" frame --
         coincides with map at t=0/Home, then drifts apart continuously
         across all 15 reps since the node is never reset mid-bag; this is
         the trace expected to visibly wander over the session)
  EKF    TF map->odom (AMCL) composed with odom->base_link (EKF) -- same
         approach as Test E Mode C / Test G

NOTE (2026-07-14): unlike test8.md's own record command, this bag does NOT
actually contain /amcl_pose, /imu/data_raw, or /cmd_vel (confirmed via
`ros2 bag info` -- only /odom, /odom_raw, /tf, /tf_static, /ground_truth_
event were recorded). A separate "AMCL-only" trace is therefore NOT
reconstructable here: the only odom->base_link TF present comes from EKF
(ekf.yaml's publish_tf: true), so composing map->odom with it yields EKF's
pose, not a pure-AMCL-plus-raw-encoder one. Only RAW vs EKF is possible.

Reference points: Home (start/end of every scenario) + B4, C4, C1 (corners
of the return_origin rectangle: Home->B4->C4->C1->Home).

Usage:
  python3 plot_test_f_trajectory.py ~/thesis_data/rosbags/test_f/test_f/
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

# Reference points relevant to Test F's scenarios (Home start/end of every
# scenario; B4/C4/C1 are the return_origin rectangle's corners). Same values
# as TITIK_KOORDINAT (confirmed 2026-07-14 as the real tape ground truth).
REFERENCE_POINTS = {
    'Home': (0.0, 0.0),
    'B4':   (1.5, -3.0),
    'C4':  (-1.0, -3.0),
    'C1':  (-1.0, -8.5),
}

# dataviz skill palette -- same role assignment as Test E for cross-plot
# consistency (green=weakest/raw, blue=AMCL, red=EKF fused).
COLOR_RAW  = '#008300'   # slot 4 green
COLOR_AMCL = '#2a78d6'   # slot 1 blue
COLOR_EKF  = '#e34948'   # slot 6 red
COLOR_GT   = '#0b0b0b'


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


def extract(bag_path):
    latest_map_odom = None
    raw_trace, ekf_trace = [], []

    for topic, msg, ts_ns in read_bag(bag_path):
        if topic == '/odom_raw':
            p = msg.pose.pose.position
            raw_trace.append((p.x, p.y))

        elif topic == '/tf':
            for tf in msg.transforms:
                fid, cid = tf.header.frame_id, tf.child_frame_id
                tr, q = tf.transform.translation, tf.transform.rotation
                yaw = U.yaw_from_quaternion(q.x, q.y, q.z, q.w)
                if (fid, cid) == ('map', 'odom'):
                    latest_map_odom = (tr.x, tr.y, yaw)
                elif (fid, cid) == ('odom', 'base_link'):
                    if latest_map_odom is not None:
                        ekf_trace.append(compose_map_base(latest_map_odom, (tr.x, tr.y, yaw)))

    return raw_trace, ekf_trace


def plot(raw_trace, ekf_trace, out_path, title, map_yaml=None):
    fig, ax = plt.subplots(figsize=(9, 9), dpi=150)
    fig.patch.set_facecolor('#fcfcfb')
    ax.set_facecolor('#fcfcfb')

    map_extent = None
    if map_yaml:
        map_img, map_extent = U.load_map_image(map_yaml)
        ax.imshow(map_img, extent=map_extent, origin='upper', cmap='gray', zorder=0)

    if raw_trace:
        xs, ys = zip(*raw_trace)
        ax.plot(xs, ys, color=COLOR_RAW, linewidth=1.2, alpha=0.85,
                 zorder=2, label='RAW - Encoder only (/odom_raw)')
    if ekf_trace:
        xs, ys = zip(*ekf_trace)
        ax.plot(xs, ys, color=COLOR_EKF, linewidth=1.4, alpha=0.85,
                 zorder=4, label='EKF fused (map TF)')

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
    print(f"  RAW samples: {len(raw_trace)}")
    print(f"  EKF fused samples: {len(ekf_trace)}")


def main():
    ap = argparse.ArgumentParser(description='Plot Test F drift-scenario XY overlay')
    ap.add_argument('bag_path', help='Path to the test_f bag directory')
    ap.add_argument('--map', default=None, help='Path to map yaml, drawn as background (full extent, not zoomed)')
    ap.add_argument('--out', default=None, help='Output PNG path')
    args = ap.parse_args()

    out_path = args.out or os.path.expanduser(
        '~/thesis_data/trajectory_test/test_f_trajectory_xy.png')
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    raw_trace, ekf_trace = extract(args.bag_path)
    plot(raw_trace, ekf_trace, out_path,
         title='Test F: Drift RAW (encoder) vs EKF Fused (seluruh sesi)',
         map_yaml=args.map)


if __name__ == '__main__':
    main()
