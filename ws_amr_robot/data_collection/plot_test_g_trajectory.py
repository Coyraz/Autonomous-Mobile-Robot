#!/usr/bin/env python3
"""
plot_test_g_trajectory.py  --  Planned path vs actual position overlay (Test G)
--------------------------------------------------------------------------------
Reads a Test G rosbag (recorded via rosbag_ground_truth.py --test navigation)
and plots, in one map-frame XY figure:

  - Planned path       every /plan message (one per goal), thin muted line
  - AMCL actual        /amcl_pose, continuous trace over the whole session
  - EKF fused actual   map->odom (AMCL) composed with odom->base_link (EKF),
                       continuous trace -- same TF composition as
                       analyze_navigation_bag.py's cur_pose()
  - Tape ground truth  gt_x/gt_y carried in each nav_goal_sent event (the
                       REAL, meter-measured target coordinate, not the Nav2
                       goal that was actually sent) -- black markers + labels

Usage:
  python3 plot_test_g_trajectory.py ~/thesis_data/rosbags/test_g_v4/
  python3 plot_test_g_trajectory.py ~/thesis_data/rosbags/test_g_v4/ --out test_g_trajectory.png
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

# True ground-truth reference points -- the warehouse's designed/tape-marked
# layout, NOT an AMCL reading. Copied from the active TITIK_KOORDINAT dict in
# robot_bringup/scripts/pengujian/collect_localization_accuracy.py (2026-07-14:
# confirmed with the user this is the real one, NOT WAREHOUSE_WAYPOINTS in
# amr_test_utils.py -- that one holds AMCL's OWN readings taken while parked
# at each tape mark, chosen historically so Nav2 could plan to them, not a
# tape measurement itself). Format: 'Name': (x_m, y_m, yaw_rad).
TITIK_KOORDINAT = {
    'Home':  ( 0.0,  0.0,  0.0),
    'Stage': ( 3.5,  0.5,  1.5),
    'A1':    ( 4.0, -8.5, -1.81),
    'A2':    ( 4.0, -7.0, -1.37),
    'A3':    ( 4.0, -4.5, -1.33),
    'A4':    ( 4.0, -3.0, -1.36),
    'B1':    ( 1.5, -8.5, -1.15),
    'B2':    ( 1.5, -7.0, -1.69),
    'B3':    ( 1.5, -4.5,  1.59),
    'B4':    ( 1.5, -3.0,  1.46),
    'C1':    (-1.0, -8.5, -1.33),
    'C2':    (-1.0, -7.0, -1.51),
    'C3':    (-1.0, -4.5, -1.53),
    'C4':    (-1.0, -3.0, -1.37),
}

# dataviz skill palette (references/palette.md) -- categorical slots 1 (blue)
# and 6 (red), well-separated (worst adjacent CVD delta-E 24.2 for the full
# 8-slot ordering). Planned path uses a muted neutral (background reference
# layer, not a series being compared).
COLOR_PLANNED = '#9a9890'   # muted neutral -- de-emphasized background layer
COLOR_AMCL    = '#2a78d6'   # categorical slot 1 (blue)
COLOR_EKF     = '#e34948'   # categorical slot 6 (red)
COLOR_GT      = '#0b0b0b'   # tape ground truth markers -- primary ink, not a series
COLOR_GOAL    = '#eb6834'   # categorical slot 8 (orange) -- Nav2/ils_gui goal markers


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
    """Compose map->odom (AMCL's correction) and odom->base_link (EKF) into
    map->base_link (x, y) -- identical to analyze_navigation_bag.py."""
    mx, my, myaw = map_odom
    ox, oy, oyaw = odom_base
    c, s = math.cos(myaw), math.sin(myaw)
    x = mx + c * ox - s * oy
    y = my + s * ox + c * oy
    return (x, y)


def extract_one(bag_path):
    latest_map_odom = None

    amcl_trace = []   # (x, y)
    ekf_trace = []     # (x, y)
    planned_paths = []  # list of [(x,y), ...] -- one per /plan message

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
                        ekf_trace.append(compose_map_base(latest_map_odom, (tr.x, tr.y, yaw)))

        elif topic == '/amcl_pose':
            p = msg.pose.pose.position
            amcl_trace.append((p.x, p.y))

        elif topic == '/plan':
            pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            if pts:
                planned_paths.append(pts)

    return amcl_trace, ekf_trace, planned_paths


def extract(bag_paths):
    """bag_paths: list of bag directories, read in order and concatenated
    (session was interrupted/restarted across multiple bags -- see test9.md
    provenance: test_g_v3 + test_g_v3_finish, test_g_v2 discarded)."""
    amcl_trace, ekf_trace, planned_paths = [], [], []
    for bag_path in bag_paths:
        a, e, p = extract_one(bag_path)
        amcl_trace += a
        ekf_trace += e
        planned_paths += p
    return amcl_trace, ekf_trace, planned_paths


def plot(amcl_trace, ekf_trace, planned_paths, out_path, title, map_yaml=None):
    gt_points = {name: (x, y) for name, (x, y, yaw) in TITIK_KOORDINAT.items()}
    fig, ax = plt.subplots(figsize=(10, 12), dpi=150)
    fig.patch.set_facecolor('#fcfcfb')
    ax.set_facecolor('#fcfcfb')

    map_extent = None
    if map_yaml:
        map_img, map_extent = U.load_map_image(map_yaml)
        ax.imshow(map_img, extent=map_extent, origin='upper', cmap='gray', zorder=0)

    # planned paths -- background layer, thin, muted, one legend entry only
    for i, pts in enumerate(planned_paths):
        xs, ys = zip(*pts)
        ax.plot(xs, ys, color=COLOR_PLANNED, linewidth=1.2, alpha=0.8,
                 zorder=1, label='Rencana jalur (/plan)' if i == 0 else None)

    # actual traces
    if amcl_trace:
        xs, ys = zip(*amcl_trace)
        ax.plot(xs, ys, color=COLOR_AMCL, linewidth=1.6, alpha=0.9,
                 zorder=2, label='Posisi aktual - AMCL')
    if ekf_trace:
        xs, ys = zip(*ekf_trace)
        ax.plot(xs, ys, color=COLOR_EKF, linewidth=1.6, alpha=0.9,
                 zorder=3, label='Posisi aktual - EKF fused')

    # tape ground-truth reference points, labeled
    for target, (gx, gy) in gt_points.items():
        ax.scatter([gx], [gy], marker='*', s=180, color=COLOR_GT,
                    edgecolors='white', linewidths=0.8, zorder=4)
        ax.annotate(target, (gx, gy), textcoords='offset points',
                     xytext=(6, 4), fontsize=9, color='#0b0b0b', zorder=5)

    ax.scatter([], [], marker='*', s=180, color=COLOR_GT, edgecolors='white',
                linewidths=0.8, label='Titik referensi meteran (ground truth)')

    # Nav2/ils_gui goal points (WAREHOUSE_WAYPOINTS -- AMCL reading at each
    # tape mark, historically re-centered for plannability, see
    # amr_test_utils.py comments). Separate marker so goal-definition error
    # (this vs the tape star) is visually distinguishable from execution
    # error (actual trace vs this marker).
    nav_goals = U.WAREHOUSE_WAYPOINTS
    for target, (gx, gy) in nav_goals.items():
        if target not in gt_points:
            continue  # only plot goals that also have a tape ground-truth pair
        ax.scatter([gx], [gy], marker='D', s=55, color=COLOR_GOAL,
                    edgecolors='white', linewidths=0.6, zorder=4)

    ax.scatter([], [], marker='D', s=55, color=COLOR_GOAL, edgecolors='white',
                linewidths=0.6, label='Titik goal Nav2 (ils_gui / WAREHOUSE_WAYPOINTS)')

    ax.set_xlabel('X (m)', color='#0b0b0b')
    ax.set_ylabel('Y (m)', color='#0b0b0b')
    ax.set_title(title, color='#0b0b0b', fontsize=13)
    ax.set_aspect('equal')
    if map_extent:
        # show the WHOLE map, not auto-zoomed to the trajectory data extent
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
    print(f"  planned paths: {len(planned_paths)}")
    print(f"  AMCL samples: {len(amcl_trace)}")
    print(f"  EKF fused samples: {len(ekf_trace)}")
    print(f"  ground-truth reference points: {len(gt_points)}")


def main():
    ap = argparse.ArgumentParser(description='Plot Test G planned-vs-actual XY trajectory')
    ap.add_argument('bag_paths', nargs='+', help='Path(s) to the test_g rosbag directory(ies), read in order')
    ap.add_argument('--map', default=None, help='Path to map yaml, drawn as background (full extent, not zoomed)')
    ap.add_argument('--out', default=None, help='Output PNG path')
    args = ap.parse_args()

    out_path = args.out or os.path.expanduser(
        '~/thesis_data/navigation_test/test_g_trajectory.png')
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    amcl_trace, ekf_trace, planned_paths = extract(args.bag_paths)
    plot(amcl_trace, ekf_trace, planned_paths, out_path,
         title='Test G: Rencana Jalur vs Posisi Aktual (AMCL & EKF Fused)',
         map_yaml=args.map)


if __name__ == '__main__':
    main()
