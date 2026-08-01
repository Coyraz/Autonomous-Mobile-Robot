#!/usr/bin/env python3
"""
plot_test_h_obstacle.py  --  Deceleration profile overlay (Test H)
--------------------------------------------------------------------------
Reads a Test H rosbag (recorded via rosbag_ground_truth.py --test obstacle)
and plots, per rep (aligned to that rep's own obstacle_nav_start at t=0),
two stacked single-axis panels sharing a time axis:

  Top     robot velocity (/odom.twist.twist.linear.x, EKF fused), m/s
  Bottom  distance to the nearest return inside a forward LiDAR cone
          (/scan_restamped), cm -- same forward-cone convention as
          test_h_detection_probe.py: |angle| <= cone_deg from angle 0

A vertical dashed line marks the operator's 'obstacle_stop' event per rep
(when they judged the robot had stopped), annotated with the operator's own
tape-measured stop_distance_cm -- letting the LiDAR-derived distance trace
and the tape measurement be cross-checked visually.

Usage:
  python3 plot_test_h_obstacle.py ~/thesis_data/rosbags/test_h_gradual \\
      --title "Test H: Profil Deselerasi -- Skenario Gradual" \\
      --out ~/thesis_data/obstacle_test/test_h_gradual_profile.png
"""

import argparse
import json
import math
import os

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

REP_COLORS = ['#2a78d6', '#e34948', '#008300', '#eb6834', '#7c4dbd']
CONE_DEG = 15.0
MOVING_THRESHOLD_MPS = 0.02

# The LiDAR is mounted rotated 180deg from base_link (see hardware.launch.py's
# static_transform_publisher: yaw=3.14159 for base_link->laser_frame), so the
# robot's forward direction is at angle +/-pi in the scan, NOT angle 0.


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


def forward_min_range(msg, cone_rad):
    """Nearest valid return within +/-cone_rad of the robot's forward
    direction, which sits at scan angle +/-pi (see module docstring note
    on the 180deg-rotated LiDAR mount)."""
    best = None
    for i, r in enumerate(msg.ranges):
        ang = msg.angle_min + i * msg.angle_increment
        ang_from_front = min(abs(ang - math.pi), abs(ang + math.pi))
        if ang_from_front > cone_rad:
            continue
        if r < msg.range_min or r > msg.range_max:
            continue
        if math.isnan(r) or math.isinf(r):
            continue
        if best is None or r < best:
            best = r
    return best


def extract(bag_path):
    odom_samples = []   # (t_s, v_mps)
    scan_samples = []   # (t_s, dist_cm)
    events = []          # dict, with '_t_s'
    cone_rad = math.radians(CONE_DEG)

    for topic, msg, ts_ns in read_bag(bag_path):
        t_s = ts_ns * 1e-9
        if topic == '/odom':
            odom_samples.append((t_s, msg.twist.twist.linear.x))
        elif topic == '/scan_restamped':
            d = forward_min_range(msg, cone_rad)
            if d is not None:
                scan_samples.append((t_s, d * 100.0))
        elif topic == '/ground_truth_event':
            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                continue
            data['_t_s'] = t_s
            events.append(data)

    return odom_samples, scan_samples, events


def group_reps(events):
    """Return {rep: {'start': t, 'stop': t, 'stop_distance_cm': v, 'cleared': t,
    'result': t}} using obstacle_nav_start/obstacle_stop/obstacle_cleared/
    obstacle_nav_result markers."""
    reps = {}
    for e in events:
        rep = e.get('rep')
        if rep is None:
            continue
        r = reps.setdefault(rep, {})
        ev = e['event']
        if ev == 'obstacle_nav_start':
            r['start'] = e['_t_s']
        elif ev == 'obstacle_stop':
            r['stop'] = e['_t_s']
            r['stop_distance_cm'] = e.get('stop_distance_cm')
        elif ev == 'obstacle_cleared':
            r['cleared'] = e['_t_s']
        elif ev == 'obstacle_nav_result':
            r['result_t'] = e['_t_s']
    return reps


def detect_motion_window(odom_samples, t0, t_search_end):
    """Auto-detect when the robot actually starts moving and when it
    actually, physically stops (distinct from the operator's 'obstacle_stop'
    marker, which lags the real stop by however long the operator took to
    notice and press Enter -- often tens of seconds, see test10.md).
    Returns (t_move_start, t_actual_stop) or (None, None) if not found."""
    win = [(t, v) for t, v in odom_samples if t0 <= t <= t_search_end]
    t_move_start = None
    for i, (t, v) in enumerate(win):
        if abs(v) > MOVING_THRESHOLD_MPS:
            # require it to stay above threshold for a few consecutive samples
            # (reject single-sample noise)
            if all(abs(v2) > MOVING_THRESHOLD_MPS for _, v2 in win[i:i + 3]):
                t_move_start = t
                break
    if t_move_start is None:
        return None, None

    t_actual_stop = None
    for i, (t, v) in enumerate(win):
        if t <= t_move_start:
            continue
        if abs(v) <= MOVING_THRESHOLD_MPS:
            # require it to STAY stopped for >=1.5s (reject a momentary dip)
            still = [v2 for t2, v2 in win[i:] if t2 <= t + 1.5]
            if len(still) >= 3 and all(abs(v2) <= MOVING_THRESHOLD_MPS for v2 in still):
                t_actual_stop = t
                break
    return t_move_start, t_actual_stop


STOPPED_CAPTURE_S = 20.0  # how long we'd LIKE to show the robot sitting stopped


def detect_resume(odom_samples, t_actual_stop, t_search_end):
    """First time after t_actual_stop the robot starts moving again (obstacle
    cleared, resumed toward the goal). Returns None if it's still stopped by
    t_search_end (i.e. stopped for the full STOPPED_CAPTURE_S window)."""
    win = [(t, v) for t, v in odom_samples if t_actual_stop < t <= t_search_end]
    for i, (t, v) in enumerate(win):
        if abs(v) > MOVING_THRESHOLD_MPS:
            if all(abs(v2) > MOVING_THRESHOLD_MPS for _, v2 in win[i:i + 3]):
                return t
    return None


def plot(odom_samples, scan_samples, reps, out_path, title):
    fig, (ax_v, ax_d) = plt.subplots(2, 1, figsize=(11, 8), dpi=150, sharex=True)
    for ax in (ax_v, ax_d):
        ax.set_facecolor('#fcfcfb')
        ax.grid(True, color='#e3e2dc', linewidth=0.6, zorder=0)
        for spine in ax.spines.values():
            spine.set_color('#c3c2b7')
        ax.tick_params(colors='#52514e')
    fig.patch.set_facecolor('#fcfcfb')

    rep_ids = sorted(reps.keys())
    for i, rep in enumerate(rep_ids):
        r = reps[rep]
        if 'start' not in r:
            continue
        t0 = r['start']
        # +3s buffer past the operator's 'stop' marker so the "stayed
        # stopped for 1.5s" check has room to confirm even when the actual
        # physical stop landed close to (or the session was interrupted
        # shortly after) that marker -- see Test H sudden rep 4, cut short
        # by a mid-session interruption right after its stop event.
        search_end = r.get('stop', t0 + 180) + 3.0
        t_move_start, t_actual_stop = detect_motion_window(odom_samples, t0, search_end)
        if t_move_start is None or t_actual_stop is None:
            continue  # never moved, or never settled -- skip (shouldn't happen)

        # end of window: capture up to STOPPED_CAPTURE_S seconds of the robot
        # sitting stopped, but cut short at resume if the obstacle was
        # cleared and the robot moved on again before that (per user request:
        # shorter is fine, just don't show stale/irrelevant post-resume data)
        t_resume = detect_resume(odom_samples, t_actual_stop, t_actual_stop + STOPPED_CAPTURE_S)
        t_win_end_abs = t_resume if t_resume is not None else t_actual_stop + STOPPED_CAPTURE_S

        # x-axis aligned so t=0 is the actual physical stop for EVERY rep,
        # so the drop-to-stop lines up across reps regardless of approach length
        t_win_end_rel = t_win_end_abs - t_actual_stop
        color = REP_COLORS[i % len(REP_COLORS)]

        v_win = [(t - t_actual_stop, v) for t, v in odom_samples
                 if t_move_start - 2.0 <= t <= t_win_end_abs]
        d_win = [(t - t_actual_stop, d) for t, d in scan_samples
                 if t_move_start - 2.0 <= t <= t_win_end_abs]

        if v_win:
            xs, ys = zip(*v_win)
            ax_v.plot(xs, ys, color=color, linewidth=1.3, alpha=0.9,
                       label=f'Rep {rep} (berhenti {t_win_end_rel:.0f}s)')
        if d_win:
            xs, ys = zip(*d_win)
            ax_d.plot(xs, ys, color=color, linewidth=1.3, alpha=0.9,
                       label=f'Rep {rep}')

        ax_v.axvline(0, color=color, linestyle='--', linewidth=0.9, alpha=0.6)
        ax_d.axvline(0, color=color, linestyle='--', linewidth=0.9, alpha=0.6)
        tape_cm = r.get('stop_distance_cm')
        if tape_cm is not None:
            ax_d.annotate(f'{tape_cm:.0f}cm (meteran)', (0, tape_cm),
                           textcoords='offset points', xytext=(4, 8),
                           fontsize=8, color=color)

    ax_v.set_ylabel('Kecepatan linear (m/s)', color='#0b0b0b')
    ax_v.set_title(title, color='#0b0b0b', fontsize=13)
    ax_v.legend(loc='upper right', frameon=True, facecolor='#fcfcfb',
                edgecolor='#c3c2b7', fontsize=9, ncol=len(rep_ids))

    ax_d.set_ylabel('Jarak ke obstacle, kerucut depan LiDAR (cm)', color='#0b0b0b')
    ax_d.set_xlabel('Waktu relatif terhadap saat robot benar-benar berhenti bergerak, t=0 (s)\n'
                     '(garis putus-putus di t=0 = titik berhenti aktual, terdeteksi dari /odom; '
                     'kurva berlanjut sampai robot bergerak lagi atau maks 20s diam)',
                     color='#0b0b0b')

    fig.tight_layout()
    fig.savefig(out_path, facecolor=fig.get_facecolor())
    print(f"Saved: {out_path}")
    print(f"  reps found: {rep_ids}")
    print(f"  odom samples: {len(odom_samples)}, scan samples: {len(scan_samples)}")


def main():
    ap = argparse.ArgumentParser(description='Plot Test H deceleration profile (velocity + LiDAR distance vs time)')
    ap.add_argument('bag_paths', nargs='+',
                     help='Path(s) to test_h bag directory(ies), read in order. '
                          'A later bag\'s own rep numbering (which always restarts at 1) '
                          'is offset to continue after the previous bag\'s highest rep -- '
                          'for stitching together a session that was interrupted mid-run.')
    ap.add_argument('--title', default='Test H: Profil Deselerasi Saat Berhenti')
    ap.add_argument('--out', default=None, help='Output PNG path')
    args = ap.parse_args()

    out_path = args.out or os.path.expanduser(
        '~/thesis_data/obstacle_test/test_h_profile.png')
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    odom_samples, scan_samples, reps = [], [], {}
    rep_offset = 0
    for bag_path in args.bag_paths:
        o, s, events = extract(bag_path)
        odom_samples += o
        scan_samples += s
        bag_reps = group_reps(events)
        for rep, r in sorted(bag_reps.items()):
            reps[rep + rep_offset] = r
        if bag_reps:
            rep_offset += max(bag_reps.keys())

    # drop reps where the robot's actual stop can't be detected (e.g. an
    # EKF NaN crash landed right on top of this rep's stop -- /odom reports
    # NaN for a few seconds, so "stayed stopped" can never be confirmed; see
    # Test H sudden's original rep 4), THEN renumber sequentially (1, 2, 3,
    # ...) so a rep lost this way doesn't leave a gap in the legend
    valid_reps = {}
    for rep, r in reps.items():
        if 'start' not in r:
            continue
        search_end = r.get('stop', r['start'] + 180) + 3.0
        t_move_start, t_actual_stop = detect_motion_window(odom_samples, r['start'], search_end)
        if t_move_start is not None and t_actual_stop is not None:
            valid_reps[rep] = r
    reps = {i + 1: r for i, (_, r) in enumerate(sorted(valid_reps.items()))}

    plot(odom_samples, scan_samples, reps, out_path, args.title)


if __name__ == '__main__':
    main()
