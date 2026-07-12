#!/usr/bin/env python3
"""
analyze_obstacle_bag.py  --  Post-process rosbag for Test H obstacle stop-and-resume
--------------------------------------------------------------------------------------
Reads a bag recorded during `rosbag_ground_truth.py --test obstacle`.

custom_path_controller does NOT replan around obstacles -- it detects a
blocked arc via /global_costmap and holds (zero velocity) until the arc
clears, then resumes the SAME goal automatically. This script measures that
behavior objectively where possible:

  stopped_safely     operator-reported (y/n) -- did it stop before touching
                      the obstacle
  stop_distance_cm   operator's tape measurement, stop point -> obstacle
  resume_time_s       OBJECTIVE, computed from the bag: time from the
                      'obstacle_cleared' marker to the first /odom sample
                      with |linear.x| above a small moving threshold --
                      i.e. how long the robot actually took to notice the
                      arc was clear and resume, independent of the
                      operator's own reaction time watching for it.
  result             success / collision / stuck / timeout (operator-judged
                      final outcome, see rosbag_ground_truth.py's run_obstacle)

Usage:
  python3 analyze_obstacle_bag.py path/to/test_h/
  python3 analyze_obstacle_bag.py path/to/test_h/ --out-dir ~/thesis_data/obstacle_test
"""

import argparse
import csv
import json
import os
from datetime import datetime

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import amr_test_utils as U

MOVING_THRESHOLD_MPS = 0.02  # |linear.x| above this counts as "resumed"


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

    # pass 1: collect all /odom samples (t_s, linear.x) and all ground_truth events
    odom_samples = []   # (t_s, v_mps)
    events = []          # (t_s, dict)
    mode = 'C'

    for topic, msg, ts_ns in read_bag(bag_path):
        t_s = ts_ns * 1e-9
        if topic == '/odom':
            odom_samples.append((t_s, msg.twist.twist.linear.x))
        elif topic == '/ground_truth_event':
            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                continue
            data['_t_s'] = t_s
            mode = data.get('mode', mode)
            events.append(data)

    if not events:
        print("ERROR: no ground_truth_event markers found. Did you run "
              "rosbag_ground_truth.py --test obstacle?")
        return

    odom_samples.sort(key=lambda p: p[0])

    def first_resume_after(t_clear):
        """First /odom timestamp after t_clear where the robot is moving."""
        for t_s, v in odom_samples:
            if t_s > t_clear and abs(v) > MOVING_THRESHOLD_MPS:
                return t_s
        return None

    # pass 2: pair up events per (rep, target) -- obstacle_nav_start ->
    # obstacle_stop -> [obstacle_cleared] -> obstacle_nav_result
    records = []
    open_rep = None   # dict accumulating fields for the current rep

    for ev in events:
        etype = ev.get('event')

        if etype == 'obstacle_nav_start':
            open_rep = {
                'rep': ev.get('rep'), 'target': ev.get('target'),
                'obstacle': ev.get('obstacle'),
                't_start': ev['_t_s'],
            }

        elif etype == 'obstacle_stop' and open_rep is not None:
            open_rep['stopped_safely'] = ev.get('stopped_safely')
            open_rep['stop_distance_cm'] = ev.get('stop_distance_cm')
            open_rep['t_stop'] = ev['_t_s']

        elif etype == 'obstacle_cleared' and open_rep is not None:
            open_rep['t_cleared'] = ev['_t_s']

        elif etype == 'obstacle_nav_result' and open_rep is not None:
            open_rep['result'] = ev.get('result')
            open_rep['resumed'] = ev.get('resumed')

            resume_time_s = None
            if open_rep.get('t_cleared') is not None:
                t_resume = first_resume_after(open_rep['t_cleared'])
                if t_resume is not None:
                    resume_time_s = round(t_resume - open_rep['t_cleared'], 2)

            records.append({
                'mode': mode,
                'rep': open_rep.get('rep'),
                'target': open_rep.get('target'),
                'obstacle': open_rep.get('obstacle'),
                'stopped_safely': open_rep.get('stopped_safely'),
                'stop_distance_cm': open_rep.get('stop_distance_cm'),
                'resumed': open_rep.get('resumed'),
                'resume_time_s': resume_time_s,
                'result': open_rep.get('result'),
            })
            open_rep = None

    if not records:
        print("ERROR: events found but couldn't pair them into complete reps.")
        return

    # ---- summary ----
    n = len(records)
    n_stopped_safely = sum(1 for r in records if r['stopped_safely'])
    n_success = sum(1 for r in records if r['result'] == 'success')
    n_collision = sum(1 for r in records if r['result'] == 'collision')
    n_stuck = sum(1 for r in records if r['result'] == 'stuck')
    n_timeout = sum(1 for r in records if r['result'] == 'timeout')

    print(f"\n{'='*64}")
    print(f"  TEST H  OBSTACLE STOP-AND-RESUME  (mode {mode})   reps={n}")
    print(f"{'='*64}")
    print(f"  Stopped safely     : {n_stopped_safely}/{n} = {100.0*n_stopped_safely/n:.1f}%")
    print(f"  Collisions         : {n_collision}/{n}")
    print(f"  Stuck (never resumed after clear): {n_stuck}/{n}")
    print(f"  Timeout (resumed but didn't arrive): {n_timeout}/{n}")
    print(f"  Full success (stop+resume+arrive) : {n_success}/{n} = {100.0*n_success/n:.1f}%")

    stop_dists = [r['stop_distance_cm'] for r in records
                  if r['stopped_safely'] and r['stop_distance_cm'] is not None]
    if stop_dists:
        print(f"  Stop distance (safe stops): mean {sum(stop_dists)/len(stop_dists):.1f} cm  "
              f"min {min(stop_dists):.1f} cm  max {max(stop_dists):.1f} cm")

    resume_times = [r['resume_time_s'] for r in records if r['resume_time_s'] is not None]
    if resume_times:
        print(f"  Resume time (objective, from bag): mean {sum(resume_times)/len(resume_times):.2f} s  "
              f"max {max(resume_times):.2f} s")

    print(f"\n  {'target':10} {'rep':>3} {'result':>10} {'stop_cm':>8} {'resume_s':>9}")
    for r in records:
        sd = f"{r['stop_distance_cm']:.1f}" if r['stop_distance_cm'] is not None else "n/a"
        rt = f"{r['resume_time_s']:.2f}" if r['resume_time_s'] is not None else "n/a"
        print(f"  {r['target']:10} {r['rep']:>3} {str(r['result']):>10} {sd:>8} {rt:>9}")

    # ---- CSV ----
    os.makedirs(out_dir, exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(out_dir, f'obstacle_mode_{mode}_{ts}.csv')
    fields = ['mode', 'rep', 'target', 'obstacle', 'stopped_safely',
              'stop_distance_cm', 'resumed', 'resume_time_s', 'result']
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        w.writeheader()
        w.writerows(records)
    print(f"\nCSV saved: {csv_path}")


def main():
    ap = argparse.ArgumentParser(description='Analyze Test H obstacle stop-and-resume rosbag')
    ap.add_argument('bag_path', help='Path to the test_h bag directory')
    ap.add_argument('--out-dir', default=None,
                    help='Output dir (default ~/thesis_data/obstacle_test)')
    args = ap.parse_args()
    out_dir = args.out_dir or os.path.expanduser('~/thesis_data/obstacle_test')
    analyze_bag(args.bag_path, out_dir)


if __name__ == '__main__':
    main()
