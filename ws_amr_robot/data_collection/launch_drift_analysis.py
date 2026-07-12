#!/usr/bin/env python3
"""
launch_drift_analysis.py
=============================================================
TASK 2: quantify the LAUNCH asymmetry (start-of-motion drift).

Steady-state wheels match within ~1%, but at the very start (0 -> target)
the stronger right motor can surge before the left integrator catches up,
nudging the robot's heading. This script measures that, it does NOT fix it.
We measure first, then decide if a fix is even needed.

For each trial it commands a step 0 -> target and records both wheels at the
full telemetry rate (every frame, firmware 'dt' timed, same clean method as
the fixed step script). Over the LAUNCH WINDOW (first --window seconds) it
computes, per trial:
  - left rise time, right rise time   (0 -> 90% of target)
  - peak |right - left| velocity difference during launch  (mm/s and % of target)
  - settle time: first instant the wheels stay within 5% of each other

Runs --reps trials and reports mean +/- std. Also saves a per-frame CSV and a
launch overlay plot.

DECISION RULE (from the agreed plan): if peak diff < 15% of target AND settle
time < 0.5 s, launch drift is acceptable -> document it, SKIP the feedforward.

USAGE (ROS killed, robot on the ground with clear space):
  python3 launch_drift_analysis.py --target 150 --reps 5 --kp 8 --ki 5 --kd 0 --set-gains
=============================================================
"""

import os
import sys
import time
import argparse

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import amr_test_utils as U

# reuse the proven serial helpers from the step script
from collect_pid_step_response import (
    SERIAL_PORT, BAUD_RATE, MM_PER_TICK, MAX_PHYSICAL_SPEED,
    connect_serial, send_velocity, drain_and_get_latest,
    read_all_packets, tick_delta, set_gains_live, SpeedTracker,
)

SETTLE_BEFORE = 0.7   # s at zero before the step (let things settle)


def record_launch(ser, target_mmps, window_s):
    """Step 0 -> target, record per-frame wheel speeds for window_s seconds.
    Returns list of (t, left, right)."""
    # settle at zero and flush
    t0 = time.time()
    while (time.time() - t0) < SETTLE_BEFORE:
        send_velocity(ser, 0)
        time.sleep(0.05)
    ser.reset_input_buffer()

    # baseline frame
    first = None
    deadline = time.time() + 1.0
    while time.time() < deadline:
        send_velocity(ser, 0)
        d = drain_and_get_latest(ser)
        if d is not None and 'l' in d and 'r' in d:
            first = d
            break
        time.sleep(0.05)
    if first is None:
        print("  ERROR: no telemetry.")
        return []

    samples = []
    tracker = SpeedTracker(window_s=0.10)   # short window: keep launch transient
    tracker.update(first, 0.0)
    t_start = time.time()
    while (time.time() - t_start) < window_s:
        send_velocity(ser, target_mmps)
        time.sleep(0.01)
        for data in read_all_packets(ser):
            if 'l' not in data or 'r' not in data:
                continue
            now = time.time() - t_start
            spd = tracker.update(data, now)
            if spd is None:
                continue
            samples.append((round(now, 4), round(spd[0], 2), round(spd[1], 2)))

    # stop
    for _ in range(10):
        send_velocity(ser, 0)
        time.sleep(0.03)
    return samples


def rise_time(times, speeds, target):
    """First time speed reaches 90% of target (simple smoothing)."""
    thr = abs(target) * 0.9
    for t, s in zip(times, speeds):
        if abs(s) >= thr:
            return t
    return None


def analyze_launch(samples, target):
    """Return per-trial launch metrics."""
    if len(samples) < 4:
        return None
    times = [s[0] for s in samples]
    lefts = [s[1] for s in samples]
    rights = [s[2] for s in samples]

    rt_l = rise_time(times, lefts, target)
    rt_r = rise_time(times, rights, target)

    # peak |right - left| during launch
    diffs = [abs(r - l) for l, r in zip(lefts, rights)]
    peak_diff = max(diffs)
    peak_diff_pct = peak_diff / abs(target) * 100

    # settle time: first t after which |R-L| stays within 5% of target
    tol = abs(target) * 0.05
    settle = None
    for i in range(len(diffs)):
        if all(d <= tol for d in diffs[i:]):
            settle = times[i]
            break

    return {
        'rise_left_s':    rt_l,
        'rise_right_s':   rt_r,
        'peak_diff_mmps': round(peak_diff, 1),
        'peak_diff_pct':  round(peak_diff_pct, 1),
        'settle_s':       settle,
    }


def main():
    p = argparse.ArgumentParser(description='Task 2: quantify launch drift')
    p.add_argument('--target', type=int, default=150, help='step target mm/s')
    p.add_argument('--reps', type=int, default=5)
    p.add_argument('--window', type=float, default=1.0, help='launch window seconds')
    p.add_argument('--kp', type=float, default=8.0)
    p.add_argument('--ki', type=float, default=5.0)
    p.add_argument('--kd', type=float, default=0.0)
    p.add_argument('--set-gains', action='store_true')
    p.add_argument('--label', default=None)
    args = p.parse_args()

    print("=" * 60)
    print(f"LAUNCH DRIFT ANALYSIS  target={args.target} mm/s  reps={args.reps}")
    print(f"window={args.window}s  gains Kp={args.kp} Ki={args.ki} Kd={args.kd}")
    print("SAFETY: robot on ground, clear space, ROS killed.")
    print("=" * 60)
    input("Press ENTER to start, Ctrl+C to cancel...")

    ser = connect_serial()
    if args.set_gains:
        if not set_gains_live(ser, args.kp, args.ki, args.kd):
            print("Gains not confirmed; aborting.")
            ser.close()
            return

    all_rows = []      # per-frame, for CSV
    metrics = []       # per-trial
    try:
        for rep in range(1, args.reps + 1):
            print(f"\n--- launch {rep}/{args.reps} ---")
            samples = record_launch(ser, args.target, args.window)
            if not samples:
                print("  no data, skipping")
                continue
            for (t, l, r) in samples:
                all_rows.append({'rep': rep, 'time_s': t,
                                 'left_speed_mmps': l, 'right_speed_mmps': r})
            m = analyze_launch(samples, args.target)
            if m:
                metrics.append(m)
                print(f"  rise L={m['rise_left_s']} R={m['rise_right_s']} s | "
                      f"peak diff={m['peak_diff_mmps']}mm/s ({m['peak_diff_pct']}%) | "
                      f"settle={m['settle_s']}s")
            time.sleep(0.8)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        for _ in range(10):
            send_velocity(ser, 0)
            time.sleep(0.03)
        ser.close()
        print("Serial closed, motors stopped.")

    if not metrics:
        print("No metrics collected.")
        return

    # save per-frame CSV
    raw_path = U.timestamped_path('launch_drift_raw', label=args.label)
    U.save_csv(raw_path, ['rep', 'time_s', 'left_speed_mmps', 'right_speed_mmps'], all_rows)

    # summary
    print("\n" + "=" * 60)
    print("LAUNCH DRIFT SUMMARY (mean/std over reps)")
    print("=" * 60)
    U.print_summary_table('rise_left_s   ', U.summarize([m['rise_left_s'] for m in metrics]), ' s')
    U.print_summary_table('rise_right_s  ', U.summarize([m['rise_right_s'] for m in metrics]), ' s')
    U.print_summary_table('peak_diff_mmps', U.summarize([m['peak_diff_mmps'] for m in metrics]), ' mm/s')
    U.print_summary_table('peak_diff_pct ', U.summarize([m['peak_diff_pct'] for m in metrics]), ' %')
    U.print_summary_table('settle_s      ', U.summarize([m['settle_s'] for m in metrics]), ' s')

    pk = U.summarize([m['peak_diff_pct'] for m in metrics])['mean']
    st = U.summarize([m['settle_s'] for m in metrics])['mean']
    print("\nDECISION (peak<15% AND settle<0.5s -> acceptable, skip feedforward):")
    import math
    ok = (not math.isnan(pk) and pk < 15.0) and (not math.isnan(st) and st < 0.5)
    if ok:
        print(f"  peak={pk:.1f}%  settle={st:.2f}s  -> ACCEPTABLE. Document it, no fix needed.")
    else:
        print(f"  peak={pk:.1f}%  settle={st:.2f}s  -> SIGNIFICANT. Consider feedforward (Task 3).")

    print(f"\nPer-frame CSV: {raw_path}")


if __name__ == '__main__':
    main()
