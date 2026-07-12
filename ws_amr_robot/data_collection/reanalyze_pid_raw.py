#!/usr/bin/env python3
"""
reanalyze_pid_raw.py  --  Recompute Test D+ metrics from an existing raw CSV
------------------------------------------------------------------------------
Re-runs collect_pid_step_response.py's analyze() over a previously recorded
raw CSV (time_s, target_mmps, rep, left_speed_mmps, right_speed_mmps), without
driving the robot again. Use this after fixing a bug in analyze() (e.g. the
overshoot-window fix, 2026-07-06) to regenerate metrics for data you already
collected.

Usage:
  python3 reanalyze_pid_raw.py path/to/test6_pid_raw_*.csv
"""

import argparse
import csv
import os
from datetime import datetime

import collect_pid_step_response as pid


def load_raw(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append({
                'time_s':           float(r['time_s']),
                'target_mmps':      float(r['target_mmps']),
                'rep':              int(r['rep']),
                'left_speed_mmps':  float(r['left_speed_mmps']),
                'right_speed_mmps': float(r['right_speed_mmps']),
            })
    return rows


def main():
    ap = argparse.ArgumentParser(description='Recompute PID metrics from a raw CSV')
    ap.add_argument('raw_csv', help='Path to test6_pid_raw_*.csv')
    ap.add_argument('--out-dir', default=None,
                    help='Output dir (default: same dir as raw_csv)')
    args = ap.parse_args()

    rows = load_raw(args.raw_csv)
    targets = sorted(set(r['target_mmps'] for r in rows))
    reps = sorted(set(r['rep'] for r in rows))

    metrics_rows = []
    for target in targets:
        for rep in reps:
            samples = [r for r in rows if r['target_mmps'] == target and r['rep'] == rep]
            if not samples:
                continue
            for wheel in ('left', 'right'):
                m = pid.analyze(samples, target, wheel)
                if m is None:
                    continue
                metrics_rows.append({
                    'target_mmps': target,
                    'target_mps': target / 1000.0,
                    'rep': rep,
                    'wheel': m['wheel'],
                    'rise_time_s': m['rise_time_s'],
                    'overshoot_pct': m['overshoot_pct'],
                    'ss_speed_mmps': m['ss_speed_mmps'],
                    'ss_error_mmps': m['ss_error_mmps'],
                    'ss_error_pct': m['ss_error_pct'],
                    'ss_rmse_mmps': m['ss_rmse_mmps'],
                })

    out_dir = args.out_dir or os.path.dirname(os.path.abspath(args.raw_csv))
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    base = os.path.basename(args.raw_csv).replace('_raw_', '_metrics_').replace('.csv', '')
    out_path = os.path.join(out_dir, f'{base}_REANALYZED_{ts}.csv')

    fields = ['target_mmps', 'target_mps', 'rep', 'wheel', 'rise_time_s',
              'overshoot_pct', 'ss_speed_mmps', 'ss_error_mmps',
              'ss_error_pct', 'ss_rmse_mmps']
    with open(out_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(metrics_rows)

    print(f"Re-analyzed metrics saved: {out_path}\n")
    for target in targets:
        rows_t = [r for r in metrics_rows if r['target_mmps'] == target]
        oshoot = [r['overshoot_pct'] for r in rows_t]
        sse = [r['ss_error_pct'] for r in rows_t]
        print(f"  target={target:.0f} mm/s  n={len(rows_t)}  "
              f"overshoot: mean={sum(oshoot)/len(oshoot):.1f}%  max={max(oshoot):.1f}%  "
              f"| ss_error: mean={sum(sse)/len(sse):.1f}%")


if __name__ == '__main__':
    main()
