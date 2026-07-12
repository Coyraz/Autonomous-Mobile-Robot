#!/usr/bin/env python3
"""
plot_heading_compare.py  --  Before/after figure for the PD heading controller
--------------------------------------------------------------------------------
Overlays two runs recorded by collect_heading_response.py:
  --nopd : heading_kd = 0  (raw Pure-Pursuit-like: prone to theta oscillation)
  --pd   : heading_kd > 0  (PD heading control: oscillation damped)

Produces a single 2-panel thesis figure (heading vs time, w_cmd vs time) plus a
printed metrics comparison table.

USAGE:
  python3 plot_heading_compare.py --pd  ~/thesis_data/heading_test/heading_pd_*.csv \
                                  --nopd ~/thesis_data/heading_test/heading_nopd_*.csv
"""

import argparse
import csv
import math
import os
from datetime import datetime


def load(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            def fv(k):
                try:
                    return float(r[k])
                except (ValueError, KeyError):
                    return float('nan')
            rows.append({k: fv(k) for k in
                         ('t', 'x', 'y', 'yaw_deg', 'v_cmd', 'w_cmd',
                          'w_odom', 'path_yaw_deg', 'heading_err_deg')})
    return rows


def metrics(rows):
    w = [r['w_cmd'] for r in rows]
    thresh = 0.02
    sign_changes, last = 0, 0
    for val in w:
        if abs(val) < thresh:
            continue
        s = 1 if val > 0 else -1
        if last != 0 and s != last:
            sign_changes += 1
        last = s
    n = len(w) or 1
    mean_w = sum(w) / n
    w_std = math.sqrt(sum((v - mean_w) ** 2 for v in w) / n)
    herr = [r['heading_err_deg'] for r in rows if not math.isnan(r['heading_err_deg'])]
    if herr:
        rmse = math.sqrt(sum(e * e for e in herr) / len(herr))
        overshoot, aligned = 0.0, False
        for e in herr:
            if abs(e) < 20.0:
                aligned = True
            if aligned:
                overshoot = max(overshoot, abs(e))
    else:
        rmse, overshoot = float('nan'), float('nan')
    return {'w_sign_changes': sign_changes, 'w_std': w_std,
            'heading_rmse_deg': rmse, 'yaw_overshoot_deg': overshoot}


def main():
    ap = argparse.ArgumentParser(description='Overlay PD vs no-PD heading runs')
    ap.add_argument('--pd', required=True, help='CSV from the heading_kd>0 run')
    ap.add_argument('--nopd', required=True, help='CSV from the heading_kd=0 run')
    ap.add_argument('--out-dir', default=os.path.expanduser('~/thesis_data/heading_test'))
    args = ap.parse_args()

    pd = load(args.pd)
    nopd = load(args.nopd)

    mp, mn = metrics(pd), metrics(nopd)
    print("=" * 60)
    print("  PD HEADING CONTROLLER — before/after comparison")
    print("=" * 60)
    print(f"  {'metric':22s} {'no-PD (kd=0)':>14s} {'PD (kd>0)':>12s}")
    for k in ('w_sign_changes', 'w_std', 'yaw_overshoot_deg', 'heading_rmse_deg'):
        print(f"  {k:22s} {mn[k]:>14.3f} {mp[k]:>12.3f}")
    print("  (PD should show fewer sign-changes, lower w_std & overshoot)")

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; table printed above, no plot.")
        return

    fig, (a1, a2) = plt.subplots(2, 1, figsize=(11, 8))

    a1.plot([r['t'] for r in nopd], [r['heading_err_deg'] for r in nopd],
            color='#F44336', lw=1.4, label='no-PD (kd=0): oscillates')
    a1.plot([r['t'] for r in pd], [r['heading_err_deg'] for r in pd],
            color='#2196F3', lw=1.6, label='PD (kd>0): damped')
    a1.axhline(0, color='k', lw=0.6)
    a1.set_ylabel('heading error to path (deg)')
    a1.set_title('Theta tracking error — PD damps the Pure-Pursuit oscillation')
    a1.legend(); a1.grid(alpha=0.3)

    a2.plot([r['t'] for r in nopd], [r['w_cmd'] for r in nopd],
            color='#F44336', lw=1.2, alpha=0.8, label='no-PD (kd=0)')
    a2.plot([r['t'] for r in pd], [r['w_cmd'] for r in pd],
            color='#2196F3', lw=1.4, label='PD (kd>0)')
    a2.axhline(0, color='k', lw=0.6)
    a2.set_ylabel('w_cmd (rad/s)'); a2.set_xlabel('time (s)')
    a2.set_title(f"Angular velocity command  "
                 f"(sign-changes: no-PD={mn['w_sign_changes']} vs PD={mp['w_sign_changes']})")
    a2.legend(); a2.grid(alpha=0.3)

    fig.tight_layout()
    os.makedirs(args.out_dir, exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    png = os.path.join(args.out_dir, f'heading_compare_{ts}.png')
    fig.savefig(png, dpi=150, bbox_inches='tight')
    print(f"\nComparison figure saved: {png}")


if __name__ == '__main__':
    main()
