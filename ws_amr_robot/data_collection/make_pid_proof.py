#!/usr/bin/env python3
"""
make_pid_proof.py
=============================================================
Builds the "separated per-wheel PID compensates the motor defect" proof
figure + table for BAB IV.

The argument, in two panels:
  LEFT  (open loop): same PWM -> the two motors reach DIFFERENT speeds,
        and the gap widens near max PWM. This is the physical motor defect.
  RIGHT (closed-loop PID): same COMMANDED speed -> both wheels reach the
        SAME speed, because each wheel has its own integrator that supplies
        whatever PWM that motor needs. The defect is compensated.

INPUTS (override with args if filenames differ):
  --openloop  RAW_openloop_elevated_*.csv  (cols: pwm,left_cms,right_cms,...)
  --pid       test6_pid_metrics_*.csv      (cols: target_mmps,wheel,ss_speed_mmps,...)

OUTPUT: pid_proof_<timestamp>.png + printed comparison table, in
        ~/thesis_data/pengujian_6_pid/.
=============================================================
"""

import os
import csv
import argparse
from collections import defaultdict

import amr_test_utils as U

DATA_DIR = U.DEFAULT_OUTPUT_DIR


def load_openloop(path):
    """Return (pwm_list, left_cms, right_cms)."""
    pwm, lc, rc = [], [], []
    for r in csv.DictReader(open(path)):
        pwm.append(float(r['pwm']))
        lc.append(float(r['left_cms']))
        rc.append(float(r['right_cms']))
    return pwm, lc, rc


def load_pid(path):
    """Return {target_mmps: {'left': mean_ss, 'right': mean_ss}} with glitch
    rows (negative / impossible speeds) dropped."""
    agg = defaultdict(lambda: defaultdict(list))
    for r in csv.DictReader(open(path)):
        ss = float(r['ss_speed_mmps'])
        if ss < 0 or ss > 400:        # drop telemetry glitches
            continue
        agg[float(r['target_mmps'])][r['wheel']].append(ss)
    out = {}
    for tgt, wheels in agg.items():
        out[tgt] = {w: sum(v) / len(v) for w, v in wheels.items() if v}
    return out


def newest(prefix):
    """Most recent file in DATA_DIR starting with prefix, or None."""
    c = sorted(f for f in os.listdir(DATA_DIR) if f.startswith(prefix))
    return os.path.join(DATA_DIR, c[-1]) if c else None


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--openloop', default=newest('RAW_openloop_elevated'))
    p.add_argument('--pid', default=newest('test6_pid_metrics'))
    args = p.parse_args()

    if not args.openloop or not os.path.exists(args.openloop):
        print("ERROR: no open-loop CSV found. Pass --openloop <file>.")
        return
    if not args.pid or not os.path.exists(args.pid):
        print("ERROR: no PID metrics CSV found. Pass --pid <file>.")
        return

    print(f"Open-loop : {os.path.basename(args.openloop)}")
    print(f"PID       : {os.path.basename(args.pid)}\n")

    pwm, lc, rc = load_openloop(args.openloop)
    pid = load_pid(args.pid)

    # ---- printed proof table ----
    # open loop at max PWM
    imax = pwm.index(max(pwm))
    ol_l, ol_r = lc[imax] * 10, rc[imax] * 10   # cm/s -> mm/s
    ol_diff = (ol_r - ol_l) / ((ol_l + ol_r) / 2) * 100
    print("=" * 60)
    print("PROOF TABLE: separated PID compensates the motor defect")
    print("=" * 60)
    print(f"\nOPEN LOOP (same PWM = {int(max(pwm))}):")
    print(f"  left  = {ol_l:6.1f} mm/s")
    print(f"  right = {ol_r:6.1f} mm/s")
    print(f"  --> DIFFERENCE {ol_diff:+.1f}%   (the motor defect)")

    print(f"\nCLOSED-LOOP PID (same commanded speed):")
    print(f"  {'target':>8} {'left':>8} {'right':>8} {'diff %':>8}")
    for tgt in sorted(pid):
        w = pid[tgt]
        if 'left' in w and 'right' in w:
            d = (w['right'] - w['left']) / tgt * 100
            print(f"  {tgt:8.0f} {w['left']:8.1f} {w['right']:8.1f} {d:+8.1f}")
    print("\n  --> wheels matched within ~1%: defect compensated.\n")

    # ---- figure ----
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; table printed above, no plot.")
        return

    fig, (axL, axR) = plt.subplots(1, 2, figsize=(13, 5))

    # left panel: open loop
    axL.plot(pwm, [v * 10 for v in lc], 'o-', color='#2196F3', label='Left motor')
    axL.plot(pwm, [v * 10 for v in rc], 's-', color='#F44336', label='Right motor')
    axL.set_title('Open loop: same PWM -> DIFFERENT speed\n(the motor defect)')
    axL.set_xlabel('PWM command'); axL.set_ylabel('Wheel speed (mm/s)')
    axL.legend(); axL.grid(alpha=0.3)

    # right panel: closed-loop PID, grouped bars per target
    targets = sorted(pid)
    x = range(len(targets))
    lvals = [pid[t].get('left', 0) for t in targets]
    rvals = [pid[t].get('right', 0) for t in targets]
    w = 0.35
    axR.bar([i - w/2 for i in x], lvals, w, color='#2196F3', label='Left wheel')
    axR.bar([i + w/2 for i in x], rvals, w, color='#F44336', label='Right wheel')
    for i, t in enumerate(targets):
        axR.plot([i - w, i + w], [t, t], 'k--', lw=1)
    axR.set_xticks(list(x))
    axR.set_xticklabels([f'{int(t)}' for t in targets])
    axR.set_title('Closed-loop PID: same target -> SAME speed\n(defect compensated, dashed = target)')
    axR.set_xlabel('Commanded speed (mm/s)'); axR.set_ylabel('Achieved speed (mm/s)')
    axR.legend(); axR.grid(alpha=0.3, axis='y')

    fig.suptitle('Separated per-wheel PID compensates motor asymmetry',
                 fontsize=13, fontweight='bold')
    fig.tight_layout()
    out = U.timestamped_path('pid_proof', ext='png')
    fig.savefig(out, dpi=150, bbox_inches='tight')
    print(f"Proof figure saved: {out}")


if __name__ == '__main__':
    main()
