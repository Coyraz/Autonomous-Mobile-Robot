#!/usr/bin/env python3
"""
measure_map_geometry.py  --  BAB IV, Stage 2, TEST C (SLAM map geometry validation)
------------------------------------------------------------------------------------
Compare distances between known points: tape-measured vs AMCL map coordinates.

We have 14 reference points with both real (tape) and AMCL coordinates.
The script computes distances between meaningful pairs (same-row, same-column,
diagonals) in both coordinate systems and reports the error.

This validates whether the SLAM map's geometry is accurate.
"""

import math
import os
import sys
from datetime import datetime

sys.path.insert(0, os.path.dirname(__file__))
import amr_test_utils as U

# Real tape-measured coordinates (meters, from physical origin)
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

# AMCL map coordinates (meters, from /amcl_pose after localization)
AMCL = {
    'Home':  (0.294,   0.018),
    'Stage': (3.760,   0.286),
    'A1':    (4.087,  -8.052),
    'A2':    (4.261,  -6.606),
    'A3':    (4.251,  -4.724),
    'A4':    (4.135,  -3.011),
    'B1':    (1.642,  -8.256),
    'B2':    (1.749,  -7.193),
    'B3':    (1.639,  -4.818),
    'B4':    (1.748,  -3.528),
    'C1':    (-0.830, -8.531),
    'C2':    (-0.972, -7.018),
    'C3':    (-1.002, -4.472),
    'C4':    (-0.944, -2.656),
}

# Meaningful distance pairs to compare
# (point_a, point_b, description)
PAIRS = [
    # Along rack A (same column, y changes)
    ('A1', 'A2', 'Rack A: A1-A2'),
    ('A2', 'A3', 'Rack A: A2-A3'),
    ('A3', 'A4', 'Rack A: A3-A4'),
    ('A1', 'A4', 'Rack A full: A1-A4'),
    # Along rack B
    ('B1', 'B2', 'Rack B: B1-B2'),
    ('B2', 'B3', 'Rack B: B2-B3'),
    ('B3', 'B4', 'Rack B: B3-B4'),
    ('B1', 'B4', 'Rack B full: B1-B4'),
    # Along rack C
    ('C1', 'C2', 'Rack C: C1-C2'),
    ('C2', 'C3', 'Rack C: C2-C3'),
    ('C3', 'C4', 'Rack C: C3-C4'),
    ('C1', 'C4', 'Rack C full: C1-C4'),
    # Across aisles (same row, x changes)
    ('A1', 'B1', 'Aisle A-B at row 1'),
    ('A2', 'B2', 'Aisle A-B at row 2'),
    ('A3', 'B3', 'Aisle A-B at row 3'),
    ('A4', 'B4', 'Aisle A-B at row 4'),
    ('B1', 'C1', 'Aisle B-C at row 1'),
    ('B2', 'C2', 'Aisle B-C at row 2'),
    ('B3', 'C3', 'Aisle B-C at row 3'),
    ('B4', 'C4', 'Aisle B-C at row 4'),
    # Full width across all racks
    ('A1', 'C1', 'Full width A-C at row 1'),
    ('A4', 'C4', 'Full width A-C at row 4'),
    # Home to Stage
    ('Home', 'Stage', 'Home to Stage'),
]


def dist(p1, p2):
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


def main():
    print("=" * 72)
    print(" TEST C  -  SLAM MAP GEOMETRY VALIDATION")
    print(" Comparing distances: tape measurement vs AMCL map coordinates")
    print("=" * 72)

    rows = []
    for pa, pb, desc in PAIRS:
        real_d = dist(REAL[pa], REAL[pb])
        amcl_d = dist(AMCL[pa], AMCL[pb])
        err_m = amcl_d - real_d
        err_cm = err_m * 100
        err_pct = (err_m / real_d * 100) if real_d > 0 else 0.0

        rows.append({
            'pair': f"{pa}-{pb}",
            'description': desc,
            'tape_m': round(real_d, 3),
            'map_m': round(amcl_d, 3),
            'error_cm': round(err_cm, 1),
            'error_pct': round(err_pct, 1),
        })

    # Also compute per-point position error (AMCL vs tape)
    point_rows = []
    for name in REAL:
        rx, ry = REAL[name]
        ax, ay = AMCL[name]
        pe = math.hypot(ax - rx, ay - ry)
        point_rows.append({
            'point': name,
            'real_x': rx, 'real_y': ry,
            'amcl_x': ax, 'amcl_y': ay,
            'pos_error_cm': round(pe * 100, 1),
        })

    # Print distance comparison
    print(f"\n{'':>2} {'Description':>25} | {'tape(m)':>8} | {'map(m)':>8} | "
          f"{'err(cm)':>8} | {'err%':>7}")
    print(" " + "-" * 68)
    for r in rows:
        print(f"  {r['description']:>25} | {r['tape_m']:8.3f} | {r['map_m']:8.3f} | "
              f"{r['error_cm']:+8.1f} | {r['error_pct']:+7.1f}%")

    abs_errs = [abs(r['error_cm']) for r in rows]
    abs_pcts = [abs(r['error_pct']) for r in rows]
    print(" " + "-" * 68)
    print(f"  mean |error|: {sum(abs_errs)/len(abs_errs):.1f} cm "
          f"({sum(abs_pcts)/len(abs_pcts):.1f}%)")
    print(f"  max  |error|: {max(abs_errs):.1f} cm ({max(abs_pcts):.1f}%)")

    # Print per-point position error
    print(f"\n{'':>2} PER-POINT POSITION ERROR (AMCL vs tape)")
    print(f"{'':>2} {'Point':>8} | {'real(x,y)':>14} | {'amcl(x,y)':>14} | {'error(cm)':>10}")
    print(" " + "-" * 56)
    for p in point_rows:
        print(f"  {p['point']:>8} | ({p['real_x']:5.1f},{p['real_y']:5.1f}) | "
              f"({p['amcl_x']:6.3f},{p['amcl_y']:6.3f}) | {p['pos_error_cm']:10.1f}")

    pt_errs = [p['pos_error_cm'] for p in point_rows]
    print(" " + "-" * 56)
    print(f"  mean: {sum(pt_errs)/len(pt_errs):.1f} cm   "
          f"max: {max(pt_errs):.1f} cm   "
          f"RMSE: {U.rmse([e/100 for e in pt_errs])*100:.1f} cm")

    # Save
    out_dir = os.path.expanduser('~/thesis_data/map_geometry')
    os.makedirs(out_dir, exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')

    csv1 = os.path.join(out_dir, f"map_geometry_distances_{ts}.csv")
    U.save_csv(csv1, ['pair', 'description', 'tape_m', 'map_m', 'error_cm', 'error_pct'], rows)

    csv2 = os.path.join(out_dir, f"map_geometry_points_{ts}.csv")
    U.save_csv(csv2, ['point', 'real_x', 'real_y', 'amcl_x', 'amcl_y', 'pos_error_cm'], point_rows)

    print(f"\n  Distance CSV: {csv1}")
    print(f"  Point CSV:    {csv2}")


if __name__ == '__main__':
    main()
