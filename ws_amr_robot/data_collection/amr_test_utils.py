#!/usr/bin/env python3
"""
amr_test_utils.py
-----------------
Shared helpers for the Chapter IV (BAB IV) data-collection scripts.

Common functions: angle normalization (wrapTo180), yaw from quaternion,
position RMSE at discrete points, simple statistics, CSV saving, and
timestamped output paths. Import this from each test script.
"""

import os
import csv
import math
import statistics
from datetime import datetime

# Physical constants (keep in ONE place; update here if calibration changes)
WHEEL_DIAMETER = 0.068      # m
WHEEL_BASE     = 0.299      # m  (corrected from 0.292 via Test B: odom overshot +2.4%, so wb must increase)
TICKS_PER_REV  = 4557.0     # measured 2026-06-20 by push test (was 4600)
M_PER_TICK     = (math.pi * WHEEL_DIAMETER) / TICKS_PER_REV

# All BAB IV data lands under ~/thesis_data/ alongside pengujian_3 / pengujian_4
# so the whole thesis dataset stays in one place (option A, 2026-06-21).
DEFAULT_OUTPUT_DIR = os.path.expanduser('~/thesis_data/pengujian_6_pid')

# ---------------------------------------------------------------- warehouse waypoints
# All coordinates are Nav2 goal positions (map frame), NOT raw tape/REAL
# coordinates -- see per-point provenance below. REAL tape values are in
# measure_map_geometry.py's REAL dict for reference/BAB IV comparison.
#
# 2026-07-07, second pass: after the 2026-07-07 map edit
# (warehouse_v1_edited_edited.yaml) broke several literal tape coordinates
# (NO_VALID_PATH at A2, near-wall-collision at A1 -- see test9.md), two
# rounds of recalibration were attempted:
#   - Round 1 (distance-transform, offline): point of max wall-clearance
#     within 50cm of the tape mark, computed from the static map. Safe but
#     not grounded in a real measurement.
#   - Round 2 (live /amcl_pose, manual park-and-read, THIS is what's used
#     below): operator parked the robot at each REAL tape mark and read
#     /amcl_pose directly via ils_gui. Verified point-by-point against
#     nearest-REAL-neighbor matching to rule out the swap/mislabel problem
#     that invalidated an earlier same-day attempt (see test9.md) -- this
#     round had NO swaps, all points self-consistent. These raw AMCL
#     readings are used AS-IS (not curve-fitted) per operator preference,
#     since they are direct ground-truth measurements of "where AMCL says I
#     am when I'm physically standing at the REAL rack" -- fitting would
#     smooth over real per-point drift, not just noise. All confirmed
#     plannable via compute_path_to_pose except C3.
#   - A2, Stage, Home: operator confirmed these were already correct,
#     kept unchanged (A2 is the Round-1 re-centered value; Stage and Home
#     are the literal REAL/tape values, already safe).
#   - C3: raw AMCL reading (-0.668,-4.585) sits in a wide (~1.2m) zone of
#     near-max costmap cost (flat ~100, not a normal wall-decay gradient --
#     likely a real obstacle/tight clearance at that exact spot when
#     measured), NO_VALID_PATH confirmed via compute_path_to_pose even
#     after searching 80cm around it. Fell back to the Round-1
#     distance-transform value for this one point only.
WAREHOUSE_WAYPOINTS = {
    # --- Start / staging ---
    'Home':       (0.0,   0.0),
    'Stage':      (3.5,   0.5),
    # --- Rack A (X = 4.0) ---
    'A1':         (4.500, -8.420),  # live AMCL read at REAL tape mark
    'A2':         (4.422, -6.805),  # kept from Round-1 (operator confirmed correct)
    'A3':         (4.313, -4.373),  # live AMCL read at REAL tape mark
    'A4':         (4.335, -3.552),  # live AMCL read at REAL tape mark
    # --- Rack B (X = 1.5) ---
    'B1':         (1.721, -8.569),  # live AMCL read at REAL tape mark
    'B2':         (1.955, -7.312),  # live AMCL read at REAL tape mark
    'B3':         (1.924, -4.563),  # live AMCL read at REAL tape mark
    'B4':         (1.961, -3.061),  # live AMCL read at REAL tape mark
    # --- Rack C (X = -1.0) ---
    'C1':        (-0.891, -8.473),  # live AMCL read at REAL tape mark
    'C2':        (-0.790, -6.968),  # live AMCL read at REAL tape mark
    'C3':        (-0.678, -5.005),  # NO_VALID_PATH at raw AMCL read -- fell back to Round-1 distance-transform value
    'C4':        (-0.706, -3.096),  # live AMCL read at REAL tape mark
    # --- Cross-section row 1 (Y = -5.7) ---
    'X1_A':       (4.0,  -5.7),
    'X1_B':       (1.5,  -5.7),
    'X1_C':      (-1.0,  -5.7),
    # --- Cross-section row 2 (Y = -1.5) ---
    'X2_A':       (3.5,  -1.5),
    'X2_B':       (1.5,  -1.5),
    # --- Cross-section row 3 (Y = 0) ---
    'X3_B':       (1.5,   0.0),
    'X3_C':      (-1.0,   0.0),
    # --- T-junction ---
    'XA_T-junc':  (4.0,  -1.5),
}


# ---------------------------------------------------------------- angles
def wrap_to_180(angle_deg):
    """Normalize an angle in DEGREES to (-180, 180]."""
    a = (angle_deg + 180.0) % 360.0 - 180.0
    return a + 360.0 if a <= -180.0 else a


def wrap_to_pi(angle_rad):
    """Normalize an angle in RADIANS to (-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def yaw_from_quaternion(qx, qy, qz, qw):
    """Yaw (rad) from a quaternion."""
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


# ---------------------------------------------------------------- metrics
def position_error(x_est, y_est, x_gt, y_gt):
    """Euclidean distance between an estimate and ground truth (same units)."""
    return math.hypot(x_est - x_gt, y_est - y_gt)


def rmse(errors):
    """Root-mean-square of a list of scalar errors."""
    errors = [e for e in errors if e is not None and not math.isnan(e)]
    if not errors:
        return float('nan')
    return math.sqrt(sum(e * e for e in errors) / len(errors))


def position_rmse(est_points, gt_points):
    """RMSE of position over matched (x,y) estimate vs ground-truth pairs.
    est_points / gt_points: lists of (x, y)."""
    errs = [position_error(ex, ey, gx, gy)
            for (ex, ey), (gx, gy) in zip(est_points, gt_points)]
    return rmse(errs)


def summarize(values):
    """Return mean/std/min/max/n for a list of numbers (ignores nan)."""
    vals = [v for v in values if v is not None and not math.isnan(v)]
    if not vals:
        return {'n': 0, 'mean': float('nan'), 'std': float('nan'),
                'min': float('nan'), 'max': float('nan')}
    return {
        'n': len(vals),
        'mean': statistics.mean(vals),
        'std': statistics.pstdev(vals) if len(vals) > 1 else 0.0,
        'min': min(vals),
        'max': max(vals),
    }


# ---------------------------------------------------------------- I/O
def timestamped_path(prefix, ext='csv', out_dir=None, label=None):
    """Build output/<prefix>[_<label>]_<YYYYmmdd_HHMMSS>.<ext>."""
    out_dir = out_dir or DEFAULT_OUTPUT_DIR
    os.makedirs(out_dir, exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    name = f"{prefix}_{label}_{ts}" if label else f"{prefix}_{ts}"
    return os.path.join(out_dir, f"{name}.{ext}")


def save_csv(path, fieldnames, rows):
    """Write a list of dict rows to CSV."""
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)
    return path


def print_summary_table(title, summary_dict, unit=''):
    """Pretty-print a summarize() result."""
    s = summary_dict
    print(f"  {title}: n={s['n']}  mean={s['mean']:.4f}{unit}  "
          f"std={s['std']:.4f}  min={s['min']:.4f}  max={s['max']:.4f}")
