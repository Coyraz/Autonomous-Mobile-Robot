#!/usr/bin/env python3
"""
plot_map_geometry.py  --  Visualize Test C: tape ground truth vs AMCL position, on the map
---------------------------------------------------------------------------------------------
Overlays the 14 tape-measured ground truth points and the AMCL-estimated points
(mean over reps) on top of the SLAM occupancy grid, with an error vector drawn
between each pair. Optionally overlays individual rep scatter points too.

Usage:
  python3 plot_map_geometry.py \
      --map ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/maps/warehouse_v3_20260721_edited.yaml \
      --points ~/thesis_data/map_geometry/map_geometry_points_TIMESTAMP.csv \
      --raw ~/thesis_data/map_geometry/map_geometry_raw_TIMESTAMP.csv \
      --out ~/thesis_data/map_geometry/test_c_overlay.png
"""

import argparse
import csv
import os

import matplotlib.pyplot as plt
import yaml
from PIL import Image

TAPE_COLOR = '#1f77b4'   # ground truth (tape)
AMCL_COLOR = '#d62728'   # AMCL mean estimate
RAW_COLOR = '#d6272733'  # individual reps, translucent


def load_map(map_yaml_path):
    with open(map_yaml_path) as f:
        meta = yaml.safe_load(f)
    map_dir = os.path.dirname(os.path.abspath(map_yaml_path))
    img_path = os.path.join(map_dir, meta['image'])
    img = Image.open(img_path)
    width, height = img.size
    resolution = meta['resolution']
    origin_x, origin_y = meta['origin'][0], meta['origin'][1]
    extent = [origin_x, origin_x + width * resolution,
              origin_y, origin_y + height * resolution]
    return img, extent


def load_points_csv(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append(r)
    return rows


def main():
    ap = argparse.ArgumentParser(description='Plot Test C ground truth vs AMCL overlay on the map')
    ap.add_argument('--map', required=True, help='Path to map .yaml (e.g. warehouse_v3_20260721_edited.yaml)')
    ap.add_argument('--points', required=True, help='Path to map_geometry_points_*.csv')
    ap.add_argument('--raw', default=None, help='Optional path to map_geometry_raw_*.csv (per-rep scatter)')
    ap.add_argument('--out', default=None, help='Output image path (default: alongside --points)')
    ap.add_argument('--scale', type=float, default=1.0, help='Multiply error vectors visually (default 1x, true scale)')
    args = ap.parse_args()

    img, extent = load_map(args.map)
    points = load_points_csv(args.points)
    raw = load_points_csv(args.raw) if args.raw else None

    fig, ax = plt.subplots(figsize=(9, 11))
    ax.imshow(img, cmap='gray', extent=extent, origin='upper')

    if raw:
        for r in raw:
            ax.scatter(float(r['amcl_x']), float(r['amcl_y']), s=14, color=RAW_COLOR,
                       zorder=2, linewidths=0)

    for p in points:
        rx, ry = float(p['real_x']), float(p['real_y'])
        ax_, ay_ = float(p['amcl_x']), float(p['amcl_y'])
        err_cm = float(p.get('mean_error_cm', p.get('pos_error_cm', 0.0)))

        ax.scatter(rx, ry, marker='^', s=70, color=TAPE_COLOR, zorder=4,
                   edgecolors='white', linewidths=0.8)
        ax.scatter(ax_, ay_, marker='o', s=55, color=AMCL_COLOR, zorder=4,
                   edgecolors='white', linewidths=0.8)
        ax.annotate('', xy=(ax_, ay_), xytext=(rx, ry),
                    arrowprops=dict(arrowstyle='->', color='#555555', lw=1.2), zorder=3)
        ax.annotate(f"{p['point']}\n{err_cm:.0f}cm", xy=(rx, ry),
                    xytext=(6, 6), textcoords='offset points', fontsize=7.5, color='#222222')

    ax.scatter([], [], marker='^', s=70, color=TAPE_COLOR, label='Ground truth (tape)')
    ax.scatter([], [], marker='o', s=55, color=AMCL_COLOR, label='AMCL (mean)')
    if raw:
        ax.scatter([], [], s=14, color='#d62728', alpha=0.3, label='AMCL (individual reps)')
    ax.legend(loc='lower left', fontsize=9, framealpha=0.9)

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Test C: Ground Truth vs AMCL Position on SLAM Map')
    ax.set_aspect('equal')

    out_path = args.out or os.path.join(os.path.dirname(os.path.abspath(args.points)),
                                          'test_c_overlay.png')
    fig.tight_layout()
    fig.savefig(out_path, dpi=200)
    print(f"Saved: {out_path}")


if __name__ == '__main__':
    main()
