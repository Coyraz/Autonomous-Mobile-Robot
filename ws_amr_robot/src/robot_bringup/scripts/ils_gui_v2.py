#!/usr/bin/env python3
"""
ils_gui.py  --  AMR Warehouse Control Panel
Flask web UI + ROS 2 node running together in one process.

How to run:
    Terminal 1: ~/kill_robot.sh && ros2 launch robot_bringup robot_navigation.launch.py
    Terminal 2: ~/ils_gui

Then open http://<pi-ip>:5000 or http://<tailscale-ip>:5000 from any device.
Logo files go in: <this script's folder>/static/
  - Tel-U_Vertikal.png
  - Logo_INACOS_MONO.png
  - ILS_Logo_Transparant.png
"""
import io
import math
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.msg import Path as NavPath
from nav2_msgs.action import NavigateToPose, FollowWaypoints, FollowPath

from flask import Flask, jsonify, request, Response

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ============================================================
# RACK COORDINATES  (map frame, verified 2026-06-15)
# C1 verified 2026-06-15 from /amcl_pose after robot parked at rack.
# ============================================================
RACK_GOALS = {
    #'Home':  ( 0.294,  0.018,  0.02),
    #'Stage': ( 3.760,  0.286,  1.54),
    #'A1':    ( 4.087, -8.052, -1.81),
    #'A2':    ( 4.261, -6.606, -1.37),
    #'A3':    ( 4.251, -4.724, -1.33),
    #'A4':    ( 4.135, -3.011, -1.36),
    #'B1':    ( 1.642, -8.256, -1.15),
    #'B2':    ( 1.749, -7.193, -1.69),
    #'B3':    ( 1.639, -4.818,  1.59),
    #'B4':    ( 1.748, -3.528,  1.46),
    #'C1':    (-0.830, -8.531, -1.33),
    #'C2':    (-0.972, -7.018, -1.51),
    #'C3':    (-1.002, -4.472, -1.53),
    #'C4':    (-0.944, -2.656, -1.37),
    
    # 2026-07-07, second pass: live /amcl_pose readings, operator parked the
    # robot at each REAL tape mark and read pose+orientation directly (see
    # amr_test_utils.py WAREHOUSE_WAYPOINTS for full provenance/rationale --
    # same values, kept in sync). Verified no swaps this round (unlike the
    # 2026-07-06 attempt, which mixed up A1/A2 and C1/C2). theta now comes
    # from the measured AMCL yaw, not a 0.0 default -- much closer to a real
    # parking heading than before. A2/Stage/Home kept from the prior
    # (Round-1 / tape) pass per operator confirmation. C3 falls back to the
    # Round-1 distance-transform value (raw AMCL reading was NO_VALID_PATH).
    'Home':  (0.0,   0.0,  0.0),
    'Stage': (3.5,   0.5,  0.0),
    'A1':    (4.500, -8.420,  1.486),
    'A2':    (4.422, -6.805,  0.0),   # kept from Round-1 (operator confirmed correct)
    'A3':    (4.313, -4.373,  1.797),
    'A4':    (4.335, -3.552,  1.652),
    'B1':    (1.721, -8.569, -1.481),
    'B2':    (1.955, -7.312,  1.683),
    'B3':    (1.924, -4.563, -1.647),
    'B4':    (1.961, -3.061,  1.747),
    'C1':    (-0.891, -8.473,  1.416),
    'C2':    (-0.790, -6.968, -1.580),
    'C3':    (-0.678, -5.005,  0.0),  # fallback to Round-1 distance-transform (raw AMCL read was NO_VALID_PATH)
    'C4':    (-0.706, -3.096,  1.627),
  
    'X1_A':      ( 4.0,  -5.7, 0.0),
    'X1_B':      ( 1.5,  -5.7, 0.0),
    'X1_C':      (-1.0,  -5.7, 0.0),
    'X2_A':      ( 3.5,  -1.5, 0.0),
    'X2_B':      ( 1.5,  -1.5, 0.0),
    'X3_B':      ( 1.5,   0.0, 0.0),
    'X3_C':      (-1.0,   0.0, 0.0),
    'XA_T-junc': ( 4.0,  -1.5, 0.0),
}

SCRIPT_DIR = Path(__file__).parent
STATIC_DIR = SCRIPT_DIR / 'static'
STATIC_DIR.mkdir(exist_ok=True)


# ============================================================
# ROS 2 NODE
# ============================================================

class NavBridgeNode(Node):
    def __init__(self):
        super().__init__('ils_gui')
        self._lock = threading.Lock()

        # State read by Flask routes
        self.pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.speed_mmps = 0.0
        self.nav_status = 'idle'
        self.current_target = None

        # Internal action tracking
        self._goal_handle    = None
        self._pending_goal   = None
        self._pending_wp     = None
        self._pending_path   = None
        self._pending_cancel = False

        # Map
        self._map_img  = None
        self._map_info = None
        self._map_lock = threading.Lock()

        # Planned path
        self._path_xs = []
        self._path_ys = []

        # /map is published LATCHED (transient_local) by map_server: it is sent
        # once and held for late subscribers. We must match that durability or
        # we miss it whenever ils_gui starts after map_server (the usual case).
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Subscriptions
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._cb_pose, 10)
        self.create_subscription(
            OccupancyGrid, '/map', self._cb_map, map_qos)
        self.create_subscription(
            Odometry, '/odom_raw', self._cb_odom, 10)
        self.create_subscription(
            NavPath, '/plan', self._cb_plan, 1)

        # Publishers
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Action client for NavigateToPose
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Action client for FollowWaypoints (multi-rack runs from the GUI)
        self._wp_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        
        self._path_client = ActionClient(self, FollowPath, 'follow_path')

        # Timer processes Flask requests inside the ROS executor (non-blocking)
        self.create_timer(0.05, self._tick)

        self.get_logger().info('NavBridgeNode ready -- waiting for topics')

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _cb_pose(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        with self._lock:
            self.pose = {'x': p.x, 'y': p.y, 'theta': yaw}

    def _cb_map(self, msg):
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        img = np.full((h, w, 3), 180, dtype=np.uint8)
        img[data == 0]   = [240, 240, 240]
        img[data == 100] = [30,  30,  30]
        with self._map_lock:
            self._map_img  = img
            self._map_info = msg.info

    def _cb_odom(self, msg):
        with self._lock:
            self.speed_mmps = msg.twist.twist.linear.x * 1000.0

    def _cb_plan(self, msg):
        xs = [p.pose.position.x for p in msg.poses]
        ys = [p.pose.position.y for p in msg.poses]
        with self._lock:
            self._path_xs = xs
            self._path_ys = ys

    # ------------------------------------------------------------------
    # Timer: process Flask requests inside the ROS thread
    # ------------------------------------------------------------------

    def _tick(self):
        with self._lock:
            goal      = self._pending_goal
            wp_goal   = self._pending_wp
            path_goal = self._pending_path 
            do_cancel = self._pending_cancel
            self._pending_goal   = None
            self._pending_wp     = None
            self._pending_path   = None
            self._pending_cancel = False

        if do_cancel:
            with self._lock:
                handle = self._goal_handle
                self._goal_handle   = None
                self.nav_status     = 'idle'
                self.current_target = None
                self._path_xs       = []
                self._path_ys       = []
            if handle:
                handle.cancel_goal_async()
        elif goal:
            if self._nav_client.server_is_ready():
                self._do_send_goal(goal)
            else:
                with self._lock:
                    self.nav_status     = 'error'
                    self.current_target = None
                self.get_logger().error('NavigateToPose server not ready')
        elif wp_goal:
            if self._wp_client.server_is_ready():
                self._do_send_wp(wp_goal)
            else:
                with self._lock:
                    self.nav_status     = 'error'
                    self.current_target = None
                self.get_logger().error('FollowWaypoints server not ready')
        elif path_goal:
          if self._path_client.server_is_ready():
                self._do_send_path(path_goal)
          else:
            with self._lock:
                self.nav_status     = 'error'
                self.current_target = None
            self.get_logger().error('FollowPath server not ready')

    def _make_pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id    = 'map'
        p.header.stamp       = self.get_clock().now().to_msg()
        p.pose.position.x    = x
        p.pose.position.y    = y
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    def _do_send_goal(self, rack_name):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._make_pose(*RACK_GOALS[rack_name])
        future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=lambda _: None)
        future.add_done_callback(lambda f: self._on_accepted(f, rack_name))

    def _on_accepted(self, future, rack_name):
        handle = future.result()
        if not handle.accepted:
            with self._lock:
                self.nav_status     = 'error'
                self.current_target = None
            self.get_logger().warn(f'Goal to {rack_name} rejected by Nav2')
            return
        with self._lock:
            self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_done)

    def _do_send_wp(self, node_names):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = [self._make_pose(*RACK_GOALS[n]) for n in node_names]
        future = self._wp_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self._on_wp_accepted(f, node_names))

    def _on_wp_accepted(self, future, node_names):
        handle = future.result()
        if not handle.accepted:
            with self._lock:
                self.nav_status     = 'error'
                self.current_target = None
            self.get_logger().warn(f'FollowWaypoints rejected: {node_names}')
            return
        with self._lock:
            self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_done)

    def _do_send_path(self, path_xy):
        goal_msg = FollowPath()
        ros_path = NavPath()
        ros_path.header.frame_id = 'map'
        ros_path.header.stamp    = self.get_clock().now().to_msg()
        for x, y in path_xy:
            ros_path.poses.append(self._make_pose(x, y, 0.0))   # yaw default 0
        goal_msg = FollowPath.Goal()
        goal_msg.path = ros_path
        future = self._path_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self._on_path_accepted(f, path_xy))

    def _on_path_accepted(self, future, path_xy):
        handle = future.result()
        if not handle.accepted:
            with self._lock:
                self.nav_status     = 'error'
                self.current_target = None
            self.get_logger().warn(f'FollowPath rejected ({len(path_xy)} titik)')
            return
        with self._lock:
            self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_done)
        
    def _on_done(self, future):
        status = future.result().status
        with self._lock:
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.nav_status = 'arrived'
            elif status == GoalStatus.STATUS_CANCELED:
                self.nav_status     = 'idle'
                self.current_target = None
            else:
                self.nav_status     = 'error'
                self.current_target = None
            self._goal_handle = None

    # ------------------------------------------------------------------
    # Flask-facing helpers
    # ------------------------------------------------------------------

    def request_goal(self, rack_name):
        if rack_name not in RACK_GOALS:
            return False, f'Unknown rack: {rack_name}'
        with self._lock:
            self._pending_goal  = rack_name
            self.nav_status     = 'navigating'
            self.current_target = rack_name
        return True, 'ok'

    def request_follow_waypoints(self, node_names):
        unknown = [n for n in node_names if n not in RACK_GOALS]
        if unknown:
            return False, f'node tak dikenal: {unknown}'
        with self._lock:
            self._pending_wp     = list(node_names)
            self.nav_status      = 'navigating'
            self.current_target  = ' -> '.join(node_names)
        return True, 'ok'
      
    def request_path(self, path_xy):
        if not path_xy:
            return False, 'path kosong'
        with self._lock:
            self._pending_path  = path_xy
            self.nav_status      = 'navigating'
            self.current_target  = f'path({len(path_xy)} titik)'
        return True, 'ok'

    def request_cancel(self):
        with self._lock:
            self._pending_cancel = True

    def publish_cmd_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x  = max(-0.20, min(0.20, float(linear)))
        msg.angular.z = max(-1.00, min(1.00, float(angular)))
        self._cmd_vel_pub.publish(msg)

    # ------------------------------------------------------------------
    # Map rendering
    # ------------------------------------------------------------------

    def render_map_png(self):
        try:
            return self._do_render()
        except Exception as e:
            self.get_logger().error(f'Map render error: {e}')
            fig, ax = plt.subplots(figsize=(4, 5), dpi=72)
            ax.set_facecolor('#ffebee')
            ax.text(0.5, 0.5, f'Render error:\n{str(e)[:80]}',
                    ha='center', va='center', transform=ax.transAxes,
                    fontsize=10, color='#c62828')
            ax.set_xticks([]); ax.set_yticks([])
            buf = io.BytesIO()
            fig.savefig(buf, format='png', dpi=72)
            plt.close(fig)
            buf.seek(0)
            return buf.read()

    def _do_render(self):
        with self._map_lock:
            bg   = None if self._map_img is None else self._map_img.copy()
            info = self._map_info

        with self._lock:
            pose   = dict(self.pose)
            target = self.current_target
            pxs    = list(self._path_xs)
            pys    = list(self._path_ys)

        if bg is None:
            fig, ax = plt.subplots(figsize=(4, 5), dpi=90)
            ax.set_facecolor('#e8eaed')
            ax.text(0.5, 0.55, 'Waiting for /map topic...',
                    ha='center', va='center', transform=ax.transAxes,
                    fontsize=13, color='#888')
            ax.text(0.5, 0.42,
                    'Make sure robot_navigation.launch.py is running',
                    ha='center', va='center', transform=ax.transAxes,
                    fontsize=8, color='#bbb')
            ax.set_xticks([]); ax.set_yticks([])
        else:
            res = info.resolution
            ox  = info.origin.position.x
            oy  = info.origin.position.y
            h, w = bg.shape[:2]

            # Portrait map: x east→west, y south→north.
            # No rotation — coordinates used directly on both image and overlays.
            fig_w = 4.0
            fig_h = min(9.0, fig_w * h / w)
            fig, ax = plt.subplots(figsize=(fig_w, fig_h), dpi=90)

            ax.imshow(bg, origin='lower',
                      extent=[ox, ox + w * res, oy, oy + h * res],
                      interpolation='nearest')

            # Rack + aisle markers. Aisles get their own color/shape (distinct
            # from racks and Home/Stage) since some aisle cross-sections sit
            # very close to real rack positions on the map (e.g. X1_A vs A3)
            # and would otherwise be hard to tell apart.
            col_map = {'Home': '#388E3C', 'Stage': '#E65100'}
            aisle_names = {'X1_A', 'X1_B', 'X1_C', 'X2_A', 'X2_B',
                           'X3_B', 'X3_C', 'XA_T-junc'}
            for name, (rx, ry, _) in RACK_GOALS.items():
                is_aisle = name in aisle_names
                c = col_map.get(name, '#8E24AA' if is_aisle else '#1565C0')
                marker = '^' if is_aisle else 's'
                ax.plot(rx, ry, marker, color=c, ms=6,
                        mec='white', mew=0.8, zorder=4)
                ax.text(rx + 0.08, ry + 0.08, name, fontsize=5,
                        color='white', fontweight='bold', zorder=5,
                        bbox=dict(boxstyle='round,pad=0.1', facecolor=c,
                                  alpha=0.85, edgecolor='none'))

            # Planned path
            if pxs:
                ax.plot(pxs, pys, '-', color='#FF8F00',
                        lw=1.2, alpha=0.85, zorder=3)

            # Target highlight ring
            if target and target in RACK_GOALS:
                tx, ty, _ = RACK_GOALS[target]
                ax.add_patch(mpatches.Circle(
                    (tx, ty), 0.30, fill=False,
                    color='#FDD835', lw=2, linestyle='--', zorder=4))

            # Robot position + heading arrow
            rx, ry, rth = pose['x'], pose['y'], pose['theta']
            al = 0.30
            ax.annotate('',
                xy=(rx + al * math.cos(rth), ry + al * math.sin(rth)),
                xytext=(rx, ry),
                arrowprops=dict(arrowstyle='->', color='#D32F2F', lw=2.5),
                zorder=6)
            ax.plot(rx, ry, 'o', color='#D32F2F', ms=8,
                    mec='white', mew=1.0, zorder=7)

            ax.tick_params(labelsize=6)
            ax.set_xlabel('x (m)', fontsize=7)
            ax.set_ylabel('y (m)', fontsize=7)

        fig.tight_layout(pad=0.3)
        buf = io.BytesIO()
        fig.savefig(buf, format='png', dpi=90)
        plt.close(fig)
        buf.seek(0)
        return buf.read()


# ============================================================
# HTML PAGE
# ============================================================

HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>AMR Warehouse Control</title>
<style>
* { box-sizing: border-box; margin: 0; padding: 0; }
body {
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
  background: #f0f2f5;
  color: #1a1a1a;
  min-height: 100vh;
}

/* ---- Header ---- */
header {
  background: #0d1b2a;
  padding: 10px 20px;
  display: flex;
  align-items: center;
  gap: 14px;
}
.logos-left  { display: flex; align-items: center; gap: 10px; }
.logos-right { margin-left: auto; display: flex; align-items: center; }
header img   { height: 36px; width: auto; object-fit: contain; }
.title-block    { flex: 1; text-align: center; }
.title-block h1 { color: #fff; font-size: 1.15rem; font-weight: 700; }
.title-block p  { color: #7ba7c7; font-size: 0.72rem; margin-top: 2px; }

/* ---- Two-column layout ---- */
.main {
  display: flex;
  gap: 12px;
  padding: 12px;
  max-width: 1400px;
  margin: 0 auto;
  align-items: flex-start;
}
.map-col  { flex: 1; min-width: 0; }
.right-col {
  width: 350px;
  flex-shrink: 0;
  display: flex;
  flex-direction: column;
  gap: 10px;
}

/* ---- Card ---- */
.card {
  background: #fff;
  border-radius: 10px;
  padding: 12px 14px;
  box-shadow: 0 1px 3px rgba(0,0,0,.09);
}
.section-label {
  font-size: 0.65rem; font-weight: 700; color: #999;
  text-transform: uppercase; letter-spacing: .06em;
  margin-bottom: 9px;
}

/* ---- Map ---- */
.map-card { overflow-y: auto; max-height: calc(100vh - 90px); }
.map-card img {
  width: 75%; height: auto; display: block; margin: 0 auto;
  border-radius: 6px;
  border: 1px solid #e0e0e0;
}
.map-ts { font-size: 0.67rem; color: #aaa; margin-top: 5px; text-align: right; }

/* ---- Status grid (2 columns inside the status card) ---- */
.stat-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 8px;
}
.stat-full { grid-column: 1 / 3; }
.stat-label {
  font-size: 0.62rem; font-weight: 700; color: #999;
  text-transform: uppercase; letter-spacing: .04em; margin-bottom: 3px;
}
.stat-value { font-size: 0.92rem; font-weight: 600; }

.badge { display: inline-block; padding: 2px 10px; border-radius: 20px;
         font-size: 0.75rem; font-weight: 700; }
.badge-idle       { background: #e8eaed; color: #5f6368; }
.badge-navigating { background: #e3f2fd; color: #1565c0; }
.badge-arrived    { background: #e8f5e9; color: #2e7d32; }
.badge-error      { background: #ffebee; color: #c62828; }

/* ---- Danger button (reused for stop nav and STOP) ---- */
.btn-danger {
  width: 100%; padding: 8px;
  background: #c62828; color: #fff;
  border: none; border-radius: 8px;
  font-size: 0.84rem; font-weight: 700;
  cursor: pointer; transition: background .15s;
}
.btn-danger:hover { background: #b71c1c; }

/* ---- D-pad teleop ---- */
.teleop-inner {
  display: flex;
  gap: 16px;
  align-items: center;
}
.dpad-wrap { text-align: center; flex-shrink: 0; }
.dpad-hint { font-size: 0.62rem; color: #aaa; margin-top: 7px; line-height: 1.4; }

.dpad {
  display: grid;
  grid-template-columns: repeat(3, 44px);
  grid-template-rows: repeat(3, 44px);
  gap: 5px;
  touch-action: none;
  user-select: none;
}
.dpad-btn {
  border: none;
  border-radius: 8px;
  background: #1565c0;
  color: #fff;
  font-size: 1.1rem;
  font-weight: 700;
  cursor: pointer;
  touch-action: none;
  user-select: none;
  display: flex;
  align-items: center;
  justify-content: center;
}
.dpad-btn:active, .dpad-btn.pressed { background: #0d47a1; transform: scale(0.95); }
.dpad-up    { grid-column: 2; grid-row: 1; }
.dpad-left  { grid-column: 1; grid-row: 2; }
.dpad-stop  { grid-column: 2; grid-row: 2; background: #c62828; font-size: 0.6rem; }
.dpad-stop:active, .dpad-stop.pressed { background: #8e0000; }
.dpad-right { grid-column: 3; grid-row: 2; }
.dpad-down  { grid-column: 2; grid-row: 3; }
.dpad-up::after    { content: '\\2191'; }
.dpad-down::after  { content: '\\2193'; }
.dpad-left::after  { content: '\\2190'; }
.dpad-right::after { content: '\\2192'; }

.teleop-stats {
  flex: 1;
  display: flex;
  flex-direction: column;
  gap: 7px;
}
.teleop-note { font-size: 0.62rem; color: #bbb; text-align: center; }

/* ---- Rack grid ---- */
.top-buttons {
  display: grid; grid-template-columns: 1fr 1fr;
  gap: 8px; margin-bottom: 10px;
}
.col-headers {
  display: grid; grid-template-columns: repeat(3,1fr);
  gap: 6px; margin-bottom: 4px;
}
.col-hdr {
  text-align: center; font-size: 0.61rem; font-weight: 700;
  color: #bbb; text-transform: uppercase; letter-spacing: .04em;
}
.rack-grid {
  display: grid; grid-template-columns: repeat(3,1fr); gap: 6px;
}
.rack-btn {
  padding: 11px 4px;
  border: 2px solid #1565c0; background: #fff; color: #1565c0;
  border-radius: 7px; font-size: 0.88rem; font-weight: 700;
  cursor: pointer; transition: all .15s; text-align: center; min-height: 44px;
}
.rack-btn:hover  { background: #e3f2fd; }
.rack-btn.active { background: #1565c0; color: #fff; }

.btn-home {
  padding: 13px;
  border: 2px solid #388e3c; background: #fff; color: #388e3c;
  border-radius: 7px; font-size: 0.92rem; font-weight: 700;
  cursor: pointer; transition: all .15s;
}
.btn-home:hover  { background: #f1f8e9; }
.btn-home.active { background: #388e3c; color: #fff; }

.btn-stage {
  padding: 13px;
  border: 2px solid #e65100; background: #fff; color: #e65100;
  border-radius: 7px; font-size: 0.92rem; font-weight: 700;
  cursor: pointer; transition: all .15s;
}
.btn-stage:hover  { background: #fff3e0; }
.btn-stage.active { background: #e65100; color: #fff; }

/* ---- Responsive: stack on narrow screens ---- */
@media (max-width: 820px) {
  .main { flex-direction: column; }
  .right-col { width: 100%; }
  .teleop-inner { justify-content: center; }
}
</style>
</head>
<body>

<header>
  <div class="logos-left">
    <img src="/static/Tel-U_Vertikal.png" alt="Tel-U" style="width: 60px; height: auto;" onerror="this.style.display='none'">
    <img src="/static/Logo_INACOS_MONO.png" alt="INACOS" style="width: 50px; height: auto;" onerror="this.style.display='none'">
  </div>
  <div class="title-block">
    <h1>AMR Warehouse Control</h1>
    <p>Integrated Logistics System</p>
  </div>
  <div class="logos-right">
    <img src="/static/ILS_Logo_Transparant.png" alt="ILS" onerror="this.style.display='none'">
  </div>
</header>

<div class="main">

  <!-- LEFT: Map -->
  <div class="map-col">
    <div class="card map-card">
      <img id="map-img" src="/map_snapshot.png" alt="Loading map...">
      <div class="map-ts" id="map-ts"></div>
    </div>
  </div>

  <!-- RIGHT: status + teleop + rack (all stacked) -->
  <div class="right-col">

    <!-- Status card -->
    <div class="card">
      <div class="section-label">Status</div>
      <div class="stat-grid">
        <div>
          <div class="stat-label">Navigation</div>
          <span class="badge badge-idle" id="nav-status">idle</span>
        </div>
        <div>
          <div class="stat-label">Target</div>
          <div class="stat-value" id="nav-target">--</div>
        </div>
        <div class="stat-full">
          <div class="stat-label">Position (x, y, &theta;)</div>
          <div class="stat-value" id="nav-pos" style="font-size:.82rem">--</div>
        </div>
        <div>
          <div class="stat-label">Speed</div>
          <div class="stat-value" id="nav-speed">--</div>
        </div>
        <div style="display:flex;align-items:flex-end">
          <button class="btn-danger" onclick="cancelNav()">Stop Nav</button>
        </div>
      </div>
    </div>

    <!-- Teleop card -->
    <div class="card">
      <div class="section-label">Manual Teleop</div>
      <div class="teleop-inner">
        <div class="dpad-wrap">
          <div class="dpad">
            <button class="dpad-btn dpad-up"    data-dir="fwd"></button>
            <button class="dpad-btn dpad-left"  data-dir="left"></button>
            <button class="dpad-btn dpad-stop"  data-dir="stop">STOP</button>
            <button class="dpad-btn dpad-right" data-dir="right"></button>
            <button class="dpad-btn dpad-down"  data-dir="back"></button>
          </div>
          <div class="dpad-hint">Hold to move, release = stop<br>Keyboard: W A S D or arrows, Space = stop</div>
        </div>
        <div class="teleop-stats">
          <div>
            <div class="stat-label">Linear</div>
            <div class="stat-value" id="joy-linear">0 mm/s</div>
          </div>
          <div>
            <div class="stat-label">Angular</div>
            <div class="stat-value" id="joy-angular">0.00 rad/s</div>
          </div>
          <div class="teleop-note">Max 150 mm/s | 0.7 rad/s<br>Any press cancels active nav goal</div>
        </div>
      </div>
    </div>

    <!-- Rack navigation card -->
    <div class="card">
      <div class="section-label">Rack Navigation</div>
      <div class="top-buttons">
        <button class="btn-home"  onclick="goToRack('Home')"  id="btn-Home">Home</button>
        <button class="btn-stage" onclick="goToRack('Stage')" id="btn-Stage">Stage</button>
      </div>
      <div class="col-headers">
        <div class="col-hdr">C (left)</div>
        <div class="col-hdr">B (mid)</div>
        <div class="col-hdr">A (right)</div>
      </div>
      <div class="rack-grid">
        <button class="rack-btn" onclick="goToRack('C4')" id="btn-C4">C4</button>
        <button class="rack-btn" onclick="goToRack('B4')" id="btn-B4">B4</button>
        <button class="rack-btn" onclick="goToRack('A4')" id="btn-A4">A4</button>

        <button class="rack-btn" onclick="goToRack('C3')" id="btn-C3">C3</button>
        <button class="rack-btn" onclick="goToRack('B3')" id="btn-B3">B3</button>
        <button class="rack-btn" onclick="goToRack('A3')" id="btn-A3">A3</button>

        <button class="rack-btn" onclick="goToRack('C2')" id="btn-C2">C2</button>
        <button class="rack-btn" onclick="goToRack('B2')" id="btn-B2">B2</button>
        <button class="rack-btn" onclick="goToRack('A2')" id="btn-A2">A2</button>

        <button class="rack-btn" onclick="goToRack('C1')" id="btn-C1">C1</button>
        <button class="rack-btn" onclick="goToRack('B1')" id="btn-B1">B1</button>
        <button class="rack-btn" onclick="goToRack('A1')" id="btn-A1">A1</button>
      </div>
    </div>

  </div><!-- end right-col -->
</div>

<script>
// ================================================================
// Status polling
// ================================================================
const BADGE = {
  idle: 'badge-idle', navigating: 'badge-navigating',
  arrived: 'badge-arrived', error: 'badge-error'
};

function allRackBtns() {
  return document.querySelectorAll('.rack-btn, .btn-home, .btn-stage');
}
function clearActive() { allRackBtns().forEach(b => b.classList.remove('active')); }
function setActive(name) {
  clearActive();
  const el = document.getElementById('btn-' + name);
  if (el) el.classList.add('active');
}

function goToRack(name) {
  setActive(name);
  fetch('/go_to_rack', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({rack: name})
  }).then(r => r.json()).then(d => {
    if (d.status === 'error') { clearActive(); alert('Error: ' + d.message); }
  }).catch(e => { clearActive(); alert('Request failed: ' + e); });
}

function cancelNav() {
  fetch('/cancel', {method: 'POST'})
    .then(r => r.json()).then(() => clearActive()).catch(() => {});
}

function updateStatus() {
  fetch('/status').then(r => r.json()).then(d => {
    const badge = document.getElementById('nav-status');
    badge.textContent = d.nav_status;
    badge.className   = 'badge ' + (BADGE[d.nav_status] || 'badge-idle');
    document.getElementById('nav-target').textContent = d.current_target || '--';
    document.getElementById('nav-pos').textContent =
      d.pose.x.toFixed(2) + ',  ' + d.pose.y.toFixed(2) + ',  ' + d.pose.theta.toFixed(2);
    document.getElementById('nav-speed').textContent = d.speed_mmps.toFixed(1) + ' mm/s';
    if (d.nav_status === 'navigating' && d.current_target) setActive(d.current_target);
    else if (d.nav_status === 'idle') clearActive();
  }).catch(() => {});
}

function updateMap() {
  const img = document.getElementById('map-img');
  img.src = '/map_snapshot.png?t=' + Date.now();
  document.getElementById('map-ts').textContent =
    'Updated ' + new Date().toLocaleTimeString();
}

setInterval(updateStatus, 1500);
setInterval(updateMap,    2000);
updateStatus();

// ================================================================
// D-pad teleop: hold a direction (button or key) to move, release = stop.
// Multiple inputs combine (e.g. forward + left). WASD or arrows, Space = stop.
// ================================================================
const MAX_LIN = 0.15;   // m/s  (150 mm/s)
const MAX_ANG = 0.70;   // rad/s

const active = new Set();   // directions currently held: fwd/back/left/right
let cmdTimer = null;

function computeCmd() {
  let lin = 0.0, ang = 0.0;
  if (active.has('fwd'))   lin += MAX_LIN;
  if (active.has('back'))  lin -= MAX_LIN;
  if (active.has('left'))  ang += MAX_ANG;
  if (active.has('right')) ang -= MAX_ANG;
  return {lin, ang};
}

function updateStats() {
  const {lin, ang} = computeCmd();
  document.getElementById('joy-linear').textContent  = (lin * 1000).toFixed(0) + ' mm/s';
  document.getElementById('joy-angular').textContent = ang.toFixed(2) + ' rad/s';
}

function sendCmd() {
  const {lin, ang} = computeCmd();
  fetch('/cmd_vel', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({linear: lin, angular: ang})
  }).catch(() => {});
}

function startSending() {
  if (cmdTimer) return;
  // cancel any active nav goal once, when manual control begins
  fetch('/cancel', {method: 'POST'}).catch(() => {});
  cmdTimer = setInterval(sendCmd, 100);
}

function stopAll() {
  active.clear();
  if (cmdTimer) { clearInterval(cmdTimer); cmdTimer = null; }
  updateStats();
  sendCmd();   // explicit zero velocity
  document.querySelectorAll('.dpad-btn').forEach(b => b.classList.remove('pressed'));
}

function press(dir) {
  if (dir === 'stop') { stopAll(); return; }
  if (active.has(dir)) return;
  active.add(dir);
  startSending();
  updateStats();
}

function release(dir) {
  if (dir === 'stop' || !active.has(dir)) return;
  active.delete(dir);
  updateStats();
  if (active.size === 0) stopAll();
  else sendCmd();
}

function dirBtn(dir) { return document.querySelector('.dpad-btn[data-dir="' + dir + '"]'); }

// --- buttons (mouse + touch unified via pointer events) ---
document.querySelectorAll('.dpad-btn').forEach(btn => {
  const dir = btn.dataset.dir;
  const down = e => { e.preventDefault(); btn.classList.add('pressed'); press(dir); };
  const up   = e => { e.preventDefault(); btn.classList.remove('pressed'); release(dir); };
  btn.addEventListener('pointerdown', down);
  btn.addEventListener('pointerup', up);
  btn.addEventListener('pointerleave', up);
  btn.addEventListener('pointercancel', up);
});

// --- keyboard (WASD + arrow keys, Space = stop) ---
const keyMap = {
  'w': 'fwd',  'arrowup': 'fwd',
  's': 'back', 'arrowdown': 'back',
  'a': 'left', 'arrowleft': 'left',
  'd': 'right','arrowright': 'right',
};
window.addEventListener('keydown', e => {
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;
  const k = e.key.toLowerCase();
  if (k === ' ' || k === 'spacebar') { e.preventDefault(); stopAll(); return; }
  const dir = keyMap[k];
  if (!dir) return;
  e.preventDefault();
  if (e.repeat) return;
  const b = dirBtn(dir); if (b) b.classList.add('pressed');
  press(dir);
});
window.addEventListener('keyup', e => {
  const dir = keyMap[e.key.toLowerCase()];
  if (!dir) return;
  e.preventDefault();
  const b = dirBtn(dir); if (b) b.classList.remove('pressed');
  release(dir);
});

// safety: stop if the window loses focus mid-press
window.addEventListener('blur', stopAll);

function emergencyStop() { stopAll(); cancelNav(); }
</script>
</body>
</html>"""


# ============================================================
# FLASK APP
# ============================================================

app = Flask(__name__, static_folder=str(STATIC_DIR), static_url_path='/static')
_node: NavBridgeNode = None


@app.route('/')
def index():
    return HTML_PAGE


@app.route('/go_to_rack', methods=['POST'])
def go_to_rack():
    data = request.get_json(silent=True) or {}
    rack = data.get('rack', '').strip()
    ok, msg = _node.request_goal(rack)
    if ok:
        x, y, theta = RACK_GOALS[rack]
        return jsonify({'status': 'navigating', 'rack': rack, 'goal': [x, y, theta]})
    return jsonify({'status': 'error', 'message': msg}), 400


@app.route('/follow_waypoints', methods=['POST'])
def follow_waypoints():
    data = request.get_json(silent=True) or {}
    node_names = data.get('nodes', [])
    if not node_names:
        return jsonify({'status': 'error', 'message': 'nodes list kosong'}), 400
    ok, msg = _node.request_follow_waypoints(node_names)
    if ok:
        return jsonify({'status': 'navigating', 'nodes': node_names})
    return jsonify({'status': 'error', 'message': msg}), 400

@app.route('/follow_path', methods=['POST'])
def follow_path():
    data = request.get_json(silent=True) or {}
    path_xy = data.get('path', [])
    ok, msg = _node.request_path(path_xy)
    if ok:
        return jsonify({'status': 'navigating', 'points': len(path_xy)})
    return jsonify({'status': 'error', 'message': msg}), 400

@app.route('/cancel', methods=['POST'])
def cancel():
    _node.request_cancel()
    return jsonify({'status': 'cancelled'})


@app.route('/cmd_vel', methods=['POST'])
def cmd_vel():
    data = request.get_json(silent=True) or {}
    linear  = float(data.get('linear',  0.0))
    angular = float(data.get('angular', 0.0))
    _node.publish_cmd_vel(linear, angular)
    return jsonify({'status': 'ok'})


@app.route('/status')
def status():
    with _node._lock:
        return jsonify({
            'nav_status':     _node.nav_status,
            'current_target': _node.current_target,
            'pose':           dict(_node.pose),
            'speed_mmps':     _node.speed_mmps,
        })


@app.route('/map_snapshot.png')
def map_snapshot():
    png = _node.render_map_png()
    return Response(png, mimetype='image/png',
                    headers={'Cache-Control': 'no-store'})


# ============================================================
# ENTRY POINT
# ============================================================

def main():
    global _node

    rclpy.init()
    _node = NavBridgeNode()

    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(_node), daemon=True)
    spin_thread.start()

    import socket, subprocess

    # Collect all non-loopback IPv4 addresses on this machine.
    # Flask binds 0.0.0.0 so every one of these is reachable.
    all_ips = []
    try:
        raw = subprocess.check_output(['hostname', '-I'], text=True, timeout=3)
        all_ips = [ip for ip in raw.split() if not ip.startswith('127.')]
    except Exception:
        pass

    ts_ip = ''
    try:
        ts_ip = subprocess.check_output(
            ['tailscale', 'ip', '-4'], text=True, timeout=3).strip()
    except Exception:
        pass

    # Separate Tailscale (100.x) from local LAN IPs
    lan_ips = [ip for ip in all_ips if ip != ts_ip]

    print()
    print('=' * 60)
    print('  ILS AMR Control Panel  (ils_gui)')
    print(f'  Local:     http://localhost:5000')
    for ip in lan_ips:
        print(f'  LAN:       http://{ip}:5000')
    if ts_ip:
        print(f'  Tailscale: http://{ts_ip}:5000  <-- Kanal (anywhere)')
    print('=' * 60)
    print()
    print('  Share a LAN address above with your PTL colleague.')
    print('  WARNING: rack buttons and joystick send REAL commands.')
    print('  Keep your hand near the emergency stop.')
    print()

    try:
        app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
