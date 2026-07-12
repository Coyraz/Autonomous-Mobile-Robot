#!/usr/bin/env python3
"""
custom_path_controller.py
-------------------------
Custom local path-tracking controller for the AMR, written from scratch to
REPLACE the Nav2 DWB local planner (dwb_core::DWBLocalPlanner).

WHY THIS EXISTS
  DWB is a sampling/optimisation planner: every cycle it samples ~200 candidate
  trajectories, simulates each 1.5 s ahead, scores them with 7 weighted critics,
  and picks the best. Between near-equal candidates the choice flips cycle to
  cycle, so the commanded heading wiggles and the robot weaves ("randomises").
  On this robot (0.20 m/s cap, 0.55 rad/s rotation deadband) it also stutters
  between "rotate hard" and "go straight".

  This node uses Pure Pursuit instead: a deterministic geometric control law
  (same input -> same output) used by line followers and self-driving cars.
  The result is smooth, repeatable, stiff path tracking.

WHAT IS HAND-CODED HERE (the custom contribution)
  1. Pure Pursuit core      : lookahead point + curvature  gamma = 2*y / L^2
  2. Adaptive lookahead     : short at low speed (tight), longer at speed
  3. Cross-track correction : Stanley-style term so the robot hugs the path
  4. Deadband feedforward   : when a gentle turn needs w < 0.55 rad/s, keep the
                              turn radius correct by lowering linear speed
                              (v = R * w_min) instead of killing the turn
  5. Velocity profiling     : slow on curves and near the goal, plus an
                              in-place final yaw alignment

ARCHITECTURE
  Keeps AMCL + NavFn global planner + costmaps + keepout filter (obstacle
  avoidance still works). This node only replaces the LOCAL controller.
  It exposes the same 'navigate_to_pose' action so ils_gui works unchanged:
    - receives a goal pose
    - asks planner_server for a global path  (ComputePathToPose action)
    - tracks that path with the control law above
    - publishes Twist to 'cmd_vel_nav'  (same topic DWB published to, so the
      existing velocity_smoother + twist_mux + STM32 chain is untouched)

  For quick testing it also accepts a goal on /goal_pose (e.g. from RViz).
"""

import csv
import math
import os
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from std_msgs.msg import Float64MultiArray
from nav2_msgs.action import NavigateToPose, ComputePathToPose

import tf2_ros


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class CustomPathController(Node):

    def __init__(self):
        super().__init__('custom_path_controller')

        # ---- Parameters (all tunable from custom_controller.yaml) ----
        self.declare_parameter('control_frequency', 25.0)
        self.declare_parameter('max_linear_speed', 0.20)    # L298N hardware cap
        self.declare_parameter('min_linear_speed', 0.05)
        self.declare_parameter('max_angular_speed', 0.60)
        self.declare_parameter('min_angular_speed', 0.55)   # motor rotation deadband
        self.declare_parameter('lookahead_min', 0.25)
        self.declare_parameter('lookahead_max', 0.60)
        self.declare_parameter('lookahead_gain', 1.0)       # Ld = gain*v + min
        # use_pd_heading: True -> prof's filtered-PD heading law (Kp/Kd/Tf below).
        # False -> raw Pure Pursuit curvature (w = v * curvature), no PD at all.
        # Defaulted False 2026-07-06: field tests showed severe oscillation with
        # PD regardless of gains/lookahead -- suspected SmacLattice path quality,
        # not the heading law itself. Flip back to True after resolving with prof.
        self.declare_parameter('use_pd_heading', False)
        self.declare_parameter('heading_kp', 2.70)          # PD heading: proportional gain
        self.declare_parameter('heading_kd', 0.44)          # PD heading: derivative gain
        self.declare_parameter('heading_tf', 0.05)          # PD heading: derivative filter time const Tf (s)
        self.declare_parameter('crosstrack_gain', 0.5)      # Stanley-style k
        self.declare_parameter('curve_slowdown', 1.5)       # higher = slower on curves
        self.declare_parameter('goal_xy_tolerance', 0.12)
        self.declare_parameter('goal_yaw_tolerance', 0.15)
        self.declare_parameter('capture_radius', 0.18)      # latch final turn within this
        self.declare_parameter('approach_dist', 0.35)       # start slowing here
        self.declare_parameter('heading_align_thresh', 0.7) # rad: rotate in place first
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel_nav')
        self.declare_parameter('enable_tuning_log', True)  # CSV of tracking error

        # ---- RPP-style obstacle awareness (needs the local costmap) ----
        self.declare_parameter('use_obstacle_costmap', True)
        # In custom mode controller_server (and its local_costmap) is not run,
        # so we read the global costmap from planner_server, which has the
        # obstacle + inflation + keepout layers and runs in every mode.
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('cost_slow_gain', 0.7)        # 0 = off, 1 = strong slowdown near cost
        self.declare_parameter('cost_slow_min_ratio', 0.25)  # never slow below this fraction of v
        self.declare_parameter('collision_cost', 99)         # >= this along the arc => stop (99 = inscribed)
        self.declare_parameter('collision_horizon', 1.2)     # seconds to project the arc forward

        gp = self.get_parameter
        self.hz          = gp('control_frequency').value
        self.v_max       = gp('max_linear_speed').value
        self.v_min       = gp('min_linear_speed').value
        self.w_max       = gp('max_angular_speed').value
        self.w_min       = gp('min_angular_speed').value
        self.ld_min      = gp('lookahead_min').value
        self.ld_max      = gp('lookahead_max').value
        self.ld_gain     = gp('lookahead_gain').value
        self.use_pd_heading = gp('use_pd_heading').value
        self.kp_h        = gp('heading_kp').value
        self.kd_h        = gp('heading_kd').value
        self.tf_h        = gp('heading_tf').value
        self.k_ct        = gp('crosstrack_gain').value
        self._prev_alpha = 0.0   # previous heading error (alpha) for D term
        self._df         = 0.0   # filtered derivative state D_f[k-1]
        # First-order derivative-filter coefficient: alpha_f = exp(-Ts/Tf).
        # Ts = control period (1/control_frequency). Larger Tf -> more smoothing.
        _ts = 1.0 / self.hz
        self._df_alpha   = math.exp(-_ts / self.tf_h) if self.tf_h > 1e-6 else 0.0
        self.curve_slow  = gp('curve_slowdown').value
        self.tol_xy      = gp('goal_xy_tolerance').value
        self.tol_yaw     = gp('goal_yaw_tolerance').value
        self.capture_r   = gp('capture_radius').value
        self.approach    = gp('approach_dist').value
        self.align_th    = gp('heading_align_thresh').value
        self.global_frame = gp('global_frame').value
        self.robot_frame  = gp('robot_frame').value
        self.enable_log   = gp('enable_tuning_log').value
        self.log_dir = os.path.expanduser('~/thesis_data/controller_tuning')

        self.use_costmap   = gp('use_obstacle_costmap').value
        self.cost_slow_k   = gp('cost_slow_gain').value
        self.cost_slow_min = gp('cost_slow_min_ratio').value
        self.collision_cost = gp('collision_cost').value
        self.collision_hz  = gp('collision_horizon').value
        self._costmap = None   # latest OccupancyGrid snapshot

        # measured angular velocity from wheel-encoder speed DIFFERENTIAL
        # (odometry_node.py: vyaw = d_theta/dt from raw ticks, NOT AMCL/EKF-
        # fused) -- the "hasil" (actual response) the professor wants checked
        # against the PP setpoint (alpha), same units (rad, rad/s), to verify
        # the PD/curvature heading law is actually correcting what it should.
        self._w_odom = 0.0
        self._dbg_alpha = 0.0

        # ---- goal preemption ----
        # 2026-07-07: rclpy's ActionServer defaults to ACCEPTING every goal
        # and running EACH one's execute_callback concurrently (one thread
        # per goal, via MultiThreadedExecutor) unless told otherwise. With no
        # goal/handle_accepted callback here, sending a second goal before
        # the first finishes (e.g. double-clicking ils_gui, or FollowWaypoints
        # racing a manual go_to_rack) started a SECOND run_path() loop that
        # published cmd_vel_nav concurrently with the first -- confirmed live
        # via two tuning-log CSVs growing at the same timestamp with
        # different trajectories. This also explained "Stop Nav doesn't
        # work": cancelling only touched whichever ONE goal_handle ils_gui
        # happened to be tracking, leaving the other to drive the robot
        # forever. Fix: a monotonic generation counter -- each new goal
        # bumps it and stores its own number; run_path()'s loop bails out as
        # soon as it sees a newer generation has taken over, regardless of
        # the action framework's own cancel state.
        self._goal_generation = 0
        self._goal_gen_lock = threading.Lock()

        # ---- TF (map -> base_link pose) ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- I/O ----
        cb = ReentrantCallbackGroup()
        self.cmd_pub = self.create_publisher(Twist, gp('cmd_vel_topic').value, 10)

        # client to the existing NavFn global planner
        self.plan_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose', callback_group=cb)

        # same action name bt_navigator used, so ils_gui is unchanged
        self.nav_server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose',
            execute_callback=self.execute_navigate,
            # Without this, rclpy REJECTS all cancels by default, so ils_gui's
            # "Stop Nav" and the teleop override could not stop an active goal.
            cancel_callback=lambda goal_handle: CancelResponse.ACCEPT,
            callback_group=cb)

        # quick-test entry: send a PoseStamped on /goal_pose (e.g. from RViz).
        # Reentrant group so its blocking plan+track runs on its own executor
        # thread, like the action server, instead of stalling the default group.
        self.create_subscription(PoseStamped, '/goal_pose', self.cb_goal_pose, 1,
                                 callback_group=cb)

        # local costmap, for the RPP-style obstacle slowdown + collision stop
        if self.use_costmap:
            self.create_subscription(
                OccupancyGrid, gp('costmap_topic').value,
                self._cb_costmap, 1, callback_group=cb)

        # measured angular velocity from wheel encoders (see _w_odom above)
        self.create_subscription(Odometry, '/odom', self._cb_odom, 20,
                                 callback_group=cb)

        # debug: [alpha_rad, w_heading_radps, w_cmd_radps, w_odom_radps] each
        # control tick -- lets an external recorder verify the PP setpoint
        # (alpha) against the actual wheel-encoder heading response (w_odom)
        # in matching units, per the professor's requested check.
        self.debug_pub = self.create_publisher(
            Float64MultiArray, 'heading_debug', 10)

        self.get_logger().info(
            'custom_path_controller ready. Replacing DWB. '
            'Publishing to "%s", serving navigate_to_pose.'
            % gp('cmd_vel_topic').value)

    # ----------------------------------------------------------------- helpers
    def get_robot_pose(self):
        """Return (x, y, yaw) of base_link in the map frame, or None."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn('TF lookup failed: %s' % str(e), throttle_duration_sec=2.0)
            return None
        x = t.transform.translation.x
        y = t.transform.translation.y
        yaw = yaw_from_quat(t.transform.rotation)
        return (x, y, yaw)

    # ------------------------------------------------- obstacle awareness (RPP)
    def _cb_odom(self, msg):
        # angular.z here is odometry_node.py's vyaw = d_theta/dt, computed
        # directly from (d_right_ticks - d_left_ticks)/wheel_base/dt -- the
        # wheel-speed-differential heading rate, independent of AMCL/EKF.
        self._w_odom = msg.twist.twist.angular.z

    def _cb_costmap(self, msg):
        # Store the latest local costmap. OccupancyGrid cost values are
        # 0 (free) .. 99 (inscribed) .. 100 (lethal), -1 unknown. Reference
        # assignment is atomic under the GIL, so no lock is needed for reads.
        self._costmap = msg

    def _cost_at(self, wx, wy):
        """Cost at a world point, or None if no costmap / outside its bounds."""
        cm = self._costmap
        if cm is None:
            return None
        info = cm.info
        mx = int((wx - info.origin.position.x) / info.resolution)
        my = int((wy - info.origin.position.y) / info.resolution)
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None
        return cm.data[my * info.width + mx]

    def _arc_blocked(self, rx, ry, ryaw, v, w):
        """Project the (v, w) arc forward collision_horizon seconds; True if it
        crosses a cell at or above collision_cost (a real obstacle)."""
        if v <= 0.0 or self._costmap is None:
            return False
        dt = 0.1
        x, y, th = rx, ry, ryaw
        for _ in range(int(self.collision_hz / dt)):
            x += v * math.cos(th) * dt
            y += v * math.sin(th) * dt
            th += w * dt
            c = self._cost_at(x, y)
            if c is not None and c >= self.collision_cost:
                return True
        return False

    def _cost_slowdown(self, rx, ry, v):
        """RPP-style: slow down when the robot sits in higher-cost cells (near
        obstacles). At the centre of a clear aisle the cost is 0, so this does
        nothing there and only bites when the robot drifts toward a wall."""
        c = self._cost_at(rx, ry)
        if c is None or c <= 0:
            return v
        factor = max(self.cost_slow_min, 1.0 - self.cost_slow_k * (c / 100.0))
        return v * factor

    def _wait_future(self, future, timeout_sec=10.0):
        """
        Wait for a future WITHOUT re-spinning the node. This callback runs in
        its own executor thread (ReentrantCallbackGroup), and the main
        MultiThreadedExecutor keeps servicing the action-client callbacks that
        complete the future. We must NOT call spin_until_future_complete here:
        the node is already owned by the main executor, so re-spinning blocks
        forever. We just poll until done.
        """
        deadline = time.time() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.time() > deadline:
                return None
            time.sleep(0.02)
        return future.result()

    def request_global_path(self, goal_pose):
        """Ask planner_server (NavFn) for a global path to goal_pose. Blocking."""
        if not self.plan_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('planner_server (compute_path_to_pose) not available')
            return None
        req = ComputePathToPose.Goal()
        req.goal = goal_pose
        req.use_start = False  # plan from current robot pose
        gh = self._wait_future(self.plan_client.send_goal_async(req))
        if gh is None or not gh.accepted:
            self.get_logger().error('global planner rejected the goal (or timed out)')
            return None
        result_wrap = self._wait_future(gh.get_result_async())
        if result_wrap is None:
            self.get_logger().error('global planner result timed out')
            return None
        result = result_wrap.result
        if result is None or len(result.path.poses) == 0:
            self.get_logger().error('global planner returned an empty path')
            return None
        return result.path

    @staticmethod
    def path_xy(path):
        return [(p.pose.position.x, p.pose.position.y) for p in path.poses]

    def closest_index(self, pts, rx, ry, start_idx):
        """Index of the path point closest to the robot, searching forward only."""
        best_i = start_idx
        best_d = float('inf')
        # small backward window guards against a bad jump; mostly search forward
        lo = max(0, start_idx - 2)
        for i in range(lo, len(pts)):
            d = (pts[i][0] - rx) ** 2 + (pts[i][1] - ry) ** 2
            if d < best_d:
                best_d = d
                best_i = i
        return best_i

    def lookahead_point(self, pts, idx, rx, ry, Ld):
        """Walk forward from idx until cumulative distance from robot >= Ld."""
        for i in range(idx, len(pts)):
            d = math.hypot(pts[i][0] - rx, pts[i][1] - ry)
            if d >= Ld:
                return pts[i], i
        return pts[-1], len(pts) - 1

    # ------------------------------------------------------------ control law
    def compute_cmd(self, pose, pts, goal_xy, goal_yaw):
        """
        Core controller. Returns (Twist, done_flag).
        Implements Pure Pursuit + cross-track correction + deadband feedforward
        + velocity profiling + final in-place yaw alignment.
        """
        rx, ry, ryaw = pose
        cmd = Twist()
        self._dbg_ct = 0.0   # cross-track error, for the tuning log
        self._dbg_he = 0.0   # heading error, for the tuning log

        dist_to_goal = math.hypot(goal_xy[0] - rx, goal_xy[1] - ry)

        # ---- Final-approach latch (hysteresis) ----
        # Once we capture the goal we COMMIT to turning in place to the goal
        # heading and never drop back to tracking. This is what kills the
        # 360 spin: without it, a slight overshoot puts the goal behind the
        # robot, the heading-align below spins around to chase it, the robot
        # drives past again, and it loops forever.
        if dist_to_goal <= self.capture_r:
            self._final_align = True

        if self._final_align:
            yaw_err = normalize_angle(goal_yaw - ryaw)
            if abs(yaw_err) <= self.tol_yaw:
                return cmd, True  # done, zero twist, latched stop
            cmd.linear.x = 0.0
            cmd.angular.z = math.copysign(self.w_min, yaw_err)
            return cmd, False

        # ---- closest point + adaptive lookahead ----
        self._last_idx = self.closest_index(pts, rx, ry, getattr(self, '_last_idx', 0))
        i0 = self._last_idx
        Ld = max(self.ld_min, min(self.ld_max, self.ld_gain * self.v_max + self.ld_min))
        (lx, ly), _ = self.lookahead_point(pts, i0, rx, ry, Ld)

        # lookahead point in the robot frame
        dx = lx - rx
        dy = ly - ry
        x_r = math.cos(ryaw) * dx + math.sin(ryaw) * dy
        y_r = -math.sin(ryaw) * dx + math.cos(ryaw) * dy
        L = math.hypot(x_r, y_r)
        if L < 1e-3:
            L = 1e-3

        # ---- heading badly off -> rotate in place to face the path ----
        # Only when FAR from the goal (start-of-path misalignment). Near the
        # goal we never do this, so an overshoot cannot trigger a spin; the
        # capture latch above owns the endgame instead.
        path_heading = math.atan2(dy, dx)
        head_err = normalize_angle(path_heading - ryaw)
        if dist_to_goal > self.approach and abs(head_err) > self.align_th:
            cmd.linear.x = 0.0
            cmd.angular.z = math.copysign(max(self.w_min, abs(head_err)), head_err)
            cmd.angular.z = max(-self.w_max, min(self.w_max, cmd.angular.z))
            return cmd, False

        # ---- PP heading setpoint: angle to lookahead point in robot frame ----
        # This is what the professor calls the "set point" from Pure Pursuit.
        # alpha > 0: lookahead is to the left; alpha < 0: to the right.
        alpha = math.atan2(y_r, x_r)

        # ---- velocity profiling: slow on curves and near the goal ----
        # (moved above the heading law so the curvature fallback below can
        # scale by the actual profiled speed, matching its original design)
        v = self.v_max / (1.0 + self.curve_slow * abs(alpha))
        if dist_to_goal < self.approach:
            v *= max(0.25, dist_to_goal / self.approach)
        v = max(self.v_min, min(self.v_max, v))

        # ---- RPP-style cost-regulated slowdown (slow near obstacles) ----
        if self.use_costmap:
            v = self._cost_slowdown(rx, ry, v)

        if self.use_pd_heading:
            # ---- PD heading controller with FILTERED derivative (prof's design) ----
            #   C_theta(s) = Kp + Kd * s/(Tf*s + 1)
            # PP supplies the setpoint (alpha); the D term damps theta oscillation,
            # and the first-order filter (Tf) rejects high-frequency noise on it.
            # Discrete filter:  D_f[k] = a*D_f[k-1] + (1-a)*D_raw[k],  a = exp(-Ts/Tf)
            d_raw = (alpha - self._prev_alpha) * self.hz
            self._prev_alpha = alpha
            self._df = self._df_alpha * self._df + (1.0 - self._df_alpha) * d_raw
            w_heading = self.kp_h * alpha + self.kd_h * self._df
        else:
            # ---- FALLBACK: raw Pure Pursuit curvature (no PD) ----
            # Temporarily reverted 2026-07-06: field tests showed severe theta
            # oscillation (>100 deg swings) with the PD law regardless of Kp/Kd
            # tuning or lookahead distance -- symptom points to the SmacLattice
            # path itself (generic, non-robot-specific primitives) rather than
            # the heading law. Flip 'use_pd_heading' back to true once this is
            # resolved with the professor. See test11.md for the diagnostic data.
            curvature = 2.0 * y_r / (L * L)
            w_heading = v * curvature

        # ---- cross-track correction (Stanley-style), tangent at closest point ----
        if i0 + 1 < len(pts):
            tx = pts[i0 + 1][0] - pts[i0][0]
            ty = pts[i0 + 1][1] - pts[i0][1]
            tang = math.atan2(ty, tx)
        else:
            tang = ryaw
        # signed lateral offset of robot from the path point (left positive)
        ex = pts[i0][0] - rx
        ey = pts[i0][1] - ry
        e_ct = -math.sin(tang) * ex + math.cos(tang) * ey

        # angular = heading term (PD or curvature) + cross-track correction
        w = w_heading + self.k_ct * e_ct

        # ---- deadband handling (the fix for straight-line oscillation) ----
        # We are in the MOVING branch here (pure in-place rotation is handled by
        # the align / final-yaw phases above, which already command w_min).
        # While translating at speed v, a gentle curve only needs a small
        # wheel-speed DIFFERENTIAL, which is achievable because both wheels
        # already spin well above the motor deadband. The 0.55 rad/s minimum
        # only applies to pivoting from near-stationary. So we must NOT bump
        # small w up to w_min here -- doing that forced 0.55 rad/s corrections
        # on a near-straight path and caused bang-bang oscillation. We only
        # null out negligible commands to avoid chattering.
        if abs(w) < 0.02:
            w = 0.0
        w = max(-self.w_max, min(self.w_max, w))

        # ---- RPP-style collision stop: if the planned arc hits an obstacle in
        #      the next collision_horizon seconds, hold still and wait. Checks
        #      only real obstacles (>= collision_cost), so inflation in the tight
        #      aisle does not false-stop us. Goal is NOT aborted; we resume when
        #      the obstacle clears. ----
        if self.use_costmap and self._arc_blocked(rx, ry, ryaw, v, w):
            self.get_logger().warn('Obstacle ahead, holding.',
                                   throttle_duration_sec=1.0)
            self._dbg_ct = e_ct
            self._dbg_he = head_err
            return Twist(), False

        # stash for the tuning log
        self._dbg_ct = e_ct
        self._dbg_he = head_err
        self._dbg_alpha = alpha

        # setpoint (alpha) vs actual wheel-encoder heading response (w_odom),
        # both in rad / rad/s -- see the professor's requested cross-check.
        dbg = Float64MultiArray()
        dbg.data = [alpha, w_heading, w, self._w_odom]
        self.debug_pub.publish(dbg)

        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        return cmd, False

    # ----------------------------------------------------- action / run loop
    def run_path(self, path, goal_pose, action_handle=None, my_gen=None):
        """Drive along path until the goal is reached. Returns True on success."""
        pts = self.path_xy(path)
        goal_xy = (goal_pose.pose.position.x, goal_pose.pose.position.y)
        goal_yaw = yaw_from_quat(goal_pose.pose.orientation)
        self._last_idx = 0
        self._final_align = False
        self._prev_alpha = 0.0   # reset D term history for each new goal
        self._df         = 0.0   # reset filtered-derivative state

        logger = self._open_log()
        t0 = time.time()
        # Plain time.sleep() instead of Node.create_rate(): rclpy's Rate
        # object is known to cause excessive executor wake-ups (near-100%
        # single-core CPU even at a modest 25Hz) when used with
        # MultiThreadedExecutor -- observed 2026-07-07 alongside high system
        # load/CPU temp, suspected of starving AMCL enough to cause delayed
        # corrections ("hang then jump" reports). A plain sleep is a
        # low-risk drop-in fix; drifts slightly under load (no compensation
        # for compute_cmd() time) but that's an acceptable trade for a
        # control loop that isn't hard-real-time.
        period = 1.0 / self.hz
        self._last_run_preempted = False
        try:
            while rclpy.ok():
                if my_gen is not None and my_gen != self._goal_generation:
                    # a newer goal has taken over this node -- yield the
                    # robot to it instead of continuing to fight for cmd_vel
                    self.stop()
                    self._last_run_preempted = True
                    return False
                if action_handle is not None and action_handle.is_cancel_requested:
                    self.stop()
                    return False
                pose = self.get_robot_pose()
                if pose is None:
                    self.stop()
                    time.sleep(period)
                    continue
                cmd, done = self.compute_cmd(pose, pts, goal_xy, goal_yaw)
                self.cmd_pub.publish(cmd)
                if logger is not None:
                    logger.writerow([
                        f'{time.time() - t0:.3f}',
                        f'{pose[0]:.4f}', f'{pose[1]:.4f}', f'{pose[2]:.4f}',
                        f'{self._dbg_ct:.4f}', f'{self._dbg_he:.4f}',
                        f'{cmd.linear.x:.4f}', f'{cmd.angular.z:.4f}',
                        f'{self._dbg_alpha:.4f}', f'{self._w_odom:.4f}'
                    ])
                if done:
                    self.stop()
                    return True
                time.sleep(period)
        finally:
            self._close_log()
        self.stop()
        return False

    def _open_log(self):
        """Open a per-run CSV of tracking error for offline gain analysis."""
        if not self.enable_log:
            self._log_file = None
            return None
        os.makedirs(self.log_dir, exist_ok=True)
        path = os.path.join(
            self.log_dir, 'run_%s.csv' % datetime.now().strftime('%Y%m%d_%H%M%S'))
        self._log_file = open(path, 'w', newline='')
        w = csv.writer(self._log_file)
        w.writerow(['t_s', 'x_m', 'y_m', 'yaw_rad',
                    'cross_track_err_m', 'heading_err_rad',
                    'v_cmd_mps', 'w_cmd_radps',
                    'alpha_rad', 'w_odom_radps'])
        self.get_logger().info('Tuning log: %s' % path)
        return w

    def _close_log(self):
        if getattr(self, '_log_file', None) is not None:
            self._log_file.close()
            self._log_file = None

    def execute_navigate(self, goal_handle):
        """navigate_to_pose action: plan a global path, then track it."""
        # Claim this generation BEFORE planning starts, so a second goal
        # arriving while we're still waiting on request_global_path() also
        # correctly preempts us (not just once run_path()'s loop begins).
        with self._goal_gen_lock:
            self._goal_generation += 1
            my_gen = self._goal_generation

        goal_pose = goal_handle.request.pose
        self.get_logger().info('Goal received. Requesting global path...')
        path = self.request_global_path(goal_pose)
        if my_gen != self._goal_generation:
            # preempted while planning -- don't even start driving
            goal_handle.abort()
            return NavigateToPose.Result()
        if path is None:
            goal_handle.abort()
            return NavigateToPose.Result()
        self.get_logger().info('Path has %d points. Tracking with Pure Pursuit.'
                               % len(path.poses))
        ok = self.run_path(path, goal_pose, action_handle=goal_handle, my_gen=my_gen)
        if ok:
            goal_handle.succeed()
            self.get_logger().info('Goal reached.')
        else:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                # covers both a real failure and being preempted by a newer
                # goal (self._last_run_preempted) -- either way this
                # specific goal did not complete
                goal_handle.abort()
        return NavigateToPose.Result()

    def cb_goal_pose(self, msg):
        """Quick test path: /goal_pose -> plan -> track (blocks the executor)."""
        with self._goal_gen_lock:
            self._goal_generation += 1
            my_gen = self._goal_generation
        self.get_logger().info('/goal_pose received (quick test).')
        path = self.request_global_path(msg)
        if path is not None and my_gen == self._goal_generation:
            self.run_path(path, msg, my_gen=my_gen)

    def stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = CustomPathController()
    # multi-threaded so the action server + planner client + control rate coexist
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
