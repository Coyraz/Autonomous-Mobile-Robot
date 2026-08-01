#!/usr/bin/env python3
"""
rosbag_ground_truth.py  --  Ground truth marker for rosbag-based data collection
----------------------------------------------------------------------------------
Publishes ground truth events to /ground_truth_event (std_msgs/String with JSON).
Run this WHILE ros2 bag record is running -- the marker messages get captured
in the bag alongside all sensor data.

WORKFLOW (example for Test E):
  Terminal 1: ros2 launch robot_bringup localization_test.launch.py mode:=A
  Terminal 2: ros2 bag record -o test_e_mode_a <topics...>
  Terminal 3: ros2 run robot_bringup teleop_keyboard
  Terminal 4: python3 rosbag_ground_truth.py --test localization --mode A --reps 5

At each reference point, drive the robot there and press Enter.
The marker is published and recorded in the bag.

After recording, analyze with:
  python3 analyze_localization_bag.py test_e_mode_a/

Usage:
  python3 rosbag_ground_truth.py --test localization --mode A --reps 5
  python3 rosbag_ground_truth.py --test trajectory --mode C
  python3 rosbag_ground_truth.py --test navigation --mode C
  python3 rosbag_ground_truth.py --test obstacle --mode C
  python3 rosbag_ground_truth.py --test map_geometry --reps 1
"""

import argparse
import json
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.srv import ClearEntireCostmap

import amr_test_utils as U


# ---------------------------------------------------------------- reference data
# All waypoints come from the shared WAREHOUSE_WAYPOINTS in amr_test_utils.py.
# These are tape-measured physical coordinates relative to Home = (0,0).
W = U.WAREHOUSE_WAYPOINTS

# Localization test (Test E): 6 well-distributed points across aisles.
LOCALIZATION_POINTS = [
    ('Home', *W['Home']),
    ('A1',   *W['A1']),
    ('A4',   *W['A4']),
    ('B2',   *W['B2']),
    ('B4',   *W['B4']),
    ('C3',   *W['C3']),
]

TRAJECTORY_SCENARIOS = [
    ('stationary',    '0m',     'Hold still for 30 seconds (do NOT move)'),
    ('straight_3m',   '3m',     'Drive straight 3 meters, stop at tape mark'),
    ('rotation_180',  '180deg', 'Rotate 180 degrees in place, stop'),
    ('rotation_360',  '360deg', 'Rotate 360 degrees in place, stop'),
    ('return_origin', 'loop',   'Drive a loop (Home→B4→C4→C1→Home), stop at start mark'),
]

# Map geometry test (Test C): all 14 points, TAPE ground truth -- NOT
# WAREHOUSE_WAYPOINTS (that dict holds AMCL's own readings, re-centered for
# Nav2 goal-plannability, not a tape measurement). This is the same
# TITIK_KOORDINAT table canonically stored in test3.md and
# plot_test_g_trajectory.py -- confirmed 2026-07-24 as the real tape/design
# ground truth. Do not delete/regenerate; see reference_titik_koordinat.md.
MAP_GEOMETRY_POINTS = [
    ('Home',  0.0,  0.0),
    ('Stage', 3.5,  0.5),
    ('A1',    4.0, -8.5),
    ('A2',    4.0, -7.0),
    ('A3',    4.0, -4.5),
    ('A4',    4.0, -3.0),
    ('B1',    1.5, -8.5),
    ('B2',    1.5, -7.0),
    ('B3',    1.5, -4.5),
    ('B4',    1.5, -3.0),
    ('C1',   -1.0, -8.5),
    ('C2',   -1.0, -7.0),
    ('C3',   -1.0, -4.5),
    ('C4',   -1.0, -3.0),
]

# Navigation test (Test G): actual destination points only (racks + Home +
# Stage). 2026-07-07: the X1_A/X1_B/.../XA_T-junc cross-section points were
# REMOVED from this list -- those are only pass-through waypoints used by
# ils_gui's FollowWaypoints bridge for the labmate's system, not real
# destinations, and testing navigation *to* them (as a final stop/goal
# rather than a transit point) was scope creep beyond what Test G is meant
# to measure. Kept them defined in WAREHOUSE_WAYPOINTS since ils_gui still
# uses them elsewhere.
NAVIGATION_TARGETS = [
    # Rack positions
    ('A1',   *W['A1']),
    ('A2',   *W['A2']),
    ('A3',   *W['A3']),
    ('A4',   *W['A4']),
    ('B1',   *W['B1']),
    ('B2',   *W['B2']),
    ('B3',   *W['B3']),
    ('B4',   *W['B4']),
    ('C1',   *W['C1']),
    ('C2',   *W['C2']),
    ('C3',   *W['C3']),
    ('C4',   *W['C4']),
    # Staging / home
    ('Stage', *W['Stage']),
    ('Home',  *W['Home']),
]


class GroundTruthMarker(Node):

    def __init__(self):
        super().__init__('ground_truth_marker')
        self.pub = self.create_publisher(String, '/ground_truth_event', 10)
        self._clear_costmap_cli = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.get_logger().info('Ground truth marker ready. Publishing to /ground_truth_event')

    def publish_event(self, event_type, **kwargs):
        """Publish a ground truth event as JSON string."""
        payload = {'event': event_type, 'ros_time': self.get_clock().now().nanoseconds}
        payload.update(kwargs)
        msg = String()
        msg.data = json.dumps(payload)
        self.pub.publish(msg)

    def clear_global_costmap(self, timeout_s=3.0):
        """Reset the global costmap's obstacle_layer before a new nav goal.

        Stale obstacle marks persist forever once the robot moves out of
        raytrace_max_range (3.0m) of where it last saw them -- the
        obstacle_layer only clears a cell when a fresh scan actually
        raytraces through it again. In Test G the robot revisits the same
        aisles from different approach angles/distances rep after rep, so a
        transient obstacle seen once (e.g. an operator standing nearby) can
        permanently block an otherwise-clear goal later in the same run.
        Safe to call here because the robot is stationary between goals
        (waiting on the s/f/t prompt), so this can't erase a real obstacle
        the controller needs mid-motion.
        """
        if not self._clear_costmap_cli.wait_for_service(timeout_sec=timeout_s):
            print("    WARNING: clear_entirely_global_costmap service not "
                  "available, skipping costmap clear.")
            return False
        future = self._clear_costmap_cli.call_async(ClearEntireCostmap.Request())
        # main() runs a background daemon thread that continuously calls
        # rclpy.spin_once() on this same node (so input() here in the main
        # thread doesn't block service/topic callbacks). spin_until_future_
        # complete() would try to enter its OWN spin on top of that already-
        # spinning node -- rclpy forbids spinning the same node from two
        # places at once ("Executor is already spinning"). Just poll the
        # future instead; the background thread is what actually completes
        # it. Same pattern as custom_path_controller.py's _wait_future().
        deadline = time.time() + timeout_s
        while rclpy.ok() and not future.done():
            if time.time() > deadline:
                print("    WARNING: clear_entirely_global_costmap timed out.")
                return False
            time.sleep(0.02)
        return future.done()


def spin_thread(node, stop_event):
    while rclpy.ok() and not stop_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.05)


# ---------------------------------------------------------------- test runners

def run_localization(node, mode, reps):
    """Test E: drive to reference points, mark each one."""
    pts = LOCALIZATION_POINTS
    print("\n" + "=" * 66)
    print(f" TEST E  -  LOCALIZATION (rosbag)  Mode {mode}  Reps {reps}")
    print("=" * 66)
    print("\nReference points:")
    for name, x, y in pts:
        print(f"  {name:6s}  ({x:+6.1f}, {y:+6.1f})")

    print(f"\nMake sure ros2 bag record is running in another terminal!")
    print(f"Topics to record:")
    print(f"  ros2 bag record -o test_e_mode_{mode.lower()} "
          f"/odom_raw /odom /tf /tf_static /amcl_pose "
          f"/imu/data_raw /scan_restamped /ground_truth_event")
    input("\nPress Enter when bag recording is started...")

    node.publish_event('test_start', test='localization', mode=mode, reps=reps)

    for rep in range(1, reps + 1):
        print(f"\n--- REP {rep} / {reps} ---")
        for name, gt_x, gt_y in pts:
            input(f"  Drive to {name} ({gt_x:+.1f}, {gt_y:+.1f}), "
                  f"align to tape, press Enter...")
            node.publish_event('point_reached',
                               test='localization', mode=mode,
                               rep=rep, point=name,
                               gt_x=gt_x, gt_y=gt_y)
            print(f"    Marker published for {name} (rep {rep})")
            time.sleep(0.5)

    node.publish_event('test_end', test='localization', mode=mode)
    print(f"\nDone! Stop bag recording (Ctrl+C in the bag terminal).")
    print(f"Then run: python3 analyze_localization_bag.py test_e_mode_{mode.lower()}/")


def run_map_geometry(node, mode, reps):
    """Test C: drive to all 14 tape-measured reference points, mark each one.

    mode is unused (kept for TEST_RUNNERS' uniform signature) -- Test C
    doesn't have A/B/C modes, it's a single map-geometry validation pass.
    reps defaults to 1 (a single measurement per point, same as the
    original 2026-06-22 Test C run) but can be raised for repeatability data.
    """
    pts = MAP_GEOMETRY_POINTS
    print("\n" + "=" * 66)
    print(f" TEST C  -  MAP GEOMETRY (rosbag)  Reps {reps}")
    print("=" * 66)
    print("\nReference points (TAPE ground truth, TITIK_KOORDINAT):")
    for name, x, y in pts:
        print(f"  {name:6s}  ({x:+6.1f}, {y:+6.1f})")

    print(f"\nMake sure ros2 bag record is running in another terminal!")
    print(f"Topics to record:")
    print(f"  ros2 bag record -o test_c_map_geometry "
          f"/odom_raw /odom /tf /tf_static /amcl_pose /ground_truth_event")
    input("\nPress Enter when bag recording is started...")

    node.publish_event('test_start', test='map_geometry', reps=reps)

    for rep in range(1, reps + 1):
        print(f"\n--- REP {rep} / {reps} ---")
        for name, gt_x, gt_y in pts:
            input(f"  Drive to {name} ({gt_x:+.1f}, {gt_y:+.1f}), "
                  f"align to tape, press Enter...")
            # 2026-07-24: also capture a manual tape-measured offset between
            # the robot's actual center and the tape mark, taken at the same
            # moment. This separates two error sources that the AMCL-vs-tape
            # comparison alone conflates: (1) how precisely the robot was
            # physically parked on the mark, vs (2) how far AMCL's own
            # reading is from where the robot actually is. Optional -- press
            # Enter to skip if not measuring this round.
            raw = input(f"    Ukur jarak pusat robot ke marka {name} dengan "
                        f"meteran (cm), atau Enter untuk skip: ").strip()
            manual_offset_cm = None
            if raw:
                try:
                    manual_offset_cm = float(raw)
                except ValueError:
                    print(f"    (input '{raw}' bukan angka, dilewati)")
            node.publish_event('point_reached',
                               test='map_geometry', mode='C',
                               rep=rep, point=name,
                               gt_x=gt_x, gt_y=gt_y,
                               manual_offset_cm=manual_offset_cm)
            print(f"    Marker published for {name} (rep {rep})"
                  + (f" -- manual offset {manual_offset_cm} cm" if manual_offset_cm is not None else ""))
            time.sleep(0.5)

    node.publish_event('test_end', test='map_geometry')
    print(f"\nDone! Stop bag recording (Ctrl+C in the bag terminal).")
    print(f"Then run: python3 analyze_map_geometry_bag.py test_c_map_geometry/")


def run_trajectory(node, mode, reps):
    """Test F: trajectory drift scenarios."""
    print("\n" + "=" * 66)
    print(f" TEST F  -  TRAJECTORY DRIFT (rosbag)  Mode {mode}  Reps {reps}")
    print("=" * 66)
    print(f"\nMake sure ros2 bag record is running!")
    print(f"  ros2 bag record -o test_f_mode_{mode.lower()} "
          f"/odom_raw /odom /tf /tf_static /amcl_pose "
          f"/imu/data_raw /cmd_vel /ground_truth_event")
    input("\nPress Enter when bag recording is started...")

    node.publish_event('test_start', test='trajectory', mode=mode, reps=reps)

    for rep in range(1, reps + 1):
        print(f"\n--- REP {rep} / {reps} ---")
        for scenario, label, desc in TRAJECTORY_SCENARIOS:
            input(f"\n  Place robot at start position for: {desc}")
            node.publish_event('trajectory_start',
                               test='trajectory', mode=mode,
                               rep=rep, scenario=scenario, label=label)
            print(f"    START marker published. Execute the trajectory now.")
            input(f"  Press Enter when trajectory is COMPLETE...")
            node.publish_event('trajectory_end',
                               test='trajectory', mode=mode,
                               rep=rep, scenario=scenario, label=label)
            print(f"    END marker published for {scenario}")
            time.sleep(0.5)

    node.publish_event('test_end', test='trajectory', mode=mode)
    print(f"\nDone! Stop bag recording.")


def run_navigation(node, mode, reps):
    """Test G: point-to-point navigation via Nav2."""
    print("\n" + "=" * 66)
    print(f" TEST G  -  NAVIGATION (rosbag)  Mode {mode}  Reps {reps}")
    print("=" * 66)
    print(f"\nMake sure ros2 bag record is running!")
    print(f"  ros2 bag record -o test_g "
          f"/odom_raw /odom /tf /tf_static /amcl_pose "
          f"/cmd_vel /plan /ground_truth_event")
    input("\nPress Enter when bag recording is started...")

    node.publish_event('test_start', test='navigation', mode=mode, reps=reps)

    for rep in range(1, reps + 1):
        print(f"\n--- REP {rep} / {reps} ---")
        for name, gt_x, gt_y in NAVIGATION_TARGETS:
            print(f"\n  Send Nav2 goal to {name} ({gt_x:+.1f}, {gt_y:+.1f})")
            print(f"    Clearing global costmap (stale obstacle marks)...")
            node.clear_global_costmap()
            node.publish_event('nav_goal_sent',
                               test='navigation', mode=mode,
                               rep=rep, target=name,
                               gt_x=gt_x, gt_y=gt_y)
            result = input(f"  Result? (s=success, f=fail, t=timeout): ").strip().lower()
            result_map = {'s': 'success', 'f': 'fail', 't': 'timeout'}

            # Manual tape-measured stop error (cm), independent of the
            # AMCL-vs-nominal-target error computed later in
            # analyze_navigation_bag.py. This is a physical measurement of
            # how far the robot ACTUALLY stopped from the taped-mark target
            # on the floor -- a cross-check against the map-frame estimate,
            # since AMCL itself can be off if the map is warped (see test3.md).
            tape_raw = input(f"  Jarak error terukur pakai meteran (cm, "
                              f"kosongkan jika tidak diukur): ").strip()
            try:
                tape_error_cm = float(tape_raw) if tape_raw else None
            except ValueError:
                print(f"    WARNING: '{tape_raw}' bukan angka, dilewati.")
                tape_error_cm = None

            node.publish_event('nav_goal_result',
                               test='navigation', mode=mode,
                               rep=rep, target=name,
                               gt_x=gt_x, gt_y=gt_y,
                               result=result_map.get(result, result),
                               tape_error_cm=tape_error_cm)
            print(f"    Result marker: {result_map.get(result, result)}"
                  + (f"  (tape error: {tape_error_cm} cm)" if tape_error_cm is not None else ""))
            time.sleep(0.5)

    node.publish_event('test_end', test='navigation', mode=mode)
    print(f"\nDone! Stop bag recording.")


def run_obstacle(node, mode, reps):
    """Test H: static obstacle stop-and-resume.

    custom_path_controller does NOT replan around obstacles -- it projects
    the robot's arc forward (collision_horizon) against /global_costmap and,
    if blocked, commands zero velocity and HOLDS (no local_costmap/
    controller_server in this mode). It resumes automatically once the
    costmap along that arc clears. This protocol measures exactly that:
    stop distance (safe or collision), and whether/how it resumes -- NOT
    route-around avoidance, which this system doesn't do.
    """
    print("\n" + "=" * 66)
    print(f" TEST H  -  OBSTACLE STOP-AND-RESUME (rosbag)  Mode {mode}  Reps {reps}")
    print("=" * 66)
    print(f"\nMake sure ros2 bag record is running!")
    print(f"  ros2 bag record -o test_h "
          f"/odom_raw /odom /tf /tf_static /amcl_pose "
          f"/cmd_vel /scan_restamped /ground_truth_event "
          f"/global_costmap/costmap")
    input("\nPress Enter when bag recording is started...")

    node.publish_event('test_start', test='obstacle', mode=mode, reps=reps)

    for rep in range(1, reps + 1):
        print(f"\n--- REP {rep} / {reps} ---")
        obstacle_desc = input("  Describe obstacle placement (e.g. 'box mid-aisle A2-A3, 1.5m from A2'): ").strip()
        target = input("  Navigation target (e.g. A3): ").strip()

        print(f"    Clearing global costmap (stale marks from earlier reps)...")
        node.clear_global_costmap()

        node.publish_event('obstacle_nav_start',
                           test='obstacle', mode=mode,
                           rep=rep, target=target,
                           obstacle=obstacle_desc)
        print(f"  Place the obstacle now (if not already), then send the Nav2 goal to {target}.")
        input(f"  Press Enter the MOMENT the robot stops moving (or collides)...")

        stopped_ok = input(f"  Did it stop SAFELY before touching the obstacle? "
                           f"(y=yes, n=collision): ").strip().lower()
        stopped_safely = stopped_ok.startswith('y')
        stop_dist_raw = input(f"  Jarak berhenti ke obstacle terukur meteran (cm): ").strip()
        try:
            stop_distance_cm = float(stop_dist_raw) if stop_dist_raw else None
        except ValueError:
            print(f"    WARNING: '{stop_dist_raw}' bukan angka, dilewati.")
            stop_distance_cm = None

        node.publish_event('obstacle_stop',
                           test='obstacle', mode=mode,
                           rep=rep, target=target,
                           obstacle=obstacle_desc,
                           stopped_safely=stopped_safely,
                           stop_distance_cm=stop_distance_cm)

        if not stopped_safely:
            # collision -- no point testing resume, log and move on
            node.publish_event('obstacle_nav_result',
                               test='obstacle', mode=mode,
                               rep=rep, target=target,
                               obstacle=obstacle_desc,
                               result='collision', resumed=False)
            print(f"    Collision logged. Recover the robot manually before the next rep.")
            time.sleep(0.5)
            continue

        input(f"  Now REMOVE the obstacle, then press Enter...")
        node.publish_event('obstacle_cleared',
                           test='obstacle', mode=mode,
                           rep=rep, target=target)
        print(f"  Watching for the robot to resume driving on its own "
              f"(no manual nudge/teleop) -- this should happen within a few "
              f"seconds once the costmap re-scans the now-empty cell.")
        resumed = input(f"  Did it resume automatically? (y/n, wait up to ~10s): ").strip().lower()
        resumed_ok = resumed.startswith('y')

        final_result = 'timeout'
        if resumed_ok:
            arrived = input(f"  Did it then reach {target} successfully? (y/n): ").strip().lower()
            final_result = 'success' if arrived.startswith('y') else 'timeout'
        else:
            final_result = 'stuck'   # stopped safely but never resumed -- a real failure mode to report

        node.publish_event('obstacle_nav_result',
                           test='obstacle', mode=mode,
                           rep=rep, target=target,
                           obstacle=obstacle_desc,
                           result=final_result, resumed=resumed_ok)
        print(f"    Marker published: {final_result}")
        time.sleep(0.5)

    node.publish_event('test_end', test='obstacle', mode=mode)
    print(f"\nDone! Stop bag recording.")
    print(f"Then run: python3 analyze_obstacle_bag.py test_h/")


# ---------------------------------------------------------------- main

TEST_RUNNERS = {
    'localization': run_localization,
    'trajectory': run_trajectory,
    'navigation': run_navigation,
    'obstacle': run_obstacle,
    'map_geometry': run_map_geometry,
}


def main():
    ap = argparse.ArgumentParser(
        description='Ground truth marker for rosbag-based thesis data collection')
    ap.add_argument('--test', required=True,
                    choices=list(TEST_RUNNERS.keys()),
                    help='Which test to run')
    ap.add_argument('--mode', default='C',
                    help='Localization mode (A/B/C)')
    ap.add_argument('--reps', type=int, default=5,
                    help='Number of repetitions')
    args = ap.parse_args()

    rclpy.init()
    node = GroundTruthMarker()
    stop_event = threading.Event()
    spinner = threading.Thread(target=spin_thread, args=(node, stop_event),
                               daemon=True)
    spinner.start()

    time.sleep(0.5)

    try:
        TEST_RUNNERS[args.test](node, args.mode.upper(), args.reps)
    except KeyboardInterrupt:
        print("\nInterrupted.")
        node.publish_event('test_interrupted', test=args.test)
    finally:
        stop_event.set()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
