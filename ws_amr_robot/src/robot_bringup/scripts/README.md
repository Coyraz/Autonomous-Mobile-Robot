# AMR Thesis Scripts

All scripts here are run directly with `python3 <path>` or `bash <path>`.
They do NOT need `colcon build` to pick up changes.

Base path shortcut (not a real variable, just for readability below):
`SCRIPTS=~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts`

---

## tools/

Operational utilities for running the robot session.

| Script | What it does |
|---|---|
| `kill_robot.sh` | Kills all ROS nodes and processes cleanly. Run this before starting a new session or before any direct-serial characterization script. |
| `record_rosbag.sh` | Records all important topics to `~/thesis_data/rosbags/`. Give a test name as argument, e.g. `record_rosbag.sh pengujian4_trial1`. |

---

## characterization/

Scripts that measure motor and encoder behavior. Most of these talk to the STM32
directly over serial, so ROS must be killed first before running them.
They save output to `~/` (CSV and PNG files).

| Script | What it does |
|---|---|
| `pid_step_response.py` | Sends a speed command to STM32 and records how fast each wheel actually reaches it. Produces rise time, overshoot, and steady-state error data. Use to verify PID tuning. |
| `pid_autotune.py` | Automatically finds PID Kp/Ki gains using Ziegler-Nichols method. Wheels must be off the ground. Takes 5-10 minutes. |
| `open_loop_characterization.py` | Sweeps PWM from 0 to 999 and measures actual wheel speed at each step. Finds the deadband (minimum PWM to move). Needs characterization firmware on STM32. |
| `openloop_step_plot.py` | Companion plotting/analysis tool for open-loop step response data. Also needs characterization firmware. |
| `step_response_collector.py` | Records cmd_vel vs actual encoder speed via ROS topics. Needs ONLY hardware.launch.py running (no navigation). Saves to `~/thesis_data/step_response/`. |
| `encoder_ripple_analysis.py` | Records encoder speed at steady state and runs FFT to find ripple frequency. Used for the SMA filter investigation. Saves to `~/thesis_data/ripple_analysis/`. |

> **Note:** `step_response_collector.py` and `encoder_ripple_analysis.py` need ROS running
> (hardware.launch.py only). The others connect to STM32 serial directly and need ROS killed.

---

## debug/

Scripts for diagnosing navigation problems while the full nav stack is running.

| Script | What it does |
|---|---|
| `nav2_oscillation_recorder.py` | Logs cmd_vel, odom, IMU data to CSV while robot navigates. Use when robot oscillates or moves strangely. Output goes to `~/nav2_oscillation_data.csv`. |

> **Note:** Output goes to `~/` (home root), not `~/thesis_data/`. This is intentional,
> it is a debug tool not thesis data. Move the CSV manually if you want to keep it.

---

## pengujian/

Data collection scripts for the formal thesis experiments (Pengujian 3 and 4).
These need the full navigation stack running.

| Script | What it does |
|---|---|
| `collect_localization_accuracy.py` | Pengujian 3. Drive robot to each reference point, run script, record position from encoder/laser/EKF. Saves to `~/thesis_data/pengujian_3/localization_data.csv`. |
| `collect_navigation_pointtopoint.py` | Pengujian 4. Sends navigation goals and records travel time, arrival error, success/fail per route. Saves to `~/thesis_data/pengujian_4/navigation_data.csv`. |

---

## integration/

Scripts for the PTL (Pick to Light) system integration with the AMR.
This is a bonus demonstration, not a thesis requirement.

| Script | What it does |
|---|---|
| `ptl_nav_bridge.py` | (Not yet written) Flask HTTP server that lets the PTL system trigger navigation goals via REST API. |
