#!/bin/bash
# =============================================================
# record_rosbag.sh
# =============================================================
# Records all important ROS 2 topics to a rosbag file for
# post-hoc analysis and thesis documentation.
#
# HOW TO USE:
#   Give the test name as argument:
#     ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts/tools/record_rosbag.sh pengujian4_trial1
#     ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts/tools/record_rosbag.sh pengujian5_obstacle_test
#     ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts/tools/record_rosbag.sh pengujian3_titik_A
#
#   If no argument given, uses timestamp as name.
#
# OUTPUT: ~/thesis_data/rosbags/<name>_<timestamp>/
#
# STOP RECORDING: Press Ctrl+C
# =============================================================

# Create output directory
OUTPUT_BASE=~/thesis_data/rosbags
mkdir -p $OUTPUT_BASE

# Build bag name from argument + timestamp
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
if [ -z "$1" ]; then
    BAG_NAME="recording_${TIMESTAMP}"
else
    BAG_NAME="${1}_${TIMESTAMP}"
fi

OUTPUT_PATH="${OUTPUT_BASE}/${BAG_NAME}"

echo ""
echo "=================================================="
echo "  ROS 2 BAG RECORDER FOR THESIS DATA"
echo "=================================================="
echo "  Bag name:   ${BAG_NAME}"
echo "  Output:     ${OUTPUT_PATH}"
echo ""
echo "  Recording topics:"
echo "    /odom          (EKF fused position)"
echo "    /odom_raw      (encoder odometry)"
echo "    /odom_rf2o     (laser odometry)"
echo "    /imu/data_raw  (IMU gyro + accel)"
echo "    /cmd_vel       (velocity commands)"
echo "    /scan_restamped (LiDAR data)"
echo "    /amcl_pose     (AMCL localization)"
echo "    /tf            (transform tree)"
echo "    /tf_static     (static transforms)"
echo "    /plan          (Nav2 planned path)"
echo "    /local_plan    (Nav2 local path)"
echo ""
echo "  Press Ctrl+C to stop recording."
echo "=================================================="
echo ""

# Source ROS 2 if not already sourced
source ~/Autonomous-Mobile-Robot/ws_amr_robot/install/setup.bash 2>/dev/null

# Start recording
ros2 bag record \
    --output ${OUTPUT_PATH} \
    /odom \
    /odom_raw \
    /odom_rf2o \
    /imu/data_raw \
    /cmd_vel \
    /scan_restamped \
    /amcl_pose \
    /tf \
    /tf_static \
    /plan \
    /local_plan \
    /wheel_encoders

echo ""
echo "Recording stopped."
echo "Bag saved to: ${OUTPUT_PATH}"
echo ""
echo "To play back this bag later:"
echo "  ros2 bag play ${OUTPUT_PATH}"
echo ""
echo "To see bag info:"
echo "  ros2 bag info ${OUTPUT_PATH}"
