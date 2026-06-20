#!/bin/bash
echo "Stopping robot..."

# Step 1: Stop ROS daemon so no auto-discovery interferes
ros2 daemon stop

# Step 2: Kill all launch processes first.
# This is critical. If you kill child processes without killing the parent
# launch process, nodes with respawn=True will be restarted automatically.
# Always kill launch processes BEFORE killing individual nodes.
pkill -9 -f "robot_navigation.launch"
pkill -9 -f "robot_mapping.launch"
pkill -9 -f "navigation.launch"
pkill -9 -f "mapping.launch"
pkill -9 -f "hardware.launch"
pkill -9 -f "robot_bringup.launch"

# Small pause to let launch processes die before killing their children
sleep 1

# Step 3: Kill all individual nodes.
# Hardware stack
pkill -9 -f "stm32_bridge"
pkill -9 -f "odometry_node"
pkill -9 -f "laser_restamper"
pkill -9 -f "static_transform_publisher"
pkill -9 -f "sllidar_node"
pkill -9 -f "sllidar_ros2"
pkill -9 -f "rf2o_laser_odometry_node"
pkill -9 -f "ekf_node"
pkill -9 -f "twist_mux"
pkill -9 -f "foxglove_bridge"

# Nav2 stack (these were missing before, causing nav terminal to stay alive)
pkill -9 -f "amcl"
pkill -9 -f "map_server"
pkill -9 -f "controller_server"
pkill -9 -f "planner_server"
pkill -9 -f "behavior_server"
pkill -9 -f "bt_navigator"
pkill -9 -f "waypoint_follower"
pkill -9 -f "velocity_smoother"
pkill -9 -f "lifecycle_manager"

# SLAM stack
pkill -9 -f "async_slam_toolbox_node"
pkill -9 -f "slam_toolbox"

# Step 4: Kill any remaining ros2 launch processes
pkill -9 -f "ros2 launch"
pkill -9 -f "python3.*launch"

# Wait for everything to fully die
sleep 3

# Step 5: Check for survivors
echo "Checking survivors..."
SURVIVORS=$(ps aux | grep -E "stm32_bridge|rf2o|slam_toolbox|ekf_node|lifecycle|amcl|controller_server|planner_server|bt_navigator|velocity_smoother|foxglove" | grep -v grep)

if [ -z "$SURVIVORS" ]; then
    echo "All clean. Safe to relaunch."
else
    echo "WARNING: These processes survived:"
    echo "$SURVIVORS"
    echo "Forcing kill on survivors..."
    # Extract PIDs and force kill
    echo "$SURVIVORS" | awk '{print $2}' | xargs -r kill -9
    sleep 1
    echo "Done force killing survivors."
fi

# Step 6: Restart ROS daemon for clean state
ros2 daemon start
echo "Robot fully stopped. Safe to relaunch."
