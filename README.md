# Autonomous-Mobile-Robot
AMR Testing for Tugas Akhir

In this case, I make an AMR (Autonomous Mobile Robot) control using IMU and Encoder, also Lidar 2D for the environment mapping.

PS:
for every terminal you want to use, you should exec this command:

    source /opt/ros/jazzy/setup.bash

or this command:

    cd ws_amr_robot
    source install/setup.bash

PS:
for running the ros2 node:
    
    ros2 launch robot_bringup robot_bringup.launch.py

for running the teleop:

    ros2 run robot_bringup teleop_keyboard

for see the map visualization using FOXGLOVE (Chrome):

    ros2 run foxglove_bridge foxglove_bridge

for outputing the map after visualization:

    cd Autonomous-Mobile-Robot/ws_amr_robot
    source install/setup.bash
    cd src/robot_bringup/maps
    ros2 run nav2_map_server map_saver_cli -t /map -f nama_peta_anda
    
