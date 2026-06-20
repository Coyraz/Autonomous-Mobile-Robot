import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_bringup = get_package_share_directory('robot_bringup')

    # 1. Hardware launch: STM32 bridge, odometry, TF, LiDAR,
    #    laser restamper, RF2O, EKF, twist_mux
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'hardware.launch.py')
        )
    )

    # 2. Navigation launch: map server, AMCL, controller, planner,
    #    behavior server, bt_navigator, waypoint_follower, lifecycle manager
    # Delayed 10 seconds to give hardware stack time to fully initialize.
    # Without this delay, AMCL and costmaps start before TF is ready
    # and the navigation stack fails silently.
    navigation_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bringup, 'launch', 'navigation.launch.py')
                )
            )
        ]
    )

    # 3. Foxglove bridge for visualization and teleop
    # Delayed 5 seconds, starts before navigation but after hardware.
    # No point starting foxglove before there is anything to visualize.
    foxglove_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='foxglove_bridge',
                executable='foxglove_bridge',
                name='foxglove_bridge',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        hardware_launch,
        foxglove_node,
        navigation_launch,
    ])
