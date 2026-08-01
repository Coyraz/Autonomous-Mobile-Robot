"""
localization_test.launch.py
---------------------------
Mode-switch launch file for Test E: 3-mode localization accuracy comparison.

Usage:
  ros2 launch robot_bringup localization_test.launch.py mode:=A   # encoder only
  ros2 launch robot_bringup localization_test.launch.py mode:=B   # AMCL only
  ros2 launch robot_bringup localization_test.launch.py mode:=C   # EKF fusion

Mode A (encoder only):
  Hardware + odom_tf_broadcaster. No AMCL, no EKF, no RF2O.
  Position source: /odom_raw (encoder-integrated pose).

Mode B (AMCL only):
  Hardware + odom_tf_broadcaster + map_server + AMCL. No EKF, no RF2O.
  odom->base_link TF comes from raw encoder odometry (no fusion).
  AMCL corrects map->odom. Position source: TF map->base_link.

Mode C (EKF fusion):
  Full hardware stack (RF2O + EKF) + map_server + AMCL.
  EKF fuses encoder velocity + RF2O position + IMU Vyaw.
  EKF publishes odom->base_link TF. Position source: TF map->base_link.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context).upper()
    pkg_bringup = get_package_share_directory('robot_bringup')

    lidar_port = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
    nav2_params = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')
    # 2026-07-25: was warehouse_v1_edited.yaml -- stale, out of sync with
    # navigation.launch.py since the 2026-07-24 remap. Synced to match.
    map_file = os.path.join(pkg_bringup, 'maps', 'warehouse_v3_20260721_edited.yaml')
    ekf_config = os.path.join(pkg_bringup, 'config', 'ekf.yaml')
    twist_mux_config = os.path.join(pkg_bringup, 'config', 'twist_mux.yaml')

    sllidar_dir = get_package_share_directory('sllidar_ros2')

    nodes = []

    # ================================================================
    # COMMON HARDWARE (all modes)
    # ================================================================
    nodes.append(Node(
        package='robot_bringup', executable='stm32_bridge',
        name='stm32_bridge', output='screen',
        respawn=True, respawn_delay=3.0))

    nodes.append(Node(
        package='robot_bringup', executable='odometry_node',
        name='odometry_node', output='screen'))

    nodes.append(Node(
        package='tf2_ros', executable='static_transform_publisher',
        # 2026-07-25: was 0.17 -- stale, out of sync with hardware.launch.py
        # since the 2026-07-21 LiDAR +12cm spacer change. Synced to match.
        arguments=['0.08', '0.0', '0.29', '3.14159', '0.0', '0.0',
                   'base_link', 'laser_frame'],
        output='screen'))

    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_dir, 'launch', 'sllidar_a1_launch.py')),
        launch_arguments={
            'serial_port': lidar_port,
            'frame_id': 'laser_frame'
        }.items()))

    nodes.append(Node(
        package='robot_bringup', executable='laser_restamper',
        name='laser_restamper', output='screen'))

    nodes.append(Node(
        package='twist_mux', executable='twist_mux',
        name='twist_mux', output='screen',
        parameters=[twist_mux_config, {'use_stamped': False}],
        remappings=[('cmd_vel_out', 'cmd_vel')]))

    # ================================================================
    # MODE A & B: odom TF broadcaster (encoder -> odom->base_link TF)
    # In mode C the EKF publishes this TF instead.
    # ================================================================
    if mode in ('A', 'B'):
        nodes.append(Node(
            package='robot_bringup', executable='odom_tf_broadcaster',
            name='odom_tf_broadcaster', output='screen'))

    # ================================================================
    # MODE B & C: map_server + AMCL (localization in map frame)
    # ================================================================
    if mode in ('B', 'C'):
        nodes.append(Node(
            package='nav2_map_server', executable='map_server',
            name='map_server', output='screen',
            parameters=[nav2_params, {'yaml_filename': map_file}]))

        nodes.append(Node(
            package='nav2_amcl', executable='amcl',
            name='amcl', output='screen',
            parameters=[nav2_params]))

        nodes.append(Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization', output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }]))

    # ================================================================
    # MODE C ONLY: RF2O laser odometry + EKF sensor fusion
    # ================================================================
    if mode == 'C':
        nodes.append(Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry', output='screen',
            respawn=False,
            parameters=[{
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom_rf2o',
                'publish_tf': False,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 20.0,
            }]))

        nodes.append(Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_filter_node', output='screen',
            parameters=[ekf_config],
            remappings=[('odometry/filtered', 'odom')]))

    return nodes


def generate_launch_description():
    foxglove = TimerAction(
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
        DeclareLaunchArgument(
            'mode', default_value='C',
            description='Localization mode: A=encoder-only, B=AMCL-only, C=EKF-fusion'),
        OpaqueFunction(function=launch_setup),
        foxglove,
    ])
