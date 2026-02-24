import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_bringup')
    
    slam_config_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'default.rviz')
    
    lidar_port = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
    
    bridge_node = Node(
        package='robot_bringup',
        executable='stm32_bridge',
        name='stm32_bridge',
        output='screen'
    )

    odom_node = Node(
        package='robot_bringup',
        executable='odometry_node',
        name='odometry_node',
        output='screen'
    )

    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )

    sllidar_dir = get_package_share_directory('sllidar_ros2')
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sllidar_dir, 'launch', 'sllidar_a1_launch.py')),
        launch_arguments={'serial_port': lidar_port, 'frame_id': 'laser_frame'}.items()
    )

    restamper_node = Node(
        package='robot_bringup',
        executable='laser_restamper',
        name='laser_restamper',
        output='screen'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_file]
    )

    # LIFECYCLE MANAGER: DIPERTAHANKAN UNTUK MEMBANGUNKAN SLAM
    # TETAPI SISTEM PEMBUNUHNYA (BOND) DIMATIKAN TOTAL
    lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': ['slam_toolbox']},
            {'attempt_establishing_bonds': False}, # KUNCI ABSOLUT: Matikan pengawasan detak jantung
            {'bond_timeout': 0.0} # Nonaktifkan timer pembunuh
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        bridge_node,
        odom_node,
        tf_node,
        lidar_launch,
        restamper_node,
        slam_node,
        lifecycle_node,
        rviz_node
    ])
