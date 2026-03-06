import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_bringup')
    
    # KITA HANYA MEMBUTUHKAN CONFIG SLAM DARI FILE
    slam_config_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    
    print(f"Loading SLAM Config from: {slam_config_file}")

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

    # --- PERBAIKAN MUTLAK EKF: PARAMETER INJEKSI LANGSUNG ---
    # Kita tidak lagi menggunakan file ekf.yaml eksternal.
    # Seluruh logika fusi matriks ditanamkan langsung ke dalam file peluncuran ini
    # untuk menjamin sistem membacanya tanpa kesalahan parsing file.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'frequency': 20.0,
            'two_d_mode': True,
            'publish_tf': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
            'world_frame': 'odom',
            'odom0': '/odom_raw',
            # KOREKSI FISIKA MUTLAK: 
            # Kita memaksa EKF menelan Posisi Absolut (X, Y, Yaw) dari roda,
            # bukan Kecepatan (Vx) yang bernilai 0.0 di odometry_node.py.
            'odom0_config': [True,  True,  False,
                             False, False, True,
                             False, False, False,
                             False, False, False,
                             False, False, False],
            'imu0': '/imu/data_raw',
            'imu0_config': [False, False, False,
                            False, False, False,
                            False, False, False,
                            False, False, True,
                            False, False, False],
            'imu0_remove_gravitational_acceleration': True
        }]
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_file,
            {'use_lifecycle_manager': False} 
        ]
    )

    lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': ['slam_toolbox']},
            {'bond_timeout': 0.0} 
        ]
    )

    return LaunchDescription([
        bridge_node,
        odom_node,
        tf_node,
        lidar_launch,
        restamper_node,
        ekf_node,  
        slam_node,
        lifecycle_node
    ])
