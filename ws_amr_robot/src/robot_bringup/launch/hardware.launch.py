import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # File ini DEDIKATIF hanya untuk menyalakan perangkat keras dan fusi sensor (EKF).
    # TIDAK ADA algoritma pemetaan (SLAM) atau navigasi di sini.
    
    lidar_port = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
    
    bridge_node = Node(package='robot_bringup', executable='stm32_bridge', name='stm32_bridge', output='screen')
    odom_node = Node(package='robot_bringup', executable='odometry_node', name='odometry_node', output='screen')
    
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

    restamper_node = Node(package='robot_bringup', executable='laser_restamper', name='laser_restamper', output='screen')

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
            'odom0_config': [True,  True,  False, False, False, True, False, False, False, False, False, False, False, False, False],
            'imu0': '/imu/data_raw',
            'imu0_config': [False, False, False, False, False, False, False, False, False, False, False, True, False, False, False],
            'imu0_remove_gravitational_acceleration': True
        }]
    )

    return LaunchDescription([
        bridge_node,
        odom_node,
        tf_node,
        lidar_launch,
        restamper_node,
        ekf_node
    ])
