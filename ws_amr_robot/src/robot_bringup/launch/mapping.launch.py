import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_bringup')
    
    # Mengambil konfigurasi SLAM murni yang sudah Anda kalibrasi
    slam_config_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    
    print(f"Memulai Mode Pemetaan Murni. Membaca konfigurasi dari: {slam_config_file}")

    # Memanggil SLAM Toolbox sebagai node biasa
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_file]
    )

    # Memanggil Lifecycle Manager untuk mengaktifkan SLAM Toolbox
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
        slam_node,
        lifecycle_node
    ])
