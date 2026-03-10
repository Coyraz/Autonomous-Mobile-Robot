import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_bringup = get_package_share_directory('robot_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    bt_navigator_dir = get_package_share_directory('nav2_bt_navigator')
    
    # Merujuk pada peta hasil iterasi kedua yang telah Anda buat
    map_file = os.path.join(pkg_bringup, 'maps', 'mapping_lab_with_ekf_attempt_2.yaml')
    nav2_params_file = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')
    
    default_bt_xml = os.path.join(bt_navigator_dir, 'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params_file,
            'use_sim_time': 'false',
            'default_bt_xml_filename': default_bt_xml,
            # [AMPUTASI MUTLAK]
            # Mencegah Lifecycle Manager membangunkan modul yang tidak memiliki konfigurasi YAML
            'use_collision_monitor': 'false', 
            'use_velocity_smoother': 'false'  
        }.items()
    )

    return LaunchDescription([
        nav2_launch
    ])
