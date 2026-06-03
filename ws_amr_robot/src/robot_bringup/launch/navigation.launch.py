import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Mendapatkan lokasi paket
    pkg_bringup = get_package_share_directory('robot_bringup')
    bt_navigator_dir = get_package_share_directory('nav2_bt_navigator')
    
    # 2. Map File Location
    map_file = os.path.join(pkg_bringup, 'maps', 'mapping_lab_with_ekf_attempt_2.yaml')
    nav2_params_file = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')
    default_bt_xml = os.path.join(bt_navigator_dir, 'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml')

    # 3. DAFTAR SIMPUL NAVIGASI MUTLAK
    # Di sinilah kita mengamputasi collision_monitor secara fisik. 
    # Kita HANYA mendaftarkan simpul yang benar-benar kita perlukan.
    lifecycle_nodes = [
        'map_server',
        'amcl',
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother'
    ]

    # --- DEKLARASI SIMPUL INDIVIDUAL ---
    
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params_file, {'yaml_filename': map_file}]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file]
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[('cmd_vel', 'cmd_vel_nav')]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[('cmd_vel', 'cmd_vel_nav')]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file, {'default_bt_xml_filename': default_bt_xml}]
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file]
    )

    # --- MANAJER SIKLUS HIDUP EKSKLUSIF ---
    # Manajer ini hanya diberi wewenang untuk membangunkan 7 simpul di atas.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': lifecycle_nodes}
        ]
    )
    
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel')
        ]
    )

    return LaunchDescription([
        map_server,
        amcl,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
        velocity_smoother
    ])
