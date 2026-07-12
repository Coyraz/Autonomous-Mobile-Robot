import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Which local controller to run. One of:
#   "custom" -> our own Pure Pursuit node (custom_path_controller)
#   "dwb"    -> stock Nav2 DWB
#   "rpp"    -> stock Nav2 Regulated Pure Pursuit (controller_rpp.yaml)
CONTROLLER = "custom"


def generate_launch_description():
    # 1. Get package locations
    pkg_bringup = get_package_share_directory('robot_bringup')
    bt_navigator_dir = get_package_share_directory('nav2_bt_navigator')

    # 2. File locations
    map_file        = os.path.join(pkg_bringup, 'maps', 'warehouse_v1_edited_edited.yaml')
    keepout_file    = os.path.join(pkg_bringup, 'maps', 'warehouse_v1_keepout.yaml')
    nav2_params_file = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')
    custom_ctrl_file = os.path.join(pkg_bringup, 'config', 'custom_controller.yaml')
    rpp_ctrl_file   = os.path.join(pkg_bringup, 'config', 'controller_rpp.yaml')
    default_bt_xml  = os.path.join(bt_navigator_dir, 'behavior_trees',
                                   'navigate_to_pose_w_replanning_and_recovery.xml')

    # 3. Lifecycle nodes list (shared base + controller-specific additions below)
    lifecycle_nodes = [
        'map_server',
        'filter_mask_server',
        'costmap_filter_info_server',
        'amcl',
        'planner_server',
        'velocity_smoother'
    ]

    # --- SHARED NAVIGATION NODES ---

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params_file, {'yaml_filename': map_file}]
    )

    filter_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'yaml_filename': keepout_file},
            {'topic_name': 'keepout_filter_mask'},
            {'frame_id': 'map'}
        ]
    )

    costmap_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'type': 0,
            'filter_info_topic': '/costmap_filter_info',
            'mask_topic': '/keepout_filter_mask',
            'base': 0.0,
            'multiplier': 1.0
        }]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file]
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel_smoothed')
        ]
    )

    nodes = [
        map_server,
        filter_mask_server,
        costmap_filter_info_server,
        amcl,
        planner_server,
        velocity_smoother,
    ]

    # --- CONTROLLER-SPECIFIC NODES ---

    if CONTROLLER == "custom":
        # Our custom Pure Pursuit controller. Serves navigate_to_pose, asks
        # planner_server for a global path, tracks it, publishes cmd_vel_nav.
        custom_path_controller = Node(
            package='robot_bringup',
            executable='custom_path_controller',
            name='custom_path_controller',
            output='screen',
            parameters=[custom_ctrl_file]
        )
        nodes.append(custom_path_controller)

        # Stock nav2_waypoint_follower for ils_gui's multi-rack GUI feature.
        # It just calls the 'navigate_to_pose' action repeatedly, which
        # custom_path_controller already serves under that same name -- no
        # bt_navigator/controller_server needed. Idle (no publishers/CPU use)
        # unless something calls the /follow_waypoints action.
        waypoint_follower = Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_params_file]
        )
        nodes.append(waypoint_follower)
        lifecycle_nodes.append('waypoint_follower')

    else:
        # Stock Nav2 controller_server stack (DWB or RPP). Both use the same
        # bt_navigator + behavior_server + waypoint_follower and the local
        # costmap from nav2_params.yaml. For RPP we append controller_rpp.yaml,
        # which overrides only the FollowPath plugin to Regulated Pure Pursuit.
        lifecycle_nodes += [
            'controller_server',
            'behavior_server',
            'bt_navigator',
            'waypoint_follower',
        ]

        if CONTROLLER == "rpp":
            controller_params = [nav2_params_file, rpp_ctrl_file]
        else:  # "dwb"
            controller_params = [nav2_params_file]

        controller_server = Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=controller_params,
            remappings=[('cmd_vel', 'cmd_vel_nav')]
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

        nodes += [controller_server, behavior_server, bt_navigator, waypoint_follower]

    # lifecycle manager must be created AFTER the node list is finalised
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
    nodes.append(lifecycle_manager)

    return LaunchDescription(nodes)
