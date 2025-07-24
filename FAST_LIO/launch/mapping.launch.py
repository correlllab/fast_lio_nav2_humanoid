import os.path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # --- fast_lio Configurations ---
    fast_lio_package_path = get_package_share_directory('fast_lio')
    fast_lio_default_config_path = os.path.join(fast_lio_package_path, 'config')

    fast_lio_config_path = LaunchConfiguration('fast_lio_config_path')
    fast_lio_config_file = LaunchConfiguration('fast_lio_config_file')

    declare_fast_lio_config_path_cmd = DeclareLaunchArgument(
        'fast_lio_config_path', default_value=fast_lio_default_config_path,
        description='Yaml config file path for fast_lio'
    )
    declare_fast_lio_config_file_cmd = DeclareLaunchArgument(
        'fast_lio_config_file', default_value='mid360.yaml',
        description='Config file for fast_lio'
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([fast_lio_config_path, fast_lio_config_file]),
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Static transform from fast_lio's 'body' frame to Unitree H1's 'pelvis' frame.
    transform_x = 0.2
    transform_y = 0.0
    transform_z = -0.71
    transform_yaw = 0.0
    transform_pitch = -0.253099
    transform_roll = 0.0

    static_tf_broadcaster_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_pelvis_broadcaster',
        arguments=[
            str(transform_x),
            str(transform_y),
            str(transform_z),
            str(transform_yaw),
            str(transform_pitch),
            str(transform_roll),
            'body',
            'pelvis'
        ],
        output='screen'
    )

    # Node for pointcloud_to_laserscan
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='cloud_registered_to_laserscan',
        output='screen',
        parameters=[
            {'target_frame': 'pelvis'},
            {'use_sim_time': use_sim_time},
            {'min_height': -0.65},
            {'max_height': 0.65},
            {'angle_min': -3.14159},
            {'angle_max': 3.14159},
            {'angle_increment': 0.0087},
            {'range_min': 0.0},
            {'range_max': 3.0},
            {'use_inf': True},
            {'scan_time': 0.0333},
            {'transform_tolerance': 0.05},
            {'queue_size': 20},
        ],
        remappings=[
            ('cloud_in', '/cloud_registered_body'),
            ('scan', '/converted_scan')
        ]
    )

    # Node for SLAM Toolbox
    slam_toolbox_config_dir = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'config'
    )
    default_mapper_params_file = os.path.join(slam_toolbox_config_dir, 'mapper_params_online_async.yaml')

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            default_mapper_params_file,
            {
                'base_frame': 'pelvis',
                'odom_frame': 'camera_init',
                'scan_topic': '/converted_scan',
                'odom_topic': '/Odometry',
                'use_sim_time': use_sim_time,
            }
        ]
    )

    # --- Nav2 Configurations ---
    nav2_bringup_package_dir = get_package_share_directory('nav2_bringup')

    nav2_params_file = LaunchConfiguration('nav2_params_file')
    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(get_package_share_directory('fast_lio'), 'config', 'nav2_config.yaml'),
        description='Full path to the Nav2 parameters file to use'
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_package_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': '',  # SLAM Toolbox publishes the map on /map, no need to specify a static map
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true',
            'namespace': '', # Use empty namespace
            'use_slam': 'False', # We are using slam_toolbox, so Nav2's internal SLAM is off
            'default_bt_xml_filename': os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees', 'compute_path_to_pose.xml'),

            'use_controller': 'true',
            'use_behaviors': 'false',
            'use_waypoint_follower': 'false',
            'use_collision_monitor': 'false',

        }.items(),

    )

    # Add all nodes to the LaunchDescription
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)

    # Fast-LIO arguments
    ld.add_action(declare_fast_lio_config_path_cmd)
    ld.add_action(declare_fast_lio_config_file_cmd)

    # Nav2 arguments
    ld.add_action(declare_nav2_params_file_cmd)

    # Core mapping and localization nodes
    ld.add_action(fast_lio_node)
    ld.add_action(static_tf_broadcaster_node)
    ld.add_action(pointcloud_to_laserscan_node)
    ld.add_action(slam_toolbox_node)

    # Nav2
    ld.add_action(nav2_launch)

    return ld