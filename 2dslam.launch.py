# import os
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     # Get the path to the slam_toolbox config directory
#     slam_toolbox_config_dir = os.path.join(
#         get_package_share_directory('slam_toolbox'),
#         'config'
#     )

#     # Path to the default online_async configuration file
#     # You can create your own custom_mapper_params.yaml in your workspace
#     # and point to it here if you need more custom parameters.
#     default_mapper_params_file = os.path.join(slam_toolbox_config_dir, 'mapper_params_online_async.yaml')

#     return LaunchDescription([
#         Node(
#             package='slam_toolbox',
#             executable='async_slam_toolbox_node',
#             name='slam_toolbox',
#             output='screen',
#             parameters=[
#                 default_mapper_params_file,
#                 {
#                     # --- IMPORTANT: Configure these parameters based on your robot ---
#                     # The frame ID of your robot's base. From your TF tree, 'pelvis' seems to be your base frame.
#                     'base_frame': 'pelvis',
#                     # The frame ID of your odometry source. From your TF tree, 'camera_init' seems to be your odometry frame.
#                     'odom_frame': 'body',
#                     # The topic name where your 2D LiDAR publishes its scans.
#                     # !! REPLACE 'my_laser_scan_topic' with your actual LiDAR topic !!
#                     'scan_topic': '/converted_scan',
#                     # The topic name where your robot publishes its odometry.
#                     # !! REPLACE 'my_robot_odom_topic' with your actual odometry topic !!
#                     'odom_topic': '/Odometry',
#                     # Set to True if you are running in simulation (e.g., Gazebo)
#                     # Set to False for real hardware unless explicitly using simulation time
#                     'use_sim_time': False,
#                     # Optional: Adjust map resolution (meters per pixel)
#                     'resolution': 0.05,
#                     # Optional: Set the update interval for the map (seconds)
#                     'map_update_interval': 0.5,
#                 }
#             ],
#             # If you are using a specific namespace for your robot, uncomment and modify:
#             # namespace='my_robot_namespace',
#             remappings=[
#                 ('/scan', '/converted_scan'),
#                 ('/odom', '/Odometry'),
#              ]
#         )
#     ])
