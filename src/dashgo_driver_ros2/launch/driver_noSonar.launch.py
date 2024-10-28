from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Paths to configuration files
    dashgo_driver_config = os.path.join(
        get_package_share_directory('dashgo_driver'), 'config', 'my_dashgo_params_noSonar.yaml'
    )
    
    velocity_smoother_config = os.path.join(
        get_package_share_directory('dashgo_driver'), 'config', 'yocs_velocity_smoother.yaml'
    )

    return LaunchDescription([
        # Dashgo driver node
        Node(
            package='dashgo_driver',
            executable='dashgo_driver.py',
            name='dashgo_driver',
            output='screen',
            parameters=[dashgo_driver_config],
            respawn=True
        ),

        # Declare launch arguments for velocity smoother and nodelet manager
        DeclareLaunchArgument('node_name', default_value='velocity_smoother'),
        DeclareLaunchArgument('nodelet_manager_name', default_value='nodelet_manager'),
        DeclareLaunchArgument('config_file', default_value=velocity_smoother_config),
        DeclareLaunchArgument('raw_cmd_vel_topic', default_value='cmd_vel'),
        DeclareLaunchArgument('smooth_cmd_vel_topic', default_value='smoother_cmd_vel'),
        DeclareLaunchArgument('robot_cmd_vel_topic', default_value='robot_cmd_vel'),
        DeclareLaunchArgument('odom_topic', default_value='odom'),

        # Nodelet manager
        Node(
            package='nodelet',
            executable='nodelet',
            name='nodelet_manager',
            arguments=['manager']
        ),

        # Velocity smoother
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('yocs_velocity_smoother'),
                'launch', 'velocity_smoother.launch.py'
            )),
            launch_arguments={
                'node_name': 'velocity_smoother',
                'nodelet_manager_name': 'nodelet_manager',
                'config_file': velocity_smoother_config,
                'raw_cmd_vel_topic': 'cmd_vel',
                'smooth_cmd_vel_topic': 'smoother_cmd_vel',
                'robot_cmd_vel_topic': 'robot_cmd_vel',
                'odom_topic': 'odom'
            }.items()
        ),

        # Dashgo check node
        Node(
            package='dashgo_tools',
            executable='check_action.py',
            name='dashgo_check',
            output='screen'
        ),

        # Static transform publishers for sonar sensors
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_sonar0',
            arguments=['0.18', '0.10', '0.115', '0.524', '0.0', '0.0', 'base_footprint', 'sonar0', '40']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_sonar1',
            arguments=['0.20', '0.0', '0.115', '0.0', '0.0', '0.0', 'base_footprint', 'sonar1', '40']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_sonar2',
            arguments=['0.18', '-0.10', '0.115', '-0.524', '0.0', '0.0', 'base_footprint', 'sonar2', '40']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_sonar3',
            arguments=['-0.20', '0.0', '0.115', '3.14', '0.0', '0.0', 'base_footprint', 'sonar3', '40']
        ),
    ])
