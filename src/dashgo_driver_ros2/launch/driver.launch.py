from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Declare parameters
    use_sim_time = False

    # Paths to config files
    # dashgo_driver_config = os.path.join(
    #     get_package_share_directory('dashgo_driver_ros2'), 'config', 'my_dashgo_params.yaml'
    # )
    
    # velocity_smoother_config = os.path.join(
    #     get_package_share_directory('dashgo_driver'), 'config', 'yocs_velocity_smoother.yaml'
    # )

    # Node configuration
    return LaunchDescription([
        # Use sim time parameter
        Node(
            package='dashgo_driver_ros2',
            executable='driver.py',
            name='dashgo_driver',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            respawn=True,
            remappings=[('/base_footprint', 'base_footprint')],
        ),

        # Declare the arguments for velocity smoother
        # DeclareLaunchArgument(
        #     'node_name', default_value='velocity_smoother'
        # ),
        # DeclareLaunchArgument(
        #     'nodelet_manager_name', default_value='nodelet_manager'
        # ),
        # DeclareLaunchArgument(
        #     'config_file', default_value=velocity_smoother_config
        # ),
        # DeclareLaunchArgument(
        #     'raw_cmd_vel_topic', default_value='cmd_vel'
        # ),
        # DeclareLaunchArgument(
        #     'smooth_cmd_vel_topic', default_value='smoother_cmd_vel'
        # ),
        # DeclareLaunchArgument(
        #     'robot_cmd_vel_topic', default_value='robot_cmd_vel'
        # ),
        # DeclareLaunchArgument(
        #     'odom_topic', default_value='odom'
        # ),

        # Include velocity smoother launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('kobuki_velocity_smoother'),
                'launch', 'velocity_smoother-launch.py'
            )),
            # launch_arguments={
            #     'node_name': 'velocity_smoother',
            #     # 'nodelet_manager_name': 'nodelet_manager',
            #     'config_file': velocity_smoother_config,
            #     'raw_cmd_vel_topic': 'cmd_vel',
            #     'smooth_cmd_vel_topic': 'smoother_cmd_vel',
            #     'robot_cmd_vel_topic': 'robot_cmd_vel',
            #     'odom_topic': 'odom'
            # }.items()
        ),

        # Check action node
        # Node(
        #     package='dashgo_tools',
        #     executable='check_action.py',
        #     name='dashgo_check',
        #     output='screen'
        # ),

        # Static transform publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_sonar0',
            arguments=[
                '--x', '0.18',
                '--y', '0.10',
                '--z', '0.115',
                '--roll', '0.524',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'sonar0',
            ],
            output='screen'
        ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_link_to_sonar1',
        #     arguments=['0.20', '0.0', '0.115', '0.0', '0.0', '0.0', 'base_footprint', 'sonar1', '40']
        # ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_sonar1',
            arguments=[
                '--x', '0.20',
                '--y', '0.0',
                '--z', '0.115',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'sonar1',
            ],
            output='screen'
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_link_to_sonar2',
        #     arguments=['0.18', '-0.10', '0.115', '-0.524', '0.0', '0.0', 'base_footprint', 'sonar2', '40']
        # ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_sonar2',
            arguments=[
                '--x', '0.18',
                '--y', '-0.10',
                '--z', '0.115',
                '--roll', '-0.524',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'sonar2',
            ],
            output='screen'
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_link_to_sonar3',
        #     arguments=['-0.20', '0.0', '0.115', '3.14', '0.0', '0.0', 'base_footprint', 'sonar3', '40']
        # ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_sonar3',
            arguments=[
                '--x', '-0.20',
                '--y', '0.0',
                '--z', '0.115',
                '--roll', '3.14',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'sonar3',
            ],
            output='screen'
        ),
    ])
