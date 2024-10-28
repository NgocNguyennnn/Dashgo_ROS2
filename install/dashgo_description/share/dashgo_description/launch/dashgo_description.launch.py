from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import ament_index_python.packages
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = ament_index_python.packages.get_package_share_directory('dashgo_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'dashgobase', 'dashgo.urdf.xacro')

    rviz_config_path = os.path.join(pkg_share, 'rviz', 'urdf_config.rviz')

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'urdf_file',
            default_value=urdf_path,
            description='Path to the URDF/Xacro file'
        ),
        DeclareLaunchArgument(
            'multi_robot_name',
            default_value='',
            description='Prefix for multi-robot names'
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('urdf_file')]),
                'publish_frequency': 20.0,
                'tf_prefix': LaunchConfiguration('multi_robot_name')
            }]
        ),
        
        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'rate': 20.0, 
                'use_gui': False
            }]
        ),
        
        # Uncomment if you want to include RViz
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz',
        #     arguments=['-d', rviz_config_path],
        #     output='screen'
        # )
    ])