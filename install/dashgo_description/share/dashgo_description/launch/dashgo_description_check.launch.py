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
    
    # Thêm đường dẫn đến file config RViz
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'urdf_config.rviz')

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'urdf_file',
            default_value=urdf_path,
            description='Path to the URDF/Xacro file'
        ),
        
        # Thêm joint_state_publisher_gui để kiểm tra chuyển động
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('urdf_file')]),
                'publish_frequency': 20.0
            }],
            output='screen'  # Thêm để xem log
        ),
        
        # RViz với config file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])