from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths to included launch files
    dashgo_driver_launch = os.path.join(get_package_share_directory('dashgo_driver_ros2'), 'launch', 'driver.launch.py')
    ydlidar_launch = os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch', 'ydlidar_launch.py')
    dashgo_description_launch = os.path.join(get_package_share_directory('dashgo_description'), 'launch', 'dashgo_description.launch.py')
    cartographer_launch = os.path.join(get_package_share_directory('dashgo_nav'),'launch','include','odom','cartographer_base.launch.py')

    # Create launch description
    ld = LaunchDescription()

    # Include launch files
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dashgo_driver_launch)
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ydlidar_launch)
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dashgo_description_launch)
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cartographer_launch)
        )
    )

    return ld
