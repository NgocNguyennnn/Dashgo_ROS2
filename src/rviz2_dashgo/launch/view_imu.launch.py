from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Lấy đường dẫn đến package
    pkg_share = get_package_share_directory('dashgo_rviz')
    
    # Đường dẫn đến file rviz
    rviz_config = os.path.join(pkg_share, 'rviz', 'pathgo_imu.rviz')

    # Node RViz2 
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='view_imu_rviz',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        rviz_node
    ])