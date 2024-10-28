from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Lấy đường dẫn đến package
    pkg_share = get_package_share_directory('dashgo_rviz')
    
    # Đường dẫn đến file rviz
    rviz_config = os.path.join(pkg_share, 'rviz', 'view_navigation.rviz')

    # Node RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config]
    )

    # Node costmap markers
    costmap_node = Node(
        package='nav2_costmap_2d',  # ROS2 sử dụng nav2_costmap_2d
        executable='costmap_2d_markers',
        name='voxel_visualizer',
        remappings=[
            ('voxel_grid', '/move_base/local_costmap/obstacle_layer/voxel_grid')
        ]
    )

    return LaunchDescription([
        rviz_node,
        costmap_node
    ])