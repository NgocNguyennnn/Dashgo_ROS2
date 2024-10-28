from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ROSSerial Python Node
        Node(
            package='rosserial_python',
            executable='serial_node.py',
            name='serial_node',
            parameters=[{
                'port': '/dev/port4',
                'baud': 115200
            }]
        ),
        
        # Serial Remote HTTP Node
        Node(
            package='serial_remote',
            executable='serial_remote_node.py',
            name='serial_remote_http'
        ),
    ])
