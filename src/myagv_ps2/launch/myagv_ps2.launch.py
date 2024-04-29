from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='myagv_ps2',
            executable='ps2_publisher',
            name='ps2_publisher',
            output='screen',
        ),
    ])
