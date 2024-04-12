import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='myagv_teleop_ros2',
            executable='myagv_teleop',
            name='myagv_teleop',
            output='screen',
            emulate_tty=True,
            parameters=[{ 'use_sim_time': False}],
        ),
    ])