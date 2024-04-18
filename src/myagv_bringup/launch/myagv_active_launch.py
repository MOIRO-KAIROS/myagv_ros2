import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    myagv_urdf = os.path.join(
        get_package_share_directory('myagv_description'),
        'urdf',
        'myAGV.urdf')
    
    with open(myagv_urdf, 'r') as file:
        myagv_desc = file.read()
        

    return LaunchDescription([
        Node(
            package='myagv_bringup',
            executable='myagv_odometry_node',
            name='myagv_odometry_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"use_sim_time": False}, {"robot_description": myagv_desc}],
            arguments=[myagv_urdf]
        ),
    ])