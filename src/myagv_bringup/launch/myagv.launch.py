import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # myAGV param 설정
    # myagv_param_dir = os.path.join(
    #     get_package_share_directory('myagv_bringup'),
    #     'param',
    #     'myAGV.yaml'
    # )

    # YDLidar pkg 경로 설정
    ydlidar_launch_file_dir = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'),
        'launch'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'
        ),
        # Include the launch file that starts the robot state publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/myagv_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        # Include the launch file that starts the ydlidar launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ydlidar_launch_file_dir, '/ydlidar_launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # Include the launch file that starts the teleop node
        # myAGV Node 실행
        Node(
            package='myagv_node',
            executable='myagv_node',
            output='screen',
        )
    ])
