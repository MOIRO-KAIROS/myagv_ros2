import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sime_time = LaunchConfiguration('use_sim_time', default='false')
    myagv_urdf = os.path.join(
        get_package_share_directory('myagv_description'),
        'urdf',
        'myAGV.urdf')
    with open(myagv_urdf, 'r') as file:
        myagv_desc = file.read()
    
    rsp_param = {'robot_description': myagv_desc}
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[rsp_param, {'use_sim_time': use_sime_time}],
        ),
    ])