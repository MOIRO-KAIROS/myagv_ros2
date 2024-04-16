import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, FindPackageShare


def generate_launch_description():
    # 파라미터 파일 경로 설정
    param_file_path = os.path.join(
        FindPackageShare('myagv_urdf'), 'urdf', 'myAGV.urdf')

    # Launch 설명 생성
    return LaunchDescription([
        # myagv_odometry_node 노드 실행
        Node(
            package='myagv_odometry',
            namespace='',
            executable='myagv_odometry_node',
            name='myagv_odometry_node',
            output='screen',
            parameters=[{'robot_description': param_file_path}]
        ),
        # joint_state_publisher 노드 실행
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
        ),
        # robot_state_publisher 노드 실행
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': param_file_path}]
        ),
        # ydlidar_ros_driver 포함
        # ROS2에서는 별도의 launch 파일을 include하는 대신, 직접 해당 노드를 여기서 실행해야 합니다.
        # 예를 들어, ydlidar_ros_driver의 노드 실행 정보가 필요합니다.
        # 아래는 가상의 ydlidar 노드 실행 예시입니다.
        Node(
            package='ydlidar_ros_driver',
            executable='ydlidar_ros_driver_node',
            name='ydlidar_ros_driver_node',
            parameters=[],
        )
    ])
