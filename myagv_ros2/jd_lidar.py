import subprocess
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
time.sleep(0.1)
GPIO.setup(20, GPIO.OUT)
GPIO.output(20, GPIO.HIGH)

time.sleep(0.05)
# SLAM drive 
launch_command = "roslaunch myagv_odometry myagv_active.launch"  # 使用ros 打开
launch_command2 = "roslaunch myagv_navigation  myagv_slam_laser.launch"  # 使用ros 打开
launch_command3 = "roslaunch myagv_teleop myagv_teleop.launch"
subprocess.run(
    ['gnome-terminal', '-e', f"bash -c '{launch_command}; exec $SHELL'"])
time.sleep(4)
# This command launch simple lidar view using rviz. 
#launch_command2 = "roslaunch ydlidar_ros_driver lidar_view.launch"  # 使用ros 打开

subprocess.run(
    ['gnome-terminal', '-e', f"bash -c '{launch_command2}; exec $SHELL'"])
time.sleep(4)
#subprocess.run(
#    ['gnome-terminal', '-e', f"bash -c '{launch_command3}; exec $SHELL'"])
