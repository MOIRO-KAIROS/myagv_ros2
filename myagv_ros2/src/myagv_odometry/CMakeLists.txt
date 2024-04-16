cmake_minimum_required(VERSION 3.5)
project(myagv_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(
  include
)

set(executable_name myagv_odometry)

add_executable(myagv_odometry_node 
               src/myAGV.cpp
               src/myAGVSub.cpp)
target_link_libraries(myagv_odometry_node ${catkin_LIBRARIES})

add_executable(myagv_start 
               src/myagv_start.cpp
               src/myagv_start_sub.cpp)

set(dependencies
  angles
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  tf2_ros
)

ament_target_dependencies(myagv_odometry_node ${dependencies})

ament_target_dependencies(myagv_start ${dependencies})

ament_package()
