cmake_minimum_required(VERSION 3.8)
project(myagv_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# find_package(message_generation REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Find and setup BFL
include_directories(
  include
  /usr/include/bfl
)

# add Library
add_library(${PROJECT_NAME}_lib
  "src/odom_estimation.cpp"
  "src/nonlinearanalyticconditionalgaussianodo.cpp"
)

# set Dependency
set(DEPENDENCIES
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "std_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)

# Apply dependencies
ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

# declare rosidl_generate_interfaces for generating service headers from srv files
rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/GetStatus.srv"
)

set(EXECUTABLE_NAME "myagv_estimation_node")

add_executable(${EXECUTABLE_NAME} 
    "src/odom_estimation_node.cpp"
)

target_link_libraries(${EXECUTABLE_NAME} ${PROJECT_NAME}_lib -lorocos-bfl)
ament_target_dependencies(${EXECUTABLE_NAME} ${DEPENDENCIES})

# ensure interfaces are generated before executables
add_dependencies(myagv_estimation ${PROJECT_NAME}__rosidl_typesupport_cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
