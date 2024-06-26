cmake_minimum_required(VERSION 3.5)
project(bt_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_OPENVINO "Build OpenVINO" OFF)
option(BUILD_AUTODOCK "Build Auto Docking" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_behaviors REQUIRED) # added by WMN
find_package(nav2_core REQUIRED)


if(BUILD_OPENVINO STREQUAL "ON")
  find_package(object_msgs REQUIRED)
  set(OTHER_DEPS ${OTHER_DEPS} object_msgs)
  add_definitions(-DSUPPORT_OPENVINO)
endif()

if(BUILD_AUTODOCK STREQUAL "ON")
  find_package(autodock_msgs REQUIRED)
  set(OTHER_DEPS ${OTHER_DEPS} autodock_msgs)
  add_definitions(-DSUPPORT_AUTODOCK)
endif()

set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav2_behavior_tree
  nav2_behaviors
  nav2_core
  nav2_msgs
  behaviortree_cpp
  ${OTHER_DEPS}
)  

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


add_executable(send_goal 
  src/send_goal
)
ament_target_dependencies(send_goal
	${dependencies}
)

add_executable(bt_ros2
  src/bt_ros2.cpp
)
ament_target_dependencies(bt_ros2
	${dependencies}
)

install (TARGETS
  send_goal
  bt_ros2
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY bt_xml 
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files.
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
