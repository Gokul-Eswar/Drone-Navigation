cmake_minimum_required(VERSION 3.8)
project(indoor_drone_nav_v2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DroneState.msg"
  "msg/Waypoint.msg"
  "msg/MissionPlan.msg"
  "action/Arm.action"
  "action/Takeoff.action"
  "action/Land.action"
  "action/GotoPosition.action"
  "action/ExecuteMission.action"
  DEPENDENCIES geometry_msgs std_msgs
)

# Declare the package to be a python package
ament_python_install_package(src)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
