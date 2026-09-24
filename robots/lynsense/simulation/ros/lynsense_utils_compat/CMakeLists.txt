cmake_minimum_required(VERSION 3.8)
project(lynsense_utils)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavToPose.action"
  "action/MoveDistance.action"
  "action/MoveWaist.action"
  "action/MoveNamedConfig.action"
  "action/SetGripper.action"
  "action/BoxPhase.action"
)

ament_package()
