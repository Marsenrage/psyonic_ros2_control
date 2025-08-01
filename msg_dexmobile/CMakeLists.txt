cmake_minimum_required(VERSION 3.16)
project(msg_dexmobile)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(msg_dexmobile
  "msg/StampedFloat64Array.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
