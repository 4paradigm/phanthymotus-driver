cmake_minimum_required(VERSION 3.8)
project(unix_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GripperCommand.msg"
  "srv/SaveMap.srv"
  "srv/LoadMap.srv"
  "srv/DeleteMap.srv"
  "srv/GetMapList.srv"
  DEPENDENCIES sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

