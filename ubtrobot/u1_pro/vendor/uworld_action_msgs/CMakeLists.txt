cmake_minimum_required(VERSION 3.8)
project(uworld_action_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotionInfo.msg"
  "srv/GetMotionInfoList.srv"
  "srv/PlayMotion.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
