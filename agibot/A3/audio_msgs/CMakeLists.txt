cmake_minimum_required(VERSION 3.8)
project(audio_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AudioChunk.msg"
  "msg/AudioData.msg"
  "msg/AudioInfo.msg"
  "msg/AudioCapture.msg"
  "msg/AudioPlayback.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
