cmake_minimum_required(VERSION 3.8)
project(lyre_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AsrEvent.msg"
  "msg/AsrIat.msg"
  "msg/AsrKeyword.msg"
  "msg/LlmEvent.msg"
  "msg/LlmRst.msg"
  "msg/PlayEvent.msg"
  "msg/PlayProgress.msg"
  "msg/TtsEvent.msg"
  "srv/LlmAsk.srv"
  "srv/PlayBinary.srv"
  "srv/PlayFile.srv"
  "srv/PlayPause.srv"
  "srv/PlayResume.srv"
  "srv/PlayStop.srv"
  "srv/PlayText.srv"
  "srv/PlayUrl.srv"
  DEPENDENCIES std_msgs
)

ament_package()
