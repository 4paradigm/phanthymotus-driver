cmake_minimum_required(VERSION 3.8)
project(q5_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorStatus.msg"
  "msg/MotorCmd.msg"
  "msg/BatteryStatus.msg"
  "msg/ImuData.msg"
  "msg/RobotState.msg"
  "msg/ChassisCmd.msg"
  "msg/JointTrajectoryPoint.msg"
  "msg/JointTrajectory.msg"
  "msg/HandCmd.msg"
  "msg/HandStatus.msg"
  "msg/HeadCmd.msg"
  "msg/LedCmd.msg"
  "msg/TtsCmd.msg"
  "msg/AsrResult.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_package()
