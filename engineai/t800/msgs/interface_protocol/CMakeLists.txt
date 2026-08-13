cmake_minimum_required(VERSION 3.8)
project(interface_protocol)

# CMake 3.22 under x86-to-ARM64 BuildKit emulation can report this existing
# ROS library as missing when it is first looked up from a nested ament package
# config (geometry_msgs -> std_msgs -> builtin_interfaces). Probe it once in
# this configure process so clean cross-architecture builds behave like native
# ARM64 builds, and fail with an explicit dependency error if it is truly absent.
find_library(
  T800_ROS_LIBRARY_PROBE
  NAMES builtin_interfaces__rosidl_generator_c
  PATHS /opt/ros/humble/lib
  NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH
)
if(NOT T800_ROS_LIBRARY_PROBE)
  message(FATAL_ERROR "Missing ROS library: builtin_interfaces__rosidl_generator_c")
endif()
unset(T800_ROS_LIBRARY_PROBE CACHE)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

file(GLOB MSG_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "msg/*.msg")
file(GLOB SRV_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "srv/*.srv")
list(FILTER MSG_FILES EXCLUDE REGEX "(^|/)\\._")
list(FILTER SRV_FILES EXCLUDE REGEX "(^|/)\\._")
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES geometry_msgs std_msgs
)

ament_package()
