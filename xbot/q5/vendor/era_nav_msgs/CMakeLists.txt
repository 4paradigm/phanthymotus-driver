cmake_minimum_required(VERSION 3.8)
project(era_nav_msgs)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find base build tools and generators
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message dependencies
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Python dependencies
find_package(ament_cmake_python REQUIRED)

# Define interface files
file(GLOB_RECURSE MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    msg/*.msg
)
message(STATUS "Found msg files: ${MSG_FILES}")

file(GLOB_RECURSE SRV_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    srv/*.srv
)
message(STATUS "Found srv files: ${SRV_FILES}")

file(GLOB_RECURSE ACTION_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    action/*.action
)
message(STATUS "Found action files: ${ACTION_FILES}")

# Generate interfaces with dependencies
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  ${ACTION_FILES}
  DEPENDENCIES builtin_interfaces action_msgs geometry_msgs sensor_msgs nav_msgs std_msgs
)

# Install python modules
ament_python_install_package(era_nav_pyclient)

# Install scripts
file(GLOB_RECURSE SCRIPT_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} scripts/*)
install(PROGRAMS
  ${SCRIPT_FILES}
  DESTINATION lib/${PROJECT_NAME}
)

# Export runtime dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(builtin_interfaces)
ament_export_dependencies(action_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(std_msgs)

ament_package()
