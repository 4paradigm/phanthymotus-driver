# DEEPRobotics Message

**English** | [简体中文](README_CN.md)

## Overview

This repository provides public ROS 2 message and service interfaces used by DEEPRobotics SDK and DEEPRobotics Simulation to exchange robot state, control commands, and peripheral data.

This repository contains only `msg/` and `srv/` interface definitions. After being built or installed, these interfaces are available to other projects as the ROS 2 package `drdds`.

## Supported Environments

| Ubuntu | ROS 2 |
| --- | --- |
| 20.04 | Foxy |
| 22.04 | Humble |
| 24.04 | Jazzy |

## Build and Install the Deb Package

```bash
cd deep-robotics-msg
source /opt/ros/<ros-distro>/setup.bash
colcon build --cmake-args -DBUILD_DEB=ON
cpack --config build/drdds/CPackConfig.cmake -B build
sudo dpkg -i ./build/deep-robotics-msgs_*.deb
dpkg -l deep-robotics-msgs
```

The deb package installs the interfaces into `/opt/ros/<ros-distro>`. After installation, each new terminal only needs to source the ROS 2 environment:

```bash
source /opt/ros/<ros-distro>/setup.bash
```

## Source Development and Validation

When modifying, adding, or debugging interfaces, build the source code and source the workspace directly:

```bash
cd deep-robotics-msg
source /opt/ros/<ros-distro>/setup.bash
colcon build
source install/setup.bash
```

This workflow is suitable for quickly validating interfaces during development. The build output can also be used directly, but each new terminal must source both the ROS 2 environment and this workspace's `install/setup.bash`.

## Verify Interfaces

After sourcing the environment for the selected workflow, inspect the ROS 2 interfaces with:

```bash
ros2 interface show drdds/msg/Joints
ros2 interface show drdds/srv/StdSrvInt32
```

## Custom Messages and Services

After adding an interface file under `msg/` or `srv/`, rebuild the package:

```bash
source /opt/ros/<ros-distro>/setup.bash
colcon build
source install/setup.bash
ros2 interface show drdds/msg/YourMessage
```

CMake automatically collects `.msg` and `.srv` files from the `msg/` and `srv/` directories. After validating custom interfaces, rebuild and install them by following the "Build and Install the Deb Package" section.

## Use in a ROS 2 Project

Declare the dependency in the downstream ROS 2 package's `package.xml`:

```xml
<depend>drdds</depend>
```

Find the package in `CMakeLists.txt` and add it as a target dependency:

```cmake
find_package(drdds REQUIRED)
ament_target_dependencies(<target> drdds)
```

Use the generated message types in C++ as follows:

```cpp
#include "drdds/msg/joints.hpp"

drdds::msg::Joints msg;
```
