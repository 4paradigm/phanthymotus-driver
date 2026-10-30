# DEEPRobotics Message

[English](README.md) | **简体中文**

## 项目简介

本仓库提供 DEEPRobotics SDK、DEEPRobotics Simulation 使用的公开 ROS 2 消息与服务接口，用于传递机器人状态、控制指令和外设数据。

本仓库只包含 `msg/` 和 `srv/` 接口定义，编译或安装后，这些接口以 ROS 2 包 `drdds` 的形式提供给其他项目使用。

## 支持环境

| Ubuntu | ROS 2 |
| --- | --- |
| 20.04 | Foxy |
| 22.04 | Humble |
| 24.04 | Jazzy |

## 编译并安装 deb

```bash
cd deep-robotics-msg
source /opt/ros/<ros-distro>/setup.bash
colcon build --cmake-args -DBUILD_DEB=ON
cpack --config build/drdds/CPackConfig.cmake -B build
sudo dpkg -i ./build/deep-robotics-msgs_*.deb
dpkg -l deep-robotics-msgs
```

deb 包会将接口安装到 `/opt/ros/<ros-distro>`。安装后，每个新终端只需要加载 ROS 2 环境：

```bash
source /opt/ros/<ros-distro>/setup.bash
```

## 源码开发与验证

修改、新增或调试接口时，可直接编译源码并加载工作空间：

```bash
cd deep-robotics-msg
source /opt/ros/<ros-distro>/setup.bash
colcon build
source install/setup.bash
```

该方式适合开发过程中快速验证接口。也可直接使用编译结果，但每个新终端除了加载 ROS 2 环境，还需要加载本工作空间的 `install/setup.bash`。

## 接口验证

根据上述使用方式加载环境后，可以检查 ROS 2 接口：

```bash
ros2 interface show drdds/msg/Joints
ros2 interface show drdds/srv/StdSrvInt32
```

## 自定义消息与服务

在 `msg/` 或 `srv/` 目录中新增接口文件后，重新编译：

```bash
source /opt/ros/<ros-distro>/setup.bash
colcon build
source install/setup.bash
ros2 interface show drdds/msg/YourMessage
```

CMake 会自动收集 `msg/` 和 `srv/` 目录中的 `.msg` 与 `.srv` 文件。自定义接口验证完成后，可按照“构建并安装 deb”章节重新构建和安装。

## 在 ROS 2 项目中使用

在下游 ROS 2 包的 `package.xml` 中声明依赖：

```xml
<depend>drdds</depend>
```

在 `CMakeLists.txt` 中查找该包，并将其添加为目标依赖：

```cmake
find_package(drdds REQUIRED)
ament_target_dependencies(<target> drdds)
```

C++ 代码中可按如下方式使用生成的消息类型：

```cpp
#include "drdds/msg/joints.hpp"

drdds::msg::Joints msg;
```
