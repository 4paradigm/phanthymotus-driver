# 快速开发（仿真）

> Source: https://wiki.pndbotics.com/robot/quick_development_sim/

本文介绍了使用 [pnd_sdk_python](https://github.com/pndbotics/pnd_sdk_python) 或 [pnd_ros2](https://github.com/pndbotics/pnd_ros2) 对Adam进行快速开发。配套仿真框架 [pnd_mujoco](https://github.com/pndbotics/pnd_mujoco)，在用户计算机上实现 [Mujoco](https://mujoco.org) 平台的模拟仿真。

## 系统环境

推荐在 **Ubuntu 22.04 x86_64** 系统下进行开发，暂不支持 Mac、Windows。

---

## 安装 Mujoco

```bash
# 安装mujoco平台
cd ~
pip3 install mujoco==3.2.0

# 安装pnd_mujoco
cd ~
git clone https://github.com/pndbotics/pnd_mujoco.git
```

> 参考链接：[Mujoco](https://mujoco.org)，[pnd_mujoco](https://github.com/pndbotics/pnd_mujoco)

---

## 安装 SDK

### pnd_sdk_python

```bash
# 安装系统依赖
sudo apt install libyaml-cpp-dev libspdlog-dev libboost-all-dev libglfw3-dev python3-pip

# 安装 Python SDK
cd ~
git clone https://github.com/pndbotics/pnd_sdk_python.git
cd pnd_sdk_python
sudo pip3 install -e . --user
```

> 参考链接：[pnd_sdk_python](https://github.com/pndbotics/pnd_sdk_python)

### pnd_ros2

（选择 pnd_ros2 标签查看对应安装步骤）

---

## 仿真测试

### Adam_Lite

按照具体型号修改配置文件 `config.py`：

```bash
# 打开配置文件
cd ~
xdg-open pnd_mujoco/simulate_python/config.py
```

修改首行代码为Lite型号：

```python
ROBOT = "adam_lite"
```

启动mujoco：

```bash
cd ~/pnd_mujoco/simulate_python
python3 pnd_mujoco.py
```

打开一个新的终端：

```bash
# 运行pnd_sdk_python控制示例
cd ~/pnd_sdk_python/example/low_level/adam_lite
python3 adam_lite_low_level_example.py

# 或运行pnd_ros2控制示例
cd ~/pnd_ros2/example/low_level/adam_lite
source /opt/ros/humble/setup.bash
source ~/pnd_ros2/install/setup.bash
python3 adam_lite_low_level_example.py
```

观察机器人脚踝运动，表示示例运行成功。

### Adam_SP

修改 `config.py` 首行为：

```python
ROBOT = "adam_sp"
```

运行示例路径对应修改为 `adam_sp` 目录。

### Adam_Pro

修改 `config.py` 首行为：

```python
ROBOT = "adam_pro"
```

运行示例路径对应修改为 `adam_pro` 目录。
