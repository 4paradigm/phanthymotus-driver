# 快速开发（真机）

> Source: https://wiki.pndbotics.com/robot/quick_development/

本文介绍了使用 [pnd_sdk_python](https://github.com/pndbotics/pnd_sdk_python) 对Adam进行快速开发。通过有线连接机器人与用户计算机，实现真机控制。

## 系统环境

推荐在 **Ubuntu 22.04 x86_64** 系统下进行开发，暂不支持 Mac、Windows。

---

## 安装 SDK

```bash
# 安装系统依赖
sudo apt install libyaml-cpp-dev libspdlog-dev libboost-all-dev libglfw3-dev python3-pip

# 安装 Python SDK
cd ~
git clone https://github.com/pndbotics/pnd_sdk_python.git
cd pnd_sdk_python && sudo pip3 install -e . --user
```

> 参考链接：[pnd_sdk_python](https://github.com/pndbotics/pnd_sdk_python)

---

## 真机连接

1. 使用网线连接机器人与用户计算机，机器人网口位于背部
2. 在用户计算机上设置网络与机器人同一网段，修改IP地址如：`10.10.20.XXX`

---

## 开发者模式

> **注意**:
> - 确认 Demo启动 已完成
> - 更多操作与模式说明参考 遥控器说明

确保机器人悬挂并处于**阻尼模式**，短按遥控器组合键 **LO + RO**（垂直下压摇杆）进入**开发者模式**。

**RCU指示灯**由**紫色慢速呼吸**变为**蓝色慢速呼吸**，表示进入**开发者模式**成功，此时可使用SDK进行开发调试。

### 运行控制示例

> 该控制示例会使机器人脚踝运动，请确保机器人已正确悬挂且双脚距离地面 >10cm。

打开一个新的终端：

#### Adam_Lite

```bash
# 获取网卡名
ip a

# 运行控制示例（替换 enp59s0 为实际有线网卡名）
cd ~/pnd_sdk_python/example/low_level/adam_lite
python3 adam_lite_low_level_example.py enp59s0
```

#### Adam_SP

```bash
cd ~/pnd_sdk_python/example/low_level/adam_sp
python3 adam_sp_low_level_example.py enp59s0
```

#### Adam_Pro

```bash
cd ~/pnd_sdk_python/example/low_level/adam_pro
python3 adam_pro_low_level_example.py enp59s0
```

### 退出开发者模式

短按遥控器组合键 **LT + B** 退出开发者模式，**RCU指示灯**由**蓝色慢速呼吸**变为**紫色慢速呼吸**，表示退出**开发者模式**成功，进入**阻尼模式**。
