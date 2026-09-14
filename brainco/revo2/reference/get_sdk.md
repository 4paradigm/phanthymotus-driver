# 获取 SDK (https://www.brainco-hz.com/docs/revolimb-hand/revo2/get_sdk.html)

Source: (see original URL)

# 获取 SDK ​

## SDK 概述 ​

灵巧手 SDK（Software Development Kit）为 BrainCo Revo 1 和 Revo 2 设备提供 Python 与 C++ 接口，包含设备连接、状态读取、电机控制、触觉数据读取和固件升级示例。

### 系统要求 ​

- Python : 3.9 ~ 3.12
- Linux : Ubuntu 20.04/22.04 LTS (x86_64/aarch64), glibc ≥ 2.31
- macOS : 10.15+
- Windows : 10/11

### 支持的通信协议 ​

| 设备 | RS-485 | Protobuf | CAN | CANFD | EtherCAT |
| --- | --- | --- | --- | --- | --- |
| Revo 1 | ✅ | ✅ | ✅ | ❌ | ❌ |
| Revo 2 | ✅ | ❌ | ✅ | ✅ | ✅ |

## 下载与安装 ​

### SDK 仓库 ​

[GitHub - brainco-hand-sdk](https://github.com/BrainCoTech/brainco-hand-sdk)

### Python SDK 安装 ​

bash

```
# 从 PyPI 安装
pip3 install bc-stark-sdk==2.0.2

# 国内镜像（阿里云 OSS）
bash install_whl.sh 2.0.2
```

### 示例代码 ​

- Python 示例
- C++ 示例
- Python GUI 调试工具

## 示例运行说明 (Running Demos) ​

在成功克隆仓库并安装 Python 或 C++ SDK 依赖后，您可以按照以下步骤直接运行 Revo 1/2 的演示示例。

⚠️ Linux 系统权限与延迟配置

在 Linux 宿主机上使用 CAN FD 或 RS-485 串口时，请确认设备权限与延迟配置：

- CAN 接口使能权限 ：请确保 SocketCAN 接口已配置为 1M 或 2M 波特率： bash sudo ip link set can0 up type can bitrate 1000000 dbitrate 2000000 fd on
- FTDI 串口延迟设置 ：若使用 RS485 串口，并且需要较短查询周期，请启用 low_latency ： bash sudo setserial /dev/ttyUSB0 low_latency

### 1. Python 示例运行 ​

进入仓库的 `python/demo/` 目录运行基础控制与多传感器时序采集：

bash

```
cd python/demo

# 运行串口与协议自动扫频发现工具
python auto_detect.py

# 运行 Revo 1/2 综合控制演示
python hand_demo.py
python hand_demo.py -m /dev/ttyUSB0 460800 127                # Modbus
python hand_demo.py -c /dev/cu.usbmodem14201 1000000 1        # ZQWL CAN 2.0
python hand_demo.py -f /dev/cu.usbmodem14201 1000000 5000000 127  # ZQWL CANFD
python hand_demo.py -B can0 127                               # SocketCAN CANFD (Linux, SDK 内置)

# 运行实时数据监控
python hand_monitor.py

# 运行固件升级工具
python hand_dfu.py /path/to/firmware.bin
```

Python 输出节选

以下为 SDK 运行日志的脱敏节选，端口、ID、序列号、固件版本和传感器数值会随设备而变化：

text

```
$ python auto_detect.py
2026-03-26T11:22:15.993405Z [INFO] 🔍 Auto-detect: scan_all=false, port=None, protocol=None
2026-03-26T11:22:16.006330Z [INFO] 🔍 Trying Modbus on /dev/tty.usbserial-21100...
2026-03-26T11:22:16.006417Z [INFO] 🔍 Trying Modbus on /dev/tty.usbserial-21100 at Baud460800...
2026-03-26T11:22:16.343617Z [INFO] TouchVendor for slave 127: ArrayPressure
2026-03-26T11:22:16.344213Z [INFO] DeviceInfo: "{\"sku_type\":\"MediumRight\",\"hardware_type\":\"Revo2TouchArrayPressure\",\"serial_number\":\"<device serial>\",\"firmware_version\":\"<firmware version>\"}"
2026-03-26T11:22:16.344343Z [INFO] ✅ Found Modbus device: port=/dev/tty.usbserial-21100, ID=0x7F, baudrate=Baud460800, hw=Revo2TouchArrayPressure
2026-03-26T11:22:16.848082Z [INFO] ✅ Auto-detect: Found 1 device(s)
2026-03-26T11:22:16.867287Z [INFO] Initialized context for Revo2TouchArrayPressure device (slave 0x7F)

Found 1 device(s)
----------------------------------------------------------------------

[Device 1] Revo2 Touch Array Pressure
  Protocol:     Modbus
  Port:         /dev/tty.usbserial-21100
  Slave ID:     0x7F (127)
  Baudrate:     460800
  Serial:       <device serial>
  Firmware:     <firmware version>
  SKU:          MediumRight
```

text

```
$ python hand_demo.py
[Init] Revo2Touch
  Protocol: StarkProtocolType.Modbus
  Port: /dev/tty.usbserial-21100
  Slave ID: 0x7F (127)
  Serial: <device serial>
  Firmware: <firmware version>

=== Demo 3: Advanced Control (Position + Time/Speed) ===
Position with duration (1000ms)...
Position with speed...
Final positions: [0, 0, 0, 0, 0, 0]
```

### 2. C++ 示例编译与运行 ​

在 `c/` 目录下编译并运行跨平台 C++ 示例：

bash

```
cd c
make

# 运行自动探测连接设备
./demo/auto_detect.exe

# 运行 Revo 2 综合控制演示
./demo/hand_demo.exe
./demo/hand_demo.exe -m /dev/ttyUSB0 460800 127               # Modbus
./demo/hand_demo.exe -c /dev/cu.usbmodem14201 1000000 1       # ZQWL CAN 2.0
./demo/hand_demo.exe -f /dev/cu.usbmodem14201 1000000 5000000 127  # ZQWL CANFD
./demo/hand_demo.exe -B can0 127                              # SocketCAN CANFD (Linux, SDK 内置)
```

C++ 输出节选

以下为 `c/demo` 示例源码对应的输出结构，端口、ID、序列号、固件版本和传感器数值会随设备而变化：

text

```
$ ./demo/auto_detect.exe
=== Stark Auto-Detect Example ===

[INFO] Auto-detecting devices...

[INFO] Found devices:

[1] Revo2Touch
    Protocol: CANFD
    Port: /dev/ttyUSB0
    Slave ID: 0x7F (127)
    Serial Number: <device serial>
    Firmware: <firmware version>

[INFO] Using the only available device
Slave[127] Serial Number: <device serial>, FW: <firmware version>
Hardware Type: Revo2Touch (6)
Slave[127] Baudrate: 460800

=== Testing Finger Control ===
Closing pinky...
Opening pinky...
Final positions: 0, 0, 0, 0, 0, 0

=== Example completed ===
```

text

```
$ ./demo/hand_demo.exe -f /dev/ttyUSB0 1000000 5000000 127 3
=== Universal Motor Control - Complete Demo ===

[Init] Mode: ZQWL CANFD
  Port: /dev/ttyUSB0, Arb: 1000000, Data: 5000000, Slave ID: 127
Slave[127] Serial Number: <device serial>, FW: <firmware version>
Hardware Type: Revo2Touch (6)

[INFO] Motor API: Revo2
[INFO] Touch type: Revo2Touch

=== Demo 3: Advanced Control (Revo2 Only) ===
[Demo] Setting unit mode to Normalized...
  Current mode: Normalized
[Demo] Position + time control (single finger)...
[Demo] Position + speed control (single finger)...
[Demo] Position + duration control (all fingers)...

[INFO] Done!
```

## 进阶参考 ​

### 接口定义文件 ​

API 查阅与代码自动补全可参考以下接口定义文件，支持直接下载或在下方展开查看：

- Python 类型存根 (.pyi) : 下载 main_mod.pyi — 包含 bc_stark_sdk.main_mod 中的类、枚举、函数类型注解。
- C/C++ 头文件 (.h) : 下载 stark-sdk.h — SDK 2.x 发布包中的 C ABI 导出头文件。

完整类型存根见 `main_mod.pyi`；完整 C ABI 头文件见 `stark-sdk.h`（均已保存在本目录下，来自本页面 "进阶参考" 章节的可展开代码块）。

### SDK 仓库说明 ​

`brainco-hand-sdk` 现仅面向 Revo 1 / Revo 2 系列示例；Revo 3 SDK 已迁移到独立的 `brainco-revo3-sdk` 仓库。旧版 `linux/`、`windows/` 示例已归档至 `archive/`，新项目建议使用 `python/demo/` 与 `c/demo/` 统一示例入口。

## 更新日志 ​

### v2.0.2 (2026/05) ​

#### 仓库变更 ​

- Revo 3 SDK 已迁移至独立仓库 brainco-revo3-sdk 。
- brainco-hand-sdk 现仅包含 Revo 1 / Revo 2 示例与预编译库。
- Python Wheel 升级至 cp39-abi3 稳定 ABI，不再支持 Python 3.8。

#### 修复 ​

- BrainCo USBCANFD 适配器新增多通道、多协议（CAN 2.0 / CANFD）自动检测。
- 修复 Revo 2 手动电机校准失败的问题。
- C/C++ ABI 变更（自 2.0.0 起）： Baudrate 枚举值顺序调整（如 BAUD5MBPS 由 6 → 7）

### v1.4.0 (2026/04/15) ​

#### 新增触觉设备类型 ​

- 新增 阵列压阻触觉版 ( Revo2TouchArrayPressure ) — 3D 力与力矩数据采集（Fx, Fy, Fz, Mx, My），通过 ArrayPressureTouchDataBuffer
- C++ 示例： hand_demo 和 hand_monitor 新增 array_pressure 模式
- Python GUI：力/力矩数据 2D 矢量罗盘可视化
- 触觉类型判断 API： is_capacitive_touch() — 电容触觉（Revo1/Revo2 Touch） is_pressure_touch() — 压阻触觉 is_force3d_touch() — 三维力触觉 is_array_pressure_touch() — 阵列压阻触觉

#### Python GUI ​

- 触觉面板：热力图可视化，支持电容/压阻/三维力/阵列压阻/视触觉
- 时序测试：Revo2 Worker，动态频率切换
- 国际化支持（中/英）

#### SDK & API 变更 ​

- 新增硬件类型： Revo2TouchForce3D 、 Revo2TouchArrayPressure
- 新增 API： uses_array_pressure_touch_api()

#### 🐛 问题修复 ​

- 修复 CAN 错误帧处理和自动检测协议分发
- SocketCAN recv_can / recv_canfd 新增 CAN_ERR_FLAG 检查

#### 📚 文档与项目结构 ​

- 弃用的 linux/ 和 windows/ 目录归档至 archive/
- 新增 install_whl.sh 脚本用于 Python Wheel 安装

### v1.1.9 (2026/03/03) ​

#### 🔧 改进 ​

- 串口打开后新增 150ms 预热延迟，提升 Modbus 自动检测首次成功率

#### 📚 新增示例 ​

- c/demo/debug_detect.cpp — C++ 调试工具，用于 Modbus 寄存器检查和原始 Protobuf 自动检测

### v1.1.6 (2026/02/28) ​

#### 🚀 新增功能 ​

- 自动检测新增 BrainCo USBCANFD 适配器支持

### v1.1.5 (2026/02/09) ​

#### 🐛 问题修复 ​

- 修复 CANFD 边界检查问题
- SocketCAN 扫描改为遍历所有接口

#### 🚀 新增功能 ​

- SocketCAN Python 绑定（ init_socketcan_canfd 、 close_socketcan 、 socketcan_scan_devices ）
- 设备上下文查询 API： stark_get_protocol_type 、 stark_get_port_name 、 stark_get_baudrate 等
- CAN 设备初始化： init_device_handler_can() / init_device_handler_can_with_hw_type()
- StarkProtocolType::Auto = 0 枚举值，支持自动检测所有协议

#### 🔧 示例改进 ​

- 运行时 CAN 后端选择，新增通信频率测试工具

### v1.1.3 (2026/02/06) ​

#### 🚀 新增功能 ​

- SocketCAN 内置支持 (Linux) - 无需外部代码
- Protobuf 协议支持 - Revo 1 串口协议，波特率 115200，Slave ID 10-254

### v1.1.0 (2025/02/05) ​

#### 🚀 新增功能 ​

- ZQWL CAN 适配器内置支持（Linux / macOS / Windows，无需额外 DLL）
- 统一设备自动检测 API： auto_detect() → init_from_detected() → close_device_handler()
- Stark 1.8 触觉能力支持（RS-485 / CAN 协议）
- 跨平台 C++ 示例（ c/demo/ ）
- Python GUI 调试工具（电机控制、触觉数据、波形监控）

#### ⚠️ Breaking Changes ​

- 硬件类型枚举重构，新增 Revo1Advanced / Revo1AdvancedTouch
- API 重命名： is_revo1() → uses_revo1_motor_api() ， is_revo2() → uses_revo2_motor_api()
- 初始化拆分： init_config() → init_logging() + init_device_handler()
- C 结构体添加 C 前缀（如 MotorStatusData → CMotorStatusData ）
- linux/ 和 windows/ 示例目录已弃用，请迁移至 c/

#### 📚 迁移指南 ​

详见 [CHANGELOG.md](https://github.com/BrainCoTech/brainco-hand-sdk/blob/main/CHANGELOG.md)

### v1.0.6 (2025/01/26) ​

#### 🚀 新增功能 ​

- 新硬件支持 : Revo1Advanced（ BCMEL/BCMER ）、Revo1AdvancedTouch（ BCMTL2/BCMTR2 ）
- 序列号自动识别 : 根据序列号前缀自动判断硬件类型
- 状态读取优化 : 多线程异步采集，应用层被动读取，性能大幅提升
- SocketCAN 支持 : Linux 平台新增 SocketCAN 后端
- 自动检测增强 : 多端口遍历、协议自动识别、Quick/Full 扫描模式

#### 🐛 问题修复 ​

- 修复物理量模式下参数 1000 倍缩放问题
- 修复 RS-485 固件升级波特率检测问题

#### 📚 示例更新 ​

- 新增 revo2_touch_collector.py 马达/触觉状态读取示例
- 新增 revo2_timing_test_gui.py 时序测试 GUI 工具

#### ⚠️ 注意 ​

- Revo1Advanced 系列设备请使用 revo2 目录下的示例代码

### v1.0.1 (2025/12/23) ​

#### 🚀 新增功能 ​

- 支持 EtherCAT 多从站通信

### v1.0.0 (2025/12/08) ​

#### 🎉 正式版本 ​

- 细节优化，正式升级到 1.0 版本

### v0.9.9 (2025/11/19) ​

#### 🚀 新增功能 ​

- 支持 Revo 1 进阶版设备 ⭐
- 统一控制参数范围：位置控制 0~1000，速度/电流/PWM 控制 -1000~+1000
- 适用于 Modbus、CANFD 和 CAN2.0 所有通信协议

#### ⚠️ 重要提示 ​

- Revo 1 进阶版设备需要 SDK v0.9.9 或更高版本

### v0.9.8 (2025/11/04) ​

#### 🚀 新增功能 ​

- CAN/CANFD 协议 : 完整的 Revo 2 CAN2.0/CANFD 通信协议栈
- ZLG CAN 支持 : Python 接口（Windows/Linux），含驱动库封装
- CANFD 分块读写 : 支持超过 29 个寄存器的大数据传输
- EtherCAT 触觉 : 触觉传感器数据采集（PDO/SDO）
- 压力触觉支持 : EtherCAT/CANFD/RS-485 多协议通信
- 保护电流接口 : ProtectedCurrent 读写，支持 CAN/EtherCAT
- 动作序列执行 : run_action_sequence 支持 CAN 2.0
- 设备类型判断 : 基于序列号判断（ get_hardware_by_sn ）
- 多设备混用 : 支持 Revo1/Revo2 同时使用

#### ⚡ 高频通信优化 ​

- C/C++ 异步调用 : set 类指令改为异步执行，避免阻塞主线程
- 高频接口禁用重试 : get/set_finger_* 、触觉读取等，避免指令堆积
- 低频接口重试优化 : 设备信息读取等降至 2 次重试

#### 🐛 问题修复 ​

- 修复 TurboConfig 字节序、Modbus C API 异步调用、OTA 升级包大小处理等问题

### v0.9.0 (2025/09/20) ​

#### 🚀 新增功能 ​

- 新增二代灵巧手 CAN2.0 协议支持
- 新增二代灵巧手触觉版 EtherCAT 协议支持

#### 🐛 问题修复 ​

- 修复已知问题，提升稳定性

### v0.8.8 (2025/09/02) ​

#### 🚀 新增功能 ​

- 新增二代灵巧手 CAN2.0 协议支持

### v0.8.6 (2025/08/20) ​

#### 🚀 新增功能 ​

- 新增一代灵巧手 CAN2.0 协议支持

#### 🔧 API 改进 ​

- 函数命名优化： modbus_ → stark_ （更通用的前缀，适用于 CANFD 和 EtherCAT） create_device_handler() 更新命名，避免歧义 canfd_init(uint8_t master_id) 更新命名，避免歧义 init_cfg 不再需要传递固件类型，通过 get_device_info 接口自动获取

#### 📚 示例代码 ​

- ROS 2 示例增加 CAN/CANFD 支持
- 新增 CAN/CANFD C++ 示例

### v0.7.8 (2025/07/31) ​

#### 🚀 新增功能 ​

- EtherCAT 固件增加 DC 同步功能

### v0.7.3 (2025/07/28) ​

#### 🐛 问题修复 ​

- 修复 CANFD 通信中协议 MasterID 配置问题
- 修复 EtherCAT 通信中 PDO 读取位置问题
- 修复 MotorStatusData 中 description 字段错误

### v0.7.0 (2025/07/18) ​

#### 🚀 新增功能 ​

- 自动检测 Modbus 从机波特率和设备 ID
- 二代灵巧手触觉版接口支持

#### 🐛 问题修复 ​

- 修复 DFU 固件升级问题

### v0.6.2 (2025/07/14) ​

#### 🚀 新增功能 ​

- EtherCAT 协议支持 ：二代灵巧手支持 EtherCAT 高速通信协议
- 扩展 RS485 波特率 ：新增 1M、2M 和 5M 波特率选项，满足高性能应用需求
- Linux 串口性能优化 ：默认启用 LOW_LATENCY 模式，460800 波特率下通信频率提升 400%

#### 🔧 API 改进 ​

- 重构设备上下文命名： ModbusContext → DeviceContext （更通用的设备上下文） ModbusHandler → DeviceHandler （统一设备操作接口）

#### 🛠 开发工具 ​

- 集成 python-stub-gen 工具链，自动生成类型存根文件（.pyi），提升 IDE 智能提示和类型检查体验

#### ⚠️ 兼容性提示 ​

- 从旧版本迁移时，请注意 API 命名变更

### v0.5.3 (2025/06/12) ​

#### 🚀 新增功能 ​

- 自定义 Modbus 读写接口
- 动作序列控制
- LED、蜂鸣器、震动马达控制接口

#### 🐛 问题修复 ​

- 修复实时位置/速度/电流返回值错误

### v0.4.5 (2025/05/14) ​

#### 🚀 新增功能 ​

- 新增 get_single_touch_status 单指触控状态检测

#### 🔧 改进 ​

- 简化 modbus_open 函数，移除冗余参数

### v0.4.4 (2025/05/06) ​

#### 🚀 新增功能 ​

- 新增 CANFD 协议支持

#### 🐛 问题修复 ​

- 修复多个已知问题，提升稳定性

### v0.3.6 (2025/04/16) ​

#### 🎉 里程碑 ​

- 首次支持二代灵巧手
- 建立基础通信框架

### v0.1.9 (2025/03/17) ​

#### 📌 初始版本 ​

- Modbus 通信协议
- Python / C 代码集成
- SDK 基础架构
