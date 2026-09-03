# PND Adam 人形机器人 - 产品说明

> Source: https://wiki.pndbotics.com/robot/humanoid_robot/

Adam 是一款融合尖端硬件架构与智能算法平台的全身动力机器人，具备卓越的运动灵活性、环境感知能力与高精度动态控制性能。其仿生结构设计与类人形态运动能力，使其能够在非结构化环境中执行高复杂度任务，广泛应用于科研实验、工业自动化、医疗辅助及服务机器人等多个前沿领域。

---

## 核心特性

### 机械系统架构

- **执行器系统**：集成多达 41 个准直驱（Quasi-Direct Drive）柔性力控关节，实现高动态精度控制与多自由度灵巧运动。
- **模块化架构**：采用全系统模块化执行器设计，优化生产与维护效率，适配规模化制造需求。
- **仿生髋关节设计**：创新性仿生髋部结构，显著增强机器人在复杂操作任务中的运动拟人性与灵活性。
- **下肢关节系统**：搭载 4 组高扭矩密度、多级减速比、高响应灵敏度的准直驱力控关节，提供卓越的步态性能与动态稳定性。

### 控制系统与通信架构

- **PND-Network**：专有高实时通信协议栈，支持激光雷达、视觉相机等多模态传感器的扩展接入，强化环境感知与闭环控制能力。
- **中央控制单元**：搭载 **Intel NUC12WSKi7** 处理器及自研机器人控制单元（RCU），统筹机器人关节驱动、能源管理、通信调度与系统监控。
- **AI 计算平台**：**Adam Standard/SP** 集成 **NVIDIA Jetson Orin NX 16GB** 嵌入式模块，专用于高性能 AI 推理、视觉计算与实时决策任务。
- **感知系统**：配备 **Intel RealSense D455** 深度视觉传感器，实现高精度三维环境建模与实时空间定位。

### 动态平衡与运动规划

- **WBC + MPC 算法架构**：基于全身动力学控制（Whole-Body Control, WBC）与模型预测控制（Model Predictive Control, MPC）的融合策略，保障机器人在动态扰动环境下的平衡鲁棒性与轨迹跟踪精度。
- **强化学习与模仿学习**：依托大规模仿真环境与深度神经网络训练，结合强化学习与模仿学习策略，持续迭代优化运动策略与控制性能。

### 能源与续航管理

- **智能电池管理系统（BMS）**：集成高精度电量监控与动态功率分配机制，支持持续高强度任务运行。
- **系统能效优化**：采用低功耗硬件设计与动态功耗管理策略，显著提升整机续航能力。

### 通信与系统扩展

- **多模通信能力**：兼容 **5G、Wi-Fi 6、蓝牙 5.0** 协议，实现高带宽、低时延的远程操控与实时调试。
- **传感器扩展接口**：PND-Network 具备优异的接口扩展性，支持多源传感器融合与系统功能定制，赋能复杂场景下的感知与决策能力。

---

## 应用领域

- **科研与实验室场景**：适用于高精度自动化实验、动态系统测试与数据采集分析等研究任务。
- **工业自动化产线**：在精密装配、在线检测、柔性制造等环节实现高可靠性作业与流程优化。
- **医疗辅助与康复**：提供手术协作、康复训练支持与病患监护等服务，适配多样化医疗场景。
- **服务与协作机器人**：适用于物流分拣、接待导引、人机协作等智能服务应用，具备高度场景适应性。

---

## 技术参数

| 型号 | Adam Lite | Adam Standard | Adam SP | Adam Pro |
|------|-----------|---------------|---------|----------|
| 全身尺寸 | 1.67m | 1.67m | 1.67m | 1.67m |
| 整机重量 | 60kg | 61kg | 62kg | 63kg |
| 全身自由度 | 25自由度 | 29自由度 | 41自由度 | 43自由度 |
| 单腿自由度 | 髋关节x3 + 膝关节x1 + 踝关节x2 = 6 | 同左 | 同左 | 同左 |
| 单手臂自由度 | 肩关节x3 + 肘关节x1 + 小臂关节x1 = 5 | 肩关节x3 + 肘关节x1 + 小臂关节x1 + 手腕关节x2 = 7 | 肩x3+肘x1+小臂x1+手腕x2+灵巧手指x6=13 | 肩x3+肘x1+小臂x1+手腕x2+灵巧手指x6=13 |
| 腰自由度 | 3自由度 | 3自由度 | 3自由度 | 3自由度 |
| 关节单元极限扭矩 | 膝关节约340N·m，髋关节约340N·m，踝关节约46N·m，手臂关节约60N·m | 同左 | 同左 | 同左 |
| 行走速度 | 最大1.5m/s | 最大1.5m/s | 最大1.5m/s | 最大1.5m/s |
| 电池 | 容量1172W·h，最大电压46.2V，最大输出电流25A | 同左 | 同左 | 同左 |
| 通信连接 | Wi-Fi 6，蓝牙5.0 | 同左 | 同左 | 同左 |
| 控制和感知算力 | 自研WBC+MPC控制算法 | 同左 | 同左 | 同左 |
| 主控制系统 | NUC12WSKi7 | NUC12WSKi7 | NUC12WSKi7 | NUC12WSKi7 |
| AI运算系统 | / | NVIDIA Jetson Orin NX 16GB | NVIDIA Jetson Orin NX 16GB | NVIDIA Jetson Orin NX 16GB |
| 感知传感器配置 | / | Intel Realsense D455 | Intel Realsense D455 | ZED MINI |
| 头 | / | 有 | 有 | 2自由度 |
| 手 | 球形手 | / | 灵巧手 | 灵巧手 |

---

## 硬件架构

（参见原文硬件架构图）

---

## 控制计算模块

### 运动控制单元（小脑）

| 参数 | 规格 |
|------|------|
| 型号 | NUC12WSKi7 |
| CPU | 12th Gen Intel Core i7-1260P |
| 内核数 | 12核 |
| 线程数 | 16线程 |
| 最大睿频频率 | 4.7GHz |
| 内存 | 16G |
| 内存类型 | DDR4 3200MHz |
| 缓存 | 18MB |
| 存储 | 128G |
| GPU | Intel Iris Xe Graphics |
| 显卡最大动态频率 | 1.40 GHz |
| 英特尔深度学习提升 | 是 |
| 英特尔Adaptix技术 | 是 |
| 英特尔超线程技术 | 是 |
| 指令集 | 64bit |

### 感知决策单元（大脑）

| 参数 | 规格 |
|------|------|
| 型号 | NVIDIA Jetson Orin NX 16GB |
| AI性能 | 100 TOPS |
| GPU | 搭载32个Tensor Core的1024核NVIDIA Ampere架构GPU |
| CPU | 8核Arm CortexA78AE v8.2 64位CPU 2MB L2+4MB L3 |
| 显存 | 16GB 128位 LPDDR5 102.4GB/s |
| 存储 | 128G M.2 SSD 固态硬盘 |
| 视频编码 | 1x 4K60(H.265) 3x4K30(H.265) 6x 1080p60(H.265) 12x 1080p30(H.265) |
| 视频解码 | 1x8K30(H.265) 2x4K60(H.265) 4x4K30(H.265) 9x 1080p60(H.265) 18x 1080p30(H.265) |
| 摄像头 | 2x MIPI CSI-2 D-PHY通道 |
| USB | 4x USB 3.2 接口 1x USB Type-C接口 |
| 显示接口 | 1x DisplayPort |
| 网络 | 千兆以太网 |
| 其他I/O | 40针接头(UART、SPI、I2S、I2C、PWM、GPIO) 4针风扇接头，直流电源插座 |
| 规格尺寸 | 103x90x34 (mm) |

---

## 关节参数

### Adam Lite

| 关节名称 | 执行器型号 | 限位（rad） | 备注 |
|----------|-----------|------------|------|
| shoulderPitch_Left/Right | PND-50-14A-50-S | +2.0420～-3.6138 | 向后为正，向前为负 |
| shoulderRoll_Left/Right | PND-50-14A-50-S | 左：+3.1416～-0.6283 右：+0.6283～-3.1416 | 左：向外为正，向内为负 右：向内为正，向外为负 |
| shoulderYaw_Left/Right | PND-30-14A-50-S | ±2.5831 | |
| elbow_Left/Right | PND-30-14A-50-S | +0.2094～-2.4958 | 向后为正，向前为负 |
| wristYaw_Left/Right | PND-20-14A-50-S | ±2.6704 | |
| waistRoll | PND-60-17-50-S | ±0.2793 | |
| waistPitch | PND-60-17-50-S | +1.3614～-0.8378 | 向前为正，向后为负 |
| waistYaw | PND-60-17-50-S | ±0.8290 | |
| hipPitch_Left/Right | PND-130A-7F-7-P | ±2.2276 | |
| hipRoll_Left/Right | PND-80-20-50-S | +1.6581～-0.7854 | 向外为正，向内为负 |
| hipYaw_Left/Right | PND-60-17-30-S | ±0.8290 | |
| kneePitch_Left/Right | PND-130-7F-P | +2.4435～0.0000 | |
| anklePitch_Left/Right | PND-50-6F5S-P | +0.4276～-1.0472 | 两关节联动，勾脚为负，反向为正 |
| ankleRoll_Left/Right | PND-50-6F5S-P | ±0.4887 | 两关节联动 |

### Adam Standard

| 关节名称 | 执行器型号 | 限位（rad） | 备注 |
|----------|-----------|------------|------|
| shoulderPitch_Left/Right | PND-50-14A-50-S | +2.0420～-3.6138 | 向后为正，向前为负 |
| shoulderRoll_Left/Right | PND-50-14A-50-S | 左：+2.7925～-0.6283 右：+0.6283～-2.7925 | 左：向外为正，向内为负 右：向内为正，向外为负 |
| shoulderYaw_Left/Right | PND-30-14A-50-S | ±2.5831 | |
| elbow_Left/Right | PND-30-14A-50-S | +0.2094～-2.4958 | 向后为正，向前为负 |
| wristYaw_Left/Right | PND-20-14A-50-S | ±2.6704 | |
| wristPitch_Left/Right | PND-20-08-50-S | ±0.9599 | 两关节联动，球关节活动范围 |
| wristRoll_Left/Right | PND-20-08-50-S | ±0.9599 | 两关节联动，球关节活动范围 |
| waistRoll | PND-60-17-50-S | ±0.2793 | |
| waistPitch | PND-60-17-50-S | +1.3614～-0.8378 | 向前为正，向后为负 |
| waistYaw | PND-60-17-50-S | ±0.8290 | |
| hipPitch_Left/Right | PND-130A-7F-7-P | ±2.2276 | |
| hipRoll_Left/Right | PND-80-20-50-S | +1.6581～-0.7854 | 向外为正，向内为负 |
| hipYaw_Left/Right | PND-60-17-30-S | ±0.8290 | |
| kneePitch_Left/Right | PND-130-7F-P | +2.4435～0.0000 | |
| anklePitch_Left/Right | PND-50-6F5S-P | +0.4276～-1.0472 | 两关节联动，勾脚为负，反向为正 |
| ankleRoll_Left/Right | PND-50-6F5S-P | ±0.4887 | 两关节联动 |

### Adam SP

| 关节名称 | 执行器型号 | 限位（rad） | 备注 |
|----------|-----------|------------|------|
| shoulderPitch_Left/Right | PND-50-14A-50-S | +2.0420～-3.6138 | 向后为正，向前为负 |
| shoulderRoll_Left/Right | PND-50-14A-50-S | 左：+2.7925～-0.6283 右：+0.6283～-2.7925 | 左：向外为正，向内为负 右：向内为正，向外为负 |
| shoulderYaw_Left/Right | PND-30-14A-50-S | ±2.5831 | |
| elbow_Left/Right | PND-30-14A-50-S | +0.2094～-2.4958 | 向后为正，向前为负 |
| wristYaw_Left/Right | PND-20-14A-50-S | ±2.6704 | |
| wristPitch_Left/Right | PND-20-08-50-S | ±0.9599 | 两关节联动，球关节活动范围 |
| wristRoll_Left/Right | PND-20-08-50-S | ±0.9599 | 两关节联动，球关节活动范围 |
| waistRoll | PND-60-17-50-S | ±0.2793 | |
| waistPitch | PND-60-17-50-S | +1.3614～-0.8378 | 向前为正，向后为负 |
| waistYaw | PND-60-17-50-S | ±0.8290 | |
| hipPitch_Left/Right | PND-130A-7F-7-P | ±2.2276 | |
| hipRoll_Left/Right | PND-80-20-50-S | +1.6581～-0.7854 | 向外为正，向内为负 |
| hipYaw_Left/Right | PND-60-17-30-S | ±0.8290 | |
| kneePitch_Left/Right | PND-130-7F-P | +2.4435～0.0000 | |
| anklePitch_Left/Right | PND-50-6F5S-P | +0.4276～-1.0472 | 两关节联动，勾脚为负，反向为正 |
| ankleRoll_Left/Right | PND-50-6F5S-P | ±0.4887 | 两关节联动 |
| hand_Left/Right | / | / | 灵巧手关节 |

### Adam Pro

| 关节名称 | 执行器型号 | 限位（rad） | 备注 |
|----------|-----------|------------|------|
| shoulderPitch_Left/Right | PND-50-14A-50-S | +2.0420～-3.6138 | 向后为正，向前为负 |
| shoulderRoll_Left/Right | PND-50-14A-50-S | 左：+2.7925～-0.6283 右：+0.6283～-2.7925 | 左：向外为正，向内为负 右：向内为正，向外为负 |
| shoulderYaw_Left/Right | PND-30-14A-50-S | ±2.5831 | |
| elbow_Left/Right | PND-30-14A-50-S | +0.2094～-2.4958 | 向后为正，向前为负 |
| wristYaw_Left/Right | PND-20-14A-50-S | ±2.6704 | |
| wristPitch_Left/Right | PND-20-08-50-S | ±0.9599 | 两关节联动，球关节活动范围 |
| wristRoll_Left/Right | PND-20-08-50-S | ±0.9599 | 两关节联动，球关节活动范围 |
| waistRoll | PND-60-17-50-S | ±0.2793 | |
| waistPitch | PND-60-17-50-S | +1.3614～-0.8378 | 向前为正，向后为负 |
| waistYaw | PND-60-17-50-S | ±0.8290 | |
| hipPitch_Left/Right | PND-130A-7F-7-P | ±2.2276 | |
| hipRoll_Left/Right | PND-80-20-50-S | +1.6581～-0.7854 | 向外为正，向内为负 |
| hipYaw_Left/Right | PND-60-17-30-S | ±0.8290 | |
| kneePitch_Left/Right | PND-130-7F-P | +2.4435～0.0000 | |
| anklePitch_Left/Right | PND-50-6F5S-P | +0.4276～-1.0472 | 两关节联动，勾脚为负，反向为正 |
| ankleRoll_Left/Right | PND-50-6F5S-P | ±0.4887 | 两关节联动 |
| hand_Left/Right | / | / | 灵巧手关节 |
| neckYaw | PND-60-17-50-S | ±1.0472 | 颈部偏航关节 |
| neckPitch | PND-60-17-50-S | ±1.0472 | 颈部俯仰关节 |

### PND Dexterous Hand

| 关节名称 | 限位（rad） | 备注 |
|----------|------------|------|
| Thumb1_Left/Right | 0.0873～1.5708 | 内收/外展 |
| Thumb2_Left/Right | 0.4538～1.0821 | 屈伸 |
| Index_Left/Right | 0～1.5533 | 屈伸 |
| Middle_Left/Right | 0～1.5533 | 屈伸 |
| Ring_Left/Right | 0～1.5533 | 屈伸 |
| Pinky_Left/Right | 0～1.5533 | 屈伸 |

PND Dexterous Hand 的各手指关节采用 **线性控制映射**：
- 控制量为 0 时，对应关节处于张开最大角度位置（即角度下限）。
- 控制量为 1000 时，对应关节处于闭合最大角度位置（即角度上限）。

---

**最新修订日期：** 2025-12-03
