# Revo 2 通讯协议概述 (https://www.brainco-hz.com/docs/revolimb-hand/revo2/introduction.html)

Source: (see original URL)

# 通讯协议 ​

BrainCo Revo 2 灵巧手最多支持三种对外通信接口：RS485、CAN FD 和 EtherCAT。

## Modbus-RTU/CANFD 协议 ​

用于机器人灵巧手，协议内容完全开放，可以通过下方协议内容直接控制灵巧手各种功能，也可以通过 SDK 和上位机进行控制。在此协议下，可以在一根总线上挂多只灵巧手或者设备，增加了通用性。做到兼容性协议设计，设计一套兼容 485 和 CANFD 两种通信接口的协议，在兼容不同通信协议的基础上确保通信的高效性、可靠性和易扩展性。

另外，Revo 2 基础版灵巧手通过拨码开关实现 CAN FD 和 Modbus-RTU协议切换，拨码开关在手腕下方，拨到 CAN FD 一侧为 CAN FD协议，拨到 Modbus-RTU 一侧为 Modbus-RTU 协议。

二代手目前有三个版本：

- V2-BASIC：二代灵巧手-基础版
- V2-PRO：二代灵巧手-进阶版
- V2-TOUCH：二代灵巧手-触觉版

## EtherCAT 协议 ​

用于机器人灵巧手，EtherCAT 通信具备高实时性、低延迟和精确同步控制的优点，采用 EtherCAT 协议可以满足高自由度灵巧手实时控制的硬性需求。灵巧手集成力觉、触觉等多模态传感器，数据量庞大，EtherCAT 的高效数据封装和硬件级时间戳能同步处理关节控制与传感器反馈，避免多协议混用的复杂性。
