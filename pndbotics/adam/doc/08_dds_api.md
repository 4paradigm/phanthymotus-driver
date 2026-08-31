# DDS通信API

> Source: https://wiki.pndbotics.com/robot/dds_api/

## DDS 简介

DDS（全称 Data Distribution Service 数据分发服务），是一个中间件，由OMG发布的分布式通信规范，采用发布/订阅模型，提供多种QoS服务质量策略，以保障数据进行实时、高效、灵活地分发，可满足各种分布式实时通信应用需求。

pnd_sdk_python 是在DDS上做了一层封装，支持 DDS 组件的 Qos 配置，对应用开发提供简单的封装接口，并实现了基于 DDS Topic 的 RPC 机制，适用于在 Adam 内部进程间以及 Adam 外部与内部的进程间通过发布/订阅和请求/响应两种方式完成不同场景下的数据通信。

---

## 接口说明

### pndbotics_sdk_py.core.channel.ChannelPublisher

ChannelPublisher 类为指定类型的消息实现了消息发布功能。

**构造函数**: `ChannelPublisher(self, name: str, type: Any)`

#### Init

| 项目 | 说明 |
|------|------|
| 函数原型 | `def Init(self)` |
| 功能描述 | 初始化 Channel，准备发送消息。 |
| 参数 | 无 |
| 返回值 | 无 |
| 备注 | 初始化通道的写入器。确保通道已准备好发送消息。 |

#### Close

| 项目 | 说明 |
|------|------|
| 函数原型 | `def Close(self)` |
| 功能描述 | 关闭通道写入器，停止发布消息。 |
| 参数 | 无 |
| 返回值 | 无 |
| 备注 | 调用 Close 后，发布器在重新初始化之前无法再写入任何消息。 |

#### Write

| 项目 | 说明 |
|------|------|
| 函数原型 | `def Write(self, sample: Any, timeout: float = None)` |
| 功能描述 | 发送指定类型的消息（样本）。 |
| 参数 | `sample: Any` (待写入的数据，可以是任何 Python 对象) |
| 返回值 | `self.__channel.Write` 的返回值 |
| 备注 | 如果提供了 timeout 参数，它表示等待写入操作完成的最长时间。 |

---

### pndbotics_sdk_py.core.channel.ChannelSubscriber

ChannelSubscriber 实现了指定类型消息的订阅功能。

**构造函数**: `ChannelSubscriber(self, name: str, type: Any)`

#### Init

| 项目 | 说明 |
|------|------|
| 函数原型 | `def Init(self, handler: Callable = None, queueLen: int = 0)` |
| 功能描述 | 初始化通道，设置读取器和处理器，并可选择指定队列长度。 |
| 参数 | `handler: Callable = None` (可选，用于处理消息的回调函数) `queueLen: int = 0` (可选，指定队列长度的整数) |
| 返回值 | 无 |
| 备注 | 仅初始化通道一次（使用 `self.__inited` 标志防止重复初始化）。如果 handler 为 None，则使用默认处理器。 |

#### Close

| 项目 | 说明 |
|------|------|
| 函数原型 | `def Close(self)` |
| 功能描述 | 关闭与通道关联的读取器，并将其标记为未初始化状态。 |
| 参数 | 无 |
| 返回值 | 无 |
| 备注 | 此函数停止从通道读取并清理状态。调用 Close 后，必须重新初始化通道才能再次使用。 |

#### Read

| 项目 | 说明 |
|------|------|
| 函数原型 | `def Read(self, timeout: int = None)` |
| 功能描述 | 从通道读取数据，带有一个可选的超时参数。 |
| 参数 | `timeout: int = None` (可选，读取操作的超时时间，单位为秒) |
| 返回值 | 取决于通道 Read 方法的实现，通常是读取到的数据，如果超时则可能返回 None。 |
| 备注 | 如果未提供 timeout 参数，通常表示读取操作没有超时限制。 |

---

### pndbotics_sdk_py.core.channel.ChannelFactoryInitialize

此函数用于初始化通道环境。

| 项目 | 说明 |
|------|------|
| 函数原型 | `def ChannelFactoryInitialize(id: int = 0, networkInterface: str = None)` |
| 功能描述 | 初始化 ChannelFactory，以在指定域中创建和管理通道。 |
| 参数 | `id: int = 0` (DDS 参与者的域 ID) `networkInterface: str = None` (可选，用于通道的网络接口名称) |
| 返回值 | 如果初始化失败则引发异常，否则返回 None。 |
| 备注 | 确保在使用通道之前创建 DDS 域和参与者。 |
