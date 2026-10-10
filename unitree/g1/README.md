# Unitree G1 Driver

G1 Driver 通过 MCP 提供运动控制、机械臂动作、麦克风、扬声器、LED 与状态监控。启用配置见 [config.yaml](config.yaml)，构建入口见[仓库 README](../../README.md)。

现有 `arm.release` 使用厂商 action 99；连续控制入口包括 [servo.py](servo.py) 和 [servo_eef.py](servo_eef.py)。SDK 请求成功与实测动作完成是不同状态。

## 双臂遥操

`teleop_control` 接收设备 Driver 的输入，在 G1 Driver 内完成相对映射、独立进程 IK 和双臂位置下发。默认注册该卡，市场清单包含 `teleop_control`。随镜像提供原厂 G1_23（每臂 5 关节、固定假手）的完整固定几何 profile；不依赖现场临时标定脚本或用户填写路径。Core 和 ActuCore 无需遥操专用修改。

1. 安装 PICO Driver，在 Canvas 添加 `teleop_device` 和 `teleop_control`，连接命令输出与控制卡输入。
2. 在设备卡齿轮页查看安装和配对说明，连接 PICO。
3. 启动项目，先松开双握把；控制卡收到有效输入后建立初始相对基准。
4. 同时按住双握把控制双臂。松开任一握把暂停，重新握住沿用原基准；跟踪空间重置后需松握重新就绪。
5. 从 Canvas 停止项目；收臂复用既有 `arm.release`，它先取消遥操输入及待发目标再交还 SDK，之后需从 Canvas 重新启动遥操。Driver 重启后也需停止再启动项目恢复绑定，无需删线重连。

用户不需要填写实例 ID、模型路径、位移比例或 Shadow/Live。这些由框架和 Driver 预设管理；启动项目本身不发送运动目标。支持 FSM 500、801，不自动切换本体模式。

适用范围仅为原厂 G1_23 固定假手，不自动适配 G1_29、灵巧手或改装 TCP。附加碰撞与工作区扫描默认关闭；Driver 不自动停止其他控制程序。`servo`、`servo_eef` 可同时注册，但本实现不提供统一控制器仲裁。

## 状态与恢复

Canvas 显示输入等待、过期和传输错误等状态。IK 子进程超时或退出后，控制卡暂停并等待后续有效输入恢复。MCP 在线不代表 DDS 接收正常。Driver 不持久化或自动恢复运动租约。

模型、话题、平滑参数、诊断与测试见 [G1 数值进程说明](g1_motion/README.md)。

## ROS 配置来源

ROS 初始化沿用主线 `rclpy.init()`，读取部署环境变量；Driver 不新增初始化校验或内置 XML 副本。
`deploy/service.yml` 统一挂载 `/opt/phanthy-motus/dds-local.xml`，指定该 profile 和 `ROS_DOMAIN_ID=42`。
开发和测试由启动环境显式提供相同配置。遥操开关不覆盖配置；Unitree SDK 的 domain 0 与网卡配置不变。
