# Unitree G1 Driver

G1 Driver 通过 MCP 提供运动控制、机械臂预设动作、麦克风、扬声器、LED 与状态监控等现有能力。启用配置见 [config.yaml](config.yaml)，构建与开发入口见[仓库 README](../../README.md)和[贡献规范](../../CONTRIBUTING.md)。

现有 `arm` 支持预设手势与 `release`；`release` 使用厂商 action 99。现有连续控制入口为 [servo.py](servo.py) 和 [servo_eef.py](servo_eef.py)，其配置、动作维度、型号探测和运行前提仍以当前实现为准。SDK 返回成功不能替代动作完成的实测确认。

## G1 双臂遥操计划

[G1 双臂 motion_control / arm 契约与验收计划](../../docs/plans/g1-dual-arm-motion-contract.md)定义拟接入的 `ext_vr → teleop → motion_control → arm` 链路，目标为北京 G1_23、每臂 5 关节。该计划区分 EEF 14 个位姿数与 joint 10 个关节角，规定连续执行、内部重力补偿、唯一公开反馈和既有 release 交接，并提供分阶段验收表。

**当前为 Draft 文档，新链路尚未实现、部署或真机验收。** 文档中的端口与行为是待实现契约，不代表 `tools/list` 已提供这些能力，也不替换现有 servo 或手势接口。天轶 #321 与本 G1 PR 并行推进。
