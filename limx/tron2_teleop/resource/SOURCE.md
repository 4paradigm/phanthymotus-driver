# 模型和代码来源

`tron2_dach.urdf` 来自 [LimX 官方 DACH_TRON2A 模型](https://github.com/limxdynamics/tron2-robot-description/blob/f547f5bc949f2a4c98e076e61cf6d3ca73d179a0/tron2a/DACH_TRON2A/urdf/robot.urdf)，固定上游提交 `f547f5bc949f2a4c98e076e61cf6d3ca73d179a0`。本副本取自本仓 `44b98789` 的 `limx/tron2/resource/tron2_dach.urdf`，SHA-256 为 `4472bebd4188911c189f95414a182f5fb66cceea3456c72c76a9a6e5654b5e2c`，运行时核对该摘要。

本仓模型保留厂家的关节名称、父子关系、原点、轴和限位，删除视觉网格、碰撞几何、惯性及 MuJoCo 元数据，并添加固定的 `display_hand_gripper_*` 显示节点。本 Driver 只按明确名称读取七关节双臂链，不使用显示节点，也不将模型用作碰撞检查或夹爪标定。未复制网格、USD、图片、SDK 二进制或现场数据。

上游模型使用 Apache-2.0；保留 [LICENSE](LICENSE)、[NOTICE](NOTICE) 和 [第三方声明](THIRD_PARTY_NOTICES.md)。第三方声明中的网格来源待确认项不涉及本副本，因为本副本不包含网格。

本 Driver 代码使用本仓 Apache-2.0 许可证。以下列出参考和直接复用的代码版本；不依赖这些 Driver 的服务或硬件实现：

| 参考 | 使用范围 |
|---|---|
| [PICO #329](https://github.com/4paradigm/phanthymotus-driver/pull/329)，`77a9fa3a` | 原样引入 `common/teleop_contract.py` 及其测试，沿用固定输入/状态 topic 和版本化字段 |
| [G1 #330](https://github.com/4paradigm/phanthymotus-driver/pull/330) | 参考相对位姿、头部偏航对齐和松握建立基准的语义；未复制 G1 硬件执行层 |
| [PiPER #395](https://github.com/4paradigm/phanthymotus-driver/pull/395)，`113fdcdf` | 复用 `common/motion` 数值工作进程，并增加 `forward` 操作；参考 NumPy FK/IK 实现，改为 TRON2 七自由度和真实姿态冗余约束 |

厂家协议依据：[TRON2 SDK 文档](https://www.limxdynamics.com/zh/documents/847884267345285120) 的关节状态、关节限位与 ServoJ 部分。16 维顺序为左臂七维、右臂七维、头部俯仰和偏航。URDF 文件遍历顺序不能作为电机顺序。厂家 ServoJ 推荐 300 Hz；该建议不是本实现的实机时序证据。
