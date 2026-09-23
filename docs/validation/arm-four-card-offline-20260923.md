# G1 arm 四卡离线实现与验证

2026-09-23；范围为北京 G1_23 的 arm 执行端。测试对象是工作树候选（文档基点 `c975093c69fa143e421ee92d2440bc2fce4c1128` 后的未提交实现），不是已部署镜像。完整实施与分阶段门禁见[契约计划](../plans/g1-dual-arm-motion-contract.md)。

## 已实现

- `arm_stream.py` 提供 SDK 独立执行循环、纯位置 10 维 `motus.control/2` 接收、连续限速、严格源截止时间、有限大小的包络缓存和每拍身份/范围检查。默认开发速度 1 rad/s，遵守模型速度限制，实际 `dt` 上限 20 ms，20 Hz 输入之间继续推进，不累计断流后的运动额度。
- 基于最终限速 q，以独立 Pinocchio 模型/Data 计算 `RNEA(q, 0, 0)`；检查力矩维度、有限值和标定/URDF 限额，计算失败不会默认为零力矩。arm 不运行 IK/FK/碰撞搜索。配置 hash 在状态提交前核验，失败保留旧配置。
- 新鲜实测起步；短时反馈中断保持，超过故障期限锁存原因；过期或包络不足保持后只接受新鲜目标。同会话可恢复保持不重建相对映射，显式 pause/resume 使用新租约防止旧帧回放。
- 原有 release 与两种 execute-99 别名统一进入 SDK 交接：撤销目标、保持、权重交还、五次实际成功的零权重写入、新鲜实测静止、调用既有 99、再等新鲜实测静止。SDK `ret=0` 与完成分开；未获得 SDK 流的旧手势路径直接用 99，不伪造零权重写入证据。没有新增回零轨迹或零位假设。
- operation_id 回执幂等；未知 RPC 结果不自动重发，需显式 `retry=true`、新 operation_id、当前无未返回 RPC 且新鲜实测静止。旧手势 RPC 返回后仍保留逻辑占用，直到显式 release 实测确认；管理 claim/resume 支持有界 nonce 重试与丢回执取消。
- 显式 stop 可取消尚未发送的 99，同时继续安全交还。99 已发送后 SDK 没有取消/动作状态接口，stop 返回 `unknown/release_action_stop_unconfirmed` 并保留占用，不伪报厂商动作已取消或静止，也不静默重新接管。
- `arm_sdk.publish_stream` 只写十个 G1_23 手臂关节，保留周六晚 visible-follow 的肩肘 `80/3`、腕 `40/1.5` 增益和 mode=1；普通 servo 的现有增益保持不变。只有 SDK Write 明确返回 True 才推进实际发布序号。
- SDK 只移植 MatchedPublisherCount / PublicationHandle 两个只读统计接口，用于观察发布者；原有手势由实际 ArmActionPlugin 路径仲裁。没有增加 base 位移或累计行程门限。

## 实际验证

执行环境为本机离线 Python 3.11.10 / Pinocchio 3.7 测试环境；没有 ROS、DDS 设备或机器人连接。命令在 Driver 根目录运行：

```sh
/private/tmp/four-card-motion-tests-20260923/bin/python -m pytest \
  unitree/g1/tests/test_arm_stream.py \
  unitree/g1/tests/test_arm_card_binding.py \
  tests/test_g1_arm_sdk.py tests/test_g1_servo.py tests/test_g1_servo_eef.py -q -rs
```

结果：**94 passed，1 skipped，1.93 s**。跳过项为旧 `test_g1_servo_eef.py:362` 的 ROS String 状态发布测试，当前宿主没有该消息类型；不能计为通过。

| 已测试路径 | 证据等级 / 观察 |
|---|---|
| 20 Hz 目标、10 ms 执行拍、1 rad/s、截止时间与长调度间隔 | 确定性时钟与有限速度替身；每拍持续推进，过期保持，无追账突跳 |
| 完整目标超出近期包络、身份/代次/模型/标定不一致 | 真实协议及包络检查；越界不继续发布，新鲜证明可续接 |
| RNEA 错误、力矩超限、SDK Write=False、硬件反馈故障 | 注入失败；无零力矩降级、不虚增 applied_sequence、不自动清故障 |
| release 的五次零权重写入、RPC ACK 与实测完成 | SDK/反馈替身；ACK 后仍保留占用，必须获得之后的新鲜静止样本 |
| RPC 阻塞/异常、未知结果、相同 operation_id、遗留手势 | 后台 RPC 替身；管理线程可返回未知，别名不绕过仲裁，不盲重发 |
| release 中途 stop、SDK 两个只读统计接口 | 99 发送前取消，已发送后未知；实际 SDK 类 AST 验证匹配计数与 unsigned handle，错误 handle 不伪报有效 |
| arm start 输入 topic、joint descriptor，旧卡片 execute/release 路径 | 当前源码的实际类/AST 路径；错误 topic 或 EEF/14 维 descriptor 不会 ready |
| RNEA 与配置 | 实际 G1_23 URDF 与 Pinocchio；十关节非零重力输出、原子 hash 拒绝。跨 Pin 3.1/3.7 的 101 姿态对照由数值实现验证记录单独提供 |

## 未验证及后续门禁

当前不勾选完整离线阶段：目标 Linux ARM64 镜像、实际 ROS/domain 42 loopback/QoS、真实 SDK 写入时延与发布者事件、总线局部恢复和真实低层状态均仍须验证。测试的固定时钟/SDK/有限速度 plant 不能替代这些证据。

本轮没有连接或操作北京 G1，没有构建/部署机器人镜像，没有申请 BOT review，没有开发或最终真机验收。严格继续执行“离线测试 → 开发真机 → 确切 HEAD BOT → 最终真机”。

09/19 周六晚 visible-follow/r3/V036 是上海行为基线来源；缺失的冻结源码完整清单、镜像 ID/digest 和现场标定 hash 仍按计划列缺。不能把上海记录称为北京通过；action 99 后自然姿态也须现场确认，离线静止回执不定义自然零位。
