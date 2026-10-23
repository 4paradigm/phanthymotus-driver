# Adam Pro arm_gesture

`plugins.arm_gesture.enabled`（默认 true）依赖 `plugins.arm.enabled`。卡片与
`arm_control`、`waist_control`、`head_control` 共用一个 31 关节控制器，保留主线
关节映射和 PD 增益；命令采用固件包络内的关节限位、五次平滑及逐帧速度上限，不需要额外依赖或控制进程。

| 动作 | side | duration_s | 手型与结束状态 |
| --- | --- | --- | --- |
| wave | left / right，默认 right | 抬手阶段过渡时长 | open_palm；抬手、摆动、回到本次接管姿态 |
| salute | left / right，默认 right | 敬礼过渡时长 | flat_hand；结束后保持敬礼 |
| reset | left / right / both，默认 both | 复位过渡时长 | 保持当前手型；回到本次控制器接管时的手臂实测姿态 |
| stop | 无 | 无 | 取消序列、冻结最后已写目标、渐退上肢增益、停止低层发布 |
| info | 无 | 无 | 反馈可用性、写入情况、错误；不判定实测到位 |

wave / salute / reset 每次调用必须明确提供布尔值 `confirm=true`，默认 false。
卡片将显示“确认现场安全与控制交接”；缺失、false、字符串或数字均在启动控制器及
联动手型之前返回 `CONFIRMATION_REQUIRED`。动作标记为危险动作；stop / info 不要求
该参数。此显式确认不能替代控制域、实测反馈或固件限位检查，也不证明硬件已安全验收。

过渡时长范围为 0.1–60 秒，是期望下限；安全限速可能延长过渡。挥手摆动及放下
所需时间另计，不能把 `duration_s` 当成整段挥手的总时长。未启用 hand 时仅执行
手臂轨迹；启用 hand 后，手型失败会拒绝手势并冻结已经接受的手臂目标。

每个动作返回唯一 `action_id`，最终向 ACP 报告 `completed`、`error` 或
`cancelled`。`completed` 需要最终指令实际写入，结果的
`completion_basis="commanded trajectory"`、`measured_arrival=null` 明确表示没有
核验实测到位。DDS 接受目标不等于机器人到达目标。

再次执行手势会取消前一个动作；手臂、腰或头的新目标也会取消手势并冻结未完成的
旧轨迹。stop 后再次执行手势会重启读取器，重新获取完整反馈。启动卡片本身不下发
运动目标。持续监测反馈，默认超过 `plugins.arm.feedback_timeout_s=0.5` 秒未收到
完整有效反馈时冻结并释放上肢增益，ACP 回报 error；需要 stop / start 并取得新反馈
后才能恢复，不会自动续播旧轨迹。

低层发布占用时，厂商 loco / motion / tracking_motion 执行请求返回
`CONTROL_BUSY`，应先 stop 上肢；厂商动作接受后，上肢动作同样被阻止，直到同一
厂商卡片显式 stop 成功。不按估计时长释放厂商所有权。tracking_motion 默认关闭，
现有卡片无 stop 接口，若手动启用并执行，需在厂商侧确认停止并重启驱动后再使用
低层控制。此互斥仅覆盖本驱动的调用，外部厂商客户端仍需操作员协调。

## 控制权与现场故障边界

[2026-10-22 现场故障及修复说明](arm_gesture_incident_20261022.md)。旧镜像
`release.261022.4fb8812` 的动作验收出现故障，不应继续测试该版本。

低层动作要求查询成功且控制域为 Traditional（0）；RL（1）或未知状态返回
`CONTROL_CONFLICT`，不自动切换域或 FSM。操作员还必须确认厂商开发者模式，
不能仅以机器人站立、有 lowstate 或 RL 的 STOP 状态判定可以接管。
活动期间独立监测控制域；确认过期或丢失即停止发布，避免与厂商控制流抢写。
此查询不提供跨进程原子租约，其他外部 DDS 写入者仍需操作员协调。

肩俯仰目标限制在实际服务包络内，wave / salute 起势为 −45°，不再使用 −85°。
控制器持续更新实测姿态，在首次接管时才捕获全身保持角；延迟不能绕过逐帧限速。
修复仅经过离线验证，尚未部署或重新进行真机动作验收。

## 用户部署后的网页及硬件验收

1. 使用 PR bot 给出的 Try it 镜像更新驱动，打开画布，确认出现 arm_gesture 卡片，
   同时保留电池、手部、腰、头及摄像头等已有卡片。
2. 启动项目，先查看 info 的 `lowstate_ready`。在操作员确认开发者模式、Traditional 控制域及无其他低层写入者后，按厂商支撑与空间要求，
   依次测试右/左挥手、敬礼、单侧/双侧 reset；检查手型、方向、限速及复位姿态。
3. 动作中 stop，确认停止后不再续播，随后重新执行；连续执行两个动作，确认前一个
   被取消。用手臂、腰、头卡片覆盖手势，确认取消回报及无旧序列反写。
4. 检查 ACP 的完成、取消、失败展示。实测到位需观察真机/关节反馈，不能依赖
   `completed`；检查厂商动作与上肢控制的双向 `CONTROL_BUSY` 和显式 stop 交接。
5. 在受控条件下验收反馈中断后的动作冻结、错误回报及重新启动恢复，并回归 PAC
   电池、手部和其他原有卡片。

以上硬件行为、镜像部署和网页展示均未在离线开发中验证，由机器人操作员完成。

## 共享低层通道的确认规则

arm_control、waist_control、head_control 与 arm_gesture 的每个运动请求均须显式
传入布尔 `confirm=true`。手臂的批量关节、单关节及预设兼容入口也执行相同检查。
未确认时返回 `CONFIRMATION_REQUIRED`，不会提交目标或启动运动。界面动作参数
均提供中文确认字段；start、stop、info 及手臂 get_state 不需要此确认。
此确认只记录操作员意图，不代表厂商已授予独占控制权，也不替代故障排除与现场验收。
