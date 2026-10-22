# Adam Pro arm_gesture

`plugins.arm_gesture.enabled`（默认 true）依赖 `plugins.arm.enabled`。卡片与
`arm_control`、`waist_control`、`head_control` 共用一个 31 关节控制器，保留主线
关节映射、PD 增益、关节限位、五次平滑及速度上限，不需要额外依赖或控制进程。

| 动作 | side | duration_s | 手型与结束状态 |
| --- | --- | --- | --- |
| wave | left / right，默认 right | 抬手阶段过渡时长 | open_palm；抬手、摆动、回到启动姿态 |
| salute | left / right，默认 right | 敬礼过渡时长 | flat_hand；结束后保持敬礼 |
| reset | left / right / both，默认 both | 复位过渡时长 | 保持当前手型；回到本次控制器启动时的手臂实测姿态 |
| stop | 无 | 无 | 取消序列、冻结最后已写目标、渐退上肢增益、停止低层发布 |
| info | 无 | 无 | 反馈可用性、写入情况、错误；不判定实测到位 |

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

## 用户部署后的网页及硬件验收

1. 使用 PR bot 给出的 Try it 镜像更新驱动，打开画布，确认出现 arm_gesture 卡片，
   同时保留电池、手部、腰、头及摄像头等已有卡片。
2. 启动项目，先查看 info 的 `lowstate_ready`。在安全站立且上肢活动空间足够时，
   依次测试右/左挥手、敬礼、单侧/双侧 reset；检查手型、方向、限速及复位姿态。
3. 动作中 stop，确认停止后不再续播，随后重新执行；连续执行两个动作，确认前一个
   被取消。用手臂、腰、头卡片覆盖手势，确认取消回报及无旧序列反写。
4. 检查 ACP 的完成、取消、失败展示。实测到位需观察真机/关节反馈，不能依赖
   `completed`；检查厂商动作与上肢控制的双向 `CONTROL_BUSY` 和显式 stop 交接。
5. 在受控条件下验收反馈中断后的动作冻结、错误回报及重新启动恢复，并回归 PAC
   电池、手部和其他原有卡片。

以上硬件行为、镜像部署和网页展示均未在离线开发中验证，由机器人操作员完成。
