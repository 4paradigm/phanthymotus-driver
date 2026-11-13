# 全身办公室跟练：联调与验收

## 当前部署方案

Mac 相机 → 本机 MediaPipe → pose_check MCP；平台 Agent 调用卡片，
G1 负责 TTS 和已验证的手臂示范。G1 内置相机不参与本轮跟练。
本地服务不保存或传出画面，只提供判定 JSON。它使用现有项目虚拟环境。
R1 ROS Plugin 仍保留 `check` 接口；以下 session 接口由 Mac MCP 服务提供，
不能把 Mac 卡片的 begin/status 命令直接发给旧 R1 Plugin。

在 `unitree/r1` 目录：

```bash
.venv/bin/python pose_local.py --model /tmp/r1_pose_landmarker_lite.task --source 0 --preview --serve
```

此时 `http://127.0.0.1:15740/mcp` 提供 `pose_check`，
`http://127.0.0.1:15740/health` 可读状态。关闭窗口、Q 或 Ctrl-C 会停止服务。
相机需要拍到单人的完整身体，尤其下蹲的髋、膝、踝；保持机位固定。
浅蹲使用 2D 膝角，斜侧取景有助于看见屈膝，但两条腿都必须足够清晰。

平台位于另一台机器，不能直接使用 Mac 的 loopback 地址。若 SSH 可用：

```bash
ssh -N -o ExitOnForwardFailure=yes -R 15740:127.0.0.1:15740 USER@ROBOT_IP
```

这条命令把服务只转发到机器人端 loopback。先确认机器人端 15740 没有被使用。
平台是否能访问该地址还取决于 Agent Core 的容器网络；需实机检查，不能猜测。
确认可达后，在平台添加 MCP 服务 URL `http://127.0.0.1:15740/mcp`，
以 tools/list 中的真实名字接入 pose_check。平台的登录、注册和 Skill 保存尚需联调。

## 跟练接口

向 `/mcp` POST JSON-RPC `tools/call`：

```json
{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"pose_check","arguments":{"action":"begin","pose":"hands_up","hold_seconds":3,"timeout_seconds":45}}}
```

保存 `result.content[0].text` 解码后的 `session_id`，之后每秒调用：

```json
{"jsonrpc":"2.0","id":2,"method":"tools/call","params":{"name":"pose_check","arguments":{"action":"status","session_id":"替换为 begin 返回的 ID"}}}
```

下蹲使用 `pose: squat, repetitions: 1`。**只有 session 的
`state: completed` 才推进下一动作**，不能把单帧 check 的 matched 当成一轮完成。
completed 是此次 session 的完成记录，会保留到下一次 begin；即时匹配看 check。
begin 拒绝覆盖正在进行的 session，要跳过先 cancel 同一个 session_id。
stop 取消 session 并禁止检查，start 恢复；摄像头释放由本地进程退出负责。

- 上肢：连续匹配到目标时长；单臂不能中途换边凑时长。
- 浅蹲：先连续站直 0.4 秒建立基准；0.45 秒内至少两帧显示下蹲，
  且髋部下降，再站回基准保持 0.4 秒才计一次。若正面取景使膝角失真，
  只有双腿投影同时缩短、双侧髋部明显下降、脚踝位置稳定且小腿长度未塌缩时
  才采用辅助判定。短暂丢失一帧可连接窗口内的两次有效下蹲观测。
  返回结果包含左右大腿、小腿和整腿投影长度，便于排查关键点错位。
- 短暂丢失腿部关键点会暂停当前判断；超过 1.5 秒才撤销站姿基准。
  已计次数和本轮超时截止时间均保留。
- 超过 1 秒没有新推理帧，进度中断；不能靠重复轮询积累时间。
- 超时返回 `state: timed_out, status: retry`，允许重试或跳过。
- 相机/模型错误单独返回 error；不要说用户动作做错了。

本地单次验证（不启动平台服务）：

```bash
.venv/bin/python pose_local.py --model /tmp/r1_pose_landmarker_lite.task --source 0 --pose squat --practice --repetitions 1 --preview
```

## 待导入平台的 Skill 内容

目标：引导用户完成一次全身办公室活动，反馈简短，不做专业动作评分。

开始前读取 pose_check.info，确认 fresh=true；确认本轮用户已完整入画。
从已绑定工具列表选择 G1 的 tts、arm 和 Mac 的 pose_check，不能猜测工具前缀。
TTS 使用 G1 原生 tts.speak；若现场只启用 Perception TTS，则必须确认
tts → G1 speaker 的接线和播放生命周期，不能把“合成成功”当成“已播出”。

动作顺序：

| 动作 | TTS 引导 | 识别目标 | G1 示范候选 |
|---|---|---|---|
| 双手举高 | 把双臂向上举高，保持三秒 | hands_up, hold_seconds=3 | arm.execute gesture="hands up" |
| 双臂展开 | 双臂向两侧平举，保持三秒 | arms_open, hold_seconds=3 | 无对应已核实手势，先语音引导 |
| 单手举高 | 举起一侧手臂，另一侧自然放下，保持三秒 | one_hand_up, hold_seconds=3 | arm.execute gesture="right hand up" |
| 浅蹲站起 | 先站直校准，校准完成后开始计时；慢慢浅蹲再站起 | squat, repetitions=N | 实时计数，过半和每次完成事件交给 TTS，不执行机器人下蹲 |

手势名称来自当前仓库 G1 driver，真机先调用 arm.list 核实。
示范候选必须经现场确认站稳、手臂空间充足并完成单动作验证后启用。
ret=0 仅能证明接口接受；当前 G1 arm 返回没有物理完成事件，不能据此
声称示范已结束，也不要用猜测的固定等待替代首次现场确认。
第一轮由操作者确认示范结束后再 begin 用户跟练。后续自动化需要可靠的
动作结束判据，未经验证前不循环叠加动作。

每步语音引导、可用的示范结束后调用 begin，保存 session_id。
每秒查询 status；反馈变化且距上次播报至少 4 秒时才播报一次，避免持续打断。
completed：说“完成得很好”，进入下一步；running：根据反馈指导；
timed_out：询问重试或跳过，最多重试一次；error：暂停并说明设备问题。
用户说停立即 cancel，不继续发手臂命令。结束时停止当前跟练并报告完成/跳过项目。
release 也是实际运动命令，只有现场验证其适用性后才作为收尾动作。

## 验收记录

- 本地几何与 session 自动测试：已覆盖保持、换边、重复帧、丢点、超时、下蹲往返。
- 全身相机现场测试：待验证。
- 平台 MCP 注册和 Skill 导入：待连接与验证。
- G1 TTS 实际播出、手臂实际动作：待连接与现场验证。
- 完整跟练闭环：待验收；以上代码完成不代表真机验收完成。
