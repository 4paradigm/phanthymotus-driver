# DetaForce 猜拳基础动作

`detaforce_rps` 是一张 actuator Card，提供六个可独立调用的动作。它不包含视觉出拳识别、胜负判定或完整游戏 Skill。参考[用户提供的动作视频](https://www.bilibili.com/video/BV1Htb26rEwa)，按 Bumi 的八个手臂关节适配；Bumi 没有手指和独立腕关节，不模拟握拳或伸指机构。

| action | 行为 |
| --- | --- |
| `invite_one` | 右臂抬起向前伸出，再拉向面部招手，重复两次 |
| `invite_both` | 双臂同时向前伸出，再拉向面部招手，重复两次 |
| `countdown` | 双臂相差半周，肩与肘配合上下交替转动 |
| `double_punch` | 双拳一前一后交替，作为“双拳猛攻”的独立原语 |
| `mouse` | 双手抬高至面部两侧，保持鼠鼠姿态 |
| `confident` | 花来，右拳举到面前，做一次肯定的下压，再回到面前举拳姿态 |

“猛攻”为双拳前后交替，“花来”为单拳举至面前、下压示意并回举。页面按五类需求编号，第 1 类邀请提供单手与双手两种动作，第 5 类为花来。花来通过 `confident` 调用，使用 `confident_recording_id` 绑定示教动作。邀请语音为“来，玩一局猜拳！”，由现有 TTS/音频能力组合播放，动作 Card 返回 `suggested_speech`。

本轮将花来的肯定动作实现为一次小幅向下点拳，随后回到面前举拳姿态、短暂停留，再放下手臂。邀请的两次招手均在面部高度完成，鼠鼠双手举至面部两侧。调整后 42 项 Card 与轨迹测试通过，四个受影响动作已通过 Gazebo 页面逐项试播、位姿回读和回到站姿的检查。

## 仿真

本机既有的 Ubuntu-22.04 / Gazebo Fortress 预览增加了 <http://127.0.0.1:15713/detaforce>。六个按钮分别调用单个动作；“依次试播全部”只用于验收动作形态，不表示大模型已完成猜拳决策。邀请试播额外组合 Ubuntu 本地 Piper 与 WSLg 音频播放。MCP 直接调用动作时只执行动作本身。

本地 PhanthyMotus <http://127.0.0.1:15878/> 的模拟设备目录已刷新，可发现 `detaforce_rps`。没有改动现有迎宾画布或启动新的大模型会话；新增卡片也尚未连接到迎宾决策核心。

MCP 地址仍为 `http://127.0.0.1:15713/mcp`，新增 `detaforce_rps`。例如：

```json
{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"detaforce_rps","arguments":{"action":"countdown"}}}
```

`status` 查询本卡运行状态，`stop_action` 停止本卡当前动作。停止后保留当前姿态，随后可按“复位站姿”；正常完成则平滑放下手臂。仿真中的动作与迎宾/行走共用执行互斥，有活跃迎宾会话时拒绝开始试播，不抢占另一任务。

预览复用现有 Ubuntu 仿真目录中的厂家 URDF、网格与 `PoseTransport`。`simulation/detaforce_gazebo.patch` 保存猜拳对该仿真器的增量接入，包括动作分发、页面入口、近景视角、邀请语音白名单与回归测试。

在已有迎宾仿真环境中，将本目录的 `detaforce_motion.py`、`detaforce_rps.py`、`detaforce_gazebo.py`、`detaforce.html` 同步到 `~/phanthymotus-sim/gazebo_minimal/`。首次接入时在该运行目录检查并应用补丁，其中 `driver_checkout` 指向本仓库在 Ubuntu 中的绝对路径。

```bash
cd ~/phanthymotus-sim/gazebo_minimal
git apply --check "$driver_checkout/noetix/bumi/simulation/detaforce_gazebo.patch"
git apply "$driver_checkout/noetix/bumi/simulation/detaforce_gazebo.patch"
bash run_bumi.sh
```

已接入的环境可通过以下命令检查补丁状态。

```bash
git apply --reverse --check "$driver_checkout/noetix/bumi/simulation/detaforce_gazebo.patch"
```

当前 WSL 环境已经完成接入，并在 `runs` 中保存了更新前的备份。运行资源由现有本地仿真环境提供，机器人镜像包含动作 Card 与轨迹定义。

轨迹采用五次插值，关节位置、速度、加速度连续；仿真最大关节速度为 2 rad/s、最大加速度为 6 rad/s²。只驱动八个手臂关节，基座、腰部、双腿保持不动。每次位姿更新均等待 Gazebo 回读，并保存阶段图像。这里验证运动学形态与接口，不证明接触力、自碰撞、动力学或站立平衡满足真机要求。

## 真机接入边界

默认注册 Card，但 `hardware_enabled: false`，六个 `*_recording_id` 均为 `-1`。启动、配置、未绑定动作都不会产生运动。底层 LowController 示例需要一次控制全身 21 个电机，缺少已验证的站立平衡集成，因此本卡不会把仿真角度直接发给真机。

真实执行通过已有 `action_recording.play_recording(recording_id)`。后续需在有人监护的条件下，用厂家示教录制并逐项验收六个动作，将实际编号填入卡片设置，确认静止、walking 模式与环境条件后再启用硬件。不能凭空分配编号，也不会用通用挥手/欢呼冒充新动作。本轮未部署或测试真机。

`running` 只代表已观察到 play_teach 模式。`playback_exited` 只代表播放模式已退出，不宣称物理动作完成。没有观察到开始时保留 `start_unconfirmed`，阻止再次发送；需先现场核验。该后端不声明 ACP 物理完成事件。共享 SDK 不提供回放任务编号，无法区分其他控制端在同一播放模式下替换任务，实机验收期间需单一操作者独占动作控制。

## 验证命令

仓库根目录：

```powershell
python -X utf8 -m pytest noetix/bumi/tests/test_detaforce_rps.py tests/test_bumi_bundle.py -q
```

Ubuntu 的现有仿真目录：

```bash
source /opt/ros/humble/setup.bash
../vision-venv/bin/python -m unittest -q test_bumi_sim test_skill_flow
```

单元测试检查 URDF 限位、整段轨迹的速度/加速度、招手和双拳的实际末端方向、交替上下运动、启动不发命令、配置校验、未确认状态、互斥与停止语义。另需以运行中的 Gazebo 检查六个动作的回读、画面与邀请语音，实机验收独立进行。

2026-11-06 本机验证：Card/轨迹测试 38 项通过，Ubuntu 仿真回归 56 项通过。通过页面依次执行六个动作，均完成并回到手臂零位；逐动作确认 Gazebo 位姿回读、基座与双腿静止，两种邀请语音均完成实际音频播放。额外验证执行中重复调用被拒绝、中途停止为 `cancelled`、停止后页面保持连接、复位站姿完成且无浏览器脚本异常。已保存六轮阶段图像与运行报告；没有真机运动、动力学验收或完整游戏 Skill 验证。
