# 天轶 pose_check 静态深蹲测试

本轮只验收 `check(squat)` 的静态判别，不用 `begin` 计数。新版增加相对身高线索：先站直建立双肩到双脚踝的图像高度基准，再比较下蹲时的高度、双髋和双肩下降量。它仍要求双肩、双髋和双脚踝可靠入框；膝盖短时被遮挡时可以继续判别。`height_ratio` 是同一机位下的图像比例，不是真实身高。

## 部署临时服务

在仓库根目录从已跟踪源码生成临时包，再把它传到天轶。这个包包含
`pose_ros_service.py`、姿态/会话模块和 socket bridge 依赖，不依赖手工维护的
`/tmp` 文件：

```bash
python3 x-humanoid/tianyi2.0/build_pose_check_bundle.py
scp /tmp/tianyi-pose-check-bridge.tar.gz nvidia@10.100.129.72:/tmp/
```

登录天轶后，只操作我们自己的 `embodied-x-humanoid-tianyi2.0` 容器和 pose 临时进程。若该容器未运行，先检查容器状态；不要操作其他人的 `-pr352` 容器。

```bash
docker inspect -f '{{.State.Running}}' embodied-x-humanoid-tianyi2.0
docker cp /tmp/tianyi-pose-check-bridge.tar.gz embodied-x-humanoid-tianyi2.0:/tmp/
docker exec -i embodied-x-humanoid-tianyi2.0 python3 - <<'PY'
import os
import signal
import time

target = b'/tmp/office-pose-check/pose_ros_service.py'
stopped = []
for name in os.listdir('/proc'):
    if not name.isdigit():
        continue
    try:
        with open(f'/proc/{name}/cmdline', 'rb') as stream:
            arguments = stream.read().split(b'\0')
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        continue
    if target in arguments:
        os.kill(int(name), signal.SIGTERM)
        stopped.append(int(name))
        print('Stopped old pose service PID', name)
for _ in range(40):
    def still_running(pid):
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as stream:
                return target in stream.read().split(b'\0')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            return False
    if not any(still_running(pid) for pid in stopped):
        break
    time.sleep(0.2)
else:
    raise SystemExit('Old pose service did not stop; inspect it before restarting')
PY
docker exec embodied-x-humanoid-tianyi2.0 bash -lc 'mkdir -p /tmp/office-pose-check && tar -xzf /tmp/tianyi-pose-check-bridge.tar.gz -C /tmp/office-pose-check'
docker exec -d embodied-x-humanoid-tianyi2.0 bash -lc 'source /opt/ros/humble/setup.bash && nohup /tmp/pose-check-venv/bin/python -u /tmp/office-pose-check/pose_ros_service.py --model /tmp/pose_landmarker_lite.task --topic /ob_camera_head/color/image_raw --message-type raw --max-fps 10 --max-width 640 </dev/null >/tmp/office-pose-check.log 2>&1 & echo $! >/tmp/office-pose-check.pid'
docker exec embodied-x-humanoid-tianyi2.0 tail -n 15 /tmp/office-pose-check.log
docker exec embodied-x-humanoid-tianyi2.0 curl -sS http://127.0.0.1:15740/health
```

启动日志应出现 `version=socket-bridge-20261007`、`max_fps=10`、`max_width=640`、`camera_domain=0`、`result_domain=42`；`/health` 应有相同 `version`，且相机收到画面后 `fresh: true`、`frames` 递增。原始 RGB 会直接送给 MediaPipe，不再经过 JPEG 编解码；服务只保留最新帧并按 10 fps 推理，因此相机仍可保持 30 fps。结果事件经天轶既有的 socket bridge 发布到 Domain 42；不要直接用 `/work/dds_profile.xml` 的进程对 Agent Core 发布。如果看到 `ModuleNotFoundError: rclpy`，检查启动命令是否包含 `source /opt/ros/humble/setup.bash`。部署后用 `/opt/phanthy-motus/dds-local.xml` 执行 `ros2 topic info /nvidia_desktop/pose_check/result -v`；应显示 `Publisher count: 1`。

MediaPipe 运行时还需要 `libEGL.so.1` 和 `libGLESv2.so.2`；PR 镜像在 Dockerfile 中安装 `libegl1` 与 `libgles2`。若复测旧镜像时出现这两项缺失错误，改用包含该依赖的新镜像。

部署后可对比姿态服务 CPU；30 fps 相机是正常的，关键是服务不再把 30 帧全部排队处理：

```bash
docker exec embodied-x-humanoid-tianyi2.0 \
  bash -lc "ps -o pid,pcpu,pmem,args -p \$(pgrep -f pose_ros_service.py | head -1)"
```

在天轶上确认 Agent Core 所在的 Domain 42 能收到结果：

```bash
docker exec -e ROS_DOMAIN_ID=42 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/opt/phanthy-motus/dds-local.xml \
  embodied-x-humanoid-tianyi2.0 \
  bash -lc 'source /opt/ros/humble/setup.bash && timeout 60 ros2 topic echo --once /nvidia_desktop/pose_check/result'
```

在这条订阅命令等待期间，到画布执行 `begin`；现在结果话题没有逐帧数据，只有事件发生时才有消息。收到包含 `"source": "pose_check"`、`"event": "task_started"` 的 JSON 才说明跨 Domain 发布已连通。然后在画布把 `pose_check` 的输出接到 `decision_core`，开启智能控制；Skill 还需根据 `event` 的 `rep_completed`、`step_completed` 决定何时写 Activity 或调用 TTS。

## 测试判别

先在卡片执行 `cancel` 清除旧跟练。选择 `ACTION=check`、`POSE=squat`，执行一次后它会持续检查。让双肩、双髋、双脚踝都在画面内，站直至少半秒；`/health` 的 `check_calibrated` 应变为 `true`。再下蹲保持片刻；需要看 `height_ratio` 等逐帧诊断值时，再执行一次 `check` 查询当前结果。稳定后应看到 `matched: true`、`status: completed`、`phase: down`；若靠身高线索判别，还会出现 `squat_evidence: body_height`。站起后应变回未匹配。结果话题只在稳定状态发生变化时各发一次 `pose_detected`、`pose_lost` 事件；`check` 不计算蹲起次数。

若未匹配，记录一条蹲下时的完整 `result`，尤其是 `reason`、`low_visibility`、`height_ratio`、`height_matched`、`knee_angles` 和 `phase`，并确认相机没有移动、脚踝没有出框。结束测试时执行 `cancel`。

## 结果话题的消息格式

`/nvidia_desktop/pose_check/result` 的 ROS 类型是 `std_msgs/msg/String`；`data` 是一条 JSON 事件，而不是每帧检测结果。`schema` 固定为 `pose_check.event.v1`。例如第 2 次深蹲完成：

```json
{
  "schema": "pose_check.event.v1",
  "source": "pose_check",
  "frame_id": 7210,
  "session_id": "本次 begin 返回的 ID",
  "state": "running",
  "progress": {"repetitions": 2, "target_repetitions": 5, "elapsed_seconds": 12.4, "calibrated": true, "phase": "ready"},
  "event": "rep_completed",
  "pose": "squat",
  "count": 2,
  "target": 5,
  "duration_seconds": 3.8,
  "narration": "第 2 次深蹲完成，用时 3.8 秒",
  "events": [{"event": "rep_completed", "pose": "squat", "count": 2, "target": 5, "duration_seconds": 3.8, "narration": "第 2 次深蹲完成，用时 3.8 秒"}]
}
```

`begin` 立即发一次 `task_started`；每种动作每新增一次计数都发一次 `rep_completed`；目标全部完成时另发一次 `step_completed`，其中 `task_completed: true`、`count` 为最终次数。上肢动作的每一轮是“稳定到位 → 放下 → 再稳定到位”，持续保持同一个姿势不会重复计数。校准完成、过半、超时、取消以及持续 `check` 的稳定进入/离开动作也只在状态变化时发事件。最后一次动作会连续发 `rep_completed` 和 `step_completed`；上层播报可只读后者，避免重复说两遍。`info` 是卡片能力/健康查询动作，`status` 是进度查询动作，两者不是结果话题里的事件名。

## 上机验收：5 次深蹲与漏检诊断

部署完成后，在天轶另开一个终端持续订阅决策核所在的结果话题：

```bash
docker exec -it \
  -e ROS_DOMAIN_ID=42 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/opt/phanthy-motus/dds-local.xml \
  embodied-x-humanoid-tianyi2.0 \
  bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic echo /nvidia_desktop/pose_check/result'
```

在画布的 `pose_check` 卡选择 `begin`、`pose=squat`、`repetitions=5`，点击执行。记下返回结果里的 `session_id`，然后做 **5 次慢而完整的蹲下再站起**。终端应依次出现：

- 一条 `task_started`；
- 一条 `calibration_completed`；
- 五条 `rep_completed`，其中同一 `session_id` 的 `count` 应严格为 `1`、`2`、`3`、`4`、`5`；
- 一条 `step_completed`，其中 `task_completed: true`、`count: 5`。

因此，实际做 5 次而只看到 `count` 到 4，就是漏计 1 次；同一 `session_id` 内 `count` 跳号或重复，就是误计问题。`progress_milestone` 是中途提示，不算一次深蹲。完成后按 `Ctrl+C` 停止订阅。

若做完某一次后约 2 秒仍未看到下一条 `rep_completed`，立刻回到卡片把 `ACTION` 改为 `status`，填入这次 `session_id` 后执行。返回中的最新姿态字段用于诊断：

| 字段 | 说明 |
| --- | --- |
| `progress.repetitions` | 当前算法已计入的次数；与终端事件的最大 `count` 应一致。 |
| `low_visibility` / `missing` | 哪个关键点不可见或缺失；脚踝、髋、膝常导致深蹲无法判断。 |
| `knee_angles` | 两侧膝角；站直接近 180°，下蹲会明显变小。 |
| `height_ratio` | 相对站立基准的双肩到脚踝图像高度；下蹲时应小于 1。 |
| `squat_evidence` | 本次有效下蹲使用 `knee_angle`、`projected_leg` 或 `body_height` 哪种证据。 |
| `progress.phase` | `ready` 表示等待下蹲，`returning` 表示已识别蹲下、等待站起。 |

把漏检前后各一次 `status` 的完整 JSON、以及对应的事件终端输出保留即可；这样可以判断是关键点丢失、阈值偏紧，还是下蹲/站起状态转换没有完成。
