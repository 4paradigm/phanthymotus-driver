# 无 R1 的视觉验证

全身跟练、保持计时、浅蹲计次及 Mac MCP 平台接入见
[OFFICE_WORKOUT.md](OFFICE_WORKOUT.md)。平台与真机联动的验收状态在该文档单独记录。

今天验证：Mac 摄像头 → JPEG → MediaPipe → `check_pose` → JSON。
`pose_local.py` 是开发入口，不启动 R1 驱动，不调用机械臂。

在 `unitree/r1` 目录运行：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-pose.txt
curl -fL https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/1/pose_landmarker_lite.task -o /tmp/r1_pose_landmarker_lite.task
.venv/bin/python pose_local.py --model /tmp/r1_pose_landmarker_lite.task --source 0 --pose hands_up --frames 30
```

Mac 需要给运行终端/应用摄像头权限。替换 `--pose` 为 `arms_open` 或
`one_hand_up`，分别验证双手举高、双臂展开、单手举高；人离开画面应返回
`reason: no_person`。默认持续运行，Ctrl-C 退出；不会保存摄像头画面。
`--image /path/to/person.jpg` 可单图验证；`--source` 也接受本地视频和
OpenCV 可读取的流地址。视频结束或相机断流会明确报错。

实时查看画面和肩膀、肘部、手腕定位，在 `unitree/r1` 终端运行：

```bash
.venv/bin/python pose_local.py --model /tmp/r1_pose_landmarker_lite.task --source 0 --pose arms_open --preview
```

绿色点表示可信度达到规则阈值，红色点表示可信度不足；点旁的数字是
适配器输出的可信度。L/R 指人体自己的左右，画面不镜像。
灰线按肩膀 → 肘部 → 手腕连接，分别表示大臂和小臂。
窗口顶部 `MATCH` 表示动作匹配，`NOT MATCHED` 表示未匹配，
`KEYPOINTS NOT RELIABLE` 表示关键点不足；中文反馈仍输出在终端。
若手腕点落错位置、变红或出画，先调整站位与光照再判断规则是否需要修改。
选中预览窗口按 Q、Esc 或关闭窗口退出，也可在终端按 Ctrl-C。

本地摄像头/视频入口默认使用 MediaPipe VIDEO 模式，利用连续帧跟踪，
减少逐帧独立检测引起的定位抖动。可加 `--frame-independent` 对比原模式。
单图和 R1 Plugin 默认仍用 IMAGE 模式（Plugin 按需检查，不一定连续调用）。
可信度阈值仍为 0.5，不复用旧的成功结果；VIDEO 模式能否改善当前画面
需现场验证，遮挡、出画或光照问题仍可能导致红点。

输出保留 `detected`、`score`、`pose`、`feedback`，新增 `matched`。
`detected` 表示关键点足够，Skill 应以 `matched == true` 判断动作完成，
不能用 `detected` 或 `score` 替代。分数是几何启发式，不是模型概率。
坐标使用原图，左右名称是人体解剖学左右；无需镜像图像。
当前选择单个人体，不支持多人动作归属。

## 本次验证记录

- Apple M1 / Python 3.9.6 / MediaPipe 0.10.35 / OpenCV contrib 4.14.0.94。
- 11 项规则与 Plugin 测试通过，`pip check` 通过。
- 官方 `https://storage.googleapis.com/mediapipe-assets/pose.jpg` 单图推理通过，
  `arms_open` 返回 `detected: true, matched: true, score: 1.0`。
- Mac 摄像头尝试返回 `not authorized to capture video`，尚未完成现场动作验证。
  在系统设置 → 隐私与安全性 → 摄像头中允许运行应用/终端访问后，重跑上述命令。
- 沙箱中的 MediaPipe 图形初始化失败；以上单图成功记录来自沙箱外执行。

## BUMI RGB 与后续 R1

BUMI 驱动发布 `sensor_msgs/CompressedImage`：
`/<bumi_namespace>/camera/color`。这是 ROS topic，不是 HTTP/RTSP 地址，
不能直接传给 `--source`。在已有 ROS 环境中实例化 `PoseCheckPlugin`，
将 `input_topic` 指向此 topic，并提供 `model_path`；使用现有 executor
接收图像。不要为了 BUMI 图像启动包含 R1 硬件初始化的 `main.py`。

R1 可用后，在已验证的运行环境安装可选推理依赖、挂载 `.task` 文件，
将 `plugins.pose_check.enabled` 设为 `true` 并设置
`plugins.pose_check.model_path`；空 `input_topic` 默认订阅
`/<r1_namespace>/camera/main`，对应 `camera_main`。
现有 `pose_check` tool 名称、`check` action、`pose` 枚举保持不变。
R1 Dockerfile 包含适配器，但未自动安装可选模型依赖；ARM64 包兼容性、
推理延迟和 ROS 链路必须在目标环境实测后再部署。

后续真机 Skill 顺序：TTS 提示 → arm 示范 → pose_check 检查 → 按反馈重试
或进入下一动作。需设置有限重试/超时；相机无数据、超过 2 秒未更新、
模型不可用均不能视为完成。真机 TTS、arm 和完整 Skill 尚待 R1 联调。

模型适配器使用 [MediaPipe PoseLandmarker 官方 API](https://ai.google.dev/edge/api/mediapipe/python/mp/tasks/vision/PoseLandmarker)。


## 肘部规则（2026-09-28）

现在必须检测到左右肩、肘、腕六个可信关键点，旧的四点输入会返回
`missing_keypoints`，不会再只凭手腕位置通过。

- `hands_up`：两侧肘部高于肩、手腕高于肘。高度差需要超过至少画面高度的
  1.5%（随肩腕高度差增加），排除肩线附近抖动与只折起小臂。
- `arms_open`：每侧肘在肩外、腕在肘外，横向各至少展开画面宽度的 2.5%；
  肩肘、肩腕和肘腕的高度差均不超过画面高度的 4%，排除垂肘和小臂内折。
- `one_hand_up`：一侧满足举高，另一侧肩、肘、腕依次向下，不能另一侧也平举。

返回字段及 Plugin 工具接口保持不变；反馈会区分大臂高度、小臂高度和肘部展开。
这是正面 2D 几何规则，不是精确的 3D 关节角测量；阈值与取景有关，需要现场验证。
停止旧程序后重启预览，先比较“只抬小臂”与“整条手臂举高”，再检查放下后退出匹配。
