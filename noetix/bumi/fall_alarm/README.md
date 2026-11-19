# fall_alarm — Bumi 视觉跌倒检测卡片

检测画面中的人是否跌倒（YOLOv11-pose 官方权重 + 姿态几何判定，无需训练），
跌倒后机器人语音提醒「检测到有人摔倒」，同时把事件发布给监控/Agent。

## 卡片类型与画布连线

`fall_alarm` 是一张 **processor** 卡：消费一路流，产出两路流。它在画布上的
三根连线如下（均为 format 匹配，不写死名字）：

```
camera (topic_out: image/jpeg)   ──▶ fall_alarm (topic_in: image/jpeg)
                                        ├─▶ topic_out: audio/pcm-16k  ──▶ speaker (topic_in)
                                        └─▶ topic_out: data/json     ──▶ 监控 / Agent
```

- `topic_in`：`image/jpeg`（`sensor_msgs/CompressedImage`），接 `camera` 卡片。
- `topic_out #1`：`audio/pcm-16k`，报警语音，接 `speaker` 卡片 → 机器人发声。
- `topic_out #2`：`data/json`，跌倒事件（置信度、包围盒、躯干角、髋部高度比、
  时间戳），接监控视图或 Agent。

Canvas 启动卡片时会以 `start(input_topic)` 把上游相机话题解析后传入；卡片据此
订阅并开始检测。`info` 返回权威 `topic_out`/`topic_in`。

## 文件

| 文件 | 职责 |
|---|---|
| `fall_detector.py` | 纯「关键点 → 跌倒」判定，无 ROS/无 YOLO，直接单测 |
| `pose_engine.py` | YOLOv11-pose 推理封装（惰性导入 ultralytics）|
| `speech.py` | 报警短语 → 16 kHz 单声道 S16LE PCM（WAV 优先，提示音兜底）|
| `plugin.py` | `FallAlarmPlugin`（MCP card 本体，导入 rclpy）|
| `scripts/generate_alarm_wav.py` | 一次性生成报警语音 WAV 资产 |
| `resource/fall_alarm.wav` | 预录报警语音（由上面脚本生成，提交后随镜像打包）|

## 跌倒判定（无训练，纯几何）

YOLOv11-pose 输出 COCO-17 关键点与人体框。判定「跌倒」要求**同时**满足：

1. 人体框扁平（`高/宽 < aspect_ratio_max`，默认 1.15）；
2. 且至少一条身体几何信号一致：
   - 躯干（髋→肩）偏离竖直方向（`> torso_angle_min_deg`，默认 45°），或
   - 髋部贴近脚踝（地面）高度（`hip_elevation < hip_elevation_max`，默认 0.25）。

三者单独都有歧义（蹲下也矮、弯腰也斜、坐下也低），所以两两组合 + 连续
`confirm_frames` 帧（默认 3）确认 + `cooldown_s`（默认 8 s）冷却：一次持续倒地
每过一个冷却周期再提醒一次，站起后再倒则立即重新提醒。

## 配置（`config.yaml` 的 `plugins.fall_alarm`）

```yaml
fall_alarm:
  enabled: false              # 默认关闭：真机跑 YOLO 需先核对帧率与权重
  model_path: yolo11n-pose.pt # n=s=nano 最快；也可填绝对路径（离线预下载）
  device: cpu                 # cpu | cuda | 0
  conf: 0.5                   # 人体检测置信度
  imgsz: 640
  rate_hz: 10                 # 检测帧率（相机 30fps，只取最新帧）
  # 报警语音 WAV（16k 单声道 S16LE）；缺省时回退到提示音并打日志
  alarm_wav_path: /work/fall_alarm/resource/fall_alarm.wav
  alarm_text: 检测到有人摔倒，请注意安全
  min_keypoint_conf: 0.5
  aspect_ratio_max: 1.15
  torso_angle_min_deg: 45.0
  hip_elevation_max: 0.25
  confirm_frames: 3
  cooldown_s: 8.0
```

`alarm_wav_path` 可省略，插件会回退到模块内的 `resource/fall_alarm.wav`。

## 模型权重

`model_path` 为文件名时，ultralytics 首次运行自动下载官方权重并缓存到
`~/.config/Ultralytics`；离线机器人请预先下载后把 `model_path` 设为绝对路径。
本卡片**直接使用官方权重，不训练、不微调**。

## 生成报警语音

```bash
pip install edge-tts
python3 scripts/generate_alarm_wav.py --text "检测到有人摔倒，请注意安全"
# 产出 resource/fall_alarm.wav（16k 单声道 S16LE）
```

## 部署

`Dockerfile` 需要新增 `ultralytics`（会带入 torch/torchvision，ARM64 为 CPU
版本）并 `COPY fall_alarm/ /work/fall_alarm/`。镜像体积会显著增大，属预期。

## 测试

```bash
python -m pytest noetix/bumi/fall_alarm/tests -q
```

纯逻辑（`fall_detector.py`、`speech.py`）无 ROS/无 YOLO 依赖，可在笔记本直接跑。

## 已知限制

- CPU 上 yolo11n-pose 推理较慢（Jetson 约数百 ms/帧），`rate_hz` 是上限而非保证；
- 阈值是面向「大致平视」相机位置的默认值，换机位需按 README 说明重测；
- 判定是几何启发式，无法区分「摔倒」与「主动躺下/坐下」，属单目姿态检测的固有边界。
