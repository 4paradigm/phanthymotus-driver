# traffic_light — Unitree AS2-W 交通灯识别卡片

识别画面中的交通灯（YOLOv8n 官方权重 + HSV 颜色判定，**无需训练**），在
红/黄/绿状态变化时通过机器人语音播报「红灯，请等待」/「黄灯，请注意」/
「绿灯，请通行」，同时把事件发布给监控/Agent。面向「辅助盲人过马路」场景。

## 卡片类型与画布连线

`traffic_light` 是一张 **processor** 卡：消费一路流，产出两路流。它在画布上的
三根连线如下（均为 format 匹配，不写死名字）：

```
camera (topic_out: image/jpeg)   ──▶ traffic_light (topic_in: image/jpeg)
                                          ├─▶ topic_out: audio/pcm-16k  ──▶ speaker (topic_in)
                                          └─▶ topic_out: data/json     ──▶ 监控 / Agent
```

- `topic_in`：`image/jpeg`（`sensor_msgs/CompressedImage`），接 `camera` 卡片。
- `topic_out #1`：`audio/pcm-16k`，播报语音，接 `speaker` 卡片 → 机器人发声。
- `topic_out #2`：`data/json`，灯态事件（颜色、置信度、亮灯像素占比、包围盒、
  时间戳），接监控视图或 Agent。

Canvas 启动卡片时会以 `start(input_topic)` 把上游相机话题解析后传入；卡片据此
订阅并开始识别。`info` 返回权威 `topic_out`/`topic_in` 与当前灯态。

## 文件

| 文件 | 职责 |
|---|---|
| `light_classifier.py` | 纯「检测框 → 红/黄/绿」判定 + 状态去抖，无 ROS/无 YOLO，直接单测 |
| `detection_engine.py` | YOLOv8n 推理封装（惰性导入 ultralytics）|
| `speech.py` | 播报短语 → 16 kHz 单声道 S16LE PCM（WAV 优先，提示音兜底）|
| `plugin.py` | `TrafficLightPlugin`（MCP card 本体，导入 rclpy）|
| `scripts/generate_speech_wav.py` | 一次性生成红/黄/绿播报语音 WAV 资产 |
| `resource/traffic_light_*.wav` | 预录播报语音（由上面脚本生成，提交后随镜像打包）|

## 识别逻辑（无训练）

YOLOv8n 输出 COCO 的「traffic light」类别（COCO 索引 9）——只负责**定位**交通灯
框；COCO 不区分红/黄/绿，颜色由 `light_classifier` 从框内像素判定：

1. 检测框内，亮度高且饱和的像素视为「亮着的灯」；
2. 按 HSV 色相把这些亮像素分成红（色相 0-12 / 168-180）、黄（18-40）、
   绿（45-95）三路；
3. 亮像素占比必须超过 `min_lit_fraction`，且胜出颜色的占比要明显领先第二名
   （`dominance_ratio`），否则判为 `unknown`（反光、混色、灯全灭）；
4. 连续 `confirm_frames` 帧保持同一颜色、且与上一次播报颜色不同，才播报一次；
   `cooldown_s` 冷却避免红→绿→红快速切换时连发。

## 模型权重

`model_path` 为 `yolov8n.pt`（文件名）时，ultralytics 首次运行自动从
[Ultralytics 官方 GitHub releases](https://github.com/ultralytics/assets/releases)
下载并缓存到 `~/.config/Ultralytics`；离线机器人请预先下载后把 `model_path` 设为
绝对路径：

```bash
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
```

本卡片**直接使用官方预训练权重，不训练、不微调**。

## 生成播报语音

```bash
pip install edge-tts
python3 scripts/generate_speech_wav.py
# 产出 resource/traffic_light_{red,yellow,green}.wav（16k 单声道 S16LE）
```

未生成 WAV 时，卡片回退到按颜色区分的提示音（红=低音、黄=中音、绿=高音）并打
日志，仍可听出状态但非真人语音。

## 配置（`unitree/as2w/config.yaml` 的 `plugins.traffic_light`）

```yaml
traffic_light:
  enabled: false              # 默认关闭：真机跑 YOLO 需先核对帧率与权重
  model_path: yolov8n.pt      # YOLOv8n 官方 COCO 权重；可填绝对路径（离线预下载）
  device: cpu                 # cpu | cuda | 0
  conf: 0.35                  # 交通灯检测置信度
  imgsz: 640
  rate_hz: 5                  # 识别帧率上限（相机 12fps，只取最新帧）
  # 红/黄/绿 HSV 判定阈值，见 light_classifier.py
  saturation_min: 90
  value_min: 100
  min_lit_fraction: 0.005
  dominance_ratio: 1.5
  confirm_frames: 2
  cooldown_s: 3.0
  texts:
    red: 红灯，请等待
    yellow: 黄灯，请注意
    green: 绿灯，请通行
  # 预录语音 WAV（16k 单声道 S16LE）；可省略，插件自动发现 resource/*.wav
  red_wav_path: /work/Traffic_Light_Recognition/resource/traffic_light_red.wav
  yellow_wav_path: /work/Traffic_Light_Recognition/resource/traffic_light_yellow.wav
  green_wav_path: /work/Traffic_Light_Recognition/resource/traffic_light_green.wav
```

## 部署

`unitree/as2w/Dockerfile` 需要新增 `ultralytics`（会带入 torch/torchvision，ARM64
为 CPU 版本）并 `COPY Traffic_Light_Recognition/ /work/Traffic_Light_Recognition/`。
镜像体积会显著增大，属预期。构建上下文通过 `driver.yaml` 的
`build_context_extras: ../Traffic_Light_Recognition` 引入。

## 测试

```bash
python -m pytest unitree/Traffic_Light_Recognition/tests -q
```

纯逻辑（`light_classifier.py`、`speech.py`）无 ROS/无 YOLO 依赖，可在笔记本直接跑；
`compute_color_scores` 的像素级用例在缺少 `cv2` 时自动跳过。

## 已知限制

- CPU 上 yolov8n 推理较慢（Jetson 约数百 ms/帧），`rate_hz` 是上限而非保证；
- HSV 颜色判定是启发式，无法完全消除逆光/反光/过曝下的误判，`dominance_ratio` 与
  `min_lit_fraction` 是面向常规机位的默认值，换机位需重测；
- 只取画面中置信度最高的一个交通灯框，多灯路口（如横杆多组灯）不做多实例区分；
- COCO 的 traffic light 类别包含行人灯/倒计时牌等，颜色判定会如实读出其灯色，不做
  机动车灯/人行灯的语义区分。
