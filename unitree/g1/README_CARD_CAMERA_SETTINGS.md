# G1 `camera_settings` Card

`camera_rgb` 提供画面；`camera_settings` 读取并调整同一颗 RealSense RGB 传感器的拍摄参数。两张 Card 无需连线。此 Card 适合固定视觉实验的拍摄条件，以及在逆光、屏幕反光等特殊场景下调整后重试 OCR 或检测；正常环境优先保留自动模式。

## 真机依据

2026-09-28 在 G1 上只读探测到 **RealSense D435I / RGB Camera**。`camera_rgb` 连续收到 1920×1080 JPEG 帧。六项参数均由该 RGB 传感器的 `supports`、`get_option_range` 和 `get_option` 返回：

| 参数 | 首次探测值 | 范围 | 步长 | 含义 |
|---|---:|---:|---:|---|
| `auto_exposure` | true | 0–1 | 1 | 自动调整曝光与增益 |
| `exposure` | 166 | 1–10000 | 1 | 曝光值；设置后切换到手动模式 |
| `gain` | 64 | 0–128 | 1 | 信号增益；设置后切换到手动模式 |
| `auto_white_balance` | true | 0–1 | 1 | 自动校正白平衡 |
| `white_balance` | 4600 | 2800–6500 | 10 | 色温值；设置后关闭自动白平衡 |
| `brightness` | 0 | -64–64 | 1 | 图像亮度 |

这些范围来自本次 D435I 探测；其他实机以 `get` 返回的范围为准。第一次探测时自动曝光开启。后续只读复核曾读到自动曝光关闭、增益为 40，说明不能把第一次探测值当成恒定状态。

## 操作

1. 在 Canvas 拖入 `camera_settings` 和 `camera_rgb`，无需连接端口。
2. 对 `camera_settings` 执行 `get`，查看当前值与合法范围；打开 `camera_rgb` 数据流观察原画面。
3. 选择 `set`，六个输入框只填写**一个**。布尔值填写 `true` 或 `false`。例如小幅测试可填 `brightness=1`。
4. 再执行 `get` 并观察新帧。完成后执行 `reset`，再用 `get` 核对。

`reset` 恢复的是**本次相机采集进程启动时读取的设置**，不是出厂默认值。如果启动时自动曝光已关闭，`reset` 也会恢复为关闭；需要自动曝光时单独设置 `auto_exposure=true`。

## 验证状态

- 真机已确认：Card 注册、`get` 返回六项当前值和范围、`camera_rgb` 持续出帧。部署前的原 RGB 流 5 帧灰度均值为 116.90–117.01；新 Driver 下另一时刻的 5 帧为 122.46–122.71。这两组采样时刻不同，不能据此声称调参改变了画面。
- 用户反馈 Canvas 中 Card 可正常使用。尚未保存一组完整的 `set → 新帧 → reset → 恢复帧` 实测数值，因此本文不宣称图像变化或 reset 的真机定量验收已完成。
- 本地单元测试覆盖读初值不写入、单项参数校验、曝光/增益/白平衡切换自动模式、reset 与 Canvas 字段提示。
