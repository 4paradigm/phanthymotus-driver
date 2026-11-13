# 单人真机实验手册（待硬件验证）

本轮未连接 Bumi，下面所有硬件结果均为待测。没有运行此手册的部署/机器人命令。由操作者确认配套 Core/Driver 已获准部署、机器人安全停放、已有 Camera 正常工作后执行；不得为了图像实验启动任何运动 Card。

## 一次性准备

1. 在 Dashboard 确认实际 namespace 和 Camera 输出，连接 RGB JPEG → ImageQualityGuard。只启动本 Card；不要点击会启动其他机器人功能的全项目启动按钮。
2. 查看 info，记录 `driver_boot_id`、`admission_epoch`、`run_id`、input/output Topic 和配置。票据应由 Core 自动签发；不要手填 run_id 或直接 Driver start 绕开 Core。
3. 在已有 ROS2 环境中按实际值设置。Domain 与生产配置核对，不能使用本地测试域 219 代替生产域：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42  # 先核实实际部署配置
export IQG_INPUT=/actual_hostname/camera/color
export IQG_OUTPUT="$IQG_INPUT/image_quality_guard"
ros2 topic info "$IQG_INPUT" --verbose
ros2 topic type "$IQG_INPUT"   # sensor_msgs/msg/CompressedImage
ros2 topic type "$IQG_OUTPUT"  # std_msgs/msg/String
python3 tools/image_quality_observe.py --topic "$IQG_OUTPUT" --seconds 30 --output iqg-normal.jsonl
```

观察器只订阅质量 JSON，不订阅/保存图像。不要用 rosbag、图片导出或屏幕录像代替指标记录。JSONL 保存在命令指定的操作者目录；其中带分析时间，不能当作曝光时刻。每次实验也记录本 Card info 与操作者观察。

## 实验表

每行单独执行 `python3 tools/image_quality_observe.py --topic "$IQG_OUTPUT" --seconds 30 --output <本行文件名>`，操作者独自制造条件并恢复。

| 项目 | 单人步骤 | 预期与判定 | 记录文件 | 实际结果 |
|---|---|---|---|---|
| 正常 | 对准有纹理、适中照明的静止场景 | 至少五个有效清洁分析样本后 HEALTHY | iqg-normal.jsonl | 待测 |
| 低光 | 降低环境照度或部分遮光，不触碰镜头 | 三个连续低光分析样本后 LOW_LIGHT；同时模糊可多原因 | iqg-low-light.jsonl | 待测 |
| 过曝 | 对准明亮漫反射区域，避免太阳/激光直射 | 三个连续过曝样本后 OVEREXPOSED | iqg-overexposed.jsonl | 待测 |
| 模糊 | 靠近有纹理目标至失焦，或轻微移动目标 | BLURRED；记录纹理/曝光，低细节不等于硬件故障 | iqg-blurred.jsonl | 待测 |
| 多异常 | 低光并对准失焦/低细节区域 | 达到各自确认次数，保留多个 reasons | iqg-multiple.jsonl | 待测 |
| 恢复 | 恢复有纹理正常照明画面 | 五个连续有效恢复样本后 HEALTHY/reasons=[] | iqg-recovery.jsonl | 待测 |
| 限频 | 保持正常图像至少三十秒，查看 runtime 计数 | 实测频率不超过配置；无积压，不能只看摄像头频率 | iqg-rate.jsonl + info | 待测 |
| 重启 | 记录 A 票据，手动停止，重新连接并启动 B | B 新 run_id，同 Topic 可重用，无重复消费者 | iqg-restart.jsonl + info | 待测 |
| 旧 stop | 在 B 运行时向 Core 重放保存的 A stop 原始请求 | ALREADY_STOPPED/STALE_RUN；B run_id 不变，继续有新 JSON | iqg-stale-stop.jsonl + 响应 | 待测 |

2Hz 下三次确认通常约 1.5 秒、五次恢复约 2.5 秒，受首样本时机、输入频率及调度影响；验收以有效分析样本计数为准。不能用一次黑屏/白屏保证只有一个原因。

## 旧 stop 重放

在浏览器 Network 面板保存 A 的 **Core** `/api/mcp/{mcp_id}/call` stop JSON；重放原票据，不从 B info 取票。示例文件内容为：

```json
{"tool":"image_quality_guard","arguments":{"action":"stop","instance_id":"原CardID"},"lifecycle":{"run_id":"A的原run_id"}}
```

在操作者已验证的 Core 地址和 TLS 证书条件下：

```bash
curl --cacert /path/to/core-ca.pem -H 'Content-Type: application/json' \
  --data-binary @iqg-A-stop.json "https://实际Core地址:15678/api/mcp/实际mcp_id/call"
```

不要增加 `_source=system`，不要关闭 TLS 验证，不要改成 B 票据。此请求只针对自己已停止的 Guard 运行，不触碰其他 Card。

## 失败排查与记录

无数据：先检查已有 Camera 是否发布、Topic 绑定、ROS_DOMAIN_ID、QoS与DDS网络；本 Card 不带断流检测。缺票/过期票：刷新 Core 生命周期状态；Core 重启时先核查 Driver 原运行，不强制覆盖。UNKNOWN：看 error.code、JPEG format 和 payload 合法性。频率偏低：检查输入实际新帧率、CPU与runtime.error；本地计时不是真机性能保证。清理失败：使用同一原票据重试，不启动新运行代替清理。

为每项填写日期、配置、run_id、样本数、期望、实际、结论、异常和 JSONL 路径；没有真实记录不得改为 PASS。停止 Guard 后用 `ros2 topic info "$IQG_OUTPUT" --verbose` 核实发布器消失，并确认其他 Camera/共享 Executor 功能保持正常。
