# TRON2 YOLO 检测

这是原 TRON2 驱动内的 processor 卡片，使用父目录唯一的 MCP 入口、容器、ROS2 context
和 publisher。没有单独 Dockerfile、driver.yaml、监听端口或视频/DDS订阅。

在画布将原 `camera`、`camera_left` 或 `camera_right` 中的一路连接到 `yolo_detect`，启动
项目。原相机卡片转发 JPEG 时，把相同的图像和接收/发布时间戳交给最新帧队列。工作线程
每秒最多处理2帧，过载只保留最新图像，不积压。标注图和检测 JSON 返回原 publisher：

- `/limx_tron2/yolo/annotated`：`image/jpeg`
- `/limx_tron2/yolo/detections`：`data/json`

卡片没有置信度、类别或帧率调节框。Core 用 `start` / `stop` / `info` 管理生命周期和相机
连接，`input_topic` / `input_topics` 由连接选择。未启用的相机输入会被拒绝。原输入头部
时间戳和源 JPEG/标注 JPEG 的 SHA256 随结果保留，时间戳是转发端接收时间，不能当成
已经验证的硬件测量时间。

GPU推理仍请求现有 `http://172.28.4.23:18080`，模型为 `yolov8l-worldv2.pt`，只接受该
部署端点，不允许卡片参数指定任意 URL。GPU `/health` 需返回 `status=ok`、
`model_loaded=true` 和对应 `model`；`POST /detect` 接受 JPEG 二进制，结果需包含原图
`image_width` / `image_height`、`box_coordinate_system=original_image_pixels`、
`detections` 数组（`label`、`confidence`、`box_xyxy`）。尺寸、检测框、返回大小及3秒
时效检查失败时不输出可用检测；故障状态清除旧检测框。

Jetson只使用 Pillow 解码和绘制标注，依赖加入父目录已有 `requirements.txt`。不安装
PyTorch、Ultralytics、CUDA或模型权重，不增加系统包和GPU设备权限。原Dockerfile的
基础镜像和安装流程保留。Pillow和业务代码会增加镜像内容，体积需实际构建后测量。

保留调度修复：只读一次时钟计算剩余等待，避免负数 sleep 杀死工作线程。health 的
`vision.yolo_detect.worker_alive` 报告线程存活；HTTP可达不等于检测正常。

构建、部署和旧独立服务迁移见父目录 [README](../README.md)。测试在父目录运行：

```bash
python3 -m unittest discover -s tests -v
```

`test_yolo_detect.py` 覆盖最新帧、图像/结果关联、失败和输入切换；`test_yolo_worker.py`
覆盖跨调度截止时间的确定性回归；`test_vision.py` 覆盖与原驱动的集成。测试不连接真实
GPU或机器人；上线仍需实际镜像构建、DDS输出和现场验收。
