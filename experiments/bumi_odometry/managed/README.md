# 历史托管视觉融合实验

这里保留此前的 ROS RealSense 采集、兼容桥接、RTAB-Map/EKF 管理器和融合距离控制代码，
用于离线分析及比较。它们不进入当前 Bumi 生产镜像，也不会随卡片自动启动。
默认相机已恢复为 `noetix/bumi/camera_runtime.py` 中的直接 pyrealsense2 采集；
默认距离移动使用 `imu_distance_control.py`，不依赖本目录。

本目录代码仍需独立 ROS 相机、RTAB-Map、robot_localization 等依赖。
在完整源码目录使用时，需要将仓库根目录和本目录加入 PYTHONPATH，
以便子进程加载 `common.logsafe` 及历史模块。不要在生产相机运行时打开第二个相机采集者。
此前 `camera.fusion_enabled` 配置项不再控制生产相机，旧配置中的该项会被忽略。

相关历史测试为 `tests/test_bumi_managed_fusion.py`、`tests/test_bumi_distance_control.py`
及需独立环境的 `tests/bumi_fusion_smoke.py`。测试通过不代表本目录已部署或实测精度通过。
