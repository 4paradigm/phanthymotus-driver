# Helios 单卡目标跟踪

`helios_face_follow` 通过独立服务注册到 Core。在画布把 Helios 的
`camera_rgb` 选成头部 RGB，再连到卡片的 JPEG 输入口。启动后使用
`set_mode` 或 `set_target` 修改目标，`info` 查看状态。监控可以同时显示
原始相机和卡片的 `data/json` 目标输出。

- `nearest_face` 从人脸内区的深度样本取中位数，选择最近的人脸。输入必须是与
  640×480 RGB 对齐的 `image/depth-zlib`，共享光学坐标系，时间差不超过
  85 ms。通过 `depth_topic` 选择已配准的流。
- `speaker` 需要已校准的说话人数据源。默认输入为
  `/rokae_helios/speech/localized`，类型是可靠 QoS 的 `std_msgs/msg/String` JSON。
  字段为 Unix 秒 `timestamp`、非空 `utterance_id`、布尔 `active`，
  以及发声时的 `image_x`（图像右侧为正，范围 [-1,1]）。
  发声更新间隔须小于 500 ms。当前发言人保持锁定直到同一个
  `utterance_id` 的结束事件。声源断流或目标丢失时清除目标输出。
- `set_target(object_name="cup")` 选择 VOP 类别。VOP 必须用同一个 RGB
  相机，默认检测话题是 RGB 话题后缀 `/objects`。默认话题无发布者时，
  卡片请求 Perception 启动 VOP。大脑可用 `set_target` 切换类别。

卡片输出像素框、归一化偏差及新鲜的头部关节反馈。当前 Helios MCP 驱动
只提供头部反馈。状态中的 `motion_available` 始终为 false。
在这台机器上，RGB 卡的默认预览报告 640×400，深度为 640×480 原生光学图。
两路没有可用的配准声明，最近人脸模式需要已验证的 RGB-D 配准输入。当前机器的麦克风服务尚未
确认能够发布逐句、相机坐标下的说话人位置。

独立镜像通过构建目标 `rokae/helios-face-follow` 生成。
