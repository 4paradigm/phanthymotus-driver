# ImageQualityGuard

ImageQualityGuard 是 Bumi 的被动 processor Card。它订阅已有 Camera Card 的 RGB JPEG，分析低光、过曝和疑似模糊，发布 `motus.image_quality/1` JSON。它不打开 RealSense、不发送运动指令、不保存原始图像。

## 实现与使用

主框架与 Driver 必须使用本次配套修改。Canvas 将 Camera 的 `image/jpeg` 输出连到 `image_quality_guard`。Core 签发运行票据后才会启动处理；Bundle 的无参 `start()` 仅为启动钩子，不订阅图像。未绑定 Core 准入会话或缺少票据的外部 start/stop 会失败。

输入由画布提供，通常是 `/{namespace}/camera/color`，消息为 `sensor_msgs/msg/CompressedImage`，Camera 的 format 为 `jpeg`。namespace 来源为 `config.yaml.ros_namespace`，空值使用清理后的 hostname；不能写死为 robot。输出为 `{input_topic}/image_quality_guard`，消息为 `std_msgs/msg/String`。

输入 QoS：BEST_EFFORT / KEEP_LAST / depth=1 / VOLATILE，与现有 Camera 一致。输出 QoS：RELIABLE / KEEP_LAST / depth=1 / VOLATILE。Core 原有 JSON topic bridge 与 Dashboard renderer 展示状态和指标。

默认 `analysis_hz=2`、`confirm_samples=3`、`recover_samples=5`。订阅回调仅覆盖最新压缩帧；steady ROS Timer 和单调时钟限制分析频率，没有新帧则不分析、不重复发布。`info.runtime.effective_analysis_hz` 为实际分析间隔测量，不承诺硬实时。分析样本数不是原始摄像头帧数。

配置的十个算法字段仍由 `QualityConfig` 校验。有效配置变化要求先停止；用户不能配置 `run_id` 或覆盖派生输出 Topic。阈值是工程初值，尚未做 Bumi 实测标定。

## 输出

```json
{
  "schema": "motus.image_quality/1",
  "run_id": "<admission_epoch_uuid>.<run_uuid>",
  "timestamp_ms": 0,
  "timestamp_kind": "analysis_time",
  "input_topic": "/hostname/camera/color",
  "state": "UNKNOWN",
  "metrics": null,
  "reasons": [],
  "candidate_reasons": [],
  "error": null
}
```

时间戳是分析时的系统墙钟毫秒，不代表曝光时间；严格 JSON 禁止 NaN/Infinity。解码错误返回 UNKNOWN、metrics=null、明确 error，保留已确认 reasons，打断恢复连续计数。恢复仍需完整五个有效恢复样本。BLURRED 只说明低细节，均匀墙面也可能触发，不能直接解释为摄像头故障。没有新增断流报警。

## 生命周期

工具顶层 `x-lifecycle` 声明 `motus.lifecycle/1`。Core 是票据签发端，`run_id=epochUUID.runUUID`；相同运行重试复用票据，新运行换票。`instance_id` 仍是画布 Card，`connection_id` 仍是连线。

票据走系统 envelope：Canvas → Core `lifecycle.run_id` → Core 校验 → MCP `arguments.run_id` → Bundle → Guard。它不出现在普通配置/表单/LLM 参数中。UUID 不是身份认证凭证；HTTP 控制面仍依赖现有受信网络与访问控制。

Driver 通过 `motus.lifecycle.status` 和 `motus.lifecycle.bind` 建立 boot challenge/epoch CAS 绑定。initialize 不更改会话。缺票、旧票、已终止票都不能关闭新运行。外部 stop 必须有票；本地 `shutdown()` 不注册为工具，永久关闭新准入并可重试清理。终态记录有固定容量，不做 TTL/LRU 淘汰；满后仅在确认空闲时显式轮换会话。

## 文件

| 文件 | 职责 |
|---|---|
| image_quality.py | 纯算法、指标、确认与恢复；阶段 A 保持原样 |
| image_quality_guard.py | MCP 契约、准入、票据、资源归属、shutdown |
| image_quality_runtime.py | 真正 rclpy 订阅/Timer/发布/释放，共用 Bundle Executor |
| main.py / config.yaml / driver.yaml / Dockerfile | Bundle 注册、发现、镜像打包 |
| tools/image_quality_observe.py | 操作者执行的纯质量 JSON 观察工具 |
| tests/test_image_quality*.py | 算法、插件、transport、真实 ROS2、观察器验证 |

测试与限制见 [测试报告](IMAGE_QUALITY_GUARD_TEST_REPORT.md)，硬件步骤见 [真机手册](IMAGE_QUALITY_GUARD_HARDWARE_TEST.md)，部署前条件见 [部署指南](IMAGE_QUALITY_GUARD_DEPLOYMENT_GUIDE.md)。当前不能把本地 DDS 的合成 JPEG 测试称为 Bumi 摄像头验收。
