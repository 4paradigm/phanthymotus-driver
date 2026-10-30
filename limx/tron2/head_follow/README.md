# TRON2 头部人物跟随

这是原 TRON2 驱动中的 Python 模块，不是独立服务。卡片由父目录 `main.py` / `device.py`
在原 `127.0.0.1:15791/mcp` 提供；没有单独 Dockerfile、容器或服务注册。

在画布启用同一驱动的头部 `camera` → `yolo_detect` 后，执行 `head_visual_follow` 的默认
`follow`。界面没有 `pause` / `status`；内部 `start` / `stop` / `info` 供项目生命周期使用。
启动容器和启动项目只使卡片就绪，不会自动转头。停用卡片或关闭智能控制阻止后续目标，
不能中途制动已经受理的动作。

人物可从整幅图像获得候选，优先靠近中心的人；至少两个不同的新鲜帧确认后跟踪。
目标靠图像检测框的空间连续性关联，不识别人脸身份。交叉、短暂遮挡或存在多个相近候选
时保持，不立即换人。已锁定的人消失，或者启用时连续未发现人，至少3个有效空帧且持续
1.5秒才回正到 pitch=0°、yaw=0°。已在回正范围内则保持。相机/模型失败、过期或重放帧
不算“无人”，不会据此回正。

跟随计算完整目标，不再分割成最多4°的步进；约6°/s，单次0.6–15秒，pitch在−55°至30°、
yaw在−45°至45°。目标用原 `move_head` 的参数验证和 `HeadCompletion.dispatch` 路径执行，
原限位、模式/连接检查、动作互斥、ACP和真实到位确认均保留。一次动作完成、解除占用并
收到之后的新鲜图像，才发送下一次目标。这仍是串行 MoveH，不是连续伺服或转动途中改目标。

工作线程通过 `vision.Runtime` 读取原 Adapter 的反馈并将头部请求交回其 asyncio 循环。
没有额外 loopback HTTP 调用、厂商写接口、机械臂或夹爪指令。机械臂移动、其它头部命令、
动作失败或结果未知时停止跟随；未知发送不重试、不自动回正。

配置需同时启用 `joint_state`、`camera`、`move_head`、`yolo_detect`、`head_visual_follow`。
检测必须来自头部 `/limx_tron2/camera`，两腕相机不能作为本控制器的输入。

构建、部署和迁移见父目录 [README](../README.md)。测试在父目录运行：

```bash
python3 -m unittest discover -s tests -v
```

`tests/test_head_follow.py` 覆盖控制状态机；`tests/test_vision.py` 覆盖单入口发现、生命周期、
同进程反馈/发送、线程死锁防护、原头部校验复用及真实 loopback HTTP。均不控制实体机器人。
