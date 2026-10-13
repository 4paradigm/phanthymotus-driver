# PiPER 机械臂控制卡

当前驱动将操作合并到中文 `arm_control` 卡片，相机 `piper_photo` 保留为可选卡。
使用现有“选择操作＋执行按钮”及通用预览确认功能，参考 Q5 的动作与参数组织。
默认操作许可关闭；模式、电机使能与操作许可独立。

- [控制卡操作与参数](CONTROL_CARD.md)
- [共享只读快照接口](MONITOR_SNAPSHOT.md)：供计划二监控插件注入使用，共用原 CAN 连接。
- [两卡集成与画布迁移](INTEGRATION.md)：计划二完成后再注册监控与模型、安排现场核对。
- [历史十卡说明](LEGACY_TEN_CARDS.md)：旧源码、回归与现场备份保留，旧工具不再对外注册。
- [历史验收记录](ACCEPTANCE.md)：原版本记录不能用作本控制卡的真机验收证据。

计划一与计划二从同一驱动提交建立独立分支。本分支交付控制卡、快照及测试；
当前尚未接入计划二监控/模型。完成集成后，每台画布放控制与监控两张核心卡，
相机可选，后台 model 不摆成卡片。两台沿用各自 MCP、CAN 和话题绑定。

## 验证

测试使用假硬件记录 CAN 调用；生命周期和监控只读接口不发送控制指令。

```bash
python3 -m unittest discover -s tests -p 'test_piper*.py' -v
```

Ubuntu 运行入口仍为 `PYTHONPATH=. python3 agilex/piper/main.py`。
Docker 打包新增 `arm_control.py`，沿用原 ROS 与固定 SDK 依赖，不新增 agent core 修改。
默认 `PIPER_MOTION_ENABLED=0`、`PIPER_CAMERA_ENABLED=0`。

现场部署前必须核对额外限位实现、现有设备状态与备份。未核对额外功能前不得覆盖现场版本；
真机动作另行取得现场授权。当前实际 Bot Version：**尚未生成**。
