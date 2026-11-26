# G1 原地小跟班：决策原型

`stationary_follow.py` 包含跟随决策层和 `g1_stationary_follow` Motus 卡片。卡片在 G1 Driver 中注册，画布可连接 `sensor/pose2d` 人物姿态输入。它将 `image_size` 与人物 `bbox`（或可见关键点）换算为画面横向位置，并在 `info` 中显示转向或停止判断。当前是**预览模式**，不调用 G1 运动接口，不能作为真机跟随功能验收。

运行离线检查：

```sh
python -m unittest discover -s unitree/g1/tests -p test_stationary_follow.py -v
```

接入前需要完成：

1. 在画布连接真实 `sensor/pose2d` 输出，核对 `image_size`、`bbox`/`keypoints`、人物 ID 稳定性与相机横坐标方向。只有标准姿态格式才会得到有效判断。
2. 明确 G1 底座控制权与停止接口，保证与现有 `loco`、`loco_relative_move` 和 SmartMotion 不会同时写入运动指令。
3. 在预览模式确认丢失目标、数据超时均产生停止判断，再实现受保护的真机运动适配。
4. 真机测试需机器人负责人在安全场地监督；原地转向也可能产生脚步位移。
