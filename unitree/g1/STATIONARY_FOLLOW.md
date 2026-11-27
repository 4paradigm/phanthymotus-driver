# G1 原地小跟班：决策原型

`stationary_follow.py` 包含跟随决策层和 `g1_stationary_follow` Motus 卡片。画布连接 `camera_rgb` → `pose` → `g1_stationary_follow`，第二条线必须使用 `pose` 的 `sensor/pose2d`（`poses/skeleton`）输出。卡片把判断与运动接口结果发布为 `data/json`，也可通过 `info` 查看。

下视相机只露出腿部时，仍使用 `pose` 给出的人物 ID 和骨架：左右膝或脚踝关键点置信度均达到 0.3 时，以两侧腿部的水平中心控制原地转向；否则回退到同一人物的 bbox。缺少人物 ID 的帧被拒绝，避免把无 ID 的不同人当作同一目标。`pose` 识别失败时不使用仅凭深度几何形状猜人的方案。

运行离线检查：

```sh
python -m unittest discover -s unitree/g1/tests -p test_stationary_follow.py -v
```

控制边界与真机验收：

1. 在画布连接真实 `sensor/pose2d` 输出，核对 `image_size`、`bbox`/`keypoints`、人物 ID 稳定性与相机横坐标方向。只有标准姿态格式才会得到有效判断。
2. 卡片只发送 `vx=0, vy=0` 的短时低速转向，经 SmartMotion 执行。只有平衡运动模式、底座空闲、没有相对运动任务时才发送；其他底座卡片的运动命令在小跟班运行时被拒绝。
3. 人物居中、目标丢失、姿态超过 0.3 秒未更新或停止卡片时不再发送新动作；已发送的定时指令最多再运行 0.12 秒后由 TimedMotion 自动结束。避免在定时指令结束后重复调用同步停止接口，造成 Core 的 5 秒停止超时。单次转向速度上限 0.2 rad/s。
4. 真机测试需机器人负责人在安全场地监督，先验证相机画面、姿态输出与转向方向；原地转向也可能产生脚步位移。单元测试不能代替真机验收。
