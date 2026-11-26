# G1 原地小跟班：决策原型

`stationary_follow.py` 是电脑仿真中使用的纯 Python 决策层。输入单人位置后，它只计算原地转向或停止的请求；前进和横移始终为零。它**没有**订阅真实感知数据、调用 G1 SDK、注册 Motus 卡片或加入 Docker 镜像，因此当前 PR 不能作为真机功能验收。

运行离线检查：

```sh
python -m unittest discover -s unitree/g1/tests -p test_stationary_follow.py -v
```

接入前需要完成：

1. 核对 Motus `pose` 卡片真实消息字段、时间戳语义、人物 ID 稳定性与横坐标方向；现有 `persons[].position[0]` 格式只是仿真假设。
2. 明确 G1 底座控制权与停止接口，保证与现有 `loco`、`loco_relative_move` 和 SmartMotion 不会同时写入运动指令。
3. 实现超时与目标丢失的实际停机路径，并以默认关闭的插件接入 `main.py`、`config.yaml`、`driver.yaml` 和 `Dockerfile`。
4. 完成无运动的消息链路测试，再由机器人负责人在安全场地监督真机测试；原地转向也可能产生脚步位移。
