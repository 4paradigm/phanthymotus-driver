# TRON2 双臂遥操

独立 MCP Driver，直接连接 DACH_TRON2A 厂家 WebSocket，并发送 `request_servoj`。不依赖 `limx/tron2`、G1 或 PiPER Driver 的运行服务。

| 项目 | 值 |
|---|---|
| Driver ID / MCP 端口 | `limx-tron2-teleop` / `15794`（仅本机回环） |
| 卡片 | `teleop_control`，单实例 actuator，支持 `info/config/start/stop` |
| 输入 / 输出 | `/teleop/command` / `/teleop/state`，ROS2 `std_msgs/msg/String` |
| 契约 / DDS | `motus.teleop.command/1` / domain 42、FastDDS、Core 回环配置 |
| 厂家连接 | 默认 `ws://10.192.1.2:5000`；可用 `TRON2_ROBOT_URL` 覆盖 |
| 身份 | 仅 `DACH_TRON2A_*`；可用 `TRON2_SERIAL` 固定身份，空值时锁定首次合法身份 |

## 使用条件与操作

部署前，隔离其他机器人运动控制来源。卡片声明 `arm_l`、`arm_r` 和 `head` 资源，因为厂家 ServoJ 同时写入头部两维。此声明不能提供跨进程硬件互斥。

机器人必须上报开发模式有效、电机 `OK`、碰撞检测已启用且柔顺模式已关闭。Driver 不切换机器人模式。操作人员应先确认物理停止手段、机械臂周围空间和正确的双臂型号。

1. 按平台部署流程安装镜像。挂载 Core 的 `/opt/phanthy-motus/dds-local.xml` 和可信 CA 证书。配置见 [部署片段](deploy/service.yml)。不要授予特权或挂载硬件设备。
2. 将 PICO `teleop_device` 输出连接到 `teleop_control`，由 Canvas 传入 `input_topic=/teleop/command`。启动进程只注册和采集状态；启动卡片也不会立即发送运动。
3. 松开双握把，并保持头显及双控制器跟踪有效。Driver 使用真实关节反馈建立双臂基准。
4. 同时按住双握把，开始相对位姿跟随。默认位置比例为 `0.5`，姿态按头部起始偏航坐标对齐。
5. 松开任意握把，停止发送目标并保留基准。再次按住双握把，沿用原基准。输入过期、跟踪丢失、连接或空间代次变化后，必须再次松开双握把建立基准。
6. 停止项目时，Driver 清除目标并取消待发请求。厂家拒绝命令或发送失败时，先排除故障，再停止并重新启动卡片。

**停止发送不等于实体急停。** 厂家文档未证明断流后的物理停止行为，且说明 `request_emgy_stop` 在运动期间不能响应。本 Driver 不调用该接口，不报告实体已停止。监控中的 `physical_stop_confirmed`、`stop_confirmed` 和 `hold_confirmed` 均为 `false`。

## 控制与监控

左右臂各七关节按厂家索引排列。FK/IK 从 `base_Link` 到 `wrist_roll_L_Link` / `wrist_roll_R_Link`，不添加未经标定的夹爪偏移。模型和算法来源见 [来源说明](resource/SOURCE.md)。

IK 在独立进程计算，只保留最新待算输入。七自由度阻尼最小二乘使用真实姿态作为冗余约束。求解预算为 100 ms；不可达、超时或越界时清除执行目标。关节限位取官方 SDK 与 URDF 的交集。

执行循环独立按 300 Hz 调度，不补发错过的周期。双臂命令变化限制为 `0.35 rad/s`；机器人跟踪误差超过 `0.2 rad` 时停止发送。头部两维保持本次基准的真实反馈值，不跟随头显；检测到头部偏移超过 `0.1 rad` 时停止发送并要求重新启动校准。一般 Linux、Python 和 WebSocket 不能保证硬实时，现场需测量实际周期。

输入和关节反馈的有效期均为 200 ms，机器人状态有效期为 3 s。关节请求按顺序发送，一次最多一个请求，响应后等待 20 ms；响应超时为 150 ms。ServoJ 发送超时为 10 ms。厂家没有 ServoJ 成功 ACK，socket 发送完成只表示数据已提交。

`/health` 和 5 Hz DDS 监控分别提供输入序号、基准代次、IK 耗时、发送状态、实际反馈、反馈年龄及错过周期数。反馈年龄按本机收包时间计算，厂家测量时间未验证。`commanded_q` 不代表机械臂已到位。

首版不含夹爪控制、头部跟随、录制回放或环境碰撞规划。厂家碰撞开关不能替代经过验证的碰撞模型；`collision_checked=false`。

## 构建与软件验证

在仓库根目录按现有 `build.sh` 流程构建：

```bash
bash build.sh limx/tron2_teleop
python3 scripts/check_service_yml.py limx/tron2_teleop
python3 -m pytest -q tests/test_tron2_teleop.py tests/test_teleop_contract.py tests/test_dockerfile_copies.py
```

`build.sh` 会根据本地注册表配置决定是否推送镜像。运行镜像只复制本 Driver、必要的 `common` 模块和模型，不包含测试文件或现场凭据。

真实 DDS 回环验证在 ARM64 镜像内运行。基础镜像须提供 pytest 供测试 fixture 导入。将 `IMAGE` 设为刚构建的镜像标签，再运行：

```bash
docker run --rm --network none --read-only --cap-drop ALL \
  --security-opt no-new-privileges \
  --tmpfs /tmp:rw,nosuid,nodev,noexec,size=32m \
  -v "$PWD/tests:/tests:ro" -e PYTHONPATH=/work:/tests "$IMAGE" \
  /bin/bash -c 'source /opt/ros/humble/setup.bash && python3 /tests/validate_tron2_teleop_dds.py'
```

该测试使用真实 ROS2/FastDDS、实际服务入口和独立 IK 进程，但厂家和 Core 都是本机假服务。它覆盖无输入启动、握把校准、跟随、暂停、断流、MCP 停止、监控和 SIGTERM 退出。单元测试另覆盖旧序号、旧代次、失踪跟踪、限位、不可达目标、身份拒绝、发送超时、断线及迟到计算结果。

软件通过不能替代现场验收。真实 PICO/Canvas 连线、300 Hz 时序、物理断流停止、头部保持及双臂碰撞风险均需现场验证。
