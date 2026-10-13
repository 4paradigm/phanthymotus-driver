# G1 数值进程

`MotionControl` 在 Driver 主进程只处理校验、最新目标和生命周期。`NumericalWorker` 在独立 CPU Python 进程加载 G1_23 模型、FK/IK、碰撞和显示；ROS、控制权、持续执行、保持和停止留在 arm。两个进程之间只有一个在途请求，协调器只保留最新待处理输入。没有GPU依赖或额外遥操构建开关。

## 代码与依赖

| 模块 | 职责 |
|---|---|
| `../teleop_control.py`、`../teleop_bus.py` | 设备输入、相对映射、会话与 ROS 订阅 |
| `../motion_control.py`、`../motion_bus.py` | 数值协调、最新目标与运动卡传输 |
| `worker.py`、`../../../common/motion/worker.py` | G1 求解器入口与独立进程 IPC |
| `kinematics.py`、`ik.py` | 固定关节、FK 与 CasADi IK |
| `profile.py`、`body23_fixed_hand.json` | 机型几何及每轮实测基准 |
| `collision.py`、`workspace.py`、`visualization.py` | 可选空间检查与诊断数据 |
| `../arm_stream.py` | 控制权、位置下发、保持与停止 |

主进程和数值子进程共用系统 `/usr/bin/python3` 及依赖，各自创建模型和 Data。数值依赖由 [Dockerfile.base](../Dockerfile.base) 固定：NumPy 2.2.6、Pinocchio 4.1.0、CasADi 3.8.1、SciPy 1.15.3。构建步骤见 [G1 构建说明](../README.md#构建)。

普通 `pin` wheel 不包含所需 CasADi 绑定。base 从源码启用 `BUILD_WITH_CASADI_SUPPORT`，运行阶段只复制安装产物。开发环境先导入 `casadi`，再导入 `pinocchio.casadi`；缺失绑定时明确报错。`CMEEL_RUN_TESTS=OFF` 仅关闭依赖上游的打包测试，项目回归须单独运行。

七个碰撞网格在构建时下载，构建和运行时均核验 SHA256。网格不进入 Git；许可证与哈希清单位于 `models/collision/`。测试专用的未标定数据位于 `../tests/fixtures/calibration.json`，不随业务镜像安装。

## 输入与状态

| 方向 | Topic | 格式 |
|---|---|---|
| 设备 → 控制卡 | `/teleop/command` | `data/teleop-cmd`，JSON schema `motus.teleop.command/1` |
| 控制卡 → Canvas 监控 | `/teleop/state` | `data/teleop-state` |

设备输入为左右控制器位姿、握把、跟踪状态、身份、代次、序号与时效。设备卡不订阅机器人反馈；PICO 只显示连接状态和握把提示。URDF、关节顺序、数值求解与执行均在机器人 Driver 内。

## 机型 profile 与每轮基准

`motion/body23_fixed_hand.json` 固定关节映射、URDF SHA、双掌 TCP（腕轴前方 0.2 m）和手柄局部变换。该文件不包含某次现场腰腿角度，也不填写 acceptance 通过标记。每次初始化映射时，从同一份新鲜 LowState 采集双臂和 13 个腰腿关节，为数值工作进程生成本轮临时 profile；工作进程重启沿用同一份基准，新的遥操会话重新采样。缺失、过期或非有限实测值会明确报告，不用零值猜测机器人姿态。松握重握不重新标定。

适用范围是原厂 G1_23 固定假手；不宣称自动适配 G1_29、灵巧手或改装 TCP。附加碰撞检查仍关闭，旧现场预览空间盒未纳入默认 profile，不能将其当作碰撞验收结果。

卡片注册、info、默认启用及空闲 stop 不申请运动权、不创建 SDK 命令通道。`servo`、`servo_eef` 可以同时注册；本实现不新增统一控制器仲裁，也不在装载时因其他卡片 enabled 而拒绝遥操。旧会话/过期指令隔离与本遥操的停止取消仍保留。

## 执行与平滑

每个有效目标仅下发一次。以后端最后成功下发的位置为起点，做 120 ms 一阶指数平滑，再将各关节增量裁剪到 ±1 rad/s × dt；dt 最大 50 ms，断帧或重握不积累追赶额度。仅处理最新目标，不排队补发历史动作。这限制的是位置参考的变化率，不是实测电机速度的硬保证。

遥操肩肘增益为 kp=80/kd=3，腕部为 40/1.5；普通 servo 增益不变，不增加重力补偿。短暂 IK 失败丢弃失败结果及其滤波历史，后续有效新帧可续接。松握恢复不等待静止阈值；`continuation_ready` 表示可以继续接收目标，与物理保持确认 `hold_confirmed` 区分。

输入有效性、反馈时效与 URDF 关节限位仍校验；附加软件碰撞/工作区扫描默认关闭，不能据此认为任意姿态均无碰撞。原始电机遥测保留，厂商保护不修改。Driver 不自动停止其他控制程序。

## 状态与恢复

Canvas 监控的 `input_status` 区分 `waiting_binding`、`stopped`、`waiting_input`、`input_stale`、`transport_error` 和 `fresh`。`transport` 提供接收/接受序号、拒绝原因、订阅代次和执行器心跳；MCP 在线不等于 DDS 接收正常。

订阅创建、读取和销毁由同一专用 ROS 线程执行；设备身份补全不重建同 topic 订阅。IK 子进程故障分别报告 `ik_worker_timeout`、`ik_worker_exited` 或 `ik_worker_unavailable`，原始异常保留在日志中。

空闲且从未接管 SDK 时，重复停止返回 no-op；不创建释放任务，也不声称已验证物理停止。未知的厂商动作结果仍报告未知，不通过重复 stop 清除。Driver 不持久化或自动恢复运动租约。

“以后端最后成功下发的位置续接”的设计参考 **@jsmy-CTH** 的 [PR #322](https://github.com/4paradigm/phanthymotus-driver/pull/322)。120 ms 平滑、1 rad/s 限幅及 50 ms 时间上限为本次 G1 适配参数。其他来源见 [NOTICE](NOTICE.md)。

## 离线验证

在仓库根目录、具备 G1 运行依赖和 pytest 的环境中执行：

```sh
python3 -m pytest -q -rs tests/test_teleop_contract.py tests/test_g1_servo_eef.py unitree/g1/tests
```

`test_motion_control_numeric.py` 使用真实模型、网格和 IK 子进程；缺少数值依赖时会跳过，不能把跳过记作通过。协议、执行器与线程用例使用硬件替身；线程用例还使用合成求解器测试超时、重启和旧代次隔离。这些测试不验证机器人动作或物理停止。

## ROS 生命周期验证

[手动验证脚本](../tests/validate_ros_lifecycle.py) 使用真实 ROS 节点和接收线程，检查订阅切换、线程归属、InvalidHandle 恢复及错误状态。控制对象使用替身，不导入硬件 SDK，不验证机器人动作。脚本不由默认 pytest 收集。

在 Linux ARM64 测试机的仓库根目录执行。将 `G1_TEST_IMAGE` 设置为已有的 G1 Driver 镜像；容器使用隔离网络、只读文件系统和只读源码，不挂载设备：

```sh
docker run --rm --network none --read-only --tmpfs /tmp \
  -v "$PWD:/src:ro" -w /src \
  -e G1_TELEOP_ISOLATED_TEST=1 -e PYTHONDONTWRITEBYTECODE=1 \
  -e PYTHONPATH=/src -e ROS_LOG_DIR=/tmp/ros-log \
  --entrypoint /bin/bash "$G1_TEST_IMAGE" -lc \
  'source /opt/ros/humble/setup.bash && python3 unitree/g1/tests/validate_ros_lifecycle.py'
```

成功时输出 `ROS ISOLATED PASS`。环境变量仅确认操作者已选择隔离环境，不会自动检查容器网络或设备挂载。真实模型验证仍使用 `test_motion_control_numeric.py`。
