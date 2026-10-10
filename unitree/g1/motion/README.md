# G1 数值进程

`MotionControl` 在 Driver 主进程只处理校验、最新目标和生命周期。`NumericalWorker` 在独立 CPU Python 进程加载 G1_23 模型、FK/IK、碰撞和显示；ROS、控制权、持续执行、保持和停止留在 arm。两个进程之间只有一个在途请求，协调器只保留最新待处理输入。没有GPU依赖或额外遥操构建开关。

## 普通镜像构建

主 Driver、`servo_eef` 和 IK 子进程共用 `/usr/local/bin/python3` 及同一套依赖。子进程使用 `sys.executable`，不再选择专用解释器；超时终止和重启机制不变。

Dockerfile 在构建阶段使用固定版本 micromamba 安装 `requirements.numeric-linux-aarch64.lock`，将共享运行时复制到 `/usr/local`。最终镜像不保留 micromamba 或 `/opt/motion` 环境，也不再额外安装 pip `pin`。ROS 基础镜像的系统 Python 由系统包保留，Driver 不使用它。

共享依赖固定为 NumPy 1.26.4、Pinocchio 3.1.0、CasADi 3.6.7。OpenCV 固定到兼容 NumPy 1.x 的 4.11.0.86，避免 pip 升级 NumPy。`LD_LIBRARY_PATH` 包含 `/usr/local/lib`，使 ROS 和数值模块使用同一套兼容的 C++ 运行库。

包清单固定下载 URL 和 MD5。开发环境也需要支持 `pinocchio.casadi` 的依赖；缺失时明确报错，不退化为未求解结果。镜像构建会检查版本并下载、验证模型资产：

```sh
python3 -c 'import numpy, casadi, pinocchio; from pinocchio import casadi as cpin; assert numpy.__version__ == "1.26.4"; assert casadi.__version__ == "3.6.7"; assert pinocchio.__version__ == "3.1.0"'
python3 /work/motion/fetch_assets.py --check
```

七个碰撞网格只在构建时下载并逐文件核验 SHA256。网格不进入Git；许可证和清单随源码，运行时再次验证哈希。`calibration.example.json` 仍是未实物标定样例，不能填写假验收或直接作为Live配置。

主进程与数值进程使用同一 Pinocchio 版本，但各自创建模型和 Data。执行器校验模型哈希和关节顺序，不能跨进程共享可变求解状态。当前遥操不增加重力补偿。

## 离线验证

真实数值用例覆盖 G1_23 模型和已核验网格的完整关节解、独立数值子进程的 EEF14→joint10 和双末端反馈，以及空配置执行器接受新标定、无效候选保留前一配置。统一运行时已在 ARM64 镜像和上海 G1 的隔离构建环境中验证：Pinocchio 3.1.0、CasADi 3.6.7、NumPy 1.26.4，G1 与 servo_eef 回归共 196 项通过、无跳过。机上验证使用已构建候选的文件系统，不访问设备，不代表业务部署或物理动作验收。

```sh
python -m pytest -q unitree/g1/tests/test_motion_control_numeric.py
python -m pytest -q unitree/g1/tests/test_motion_control_contract.py
python -m pytest -q unitree/g1/tests/test_motion_control_threads.py
```

第二组6项为明确执行器/数值替身的卡片协议测试，覆盖最新值、会话栅栏、失败后同映射恢复、完整参考及包络。它不证明SDK执行或物理完成。缺少真实数值ABI时第一组会skip，不能把skip记作通过。

第三组运行真实遥操线程、运动协调线程和数值 IPC 子进程，验证最新待处理帧覆盖、通信超时、旧进程退出、重启、新代次拒绝旧结果以及新帧恢复发布。测试复用硬件替身，并以可阻塞的合成求解器替代 IK；需要 pytest、NumPy 和 SciPy，不需要 ROS、SDK 或 Pinocchio。它不验证实际求解精度或物理停止。

## 输入与状态

| 方向 | Topic | 格式 |
|---|---|---|
| 设备 → 控制卡 | `/teleop/command` | `data/teleop-cmd`，JSON schema `motus.teleop.command/1` |
| 控制卡 → Canvas 监控 | `/teleop/state` | `data/teleop-state` |

设备输入为左右控制器位姿、握把、跟踪状态、身份、代次、序号与时效。设备卡不订阅机器人反馈；PICO 只显示连接状态和握把提示。URDF、关节顺序、数值求解与执行均在机器人 Driver 内。

## 机型 profile 与每轮基准

`motion/body23_fixed_hand.json` 固定关节映射、URDF SHA、双掌 TCP（腕轴前方 0.2 m）和手柄局部变换，沿用本轮 r11b 已实测的原厂固定手配置。该文件不包含某次现场腰腿角度，也不填写 acceptance 通过标记。每次初始化映射时，从同一份新鲜 LowState 采集双臂和 13 个腰腿关节，为数值工作进程生成本轮临时 profile；工作进程重启沿用同一份基准，新的遥操会话重新采样。缺失、过期或非有限实测值会明确报告，不用零值猜测机器人姿态。松握重握不重新标定。

适用范围是原厂 G1_23 固定假手；不宣称自动适配 G1_29、灵巧手或改装 TCP。附加碰撞检查仍关闭，旧现场预览空间盒未纳入默认 profile，不能将其当作碰撞验收结果。模型及上述 TCP 有本轮现场使用依据，但没有补造独立尺寸或长时机械验收记录。

卡片注册、info、默认启用及空闲 stop 不申请运动权、不创建 SDK 命令通道。`servo`、`servo_eef` 可以同时注册；本实现不新增统一控制器仲裁，也不在装载时因其他卡片 enabled 而拒绝遥操。旧会话/过期指令隔离与本遥操的停止取消仍保留。

## 执行与平滑

每个有效目标仅下发一次。以后端最后成功下发的位置为起点，做 120 ms 一阶指数平滑，再将各关节增量裁剪到 ±1 rad/s × dt；dt 最大 50 ms，断帧或重握不积累追赶额度。仅处理最新目标，不排队补发历史动作。这限制的是位置参考的变化率，不是实测电机速度的硬保证。

遥操肩肘增益为 kp=80/kd=3，腕部为 40/1.5；普通 servo 增益不变，不增加重力补偿。短暂 IK 失败丢弃失败结果及其滤波历史，后续有效新帧可续接。松握恢复不等待静止阈值；`continuation_ready` 表示可以继续接收目标，与物理保持确认 `hold_confirmed` 区分。

输入有效性、反馈时效与 URDF 关节限位仍校验；附加软件碰撞/工作区扫描默认关闭，不能据此认为任意姿态均无碰撞。原始电机遥测保留，厂商保护不修改。Driver 不自动停止其他控制程序。

## 状态与恢复

Canvas 监控的 `input_status` 区分 `waiting_binding`、`stopped`、`waiting_input`、`input_stale`、`transport_error` 和 `fresh`。`transport` 提供接收/接受序号、拒绝原因、订阅代次和执行器心跳；MCP 在线不等于 DDS 接收正常。

订阅创建、读取和销毁由同一专用 ROS 线程执行；设备身份补全不重建同 topic 订阅。IK 子进程故障分别报告 `ik_worker_timeout`、`ik_worker_exited` 或 `ik_worker_unavailable`，原始异常保留在日志中。

空闲且从未接管 SDK 时，重复停止返回 no-op；不创建释放任务，也不声称已验证物理停止。未知的厂商动作结果仍报告未知，不通过重复 stop 清除。Driver 不持久化或自动恢复运动租约。

## 平滑与轨迹衔接的参考来源

感谢 **@jsmy-CTH** 在 [PR #322：提供实际下发状态用于轨迹衔接](https://github.com/4paradigm/phanthymotus-driver/pull/322) 中的贡献。本实现参考其“以实际成功下发的关节参考作为后续轨迹衔接起点”的做法，避免反复以存在滞后的实测位置作为推进起点。请将 #322 作为这部分设计来源保留。120ms一阶指数平滑及本卡1rad/s、dt最大50ms的组合是本次G1适配追加的实现，不冒称为#322原有全部算法，也不将参考部分描述为本PR独立首创。

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
