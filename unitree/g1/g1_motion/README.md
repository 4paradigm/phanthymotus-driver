# G1 数值进程

`MotionControl` 在 Driver 主进程只处理校验、最新目标和生命周期。`NumericalWorker` 在独立 CPU Python 进程加载 G1_23 模型、FK/IK、碰撞和显示；ROS、控制权、持续执行、保持和停止留在 arm。两个进程之间只有一个在途请求，协调器只保留最新待处理输入。没有GPU依赖或额外遥操构建开关。

## 普通镜像构建

将当前 bundle 的 `g1_motion/` 和公共 `common/motion/` 一并放进镜像。构建阶段使用项目既有受固定版本控制的 micromamba，直接安装本目录的显式 ARM64 包清单：

```sh
micromamba create --yes --prefix /opt/g1-motion --file /work/g1_motion/requirements.numeric-linux-aarch64.lock
/opt/g1-motion/bin/python -c 'import numpy, casadi, pinocchio; from pinocchio import casadi as cpin; assert numpy.__version__ == "1.26.4"; assert casadi.__version__ == "3.6.7"; assert pinocchio.__version__ == "3.1.0"'
python3 /work/g1_motion/fetch_assets.py
python3 /work/g1_motion/fetch_assets.py --check
```

清单来自既有 G1 CPU 版本，固定包URL和包MD5；复用文件SHA256为 `20ce04ace4936c26281be963cc71fd163ee6966d2c5f996f11f235abef909d7d`。新镜像仍需在目标 Linux ARM64 构建验证，不把历史镜像结果当本轮构建通过。`worker.py` 自动优先 `/opt/g1-motion/bin/python`；开发机器不存在该路径时使用当前解释器，缺少 CasADi-enabled Pinocchio 会报错，不退化为无碰撞或未求解结果。

七个碰撞网格只在构建时下载并逐文件核验 SHA256。网格不进入Git；许可证和清单随源码，运行时再次验证哈希。`calibration.example.json` 仍是未实物标定样例，不能填写假验收或直接作为Live配置。

主进程最终指令的 RNEA 不依赖 CasADi；arm 应使用其独立模型和Data，并验证模型hash、关节顺序与数值进程一致。允许主进程使用正常 Pinocchio ABI，须用同一模型和q做数值对照；不得复用 IK 队列、等待重计算或在失败时填零力矩。

本轮用同一 G1_23 URDF、同一固定关节值、零位及100组确定随机姿态，对比本地 Pinocchio 3.1.0 与 3.7.0 的最终位置重力补偿：101组的最大绝对差为0 N·m。该结果只验证本机两套ABI的模型计算一致，不证明目标机器的SDK下发或力矩效果。

## 离线验证

本地已验证的三个数值用例：真实 G1_23模型和已核验网格的完整关节解、独立数值子进程的 EEF14→joint10 和双末端反馈，以及空配置执行器接受新标定、无效候选保留前一配置。使用 macOS ARM64、Pinocchio3.1.0/CasADi3.6.7/NumPy1.26.4；不是Linux ARM64构建或机器人验收。

```sh
python -m pytest -q unitree/g1/tests/test_motion_control_numeric.py
python -m pytest -q unitree/g1/tests/test_motion_control_contract.py
```

第二组6项为明确执行器/数值替身的卡片协议测试，覆盖最新值、会话栅栏、失败后同映射恢复、完整参考及包络。它不证明SDK执行或物理完成。缺少真实数值ABI时第一组会skip，不能把skip记作通过。
