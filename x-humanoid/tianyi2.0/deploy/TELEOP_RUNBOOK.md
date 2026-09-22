# 天轶遥操：构建、配置与部署

遥操输入接入与末端映射运行在普通 ActuCore 内；同一 Driver 的 motion_control 提供 IK、碰撞和收臂，arm 提供连续关节执行。Canvas 的 `teleop` 卡片是用户配置/连接入口；PICO 的开始、结束并收臂和立即停止控件使用同一会话。不要部署第二个 ActuCore，也不要把本说明当成绕过当前占用或设备验收的授权。

## 机上构建

上传源码而非本地镜像。先确认目标架构、个人目录属主、数据盘、Docker 数据根和空间；不要借用其他项目目录。下面在已准备好的个人事项目录中执行，`src/driver` 为本仓源码：

```sh
mkdir -p build-tmp evidence
export TMPDIR="$PWD/build-tmp"
bash src/driver/x-humanoid/tianyi2.0/deploy/build_teleop.sh \
  local/phanthy-motus/tianyi-driver:teleop-candidate \
  > evidence/driver-build.log 2>&1
```

脚本只构建，不推送、不启动服务。临时上下文复用仓内 common；audio_msgs 使用基础镜像已有的 `/ros_ws/install` 环境，不再复制和重复编译。厂商消息编译或实际主入口导入失败立即报错。基础镜像按 digest 固定，apt 保留签名验证；镜像 ID 与源码哈希应写入本次私有部署记录。

标准 CI / bot 入口是仓根的 `bash build.sh --mirror tencent x-humanoid/tianyi2.0`，它依据 `driver.yaml` 的 `build_context_extras` 将仓根 `common/` 复制到临时上下文的同名目录。专用脚本使用同一布局；源码上传须包含该共享目录，不能只上传 Tianyi 子目录。标准入口配置仓库凭据时会推送，不能当作专用脚本的“仅构建”替代。

Docker 构建的 RUN 不执行基础镜像 ENTRYPOINT。编译和导入检查因此显式加载 `/opt/ros/humble` → `/ros_ws/install`，完成编译后再加载 `/tianyi_ws/install`；运行 CMD 也使用相同顺序。替换 ROS_BASE_IMAGE 时必须保留兼容的 audio_msgs overlay，缺失时构建或启动明确失败，不忽略错误。`AudioChunk` 与实际 Tianyi 主入口导入检查保留。

镜像体积核验（2026-09-22）：registry 中 `release.260922.2255037` 的压缩层合计 **341,727,193 B**；其前 15 层与基础镜像 `sha256:82d45949e7c3fd85e6baf4a2b24b384a3ec020a5e237c5f801bc2f2269ca649f` 完全一致，基础层合计 **261,578,370 B**。该成功版本曾额外复制 audio_msgs（独立层 **743 B**）并与两个厂商包一起重编译（三包共同层 **6,754,542 B**）；后者不能全部计为 audio_msgs 增量。此前日志/overlay 修复已移除冗余副本和编译；下述三段架构迁移另外增加了锁定的数值依赖，不能沿用此前体积结论。由于前一失败版本没有可比成功镜像，新版本净体积变化需按新 manifest 实测，不声称“零增量”。此前日志修复只复用镜像已有 common。新 motion_control 的 NumPy/SciPy/Pinocchio 及 cmeel ABI 依赖由 tianyi_motion/requirements.lock 固定并验证哈希，必须另行构建并记录真实体积。

ActuCore 使用配套主仓的普通 bundle 构建入口和遥操依赖，不沿用早期独立服务示例。Jetson 部署使用主仓 `deploy/build_actucore.sh --jp-version 6.1 --with-teleop`，具体镜像和设备架构由实际部署选择。该主仓脚本在配置仓库凭据时还会推送，使用前应核对其发布设置与授权。不能把 Jetson 产物当作 x86/G1 通用镜像。

## Driver 配置

保留当前插件和 ROS 网络配置，只增补相关项。以下为 Shadow 示例，路径必须替换为该部署已准备并只读挂载的标定文件：

```yaml
motion_control:
  enabled: true
  calibration_path: /calibration/tianyi.json
teleop:
  enabled: true
  live_enabled: false
  calibration_path: /calibration/tianyi.json
  first_acceptance_enabled: false
  operator_session_enabled: false
  continuation_timeout_ms: 300
  feedback_fault_timeout_ms: 300
```

新三段卡片接线、动作空间与管理调用见 [MOTION_CONTROL.md](../MOTION_CONTROL.md)；旧执行契约见 [TELEOP.md](../TELEOP.md)。双臂阶段明确 `hands_enabled: false`，不伪造手部端点或验收标志。新 motion_control 运行期速度取标定值；未指定时为 1 rad/s，最大仍受 URDF 与 1.5 rad/s 的已有边界约束。旧 v1 单独运行的默认值维持 0.2 rad/s。

普通 ActuCore 的 teleop 配置选择 `robot_profile=tianyi2`、匹配机器人 namespace，连接同机 Driver MCP。默认 MCP 为 15730，PICO WSS 为 15741；每次部署核对实际服务端口。Agent Core 的 `TELEOP_MANAGEMENT_URL` 指向同机 ActuCore `/mcp`，两端挂载同一管理密钥。证书、配对凭据、标定实测资料和回放租约 journal 不入仓库。

连续目标的 domain 42 只允许本机 DDS；Driver 的厂商 domain 0 继续使用已核验的机器人网络配置。不能把本机隔离 profile 误用到厂商总线。统一 ActuCore 必须包含 DDS 参考文件并挂载对应配置。

## 离线验证

从 Driver 仓根运行相关状态机、管理、反馈、发布及生命周期测试：

```sh
python3 -m pytest -q \
  x-humanoid/tianyi2.0/tests/test_motion_stream.py \
  x-humanoid/tianyi2.0/tests/test_first_acceptance.py \
  x-humanoid/tianyi2.0/tests/test_teleop_*.py \
  x-humanoid/tianyi2.0/tests/test_ros_scheduler.py
```

这些测试用假时钟、合成反馈、记录式发布器和本地 socket，不连接机器人。`tests/benchmark_feedback_scheduler.py` 则是真实 ROS 只读订阅工具，不能混同于离线测试，也不应在设备承担任务时擅自运行。

构建回归使用 `python3 -m pytest -q tests/test_tianyi_build_context.py`：在独立临时源码副本中运行两个真实 Shell 入口，用 Docker 记录器检查上下文；提取 Dockerfile 的编译、导入、启动 Shell 命令，在临时环境中验证 overlay 加载顺序及缺失失败。不连接 Docker daemon，不构建或推送镜像；实际 ARM64 编译和依赖导入仍须由镜像构建验证。

`tests/test_teleop_logsafe.py`（相对 Tianyi 目录）真实启动 `teleop_executor.py --bus` 子进程，以 ROS stub 和匿名本地 socketpair 验证：源码/镜像布局均在导入 ROS 前安装 common.logsafe，stdout、stderr 的 Python 输出及启动异常移除控制字符并按行写入。它不创建 DDS 参与者；logsafe 不包装 C/C++ 库直接写文件描述符的原生日志，不能据此声称所有原生输出已受保护。

将源码中的 `tests/image_shadow_smoke.py` 只读挂载到候选容器；生产镜像默认不打包测试脚本。需在 network none、只读根、仅 /tmp 可写、无设备/真实配置/凭据的隔离环境中执行，并加载镜像 ROS 环境。它验证实际 bundle 双轮启停和 Shadow 拒绝 claim，硬件发布器必须为零。合成 ROS/MCP 测试不能替代真机。

## 发布前后检查

1. 读取当前 Compose、镜像、配置、挂载和占用，确认项目/编辑任务空闲、Driver 无租约/输出、停止已确认。仅凭 Compose 与旧快照不同不能判断设备不安全，但发布前必须审查实际差异。
2. 根据当前部署生成目标服务的增量及回滚方案。保留用户 Agent Core 代码、镜像和业务配置；统一 ActuCore 所需管理地址变更单独列出，不恢复历史整份配置。
3. 构建和预检先完成，再切换已授权服务。修改前再次核对占用，避免与页面部署并发。未知镜像或并发 Compose 修改需要重新审查，不覆盖。
4. 回查实际镜像、MCP 工具、配置、ROS 节点/topic/QoS、反馈年龄及错误、PICO 配对和重连。普通 ActuCore 中应同时保留已有功能和 teleop，且只存在一个 ActuCore 注册入口。
5. 应用/服务启动不得自动获取执行权。Shadow 下 Driver 应为无执行权、无运动输出；Live 开始必须走显式操作者会话。准备或发布成功不表示硬件已运动。
6. 回滚前确认停止释放，不通过重启清除未知停止或租约锁存。候选崩溃时检查 Driver 和当前实际镜像；不能依赖崩溃候选自己的 MCP 才允许回滚，也不能在 Driver 状态未知时盲退。

## 操作与验收

在 Canvas 找到 ActuCore `teleop` 卡片，配置连接、模式及映射，将其 `control/eef` 输出连接到同机 Driver 的 `motion_control` 输入，再将 motion_control 的 `joints` 输出连接到 `arm`，完成 PICO 配对。先在 Canvas **开启智能控制**：Core 校验保存的三段连线和当前 Driver 声明，遥操卡片只进入 `armed`（等待 PICO 开始），尚不申请执行权或发送双臂目标。然后在 PICO 透视页点击 **开始遥操**，由 ActuCore 请求 Driver 准备本次会话。未开启智能控制时不能从 PICO 开始；应用或服务重启不会恢复 `armed` 权限。旧 v1 图仍按 TELEOP.md 的两段入口兼容，不能同时启用。

开始后先松开两侧握把；同时握住才跟随，松开任意握把保持，重新握住按实测姿态重建相对基准。正常结束在 PICO 或卡片选择 **结束并收臂**，也可在 Canvas **关闭智能控制**：ActuCore 撤销输入，Driver 的 motion_control 生成并检查回自然姿态的轨迹，确认到位及停止后释放控制权。关闭智能控制会同时撤销 `armed`，PICO 断连也可完成；收臂或释放失败会保留故障状态与反馈通路，处理原因后显式重试，不能将按钮受理当作完成。立即停止、断网或故障只请求保持，不自动收臂或张手。

日常使用上述 Canvas / PICO 流程，不依赖后端脚本。历史独立启动、回放和准备脚本仅供离线或已授权的专项诊断，不作为绕过智能控制生命周期的日常入口。新连线和项目启停生命周期目前只有离线验证，需单独完成现场验收；此前现场跟随、恢复及收臂证据不自动覆盖它。

首次试验与持续操作的开关及证据要求见执行契约。反馈中的 applied_sequence 与 last_vendor_command 不等于到位；验收必须比较实测 q/dq，记录方向、幅度、延迟、恢复和结束收臂。跟随误差如实报告，不额外设置精度门槛。碰撞、不可达、丢失反馈和停止故障保留首因，不能把 UI “运行中”当作硬件正在执行。

现场已反馈双臂跟随及结束收臂可用；手部、长期运行和全部故障注入需分别验证。设备承担接待等现有任务期间，只继续本地代码、离线回放和审查，不操作该设备。
