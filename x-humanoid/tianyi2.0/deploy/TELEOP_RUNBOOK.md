# 天轶遥操：构建、配置与部署

遥操运行在普通 ActuCore 内，Driver 只提供执行器。Canvas 的 `teleop` 卡片是用户配置/连接入口；PICO 的开始、结束并收臂和立即停止控件使用同一会话。不要部署第二个 ActuCore，也不要把本说明当成绕过当前占用或设备验收的授权。

## 机上构建

上传源码而非本地镜像。先确认目标架构、个人目录属主、数据盘、Docker 数据根和空间；不要借用其他项目目录。下面在已准备好的个人事项目录中执行，`src/driver` 为本仓源码：

```sh
mkdir -p build-tmp evidence
export TMPDIR="$PWD/build-tmp"
bash src/driver/x-humanoid/tianyi2.0/deploy/build_teleop.sh \
  local/phanthy-motus/tianyi-driver:teleop-candidate \
  > evidence/driver-build.log 2>&1
```

脚本只构建，不推送、不启动服务。临时上下文复用仓内 common 和 audio_msgs；厂商消息编译或实际主入口导入失败立即报错。基础镜像按 digest 固定，apt 保留签名验证；镜像 ID 与源码哈希应写入本次私有部署记录。

ActuCore 使用配套主仓的普通 bundle 构建入口和遥操依赖，不沿用早期独立服务示例。Jetson 部署使用主仓 `deploy/build_actucore.sh --jp-version 6.1 --with-teleop`，具体镜像和设备架构由实际部署选择。该主仓脚本在配置仓库凭据时还会推送，使用前应核对其发布设置与授权。不能把 Jetson 产物当作 x86/G1 通用镜像。

## Driver 配置

保留当前插件和 ROS 网络配置，只增补相关项。以下为 Shadow 示例，路径必须替换为该部署已准备并只读挂载的标定文件：

```yaml
teleop:
  enabled: true
  live_enabled: false
  calibration_path: /calibration/tianyi.json
  first_acceptance_enabled: false
  operator_session_enabled: false
  continuation_timeout_ms: 300
  feedback_fault_timeout_ms: 300
```

标定及执行契约见 [TELEOP.md](../TELEOP.md)。双臂阶段明确 `hands_enabled: false`，不伪造手部端点或验收标志。运行期速度取标定值，默认 0.2 rad/s，最大 1.5 rad/s；现场配置不是仓库默认值。

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

将源码中的 `tests/image_shadow_smoke.py` 只读挂载到候选容器；生产镜像默认不打包测试脚本。需在 network none、只读根、仅 /tmp 可写、无设备/真实配置/凭据的隔离环境中执行，并加载镜像 ROS 环境。它验证实际 bundle 双轮启停和 Shadow 拒绝 claim，硬件发布器必须为零。合成 ROS/MCP 测试不能替代真机。

## 发布前后检查

1. 读取当前 Compose、镜像、配置、挂载和占用，确认项目/编辑任务空闲、Driver 无租约/输出、停止已确认。仅凭 Compose 与旧快照不同不能判断设备不安全，但发布前必须审查实际差异。
2. 根据当前部署生成目标服务的增量及回滚方案。保留用户 Agent Core 代码、镜像和业务配置；统一 ActuCore 所需管理地址变更单独列出，不恢复历史整份配置。
3. 构建和预检先完成，再切换已授权服务。修改前再次核对占用，避免与页面部署并发。未知镜像或并发 Compose 修改需要重新审查，不覆盖。
4. 回查实际镜像、MCP 工具、配置、ROS 节点/topic/QoS、反馈年龄及错误、PICO 配对和重连。普通 ActuCore 中应同时保留已有功能和 teleop，且只存在一个 ActuCore 注册入口。
5. 应用/服务启动不得自动获取执行权。Shadow 下 Driver 应为无执行权、无运动输出；Live 开始必须走显式操作者会话。准备或发布成功不表示硬件已运动。
6. 回滚前确认停止释放，不通过重启清除未知停止或租约锁存。候选崩溃时检查 Driver 和当前实际镜像；不能依赖崩溃候选自己的 MCP 才允许回滚，也不能在 Driver 状态未知时盲退。

## 操作与验收

在 Canvas 找到 ActuCore `teleop` 卡片，配置机器人、模式及映射，完成 PICO 连接/配对并查看状态。PICO 与卡片操作作用于同一个会话，不要求启动整个 Canvas 业务项目。

开始后先松开两侧握把；同时握住才跟随，松开任意握把保持，重新握住按实测姿态重建相对基准。正常结束使用“结束并收臂”，由 ActuCore 生成并检查回自然姿态的轨迹，再确认停止释放。立即停止、断网或故障只请求保持，不自动收臂或张手。

首次试验与持续操作的开关及证据要求见执行契约。反馈中的 applied_sequence 与 last_vendor_command 不等于到位；验收必须比较实测 q/dq，记录方向、幅度、延迟、恢复和结束收臂。跟随误差如实报告，不额外设置精度门槛。碰撞、不可达、丢失反馈和停止故障保留首因，不能把 UI “运行中”当作硬件正在执行。

现场已反馈双臂跟随及结束收臂可用；手部、长期运行和全部故障注入需分别验证。设备承担接待等现有任务期间，只继续本地代码、离线回放和审查，不操作该设备。
