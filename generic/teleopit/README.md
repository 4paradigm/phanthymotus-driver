# Teleopit 动作仿真 Driver

在 PhanthyMotus 画布内启动 BVH/PICO 全身动作输入，查看 **G1 29DoF MuJoCo 仿真**、关节目标和处理耗时。控制和画面都走已有卡片，不需要另开遥操作网页。

此 Driver 是额外的可选后端，不替换既有 OpenXR/WebXR 双臂链路。实际复用 [Teleopit 0.5.0](https://github.com/BotRunner64/Teleopit/tree/v0.5.0) 的 `TeleopPipeline`：输入 → GMR 重定向/IK → ONNX 策略 → MuJoCo；这里新增的代码负责卡片、进程会话、配置、诊断和图像发布，没有自写替代算法。

## 这一版能做什么

| 卡片 | 可操作/可看到的结果 |
| --- | --- |
| `teleopit_sim`（执行器） | 配置输入、预检查、显式运行、暂停、恢复、停止；停止可取消模型加载和等待头显 |
| `teleopit_state`（传感器） | 会话、输入新鲜度、29 个关节目标/仿真测量值、各阶段计算耗时、错误原因 |
| `teleopit_preview`（传感器） | 640×480 MuJoCo JPEG 画面，最高约 5 Hz，直接由 Core 的图像卡渲染 |

不启动仿真也能启动 Driver、查看卡片及缺失依赖。启用控制卡的 `start` 只返回 ready；点击 `run` 才启动独立子进程。停止预览/状态卡不会停止仿真；停止控制卡会结束会话。`config` 和卡片齿轮设置作用于**下次运行**。

只有 `source=bvh|pico`；没有 live 开关，也不加载 Unitree SDK、G1 Bridge 或真机电机发布器。仿真状态不会被转发为机器人命令。现成策略面向 G1 29DoF，不能直接用于此前联调的 G1_23；真机适配和 PICO 设备验收不在本版已验证结果之内。

## 本地安装与启动

仿真环境使用 Python 3.10–3.12，建议与 ROS 2 的解释器分开。先阅读下文的第三方许可说明，再显式安装。源码/模型下载只在安装脚本执行；卡片不自动安装代码。

```bash
# 在 phanthymotus-driver 仓库根目录执行；工作目录可自选，不要放入待提交源码树。
uv venv --python 3.11 --seed /tmp/phanthy-teleopit-venv
/tmp/phanthy-teleopit-venv/bin/python generic/teleopit/setup_teleopit.py \
  --root /tmp/phanthy-teleopit-source --accept-third-party-licenses

export TELEOPIT_ROOT=/tmp/phanthy-teleopit-source
export TELEOPIT_PYTHON=/tmp/phanthy-teleopit-venv/bin/python

# 无 ROS 的开发机：只提供 MCP，状态可通过 info 查询；不宣称卡片已收到 DDS 画面。
"$TELEOPIT_PYTHON" generic/teleopit/main.py --no-ros --no-register
```

有 ROS 2/Core 的主机使用已配置 ROS 2 的 Python 启动 `generic/teleopit/main.py`（不加 `--no-ros`）；仿真仍由 `TELEOPIT_PYTHON` 执行。MCP 默认只监听 `127.0.0.1:15719`，通过已有本机 Core 注册和代理访问。Core DDS Domain 默认 42；与 Core 使用相同 DDS 配置。此 Driver 不创建机器人 DDS Domain 的参与者。

资产最小下载约 44 MB（不含 Python 依赖）：官方 `track_g1.onnx`、机器人模型、BVH 样例。安装器固定源码 commit `f9263865c581802ad531854b8e547e2403a945f3` 与资产 revision `94cf996444fea6894b87c28e86606cd4c2f1408f`，核对大小和 SHA-256，拒绝覆盖已有不同内容，拒绝归档路径穿越和链接。G1 不需要另下载 365 MB 的其他 GMR 资源。

## 在画布上验证

1. 添加并启动 `teleopit_state`、`teleopit_preview`，再添加 `teleopit_sim`。
2. 控制卡选择 `source=bvh`，`bvh_path` 留空用固定样例，`max_steps=500`，`render=true`。启动卡片后执行 `preflight`，确认 `ready=true`。
3. 执行 `run`。状态会从 starting 变为 running；`snapshot.step` 和 `sim_time_s` 增长，画面与关节数据更新。默认最多 500 个策略步，也会在 BVH 结束时完成。
4. 执行 `pause`，等到 paused 后确认步数冻结；`resume` 后继续。`stop` 后为 idle，子进程和输入资源释放。再次测试需重新 `start` 控制卡。

`policy_hz=50` 是策略的目标频率，不代表当前主机实际达到 50 Hz。`step_compute_ms` 是重定向、观测、推理和物理计算耗时之和，**不是 VR→真机端到端延迟**；不含输入等待、渲染、传输和卡片显示。慢帧不补发追帧。`snapshot_age_ms` 用于识别陈旧数据；画面仅用于仿真观察，不是头显实时视频回传。

无需 ROS 的 MCP 检查示例（状态/关节可以使用，图像推送需要 ROS 2）：

```bash
curl -s http://127.0.0.1:15719/mcp -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"teleopit_sim","arguments":{"action":"start"}}}'
curl -s http://127.0.0.1:15719/mcp -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":2,"method":"tools/call","params":{"name":"teleopit_sim","arguments":{"action":"run","source":"bvh","max_steps":50,"render":false}}}'
curl -s http://127.0.0.1:15719/mcp -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":3,"method":"tools/call","params":{"name":"teleopit_sim","arguments":{"action":"info"}}}'
```

## PICO Ultra 输入

此入口使用 [pico-bridge 0.2.1](https://github.com/BotRunner64/pico-bridge/releases/tag/v0.2.1) 的全身骨架数据，**不是**原有只传头部/控制器位姿的 OpenXR 客户端。按照 [Teleopit PICO 仿真教程](https://github.com/BotRunner64/Teleopit/blob/v0.5.0/docs/docs/tutorials/pico-sim2sim.md) 配置头显、全身追踪及对应 Motion Trackers，并安装匹配 APK。

1. 在同一虚拟环境重跑安装脚本，加 `--pico` 安装固定版本 Python bridge；不会安装真机 SDK。
2. PICO 和 Driver 主机使用互通的局域网。接收端默认绑定 `0.0.0.0:63901`；多网卡时，在卡片填写 `pico_advertise_host` 为头显可达的主机 IP。允许 bridge 所需的局域网流量。
3. 头显完成应用启动、授权和全身追踪校准后，卡片选择 `source=pico`、`max_steps=0`，执行 `preflight` → `run`。
4. 仿真开始/暂停/恢复/结束由卡片负责，不依赖头显的 Y/A/B 按钮。头显首次授权、应用启动和追踪校准仍需按 PICO 应用要求完成。

`running` 只说明后端已启动；必须看到 `snapshot.step` 增长才说明已完成输入→算法→仿真。`backend.waiting_for_input=true` 表示还没收到可用骨架。超过 0.5 秒未更新标记 `input_stale`，超过配置的 `input_timeout_s`（默认 10 秒）结束会话并返回错误；不会默默宣称连接成功。本地测试覆盖解析、超时和控制隔离，PICO 实机待连接验收。

## 容器部署

```bash
./build.sh generic/teleopit
```

镜像只包含 Driver 和 ROS 运行时，不内置第三方 Teleopit 源码、策略和样例。按 `deploy/service.yml` 挂载持久化 `/opt/phanthy-motus/teleopit-runtime` 后，由操作者在已启动容器里显式准备独立后端环境：

```bash
docker exec embodied-teleopit-simulation bash /work/prepare_runtime.sh \
  --accept-third-party-licenses --pico
```

首次准备完成后可直接在卡片 `preflight` / `run`，无需重启 Core。默认使用 CPU 推理与 OSMesa 无显示器渲染，不需要 privileged、机器人设备挂载或 GPU。镜像中的 ROS Python 负责发布，持久化 venv 负责仿真。Linux ARM64 镜像构建和真实 ROS/Core 渲染仍需部署环境验证；不要把本地 Intel Mac 仿真测试视为这两项已通过。

## 验证命令

```bash
# 轻量契约/生命周期/安装器测试；真实仿真测试默认明确跳过。
"$TELEOPIT_PYTHON" -m pip install pytest
"$TELEOPIT_PYTHON" -m pytest generic/teleopit/tests -q

# 实际加载固定 Teleopit、官方 BVH/模型/ONNX，检查真实 worker 与 HTTP 控制。
TELEOPIT_TEST_ROOT="$TELEOPIT_ROOT" \
  "$TELEOPIT_PYTHON" -m pytest generic/teleopit/tests -q
```

本地验证包括真实 GMR→ONNX→MuJoCo、29 关节输出、有效 JPEG、HTTP 控制及停止/重启；不包含 PICO 实机、G1 真机和 ROS/Core 浏览器端验收。`diagnostics` 列表中 `code=preview_unavailable` 的记录表示 OpenGL/OSMesa 不可用，计算仍可继续，但不能把无画面认作预览成功。

## 第三方来源与许可

Teleopit 根目录为 Apache-2.0，固定模型仓库元数据标注 MIT；**不能据此推断所有第三方代码、模型和样例都适用相同许可**。上游 `teleopit/retargeting/gmr/utils/lafan_vendor/license.txt` 写明 CC-BY-NC-ND-4.0，默认示例来自 LAFAN1；解包后的 `assets/robots/unitree_g1/LICENSE` 是 Unitree BSD-3-Clause，保留其声明。商业部署、修改或再分发前应核对对应材料的授权范围；本 Driver 不替第三方授予许可。

Driver 仓库不提交这些外部源码/资产，不在默认镜像中打包它们；安装必须由操作者阅读提示并显式确认。后续若要产品化全身真机控制，还需确认具体 G1 型号、匹配模型/策略、现场验收及上述授权问题。
