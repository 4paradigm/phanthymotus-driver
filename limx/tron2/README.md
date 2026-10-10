# LimX TRON2 双臂固定姿态复位

将操作者记录的双臂姿态暴露为 Phanthy Motus `reset_pose` 执行器卡片。
`execute` 发出一次厂家 MoveJ，并以关节反馈确认到位。其他操作只读。
不发送头部、夹爪、模式切换、拖动示教、负载辨识或校准指令。

这是固定姿态复位驱动，不是完整 TRON2 控制器或碰撞规划器。手动确认运动空间仍是执行前提。
服务每次启动都禁用运动调用。平台开启智能控制时，通过卡片 `start` 启用复位；`stop` 禁用后续调用。已 commissioning 的目标可通过网页或决策流程调用，不需要本机 `arm`。

## 接口与运行环境

- Linux / Python 3.10+；Jetson Orin NX、JetPack 6 的 ARM64 环境是已测试原型的平台。
- 原有机器人主控制器保留；Jetson 通过以太网访问厂家 WebSocket `ws://<robot>:5000`。
- MCP：同机 `http://127.0.0.1:15791/mcp`；诊断：`GET /health`。
- 需要同机 Agent Core；向其 `/api/mcp` 注册，并校验 HTTPS 证书。
- 不依赖原型的只读容器，不需要 GPU、Docker socket、privileged、ROS DDS 或 CAN 设备权限。
- 提供一个 `actuator` 卡片 `reset_pose`，`multiInstance: false`。

厂家协议与关节顺序依据：[SDK](https://www.limxdynamics.com/zh/documents/847884267345285120)
§3.6.1（MoveJ）、§3.6.7（关节状态）、§3.6.10（示教管理），以及
[用户手册](https://www.limxdynamics.com/zh/documents/844648486841487360) §1.5。
关节 API 未返回名称时，采用手册 0–13 序号：左臂 7 关节、右臂 7 关节。
不得把这份映射用于不同机器人型号。位置单位为 rad。

## 准备目标与配置

先在机器人现场确认正确序列号、固件、关节顺序和厂家模式切换方法。
用厂家拖动示教设置目标姿态，录制期间保持静止；不要在上层开发模式下硬掰机械臂。
退出示教后，在厂家上层开发模式进行复位测试。Teleoperator 不能代替已确认的上层开发模式。

在 Jetson 的仓库根目录安装开发依赖：

```bash
python3 -m venv .venv-tron2
.venv-tron2/bin/pip install -r limx/tron2/requirements.txt
mkdir -p "$HOME/tron2-data" "$HOME/tron2-runtime"
export TRON2_SERIAL='<本机机器人实际序列号>'
export TRON2_ROBOT_URL='ws://10.192.1.2:5000'
export TRON2_HOME_FILE="$HOME/tron2-data/home.json"
export TRON2_COMMISSION_FILE="$HOME/tron2-data/commissioning.json"
.venv-tron2/bin/python limx/tron2/record.py \
  --serial "$TRON2_SERIAL" --url "$TRON2_ROBOT_URL" --pose "$TRON2_HOME_FILE"
```

`record.py` 仅采集三份具有递增时间戳的静止状态，校验序列号、设备健康与角度限位，
然后独占创建目标文件。已有文件会拒绝覆盖。复位测试中移动机械臂后，不要再次录制目标。
复用原型的目标时，原样复制已有 `home.json`；不要修改目标值、元数据或摘要来绕过验证。
不要把真实机器人目标、序列号或运行文件提交到 Git。

## 构建与部署

正式镜像按仓库的 `build.sh limx/tron2` 和 PR 构建流程生成；未发布前不能从官网选择此驱动。
`driver.yaml` 的 `build_context_extras` 会把 `common/` 加入构建上下文。
需要本地 ARM64 构建时，在仓库根目录准备临时上下文：

```bash
TRON2_BUILD_CONTEXT=$(mktemp -d)
cp -R limx/tron2/. "$TRON2_BUILD_CONTEXT/"
cp -R common "$TRON2_BUILD_CONTEXT/common"
sudo docker build --platform linux/arm64 -t limx-tron2:local "$TRON2_BUILD_CONTEXT"
```

`deploy/service.yml` 是服务片段，由平台合并到 Compose；手动试部署可生成完整 Compose：

```bash
export TRON2_POSE_DIR="$HOME/tron2-data"
export TRON2_RUNTIME_DIR="$HOME/tron2-runtime"
export TRON2_CA_CERT='/实际路径/Core证书.pem'
export TRON2_AGENT_CORE_URL='https://phanthy-motus:15678'
.venv-tron2/bin/python - <<'PY'
from pathlib import Path
import yaml
fragment = yaml.safe_load(Path('limx/tron2/deploy/service.yml').read_text())
fragment['limx-tron2']['image'] = 'limx-tron2:local'
Path('tron2-compose.local.yml').write_text(yaml.safe_dump({'services': fragment}))
PY
sudo -E docker compose -f tron2-compose.local.yml up -d --pull never
```

证书必须是实际 Core 证书/信任链，URL 主机名必须匹配证书 SAN。
默认服务将 `phanthy-motus` 解析为同机 127.0.0.1。其他主机名需在服务中配置对应解析。
证书、目标目录和运行目录要提前创建；证书错误时查看日志修正，不关闭 TLS 验证。
容器根文件系统和 `/data` 只读，只有 `/runstate` 与临时目录可写。

**从原型迁移：**正式版也用 15791，同一时间只能有一个复位服务。
必须先关闭平台智能控制，确认原型无运动正在执行且无未确认运动标记，再停原型容器并启动正式服务。旧版如仍有一次授权，也应先撤销。
保留旧目标、commissioning 和独立 runtime 目录，正式版继续挂载同一 runtime 目录；
不要通过切换目录丢弃 `unconfirmed-motion.json`。备份只能保存数据，不能撤销实际运动。
正式版采用严格 TLS 注册，迁移后需要重新验证 Core 连接与网页卡片。

## 固件状态验收与 commissioning

容器启动、自动注册、卡片 `start` 都不发送运动。先查询：

```bash
python3 limx/tron2/control.py status
python3 limx/tron2/control.py preview
```

在实体手柄确认上层开发模式，并核对 `working_mode_valid`、`working_mode` 和 `teach_status`。
API 成功不能证明已经退出拖动示教。选择能够表示实际示教状态的字段，
不能用 `result`、`success` 或 `cmd` 充当模式证明。

验收原型固件中观察到 `working_mode=developer_mode`，示教消息
`state=ST_DEVED;sub=;rec=0`。对应 commissioning 示例如下；
只有本机固件及实体手柄观察一致时才可使用，不是所有固件的通用默认值：

```bash
python3 limx/tron2/control.py commission --confirm-high-level \
  --teach-state-pointer '/message#state' --teach-inactive-json '"ST_DEVED"'
python3 limx/tron2/check.py
```

`/message#state` 解析分号分隔的键值消息；必须无重复键、无示教子状态且 `rec=0`。
commissioning 绑定目标文件 SHA256、模式字符串及示教字段。
CLI 在宿主写入已挂载的目标目录，需要保持前述 `TRON2_SERIAL`、`TRON2_HOME_FILE`、
`TRON2_COMMISSION_FILE` 环境变量。此步骤不运动。
`check.py` 验证注册、状态、目标及带 `instance_id` 的卡片生命周期；也不运动。

## Motus 网页操作

刷新同机 Motus 控制台，将 `reset_pose` 拖入画布。先选择 `preview`，再开启智能控制：
当前控制台仅在运行状态启用卡片的“执行”按钮。卡片 `start` 只准备接口，不自动复位。
本流程先手动调用卡片，暂不接入决策核心或开机自动执行。

| ACTION | 行为 |
|---|---|
| `info` | 查看固定目标及摘要 |
| `status` | 查询厂家设备、关节及示教状态 |
| `preview` | 比较当前与目标，列出每个关节差值 |
| `verify_at_home` | 只读检查当前 14 个关节是否都在目标 1° 内 |
| `execute` | 平台启用后执行 MoveJ 并等待到位验收 |

确认机器人在上层开发模式、已退出示教、无人扶持且路径无障碍后，在网页选择 `execute`，点击一次“执行”。
不需要在 Jetson 执行 `arm`，也没有 60 秒授权窗口。成功后平台仍启用，可在下一次任务完成后再次调用。
关闭智能控制会通过 `stop` 禁用新的复位；已进入发送阶段的物理运动仍继续验收，`stop` 不是急停。
服务重启后平台状态不持久化，需要先关闭再开启智能控制。

若把此卡片接入决策流程，`execute` 可以作为操作完成后的最后一步；每次调用仍只发送一个固定目标 MoveJ，
错误或不确定结果不自动重试。工作流程应等待返回到位结果后再进行下一步。
一次成功调用不代表未来任意起点的回位路径无碰撞，部署者须验证任务结束姿态到保存目标的路径。

移除了原型首测阶段自定的 15° 门槛；当前位置和目标仍必须位于厂家关节限位内。
预计时间为 `max(8, ceil(最大关节差值度数 / 2.5))` 秒：例如 45.41° 用 19 秒，
100° 用 40 秒。此时间规则控制计划的平均变化量，不是厂家保证的峰值速度限制，
也不表示路径已通过碰撞检查。授权前须确认整个回位路径。
服务不接受自定义角度或任意持续时间。

成功要求：MoveJ 应答成功、至少经过预览显示的预计时间、连续三份递增时间戳反馈中，
所有双臂关节误差 ≤1° 且速度绝对值 ≤0.01 rad/s，同时模式、示教状态与健康检查仍通过。
结果 `status=arrived`、`at_home=true` 才表示到位。单纯应答 success 不算完成。
`verify_at_home` 只检查当前位置，不证明新运动的完整执行过程。

## 中断与不确定结果

- 发指令前先创建并同步持久标记；断电、重启和网络断开均不会自动重发 MoveJ。
- 请求方断开时保留后台反馈验收；并发执行拒绝。SIGTERM 在未发送阶段禁用发送，已发送时继续等待验收。
- 可能已发送但未验收到位时返回 `motion_sent: "unknown"`、`arrival_verified: false`，需要现场检查。
- MCP `stop` / 关闭智能控制禁用后续复位请求，**不是物理急停**。请使用已掌握的厂家实体停止措施。
- 确認机器人停稳并人工检查后，才可执行 `control.py clear --confirm-inspected`；它不停止、不复位，并将平台执行状态设为禁用；检查后需先关闭再开启智能控制。
- 本驱动不提供自动碰撞检测或路径规划，始终报告 `path_validated=false`。

## 测试和已知验证边界

在仓库根目录运行：

```bash
python3 -m unittest discover -s limx/tron2/tests -v
python3 -m pytest tests/test_dockerfile_copies.py tests/test_deploy_restart_policy.py -q
```

协议与真实 loopback HTTP 测试使用模拟机器人，包括状态校验、卡片生命周期、
平台 start/stop 门控、并发拒绝、发送不确定标记、断开后的反馈验收以及到位条件。

2026-10-10，操作者提供的 Jetson 原型 0.2.2 实体日志记录一次成功复位：
最大关节变化 13.2926°、MoveJ 8 秒、连续到位反馈 3 份、最大到位误差 0.14324°、
`status=arrived`、`at_home=true`。该证据来自原型 CLI，与网页调用相同控制处理函数。
**正式 0.3.0 包的镜像构建、TLS 注册和实体/网页回归仍需独立验收**；
本记录不能代替 CI 镜像与正式版本实机测试。
移除 15° 门槛后的 45.41° / 19 秒场景已做模拟协议回归。随后操作者反馈 0.2.4 原型的网页复位已完成，未附新的差值、用时或误差日志；不据此声称该大幅场景或正式镜像已通过实体验收。
