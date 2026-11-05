# Dobot CRA joints and joint_move

基于官方 V4 Python SDK 的 Jetson Docker 驱动。本版提供并列的 `joints` 与 `joint_move`
两张可见卡片，URDF 由后台 `model` 资源提供。SDK 版本见 [SDK.md](SDK.md)。

## 卡片

| 名称 | 类型 | 内容 |
|---|---|---|
| `joint_move` | actuator | 绝对关节运动、停止、暂停及继续，详见 [JOINT_MOVE.md](JOINT_MOVE.md) |
| `joints` | sensor | 六轴实际角度驱动的 CR3A 结构骨架、逐关节转角 JSON，以及共享连接配置 |

`model` 为渲染器内部读取的 resource，不是可见卡片。
本版不注册 `safe_skin` 或 `safe_skin_control`，不发布安全皮肤 topic，也不提供皮肤设置入口。
升级后请移除画布中的旧安全皮肤卡片及连线；不会自动修改已有画布。

## 关节输出

默认以 30Hz（约每 33ms）发布 `std_msgs/String` JSON，两个输出共用同一份反馈快照：

- `/dobot_cra/state/joints`，`sensor/skeleton`；仅包含 `timestamp_ms/available/joints`。
  每个关节包含 `name/q`，`q` 为弧度，关节名为 `joint1..joint6`。
- `/dobot_cra/state/joint_angles`，`data/json`；包含 `timestamp_ms/available` 和
  `joint1_deg` 至 `joint6_deg`，每个关节转角单独一行，单位为度。

JSON 输出示例（数值为示例）：

```json
{
  "timestamp_ms": 1793856449751,
  "available": true,
  "joint1_deg": 90,
  "joint2_deg": -30,
  "joint3_deg": 0,
  "joint4_deg": 0,
  "joint5_deg": 0,
  "joint6_deg": 0
}
```

断线或过期时 `available=false`，骨架 `joints=[]`，六个角度字段均为 `null`，不输出旧角度。
角度 JSON 不依赖 URDF 是否可用；型号不匹配时仅骨架停止提供关节位置。
输出不含 IP/端口、连接诊断、包计数、SDK 来源或冗余角度数组。
旧下游若依赖 `joint_names/angles_deg`，请改读 `joint1_deg` 至 `joint6_deg`。

## 卡片介绍

内部 `info` 只返回卡片介绍、参数 schema、配置 schema 和输入输出口定义。
`joints` 通过输出口持续发布数据，不提供业务 action；joint_move 按需建立 Dashboard 命令连接。

## 固定直角结构

反馈型号 113 自动识别 CR3A。保留上游原始 `resource/cr3_robot.urdf`，`model` 返回
从该文件及同版本 STL 派生显示骨架：两根长杆分别沿杆身中心面走线，再接回关节原点，
每根杆有两个固定直角拐点，共四处。J2 杆身中心面为局部 Z=0.1288m，J3 为 Z=0.0123m，
依据官方 CAD 中段截面确定，来源记录见 `resource/README.md`。
转角通过固定坐标系表示，末端补偿旋转使原有坐标变换不变。附加节点全部为 `fixed`，不接受控制角度。
原有六个可动轴的名称、轴向、限位及所有物理 link 的变换保持不变。

同时将原始复合 RPY 分解为单轴固定旋转，兼容平台现有 XYZ Euler 渲染顺序，
并保持标准 URDF 的 Rz*Ry*Rx 变换。测试对零位、当前样例姿态和 30 组随机角度逐一验证
全部物理 link 的位置与朝向，避免只画出直角却改变机械臂姿态。

模型只用于结构显示，不是运动规划模型或完整 CAD 外壳。未知/不匹配型号不会套用 CR3A。
先确认反馈有效再打开 `joints`，以便平台首次加载模型时取得正确型号。

## 网络与配置

Jetson 有线口接控制柜 LAN2：Jetson `192.168.200.10/24`，控制柜默认 `192.168.200.1`，
无需网关，Wi-Fi 可继续用于 SSH/互联网。

`DOBOT_CONTROLLER_IP`、`DOBOT_FEEDBACK_PORT`、`DOBOT_MODEL_NAME` 覆盖初始配置。
`config.yaml` 默认 IP 空，不配置时不联系控制器；部署片段默认 IP 为 LAN2。
`CONFIG_PATH` 可覆盖配置文件。`joints` 的共享配置提供 IP 和端口，变更时先关闭
旧反馈连接、清空缓存，再连接新地址。IP 声明为敏感字段，不打包到共享 Solution。
支持平台保存配置及启动前自动下发的 `config` 动作；返回 `state="configured"`。
随后 `start` 无需重复传入 IP/端口；重复应用相同配置不会断开现有连接，非法配置保留原连接。

反馈端口 `30004/30005/30006` 默认周期为 8/200/1000ms；接收和过期超时至少覆盖三个周期。
默认 `30004/30005` 为 1 秒接收、2 秒过期，`30006` 均为 3 秒；支持配置更大值。
SDK 清理异常不会中断重连，诊断字段 `cleanup_error` 单独记录。

## 构建与检查

```bash
bash build.sh --mirror none dobot/cra
```

SDK 及构建工具固定版本，源码校验 SHA-256；构建机需访问 GitHub codeload。
两阶段构建仅在 builder 中安装构建工具，运行镜像仍包含 SDK、numpy、requests、PyYAML 等必要第三方依赖。
这些运行依赖会增加镜像体积；本次未测量实际 ARM64 镜像体积。
部署使用 `/deploy/service.yml`、host 网络、平台 DDS loopback 配置，domain 42，
MCP 端口 `15746`，容器名 `embodied-dobot-cra`；无需主机 SDK 或 privileged。

```bash
python3 -m venv .venv
.venv/bin/pip install -r dobot/cra/build-requirements.txt
.venv/bin/pip install --no-build-isolation -r dobot/cra/requirements.txt pytest
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest tests/test_dobot_cra_sensors.py tests/test_dobot_skeleton_structure.py tests/test_dobot_joint_move.py tests/test_dobot_completion.py -q
.venv/bin/python scripts/check_service_yml.py
```

## PR 验收

1. 可见卡片仅有 `joints` 和 `joint_move`；`model` 保留为后台资源。
2. 骨架使用六轴实际角度，默认 30 Hz；JSON 的六个角度字段分别占一行。
3. systemcontrol 不注册，joint_move 不提供独立使能或上电动作；运动前未使能时自动使能。
4. 反馈失效时骨架为空、角度为 null；`info` 只返回卡片介绍。
5. 本地模拟验证不操作真机，模型与实际机械臂的显示效果由现场验收。

## 关节运动

joint_move 与 joints 共用底层反馈服务和连接配置，均可独立启动。
move_j/resume 先 ClearError，再检查使能，未使能时发送一次 EnableRobot 并等待反馈确认。
然后发送 EnableSafeSkin(0)；move_j 再发送 ClearError 和 MovJ，resume 发送 Continue。
失败不继续也不重试。
不自动重新 enable 安全皮肤。模式 11 等保护状态不会提前阻止准备命令；
仅在准备完成后确认模式 5（move_j）或模式 10（resume）、已使能且无保护触发，才发送最终运动命令。

四个运动 action 均返回 action_id，通过 ACP 完成回调报告真实终态；命令应答不等于运动完成。
暂停/停止也要等待新反馈确认状态与实际速度，缺失反馈不宣称停稳。
部署增加 Agent Core CA 证书只读挂载与主机名映射，保持 TLS 校验；详细要求见 JOINT_MOVE.md。
