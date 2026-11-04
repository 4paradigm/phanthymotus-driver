# Dobot CRA joints and safe skin

基于官方 V4 Python SDK 的 Jetson Docker 驱动。本版只保留关节骨架，并新增安全皮肤
状态和控制；不修改平台前端，不加载 STL 外壳。SDK 版本见 [SDK.md](SDK.md)。

## 卡片

| 名称 | 类型 | 内容 |
|---|---|---|
| `joints` | sensor | 六轴实际角度驱动的 CR3A 结构骨架；共享连接配置入口 |
| `safe_skin` | sensor | 小臂、J4/J5/J6 接近暂停标志及状态有效性、最近控制请求 |
| `safe_skin_control` | actuator | 显式请求 TCP 控制、总开关、分区灵敏度 |
| `model` | resource | 平台渲染器使用的固定直角骨架 URDF，不发布 ROS topic |

已移除 `connection`、`joint_states`、`robot_info`、`software_info`、`arm_all_state`、
`controller_state` 卡片。升级后请删除画布中的旧卡片，在 `joints` 齿轮处重新配置连接。
不会自动改动已有画布或把旧 sensor 重命名为新控制功能。

两个 sensor 共用一个反馈连接，默认 10Hz 发布 `std_msgs/String` JSON：

- `/dobot_cra/state/joints`，`sensor/skeleton`；六关节 `q/dq` 为 rad、rad/s，
  `degree` 保留度数，关节名为 `joint1..joint6`。
- `/dobot_cra/state/safe_skin`，`data/json`。

断线或过期时 `connected=false, stale=true`；骨架不再输出旧角度，`joints=[]`；
皮肤触发状态为 `null`，不会把未知当作“没有触发”。所有卡片可查看连接新鲜度。

## 固定直角结构

反馈型号 113 自动识别 CR3A。保留上游原始 `resource/cr3_robot.urdf`，`model` 返回
从该文件派生的显示骨架：J4 前的 `(-0.23, 0, 0.1283)` 偏置拆为两段垂直刚性连杆，
转角通过 +90° / -90° 固定坐标系表示。附加节点全部为 `fixed`，不接受控制角度。
原有六个可动轴的名称、轴向、限位及所有物理 link 的变换保持不变。

同时将原始复合 RPY 分解为单轴固定旋转，兼容平台现有 XYZ Euler 渲染顺序，
并保持标准 URDF 的 Rz*Ry*Rx 变换。测试对零位、当前样例姿态和 30 组随机角度逐一验证
全部物理 link 的位置与朝向，避免只画出直角却改变机械臂姿态。

模型只用于结构显示，不是运动规划模型或完整 CAD 外壳。未知/不匹配型号不会套用 CR3A。
先确认反馈有效再打开 `joints`，以便平台首次加载模型时取得正确型号。

## 安全皮肤状态

`safe_skin.regions` 包含 `arm/j4/j5/j6` 四个区域，每个区域输出 `part`、`raw_flag`、
`approach_paused`；`any_approach_paused` 汇总区域触发状态。`robot_status` 提供模式、
使能、运行、报警、碰撞、安全字节及当前队列指令号，便于区分普通碰撞与皮肤接近暂停。

反馈不提供皮肤安装检测、总开关或灵敏度读回，因此：

- `hardware_presence="unknown"`；全零标志不证明安全皮肤已安装或已开启。
- `enabled=null`、`sensitivity` 各区为 `null`，`configuration_readback_available=false`。
- `control.last_request` 仅记录本驱动最近提交的设置，不能当作当前真实配置。
- 本版不虚构距离读数或 TCP 复位指令；`reset_supported=false`。

## 安全皮肤控制

启动驱动、添加卡片、点击 start/info 均不打开 Dashboard、不接管控制、不写入设置。
只有显式控制请求才懒连接 `29999`。所有请求通过一条序列化连接调用官方 SDK；
接收按分号完整组帧，断线/超时不自动重放写入。停止控制卡或容器会关闭该连接，
但不取消控制器已经接受的队列指令，也不自动退出设备的 TCP 模式。

| action | 参数 | 官方 SDK |
|---|---|---|
| `info` | 无 | 只读本地状态 |
| `request_control` | `confirm_change=true` | `RequestControl()` |
| `set_enabled` | `enabled=true/false`, `confirm_change=true` | `EnableSafeSkin(1/0)` |
| `set_sensitivity` | `part=3/4/5/6`, `level=0/1/2/3`, `confirm_change=true` | `SetSafeSkin(part,level)` |

部位 3=小臂，4/5/6=J4/J5/J6；等级 0=关闭，1=低，2=中，3=高。
每次写操作都要明确确认，平台按 actuator 危险操作处理。驱动要求新鲜反馈、机器人停止，
没有报警、碰撞、皮肤触发、拖拽、暂停或抱闸松开。请求 TCP 控制另要求未使能/下电，
且关闭手自动模式切换功能。条件不满足时返回原因，不代为使能、清错、关闭其他保护或恢复运动。
设置过程中再次检查反馈，避免连接耗时后仍按旧状态发送。

开关和灵敏度是**队列指令**：返回 0 只说明接受请求。返回 ResultID 时状态为
`submitted`；旧固件不返回 ID 时为 `accepted_unverified`。无配置读回接口，所以即使
接受也始终标记 `configuration_verified=false`。可在 sensor 查看当前队列指令号，
并在厂家软件中核对实际设置；下使能时接受的队列可能等待后续使能，不会由驱动自动执行使能。
明确错误码返回 `rejected`，通信不确定性返回 `failed_or_outcome_unknown`，均不自动重试。

文档没有单独的 TCP 皮肤复位函数；关闭/重开皮肤不作为复位动作。TCP 设置只在对应
控制模式中生效，退出后按厂家规则恢复原配置。SDK 错误 -1 还可能表示没有安装安全皮肤。

## 网络与配置

Jetson 有线口接控制柜 LAN2：Jetson `192.168.200.10/24`，控制柜默认 `192.168.200.1`，
无需网关，Wi-Fi 可继续用于 SSH/互联网。

`DOBOT_CONTROLLER_IP`、`DOBOT_FEEDBACK_PORT`、`DOBOT_MODEL_NAME` 覆盖初始配置。
`config.yaml` 默认 IP 空，不配置时不联系控制器；部署片段默认 IP 为 LAN2。
`CONFIG_PATH` 可覆盖配置文件。`joints` 的共享配置提供 IP 和端口，变更时先关闭
Dashboard 和旧反馈连接、清空缓存，再连接新地址。IP 声明为敏感字段，不打包到共享 Solution。

反馈端口 `30004/30005/30006` 默认周期为 8/200/1000ms；接收和过期超时至少覆盖三个周期。
默认 `30004/30005` 为 1 秒接收、2 秒过期，`30006` 均为 3 秒；支持配置更大值。
SDK 清理异常不会中断重连，诊断字段 `cleanup_error` 单独记录。

## 构建与检查

```bash
bash build.sh --mirror none dobot/cra
```

SDK 及构建工具固定版本，源码校验 SHA-256；构建机需访问 GitHub codeload。
部署使用 `/deploy/service.yml`、host 网络、平台 DDS loopback 配置，domain 42，
MCP 端口 `15746`，容器名 `embodied-dobot-cra`；无需主机 SDK 或 privileged。

```bash
python3 -m venv .venv
.venv/bin/pip install -r dobot/cra/build-requirements.txt
.venv/bin/pip install --no-build-isolation -r dobot/cra/requirements.txt pytest
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest tests/test_dobot_cra_sensors.py tests/test_dobot_safe_skin.py tests/test_dobot_skeleton_structure.py -q
.venv/bin/python scripts/check_service_yml.py
```

## PR 验收

1. 升级镜像后确认只有两个 sensor、一张控制卡和 model 资源，清理旧画布卡片。
2. 配置并启动 `joints`，确认六轴与实时角度一致，J4 前连杆可见直角而非斜杆；固定拐点不可控。
3. 核对 `safe_skin` 各分区标志；断网时应变为未知，重连后恢复。
4. 在受控现场停止机器人后，显式执行所需皮肤设置，对照厂家软件确认队列执行及实际配置。
   本地模拟测试不替代这一项硬件验证。
5. 本次开发验证不得自动在真机上关闭安全皮肤、使能或恢复运动。
