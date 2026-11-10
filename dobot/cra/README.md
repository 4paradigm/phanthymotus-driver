# Dobot CRA joints, joint_move and teach

基于官方 V4 Python SDK 的 Jetson Docker 驱动。本版提供并列的 `joints`、`joint_move` 和 `teach`
三张可见卡片，URDF 由后台 `model` 资源提供。SDK 版本及源码校验值固定在 [requirements.txt](requirements.txt)。

## 卡片

| 名称 | 类型 | 内容 |
|---|---|---|
| `joint_move` | actuator | 绝对/相对关节运动、停止、暂停及继续 |
| `joints` | sensor | 六轴实际角度驱动的 CR3A 结构骨架、逐关节转角 JSON，以及共享连接配置 |
| `teach` | actuator | 后台拖拽录制、轨迹管理、到起点运动、原生回放及暂停/恢复 |

`model` 为渲染器内部读取的 resource，不是可见卡片。
本版不注册 `safe_skin` 或 `safe_skin_control`，不发布安全皮肤 topic。
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
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest tests/test_dobot_*.py -q
.venv/bin/python scripts/check_service_yml.py
```

## PR 验收

1. 可见卡片为 `joints`、`joint_move` 和 `teach`；`model` 保留为后台资源。
2. 骨架使用六轴实际角度，默认 30 Hz；JSON 的六个角度字段分别占一行。
3. systemcontrol 不注册，joint_move 不提供独立使能或上电动作；运动前未使能时自动使能。
4. 反馈失效时骨架为空、角度为 null；`info` 只返回卡片介绍。
5. 本地模拟验证不操作真机，模型与实际机械臂的显示效果由现场验收。

## 关节运动

joint_move 与 joints 共用底层反馈服务和连接配置，均可独立启动。
move_j/rel_joint_mov_j 先 ClearError，再检查使能，未使能时发送一次 EnableRobot 并等待反馈确认。
然后按 detect_collision 配置安全皮肤：默认 false 关闭；true 开启并将四个区域设为 low。
新运动再发送 ClearError 和 MovJ/RelJointMovJ。
resume 只检查已使能、模式 10 和保护状态，然后复用命令连接发送 Continue；
不清错、不自动使能、不设置安全皮肤，保持暂停前配置。未使能或保护未解除时拒绝继续。
失败不继续也不重试。
皮肤设置由 detect_collision 显式选择。模式 11 等保护状态不会提前阻止准备命令；
新运动准备完成后必须确认模式 5、已使能且无保护触发，才发送位移命令。

五个运动 action 均返回 action_id，通过 ACP 完成回调报告真实终态；命令应答不等于运动完成。
暂停/停止也要等待新反馈确认状态与实际速度，缺失反馈不宣称停稳。
部署增加 Agent Core CA 证书只读挂载与主机名映射，保持 TLS 校验。

rel_joint_mov_j 新增相对关节偏移运动，单位为度，0 表示该轴不变；其余设置复用 move_j。
准备前及下发前校验实际角度加偏移后的最终目标，完成监测使用最终目标角度。

绝对运动的空白关节保持下发前实际角度，相对运动的空白偏移按 0；至少填写一个关节。

| action | 输入与行为 |
|---|---|
| `move_j` | `joint1_deg` 至 `joint6_deg` 为绝对目标角度，留空保持下发前实际角度 |
| `rel_joint_mov_j` | `joint1_offset_deg` 至 `joint6_offset_deg` 为相对偏移，留空按 0 |
| `pause` | 暂停控制器队列或工程，不适用于点动 |
| `resume` | 仅需手动 confirm_move=true，检查后直接 Continue，保持暂停前安全皮肤配置 |
| `stop_motion` | 停止控制器队列或工程，包括点动 |

两种角度运动至少填写一个关节，明确填入绝对角度 0 表示回到零度。
每次操作要求手动输入 `confirm_move=true`。速度和加速度为 1–100%，默认 10%，固定 cp=0。
目标行程为 J1/J2/J4/J5/J6 ±360°、J3 ±155°；实际还受控制器自定义软限位约束。
安全皮肤开关与灵敏度设置均等待新反馈确认对应队列完成，允许本次设置产生的短暂模式 7，
失败或超时不继续发送运动命令。暂停、继续和停止作用于整个控制器队列，不限于本卡片。

完成通知使用 `AGENT_CORE_URL` 和 `AGENT_CORE_CA_CERT`，默认地址为 https://phanthy-motus:15678，
CA 路径为 /opt/phanthy-motus/data/certs/cert.pem。部署支持 DOBOT_AGENT_CORE_URL、
DOBOT_AGENT_CORE_HOSTNAME、DOBOT_AGENT_CORE_ADDRESS、DOBOT_CA_DIR、DOBOT_AGENT_CORE_CA_CERT 覆盖。
TLS 保持校验；停止在反馈或证书配置异常时仍可下发，但不能宣称已停稳或已成功通知平台。

暂停完成以模式 10 及连续两包新反馈确认停稳为依据，不强制要求 pause_cmd_flag=1；
实测该字段可以为 0。resume 同样以模式 10 判断暂停状态，不依赖该标志。

## 示教录制与回放

`teach` 与 `joint_move` 共用 Dashboard 连接、反馈服务和机械臂占用管理。
录制、运动以及暂停的回放都保留机械臂占用；另一张卡不能插入新运动。
任一卡片的暂停、继续、停止请求转交给当前占用者，并以实际调用卡片的名称发送 ACP 结果。
`info` 仅返回卡片说明，没有业务状态查询 action 或新增 sensor 输出。

| action | 行为 |
|---|---|
| `start_record` | 指定 `name`，ClearError、按需 EnableRobot，不发送 StartDrag；之后等待操作者按机械臂拖拽键首次进入拖拽 |
| `stop_record` | 结束整次录制；仍在拖拽才 StopDrag；以新反馈确认模式 5 停稳后保存，并尝试原生文件导出 |
| `list_records` | 返回轨迹编号、名称、完整性、结束原因、时长、点数、原生文件状态 |
| `delete_record` | 删除本地录制，保留控制器副本；禁止删除正在使用的录制 |
| `go_to_start` | 校验原生文件和坐标系，使用 MovJ 到起点并确认到位 |
| `play` | 确认已在起点，复用新运动准备，执行 StartPath，等待实际结束 |
| `pause` | 暂停当前队列；模式 10 可对应 pause_cmd_flag=0 |
| `resume` | 直接 Continue，保留原队列和安全皮肤设置，不清错、不 Enable、不重新设置皮肤 |
| `stop_motion` | 终止回放；录制时作为中断处理，尝试退出拖拽，保留标为不完整的采集数据 |

除列表查询外，业务 action 都需要手动填写 `confirm_move=true`；该字段不设默认值或枚举。

### 一次录制、多次拖拽

流程：start_record → 等待首次拖拽 → 拖拽（模式 6）⇄ 非拖拽停顿（模式 5），可反复多次 → stop_record → 确认退出拖拽并保存。
按拖拽键退出拖拽**不会**结束录制；正常结束只有 stop_record 或达到上限。stop_motion、驱动停止与异常均视为中断。

- **时间轴**：首次进入拖拽前不产生轨迹点。之后模式 6 与模式 5 的反馈都按实际关节角和控制器时间戳采集，
  停顿的真实时长保留在同一条时间轴上，不复制旧点、不重置时间、不拼接分段。
- **上限**：等待首次拖拽与录制各受 `max_recording_s` 约束；录制时长从首次拖拽起算，包含停顿，
  再次进入拖拽不会重置；最多 10000 点。达到上限按 stop_record 相同流程退出并保存，`end_reason=limit_reached`。
- **录制中允许的状态**：模式 6；或模式 5 且已使能、无运行/点动/队列/暂停标志。模式 5 下仍在减速滑行时继续采集，
  不视为结束。模式 7/8/9/10/11 等其他模式即使标志干净也以不完整结束。保护检查（报警、碰撞、皮肤、下使能、
  抱闸松开、型号）先于模式判断。
- **stop_record 确认**：先停止采集，仅当仍在拖拽时发送 StopDrag；只使用请求之后（发送了 StopDrag 则为其之后）
  收到的新反馈，要求连续两包满足：模式 5、已使能、drag_status=false、无运行/点动/队列/暂停、
  六轴 |qd| ≤ 0.1°/s、无报警/碰撞/皮肤/抱闸松开。超时、断线或保护状态时返回失败，数据标为不完整，不宣称成功。
- **异常**：报警、碰撞、皮肤、下使能、抱闸松开、非法模式、运行/点动、断线/过期/溢出/时间戳缺口、坐标系变化、
  stop_motion、驱动停止/重启，均标为不完整并保留原始数据、轨迹点和明确原因；不自动清错、使能或 Continue。
  可尝试退出拖拽，但未经新反馈确认不报告成功。中断标记不可撤销，后续的 stop_record 不会清除；
  退出确认期间收到的 stop_motion 或驱动停止，会在最终保存前同步核对，结果仍为不完整。
- **占用**：等待、拖拽和停顿期间 teach 一直占用机械臂，joint_move 不能插入运动。退出无法确认时继续占用，
  直到后续 stop_motion/stop_record 确认停稳；成功后才释放。旧任务不会释放新任务的占用。
- **ACP**：start_record 完成仅表示“已准备，等待首次拖拽”；进入/退出拖拽不发送完成通知；
  stop_record 在退出确认和保存后才完成。只有采样完整、至少 2 点、退出已确认且保存成功才为 `recorded`。
  `end_reason` 为 `stop_record`、`limit_reached` 或 `interrupted`。`native_state`（含导出失败 `export_failed`
  及 `native_reason`）独立于本地录制状态，导出失败不改变 `recorded`。
- start_record 前机械臂不能已处于拖拽，否则拒绝且不发送任何命令。从未进入拖拽就结束会标为不完整。

采集从共享 30004 接收线程进入独立有界缓冲，不经过网页、ROS 发布或 joints 的 30 Hz 定时器。
原始数据逐包落盘（含模式）；按控制器时间戳线性重采样为 50 ms 一点，最多 10000 点、约 500 秒。
断线、缓冲溢出、超过 50 ms 的采样缺口、时间倒退、保护变化和用户/工具坐标系变化
均将录制标为不完整，不自动拼接。重启遗留的录制也标为不完整。坐标范围按 CR3A 限制校验。
小于 50 ms 的采样间隙才参与重采样；原始文件保留实际时间，不把重复旧包当新数据。

录制不改安全皮肤或本体碰撞等级。go_to_start/play 的 detect_collision 沿用 joint_move 的定义，
默认 false，true 开启安全皮肤并将四个区域设为 low。play 按录制时间比例回放，
speed_multiplier 为 0.25–2，默认 1；固定 sample=50、isConst=0、freq=1（不增加额外滤波）。
回放超时按时长/倍率增加余量，不直接套用 300 秒。到起点使用 velocity/acceleration，默认各 10%。
StartPath 的 ACK 可以不带队列号；此时必须观察到本次运行，再通过连续新反馈的空闲、停稳和末点角度确认完成。
通信中断不重复发送 StartPath/MovJ/Continue；错误后尝试一次 Stop，无法确认停止则保留错误状态。

### 原生 CSV 与部署边界

**本地原始录制和原生回放文件是不同格式。当前没有这台控制器导出的原生 CSV 样本，
因此没有随驱动内置任何“已兼容”CSV 布局。默认可以录制，回放明确返回 `format_required`，
不会把内部 JSON 当原生轨迹，也不会在此状态下发送准备或位移命令。**

要启用原生导出，需要从当前控制器样本确认字段后提供数据格式配置，并把控制器轨迹共享目录
预先通过 CIFS/SMB 或 NFS 挂载到驱动可访问的目录。该桥接只做文件写入与 SHA-256 回读校验，不开启 SSH、自动挂载、
尝试未知 HTTP 写接口或安装 SMB 客户端。GetStartPose 会进一步验证控制器读出的起点；
这不替代现场对整条 CSV 的兼容性验证。当前测试中的 CSV 明确标记为 SIMULATOR_ONLY，不能部署为真机格式。

配置项（config.yaml 的 teach 或对应环境变量）：

| 配置/变量 | 作用 |
|---|---|
| data_dir / DOBOT_TEACH_DATA_DIR | 本地持久化录制目录；部署默认 /work/data/dobot-teach |
| controller_directory / DOBOT_TEACH_CONTROLLER_DIR | **容器内**的控制器轨迹目录；原生覆盖配置固定为 /work/controller-trajectories |
| native_csv_profile / DOBOT_TEACH_CSV_PROFILE | **容器内**的 CSV 布局文件路径；原生覆盖配置固定为 /work/config/dobot-teach/profile.json |

布局 JSON 使用 version=1、非空 format_id、columns、可选 header（CSV 行数组）及 delimiter。
columns 支持 joint1_deg..joint6_deg、time_ms、time_s、user、tool，或者 {"literal": 值} 常量列；
六轴必须各出现一次，单位为度。header 为原样输出的常量行。若控制器样本需要其他结构，应先扩展并验证
编码器，不能用常量伪造动态位姿或 IO。不自动记录/回放 DO，也不改变工具负载设置。

默认 service.yml 仅启用本地录制，两个原生路径环境变量为空，因此不要求主机上存在 profile 或控制器共享目录。
Agent Core 只自动提取 service.yml，不会自动加载可选覆盖文件。启用原生回放须按以下步骤，
将镜像自带的 deploy/native-teach.compose.yml 与平台统一 Compose 文件一起加载。

1. 在 Linux 部署主机上，先通过现场已有的共享配置，将控制器的 process/trajectory 目录挂载到
   例如 /mnt/dobot-trajectory。该目录必须实际位于 CIFS/SMB 或 NFS 文件系统中，且允许读写。
   普通 mkdir 创建的目录、失效后落到本地磁盘的目录、只读挂载不能代替控制器共享。
2. 准备从当前控制器样本确认的 profile JSON 文件。使用下面两个**宿主机路径变量**：

| 宿主机变量 | 内容 | 容器映射及权限 |
|---|---|---|
| DOBOT_TEACH_PROFILE_HOST_FILE | 已存在的 profile JSON 绝对路径 | /work/config/dobot-teach/profile.json，只读 |
| DOBOT_TEACH_CONTROLLER_HOST_DIR | 已挂载共享中轨迹目录的绝对路径 | /work/controller-trajectories，读写 |

3. 默认录制容器由平台创建后，在有部署目录写权限及 Docker 权限的账号下执行：

```bash
cd /opt/phanthy-motus
export DOBOT_TEACH_PROFILE_HOST_FILE=/opt/phanthy-motus/data/dobot-teach/native-profile.json
export DOBOT_TEACH_CONTROLLER_HOST_DIR=/mnt/dobot-trajectory

docker cp embodied-dobot-cra:/deploy/native-teach.compose.yml ./dobot-native-teach.compose.yml
docker cp embodied-dobot-cra:/work/dobot/cra/native_mounts.py ./check-dobot-native-mounts.py
python3 ./check-dobot-native-mounts.py

docker compose -f docker-compose.yml -f dobot-native-teach.compose.yml config --quiet
docker compose -f docker-compose.yml -f dobot-native-teach.compose.yml up -d --no-deps dobot-cra
```

宿主机预检查仅依赖 Python 标准库，无需安装机械臂 SDK。它会读取 profile JSON，核对 Linux 挂载信息，
并在共享目录创建、回读及删除一个临时探测文件。它不向机器人命令端口发送指令。
覆盖配置使用长格式 bind mount 和 create_host_path=false，缺失的宿主机路径不会被自动创建为空目录。
运行时导出也重新检查 profile 可读、共享确为网络文件系统及可写，失败时不会向该目录写入轨迹。

将这两个 HOST 变量保存到 /opt/phanthy-motus/.env 后，后续维护仍应使用同一组 -f 文件。
平台重新部署驱动时只使用基础 service.yml，原生回放会恢复关闭状态；须重新提取新版覆盖文件并按上述命令启用。
只设置 DOBOT_TEACH_CSV_PROFILE/DOBOT_TEACH_CONTROLLER_DIR 为宿主机路径不能启用回放。

CSV 文件名由轨迹编号生成，不覆盖不同内容的已有控制器文件。
控制器 IP、坐标系、原始文件校验值、原生文件起点不匹配时拒绝播放。

本次新增代码仅使用标准库，requirements.txt 与 SDK pin 不变。Dockerfile 仅增加 Python 文件打包，
并非新增运行时第三方依赖；原有 SDK/numpy/requests/PyYAML 的镜像占用仍然存在。
本地测试覆盖真实回环 TCP 的官方 SDK、1440 字节反馈、MCP HTTP、ACP、录制/传输/回放模拟，
不验证控制器固件接受 CSV，不进行 Jetson 或真机测试。
