# LimX TRON2 参数与双臂控制驱动

为 Phanthy Motus 提供关节骨架、电池两张传感器卡片和一张双臂控制卡片，另有骨架渲染必需的model资源。适用于通过以太网连接DACH_TRON2A双臂/头部主控制器的Jetson / ARM64主机。驱动版本：`0.5.2`；MCP仅监听`127.0.0.1:15791`。

| 卡片 | 类型 | 内容 | 数据流 |
| --- | --- | --- | --- |
| `joint_state` | sensor | 左右臂及头部16关节URDF实时骨架；info保留原始rad/deg、dq/tau | `/limx_tron2/joint_state`（sensor/skeleton） |
| `battery_state` | sensor | 电量、电压原始值/电流绝对值、按mV/mA假设的估算功率、单个温度原始值（五路最大有效值）、循环次数 | `/limx_tron2/battery_state` |
| `arm_control` | actuator | move自定义目标、pose预设、reset复位、只读预览/列表/到位检查 | 无 |
| `model` | resource | 厂家DACH_TRON2A骨架URDF，供平台自动加载 | 无 |

传感器通过Motus本机ROS2总线发布JSON：joint_state声明sensor/skeleton，battery_state保留data/json。默认每0.5秒查询关节并发布缓存，实际频率受厂家响应影响。电池来自notify_robot_info通知。robot_status不再作为卡片或ROS2 topic公开，但通知中的连接、固件、模式等仍供内部运动校验；值不变不表示这些信息没有用途。

## 读取边界与字段说明

传感器后台仅发送request_get_joint_state。arm_control仅在显式move/pose/reset调用时发送一次request_movej；没有模式切换、拖动示教或自动演示循环。

- 名称由机器人返回。`names` 缺失或为空时，`labels` 使用 `joint_index_0` 等索引，`labels_are_indices: true`；不推测关节名称及身体部位对应关系。
- `q` 按厂家关节接口的弧度值保留；`positions_deg` 仅进行弧度到度的换算。`dq` / `tau` 原样保留，缺失时为 `[]`。
- 0.5.2的骨架流每项增加q_deg（由同一q换算，不舍入）、group（左臂/右臂/头部）及label（简短关节名称）。完整16项角度都输出；更新后的Motus骨架组件可展开角度表，同时显示度与rad三位小数，完整精度留在数据和数值提示中。旧平台会忽略这些可选字段，不会自动出现表格，必须同时更新Agent Core前端。表格是实际反馈，不是arm_control的目标角；停更/断连应显示不可用，不把未知值补成0。
- 骨架使用Motus既有joints协议和model资源。名称缺失时按官方SDK索引顺序映射到URDF：左臂0–6、右臂7–13、头部俯仰14/偏航15；URDF文本中头部关节顺序相反，不使用文件遍历顺序。若厂家提供名称则按名称匹配，不支持的关节数或名称返回空骨架和不可用诊断。没有可用IMU姿态时省略可选imu_quat；不能发送空数组，因为平台会将其解构成四个undefined值、破坏根节点旋转。模型来源和Apache-2.0归属见resource/SOURCE.md；只保留厂家运动学，不包含网格资产。
- 电池数据schema版本为3。MCP/health缓存字段为 `soc/voltage/current/power/temperature/cycle`；`soc`为百分比，`voltage/current/temperature`为厂家原始单位。`temperature`只显示五路有效原始值中的最大值，不标成℃、不判断正常/过热；五路原值仍在`temperature_channels_raw`中按`bms_temp1`至`bms_temp5`保留，无效通道保留null。所有通道无效时单个温度也是null。`cycle`只接受厂家`bms_charging_cycle`的非负整数。
- 电流`current`按用户要求只展示绝对值，例如厂家`-2620`显示`2620`；0保留，无效值仍为null。缩放保持厂家原始单位，这个字段不再保留正负方向，不用于判断充放电状态。
- 按用户授权启用估算：假设电压mV、电流mA，以`(voltage/1000)*(current/1000)`计算功率大小，保留两位小数。`power_status: estimated_assuming_mV_mA`明确表示单位假设，未经厂家确认。电压或有效电流缺失、计算溢出时仍为null；有效0则显示0。电流不保留方向，因此功率也是大小，不能表示充放电方向或外接电源下整机耗电。
- `validity.power`只表示估算输入/计算有效，不代表单位已确认。温度单值有效性为布尔值，五路通道有效性在`validity.temperature_channels_raw`。有效0保留，缺失/无效/非有限值为null。
- 厂家的 `UNKNOW` 等状态字符串原样保留；缺失字段为 `null`。
- 电池主要显示六项：`soc/voltage/current/estimated_power_w/temperature_raw/cycle`，用字段名直接标明估算功率和原始温度。schema、单位、有效性、五路温度和接收时间集中在`status`诊断中；`info.sample.data`仍提供数值型`power/temperature`。`received_at/reception_age_seconds`只表示整条通知的接收时间，不能证明每个电流/温度字段的采样时间。`current_updated_at_raw`是未验证时钟的厂家标记，不与主机时间相减。读数随通知持续更新，不保证厂家独立测温刷新频率。
- 厂家关节时间戳的时钟及单位尚未验证，因此 `robot_timestamp` 仅作为原始值保留，`measurement_time_verified` 为 `false`。接收年龄不能证明测量数据的真实年龄。
- 默认超过5秒没有对应数据则`available: false`、`sample_status: stale`、缓存`data: null`。电池发布包清空全部六项读数，诊断的五路温度也全部为null、有效性false；不保留旧温度或估算功率。断连时立即清空。

电池卡片“查看数据流”示例（电压、电流单位待厂家确认，示例不是实时测量）：

```json
{
  "soc": 85,
  "voltage": 52650,
  "current": 70,
  "estimated_power_w": 3.69,
  "temperature_raw": 104,
  "cycle": 22,
  "status": {
    "schema_version": 3,
    "power_status": "estimated_assuming_mV_mA",
    "temperature_policy": "maximum_valid_raw_channel",
    "temperature_channels_raw": [74, 64, 74, 104, 94],
    "units": {"soc": "%", "voltage": "vendor_raw_unverified", "current": "vendor_raw_unverified", "power": "W", "temperature": "vendor_raw_unverified", "cycle": "count"},
    "validity": {"soc": true, "voltage": true, "current": true, "power": true, "temperature": true, "temperature_channels_raw": [true, true, true, true, true], "cycle": true},
    "current_updated_at_raw": 1791780203689,
    "connected": true, "available": true, "sample_status": "recently_received",
    "received_at": 1791780232.899, "reception_age_seconds": 0.398,
    "measurement_time_verified": false
  }
}
```

电池沿用0.4.8的schema v3，功率为estimated_power_w，温度为单值temperature_raw；诊断位于status。MCP/health仍提供power和temperature，五路原数组为temperature_channels_raw。本次0.5.0不修改电池计算、显示字段或本机复位目标。

传感器start/stop兼容单实例生命周期，不启动/停止机器人，也不停止后台采样。info读取缓存，不额外请求机器人。公开两张sensor、一张arm_control actuator及model resource；不提供独立robot_status/reset_pose卡片或助手卡片。

兼容Agent Core自动注入的instance_id和_trace_id：最长128字符、无控制字符，不参与机器人请求或回显。两张传感器通过info查看当前原始缓存。

## 配置

`config.yaml` 控制轮询间隔、超时、接收过期阈值及各卡片是否启用。至少启用一张。部署环境可覆盖：

| 环境变量 | 默认值 | 用途 |
| --- | --- | --- |
| `TRON2_ROBOT_URL` | `ws://10.192.1.2:5000` | 厂家 WebSocket（仅接受端口 5000） |
| `TRON2_SERIAL` | 空 | 可指定序列号锁定目标；为空则从首次合法通知学习，并在本次进程的重连期间固定该序列号 |
| `TRON2_AGENT_CORE_URL` | `https://phanthy-motus:15678` | Compose 中的 Core 地址，传入容器的 `AGENT_CORE_URL` |
| `TRON2_CA_CERT` | `/opt/phanthy-motus/data/certs/cert.pem` | 宿主 Core 信任证书文件，挂载至 `/certs/core.pem` |

生产部署建议明确设置 `TRON2_SERIAL`。序列号和证书只在部署主机配置，不提交到仓库。自动学习模式在进程重新启动后重新学习，不提供跨重启身份锁定。

## 构建与部署

从仓库根目录使用标准流程构建：

```bash
bash build.sh --mirror none limx/tron2
```

`driver.yaml` 的 `build_context_extras` 将仓库 `common/` 加入构建上下文。Dockerfile 在实际 `/work` 布局下检查入口和 ROS2 模块导入；这项构建检查不连接机器人或创建 DDS participant。

PR 构建完成后，使用机器人评论给出的新镜像 / Try it 链接部署到同机 Motus。`deploy/service.yml` 由 Core 提取并合入统一 Compose，镜像占位符 `__IMAGE__` 由平台替换。

部署前确认：

1. 主机已能通过机器人网口访问厂家 WebSocket；保留机器人原有主控制器。
2. `/opt/phanthy-motus/dds-local.xml` 是 Core 生成的实际文件，不是目录。驱动挂载该文件，使用 domain 42、FastDDS 和 loopback 数据总线；文件缺失则拒绝创建 participant。
3. Core 信任证书是实际文件。HTTPS 保持证书及主机名验证；`TRON2_AGENT_CORE_URL` 的主机名必须与证书 SAN 匹配。需要时使用证书包含的主机名，并相应调整 Compose 的 `extra_hosts`；不要关闭 TLS 验证。
4. MCP 端口 15791 未被其他本地原型占用。

通过 WebSocket 采样和显式 MoveJ 调用，不挂载 `/dev`、Docker socket、姿态文件或可写控制目录，也不需要 privileged 或 GPU runtime。容器使用只读根文件系统和临时 `/tmp`；ROS2 日志显式写入 `/tmp/ros-log`；`restart: always`。

如果同机部署过早期复位原型，先备份其文件，再停止旧服务以释放 15791，并在 Motus 设置中移除旧复位服务注册及画布卡片。本驱动注册名称为 **LimX TRON2 Sensors and Arm Control**；不会自动删除旧容器、旧注册或已保存的姿态文件。

## 网页查看与验收

在 Jetson 宿主运行只读检查，或从仓库运行同一脚本：

```bash
sudo docker exec embodied-limx-tron2 python3 /work/check.py
# 或：python3 limx/tron2/check.py
```

脚本等待就绪，检查Core注册、两张sensor/arm_control/model类型、骨架格式和URDF名称，验证生命周期及关节接收时间更新。它不调用move/pose/reset，也不验证真实运动。

升级到0.5.0后刷新Motus，移除画布上的旧robot_status与reset_pose实例，重新添加joint_state以更新缓存的渲染格式。使用joint_state、battery_state、arm_control三张业务卡片；model由骨架组件自动读取，不需要手动执行或连接。开启后joint_state应显示厂家双臂/头部骨架。断连/过期发布空joints列表和不可用诊断，原始info缓存data为null。恢复并收到新关节回复后更新骨架。已保存画布不会随tools/list改变而自动删除旧实例；部署工具不得在未确认时覆写整张画布。

`GET http://127.0.0.1:15791/health` 返回卡片状态：全部已启用传感器有最近接收数据时为 HTTP 200，否则为 503。`registered_mcp_id` 单独表明 Core 注册是否完成；机器人可读与 Core 已注册是两项独立条件。

## 开发测试与验证范围

```bash
python3 -m pip install -r limx/tron2/requirements.txt
python3 -m unittest discover -s limx/tron2/tests -v
python3 -m pytest tests/test_dockerfile_copies.py -q
```

测试包含真实本机 HTTP / WebSocket 服务、请求关联及超时、序列号约束、重连、断连清空、过期数据、卡片生命周期、ROS2 JSON 发布的模拟验证和镜像入口布局。测试不驱动实体机器人。

此前现场验证了16项关节值和运动ACK；0.5.0的骨架网页效果和新增pose/reset操作仍需部署后单独验收。摄像头工作继续暂停。

## arm_control 动作

0.5.1修正骨架数据兼容：缺少真实IMU四元数时不再发送imu_quat空数组。升级后重新加载网页/骨架组件，清除旧数据留下的无效根旋转；后续关节反馈仍按原名称应用到URDF。发布回归验证两帧肘角变化，检查脚本同时显示左右肘角，便于区分真实反馈与网页显示问题。电池和机械臂动作保持0.5.0实现。

参考现有宇树arm卡片的命名姿态列表与x-action-params参数分组；实际执行仍使用TRON2 MoveJ，不复制宇树动作ID或硬件命令。

| ACTION | 参数 | 行为 |
| --- | --- | --- |
| move | joint、time（默认8秒） | 14项绝对目标，左7右7，rad，不包含头部 |
| pose | pose_name、time（省略则自动计算） | 执行以本机保存home为基准的命名姿态 |
| reset | 无 | 回到本机保存的自然垂下目标 |
| preview | pose_name（默认arms_down） | 读取目标/角度差/预计时长，不发送运动 |
| list | 无 | 列出可选姿态，不发送运动 |
| verify_at_home | 无 | 单个最近关节样本与home误差≤1度检查 |

pose_name选项：arms_down、left_elbow_20、right_elbow_20、both_elbows_20。后3项分别将home的左肘索引3、右肘索引10或两者减去20度（转成rad），其余关节保持home目标。它们是绝对目标，不会重复点击累积增加角度。自定义move继续接受数组或≤4096字符的JSON数组文本，不接受NaN/Infinity、bool、数字字符串、头部角度或超限值。

reset及pose使用现有`/config/tron2/home.json`（宿主`/opt/phanthy-motus/data/tron2`只读目录挂载）。沿用tron2-arms-reset-pose-v1、14项rad、标准顺序、序列号与固件绑定。升级不重新采集或覆盖home，也不从历史home文件回拷。文件缺失/不匹配时info报告未配置、实际运动拒绝，但平台仍可启动。

pose/reset时长至少max(8,ceil(最大关节变化度数/5))秒，超过60秒拒绝；pose显式填写time时不得短于此值。所有目标仍校验[厂家SDK限位](https://www.limxdynamics.com/zh/documents/847884267345285120)。头部、夹爪不发命令，不进行路径碰撞规划。

start返回ready，stop返回idle，均不发送运动，也不是机器人急停。只有显式move/pose/reset发送一次request_movej并等待GUID/序列号匹配的ACK；accepted不表示到达，arrival_verified为false。超时、断连、未知结果不自动重试；等待运动时长后可单独检查实际关节反馈。

check.py只调用生命周期/信息和model资源，不执行运动。源码和模拟接口通过不代表网页渲染、实际运动到位或PR审批已经通过。正式镜像由上游PR Review Agent的/request_bot_review发布；部署后仍需现场验收。
