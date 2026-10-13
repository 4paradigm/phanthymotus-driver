# LimX TRON2 参数与双臂控制驱动

为 Phanthy Motus 提供三张持续更新的传感器卡片和两张显式运动执行卡片。适用于与 TRON2 原有主控制器通过以太网连接的 Jetson / ARM64 主机。驱动版本：`0.4.8`；MCP 仅监听 `127.0.0.1:15791`。

| 卡片 | 类型 | 内容 | 数据流 |
| --- | --- | --- | --- |
| `joint_state` | sensor | 关节位置 `q`（rad）、`positions_deg`（度）、厂家返回的 `dq` / `tau` | `/limx_tron2/joint_state` |
| `robot_status` | sensor | 工作模式、固件版本、IMU / 电机 / 相机等状态 | `/limx_tron2/robot_status` |
| `battery_state` | sensor | 电量、电压原始值/电流绝对值、按mV/mA假设的估算功率、单个温度原始值（五路最大有效值）、循环次数 | `/limx_tron2/battery_state` |
| `arm_control` | actuator | 双臂14项角度目标（rad），显式 MoveJ | 无 |
| `reset_pose` | actuator | 从本机保存姿态读取固定双臂目标，预览/执行/到位检查 | 无 |

三张传感器卡片使用 `data/json` 格式，通过 Motus 本机 ROS2 总线发布。默认每 0.5 秒查询一次关节状态并发布缓存，实际更新速率受机器人响应时间影响。状态和电池来自机器人主动发送的 `notify_robot_info`，其更新频率由机器人决定。

## 读取边界与字段说明

传感器后台仅发送 `request_get_joint_state`。新增 `arm_control` actuator 通过显式 move 调用发送一次 `request_movej`；驱动不再是全只读服务。reset_pose使用本机固定目标显式调用MoveJ；没有模式切换或拖动示教接口。

- 名称由机器人返回。`names` 缺失或为空时，`labels` 使用 `joint_index_0` 等索引，`labels_are_indices: true`；不推测关节名称及身体部位对应关系。
- `q` 按厂家关节接口的弧度值保留；`positions_deg` 仅进行弧度到度的换算。`dq` / `tau` 原样保留，缺失时为 `[]`。
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

0.4.8的schema v3将主显示精简为六项，功率消费者改读`estimated_power_w`，温度消费者改读单值`temperature_raw`；诊断字段移入`status`。MCP/health中的`temperature`由数组改成单值，原数组改名`temperature_channels_raw`，`power`恢复有标记的估算。关节/设备状态结构、卡片名称和topic不变；升级后刷新网页。此显示更新不修改机械臂控制或本机复位目标。

卡片的 `start` / `stop` 是单实例常开传感器的生命周期兼容调用，不启动或停止机器人，也不停止后台采样。`info` 返回该传感器的当前状态和缓存，不额外发送机器人请求。驱动提供三张 sensor 及 arm_control/reset_pose actuator，不提供缓存聚合 resource 或助手卡片。

兼容 Agent Core 自动注入的 `instance_id` 和 `_trace_id`：两者仅接受最长 128 字符、无控制字符的字符串，不参与机器人请求、不回显到采样数据。Schema 与运行时均接受这两项元数据；其他未声明参数仍被拒绝。0.4.2 保留追踪元数据兼容，并移除之前的缓存聚合资源工具；三张传感器仍能通过 info 查看当前缓存。

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

脚本等待就绪、检查 Core 注册和 MCP 卡片类型、验证生命周期以及关节回复接收时间持续更新。它不检查或调用运动接口。该脚本按默认三张卡片全部启用验收。

刷新 Motus 控制台，找到 **LimX TRON2 Sensors and Arm Control**，将三张 sensor 卡片拖入画布并开启，再点击“查看数据流”。从旧版升级时，应从画布移除旧缓存读取、文字输入和决策卡片；助手 Skill 是单独的本机配置，不属于此 Driver。数据存在时应看到 `available: true`；断开机器人链路后应看到 `data: null` 和不可用状态，恢复连接后收到新通知 / 关节回复才恢复数值。各传感器的 `info` 可查看该卡片当前缓存。

`GET http://127.0.0.1:15791/health` 返回卡片状态：全部已启用传感器有最近接收数据时为 HTTP 200，否则为 503。`registered_mcp_id` 单独表明 Core 注册是否完成；机器人可读与 Core 已注册是两项独立条件。

## 开发测试与验证范围

```bash
python3 -m pip install -r limx/tron2/requirements.txt
python3 -m unittest discover -s limx/tron2/tests -v
python3 -m pytest tests/test_dockerfile_copies.py -q
```

测试包含真实本机 HTTP / WebSocket 服务、请求关联及超时、序列号约束、重连、断连清空、过期数据、卡片生命周期、ROS2 JSON 发布的模拟验证和镜像入口布局。测试不驱动实体机器人。

此前现场已验证厂家 WebSocket 能返回机器人信息与 16 项关节值；本版本的 ROS2 / Compose 正式镜像仍需在 Jetson 上按上述步骤验收。摄像头不包含在当前版本；arm_control和reset_pose真实运动仍需现场单独验收。

## arm_control 与构建修复（0.4.4）

0.4.6兼容Motus画布将JOINT作为文本提交的行为：接受真正的数组或不超过4096字符的JSON数组文本，解析后仍要求14个有限数值及厂家绝对关节限位。网页JOINT填写带方括号的完整14项数组；不接受逗号列表、数字字符串、NaN、Infinity或头部的末尾两项。此兼容不触发自动运动或重试。

`arm_control` 的 `info` 只报告连接状态；`move` 必须传入双臂14项 `joint`（左臂7项、右臂7项，单位rad）和0.1–60秒的 `time`（默认2秒）。读取关节状态虽然有16项，MoveJ不接受末尾两项头部目标。依据[厂家SDK指南3.6.1](https://www.limxdynamics.com/zh/documents/847884267345285120)校验14项及公布的绝对关节限位。没有15度变化限制，也没有本机一次授权步骤。

运动仅由显式MCP调用触发。运行时先拒绝非法值、超限、断连及过期状态，然后在原有WebSocket发送一次并按GUID/序列号等待response_movej。`status: accepted`只表示厂家回复success；`arrival_verified: false`，不保证到达或碰撞路径。超时/断连/发送错误不自动重试，返回结果未知或失败。

本次修复缺失if代码块的语法错误、motion参数被拒绝、错误后台任务成功提示、actuator被放进传感器采样/topic列表的问题；电池schema补齐实际接受的action并保留无参数调用兼容。check.py只检查三个传感器及卡片类型，不执行机械臂move。模拟接口测试不连接真机。

自定义GitHub Actions改为源码、离线合同和部署配置验证；不再在未配置私有仓库凭据时尝试登录发布。正式PR镜像沿用上游PR Review Agent，通过/request_bot_review构建。摄像头工作已暂停，未包含在此版本。

## 平台启动兼容与 reset_pose（0.4.5）

平台开启智能控制会对每张卡片调用start。arm_control和reset_pose均接受info/start/stop；start/stop只返回生命周期状态，不调用机器人，也不是急停。只有arm_control.move或reset_pose.execute发送运动。

reset_pose使用`/config/tron2/home.json`，宿主目录`/opt/phanthy-motus/data/tron2`只读挂载到容器。文件沿用此前保存的tron2-arms-reset-pose-v1格式，要求双臂14项弧度、标准关节顺序、机器人序列号和固件绑定；公共代码不带示例/默认运动目标。原保存姿态可以原样复制，不能用当前随机姿态覆盖原自然下垂目标。文件缺失或不匹配时，平台仍能启动，但execute会明确拒绝发送。

- info：报告是否配置目标；不发运动。
- preview：返回目标、最大关节变化和预计时间；不发运动。
- execute：读取固定目标并发送一次MoveJ，等待厂家受理回复；不接收调用方传入的目标角。
- verify_at_home：用当前最近收到的关节样本检查双臂最大误差≤1度，属于单样本检查，不是连续稳定验收。

复位时间为max(8,ceil(最大角度差/5))秒；超过现有60秒MoveJ时间上限时拒绝，不放宽厂家绝对限位。头部和夹爪不发命令，没有15度变化限制或本机一次授权。execute回复accepted不表示已经到达；等待运动时间后可调用verify_at_home。不进行碰撞路径规划。

首次部署前，将此前确认的姿态复制到宿主目录：

```bash
sudo install -d -m 0755 /opt/phanthy-motus/data/tron2
sudo install -m 0600 ~/motus-local-drivers/tron2-reset/home.json /opt/phanthy-motus/data/tron2/home.json
```

check.py覆盖两个执行卡片的info/start/stop，明确验证motion_sent=false；不会调用move或execute。本次仅新增必要的本机姿态只读挂载及纯Python文件，不添加依赖或扩大硬件权限。
