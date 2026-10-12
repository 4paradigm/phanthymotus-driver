# LimX TRON2 只读参数驱动

为 Phanthy Motus 提供三张持续更新的传感器卡片。适用于与 TRON2 原有主控制器通过以太网连接的 Jetson / ARM64 主机。驱动版本：`0.4.3`；MCP 仅监听 `127.0.0.1:15791`。

| 卡片 | 类型 | 内容 | 数据流 |
| --- | --- | --- | --- |
| `joint_state` | sensor | 关节位置 `q`（rad）、`positions_deg`（度）、厂家返回的 `dq` / `tau` | `/limx_tron2/joint_state` |
| `robot_status` | sensor | 工作模式、固件版本、IMU / 电机 / 相机等状态 | `/limx_tron2/robot_status` |
| `battery_state` | sensor | 电量 `soc`、电压 `voltage`、电流 `current`、功率 `power` | `/limx_tron2/battery_state` |

三张传感器卡片使用 `data/json` 格式，通过 Motus 本机 ROS2 总线发布。默认每 0.5 秒查询一次关节状态并发布缓存，实际更新速率受机器人响应时间影响。状态和电池来自机器人主动发送的 `notify_robot_info`，其更新频率由机器人决定。

## 读取边界与字段说明

机器人链路上唯一的应用请求是 `request_get_joint_state`。本驱动没有复位、关节目标设置、模式切换、拖动示教、负载辨识或其他运动接口，也不需要开发模式或本机运动授权。

- 名称由机器人返回。`names` 缺失或为空时，`labels` 使用 `joint_index_0` 等索引，`labels_are_indices: true`；不推测关节名称及身体部位对应关系。
- `q` 按厂家关节接口的弧度值保留；`positions_deg` 仅进行弧度到度的换算。`dq` / `tau` 原样保留，缺失时为 `[]`。
- 电池数据流使用平铺的四个字段，与 Unitree 电池卡片的展示方式一致：`soc` 为 0–100 的电量百分比；`voltage` / `current` 为有限数值形式的厂家 `bms_voltage` / `bms_current` 原始值，不作缩放、不标成 V / A。电流仅在 `bms_current_valid` 严格为 `true` 时显示，保留厂家符号但不推测充放电方向。`power` 暂为 `null`：电压、电流单位未确认，不能将原始值直接相乘并宣称瓦数。温度、循环次数和 `raw` 已移除。
- 厂家的 `UNKNOW` 等状态字符串原样保留；缺失字段为 `null`。
- 关节和设备状态数据包包含 `connected`、`available`、`sample_status`、`received_at` 和 `reception_age_seconds`；电池卡片的 `info.sample` 同样保留这些诊断字段。接收年龄使用主机单调时钟计算；`received_at` 是主机接收时的 Unix 秒数，只表示整条通知的接收年龄，不能证明其中电流字段的采样年龄。
- 厂家关节时间戳的时钟及单位尚未验证，因此 `robot_timestamp` 仅作为原始值保留，`measurement_time_verified` 为 `false`。接收年龄不能证明测量数据的真实年龄。
- 默认超过 5 秒没有收到对应数据则 `available: false`、`sample_status: stale`、`data: null`。断连时立即清空缓存，再尝试重连；不会继续显示旧关节值。电池数据流断连或过期时仍只含四个字段，全部为 `null`；具体原因通过该卡片 `info` 或 `/health` 查看。

电池卡片“查看数据流”示例（电压、电流单位待厂家确认，示例不是实时测量）：

```json
{"soc": 85, "voltage": 52650, "current": -70, "power": null}
```

0.4.3 的电池发布格式有变更：消费端应使用 `soc` 代替 `data.battery_percent`，直接读取 `voltage` / `current`。MCP `info` 仍返回诊断封装，其中 `sample.data` 使用这四项新字段。关节和设备状态的发布结构、三张卡片名称和 topic 均保留。

卡片的 `start` / `stop` 是单实例常开传感器的生命周期兼容调用，不启动或停止机器人，也不停止后台采样。`info` 返回该传感器的当前状态和缓存，不额外发送机器人请求。驱动仅暴露表中的三张 sensor，不提供缓存聚合 resource 或助手卡片。

兼容 Agent Core 自动注入的 `instance_id` 和 `_trace_id`：两者仅接受最长 128 字符、无控制字符的字符串，不参与机器人请求、不回显到采样数据。Schema 与运行时均接受这两项元数据；其他未声明参数仍被拒绝。0.4.2 保留追踪元数据兼容，并移除之前的缓存聚合资源工具；三张传感器仍能通过 info 查看当前缓存。

## 配置

`config.yaml` 控制轮询间隔、超时、接收过期阈值及三张卡片是否启用。至少启用一张。部署环境可覆盖：

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

仅使用 WebSocket 读取参数，不挂载 `/dev`、Docker socket、姿态文件或可写控制目录，也不需要 privileged 或 GPU runtime。容器使用只读根文件系统和临时 `/tmp`；ROS2 日志显式写入 `/tmp/ros-log`；`restart: always`。

如果同机部署过早期复位原型，先备份其文件，再停止旧服务以释放 15791，并在 Motus 设置中移除旧复位服务注册及画布卡片。本驱动注册名称为 **LimX TRON2 Read-only Sensors**；不会自动删除旧容器、旧注册或已保存的姿态文件。

## 网页查看与验收

在 Jetson 宿主运行只读检查，或从仓库运行同一脚本：

```bash
sudo docker exec embodied-limx-tron2 python3 /work/check.py
# 或：python3 limx/tron2/check.py
```

脚本等待就绪、检查 Core 注册和 MCP 卡片类型、验证生命周期以及关节回复接收时间持续更新。它不检查或调用运动接口。该脚本按默认三张卡片全部启用验收。

刷新 Motus 控制台，找到 **LimX TRON2 Read-only Sensors**，将三张 sensor 卡片拖入画布并开启，再点击“查看数据流”。从旧版升级时，应从画布移除旧缓存读取、文字输入和决策卡片；助手 Skill 是单独的本机配置，不属于此 Driver。数据存在时应看到 `available: true`；断开机器人链路后应看到 `data: null` 和不可用状态，恢复连接后收到新通知 / 关节回复才恢复数值。各传感器的 `info` 可查看该卡片当前缓存。

`GET http://127.0.0.1:15791/health` 返回卡片状态：全部已启用卡片有最近接收数据时为 HTTP 200，否则为 503。`registered_mcp_id` 单独表明 Core 注册是否完成；机器人可读与 Core 已注册是两项独立条件。

## 开发测试与验证范围

```bash
python3 -m pip install -r limx/tron2/requirements.txt
python3 -m unittest discover -s limx/tron2/tests -v
python3 -m pytest tests/test_dockerfile_copies.py -q
```

测试包含真实本机 HTTP / WebSocket 服务、请求关联及超时、序列号约束、重连、断连清空、过期数据、卡片生命周期、ROS2 JSON 发布的模拟验证和镜像入口布局。测试不驱动实体机器人。

此前现场已验证厂家 WebSocket 能返回机器人信息与 16 项关节值；本版本的 ROS2 / Compose 正式镜像仍需在 Jetson 上按上述步骤验收。复位动作应作为后续 action 类功能单独实现，不包含在本 PR 中。
