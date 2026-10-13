# LimX TRON2 参数与双臂控制驱动

为 Phanthy Motus 提供关节骨架、电池及头部/左右腕RGB摄像头五张传感器卡片和一张双臂控制卡片，另有骨架渲染必需的model资源。适用于通过以太网连接DACH_TRON2A双臂/头部主控制器的Jetson / ARM64主机。驱动版本：`0.5.7`；MCP仅监听`127.0.0.1:15791`。

| 卡片 | 类型 | 内容 | 数据流 |
| --- | --- | --- | --- |
| `joint_state` | sensor | 左右臂及头部16关节URDF实时骨架，附随腕运动的左右夹爪示意；info保留原始rad/deg、dq/tau | `/limx_tron2/joint_state`（sensor/skeleton） |
| `battery_state` | sensor | 电量、电压原始值/电流绝对值、按mV/mA假设的估算功率、单个温度原始值（五路最大有效值）、循环次数 | `/limx_tron2/battery_state` |
| `camera` | sensor | 按需读取头部RGB，原始JPEG限10fps发布，不录像 | `/limx_tron2/camera`（image/jpeg） |
| `camera_left` | sensor | 左腕RGB校正图像，独立启停，限10fps | `/limx_tron2/camera_left`（image/jpeg） |
| `camera_right` | sensor | 右腕RGB校正图像，独立启停，限10fps | `/limx_tron2/camera_right`（image/jpeg） |
| `arm_control` | actuator | move自定义目标、pose预设、reset复位并自动校验；ACP完成通知 | 无 |
| `model` | resource | 厂家DACH_TRON2A骨架URDF，供平台自动加载 | 无 |

传感器通过Motus本机ROS2总线发布JSON：joint_state声明sensor/skeleton，battery_state使用现有data/json。默认每0.5秒查询关节并发布缓存，实际频率受厂家响应影响。电池来自notify_robot_info通知。robot_status不再作为卡片或ROS2 topic公开，但通知中的连接、固件、模式等仍供内部运动校验；值不变不表示这些信息没有用途。

## 读取边界与字段说明

传感器后台仅发送request_get_joint_state。arm_control仅在显式move/pose/reset调用时发送一次request_movej；没有模式切换、拖动示教或自动演示循环。

- 名称由机器人返回。`names` 缺失或为空时，`labels` 使用 `joint_index_0` 等索引，`labels_are_indices: true`；不推测关节名称及身体部位对应关系。
- `q` 按厂家关节接口的弧度值保留；`positions_deg` 仅进行弧度到度的换算。`dq` / `tau` 原样保留，缺失时为 `[]`。
- 左右夹爪通过安装点上的固定双指轮廓显示，跟随对应手腕运动。开合是静态示意，不表示当前实际夹持状态；没有夹爪开合反馈时不伪造角度或开合动画。附加display_hand_gripper_*节点只供渲染，全部fixed，不增加16关节反馈数量，不改变原始运动学、控制限位或夹爪命令。显示间距/指长为示意尺寸，不是标定的实物外形、TCP或碰撞模型。
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

电池沿用0.4.8的schema v3，功率为estimated_power_w，温度为单值temperature_raw；诊断位于status。MCP/health仍提供power和temperature，五路原数组为temperature_channels_raw。本次0.5.3不修改电池计算、显示字段或本机复位目标。

关节/电池传感器start/stop兼容单实例生命周期，不启动/停止机器人，也不停止后台采样。info读取缓存，不额外请求机器人。camera的start/stop实际开启/释放只读视频连接。公开六张sensor、一张arm_control actuator及model resource；不提供独立robot_status/reset_pose卡片或助手卡片。

兼容Agent Core自动注入的instance_id和_trace_id：最长128字符、无控制字符，不参与机器人请求或回显。三张传感器通过info查看当前原始缓存。

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

脚本等待就绪，检查Core注册、三张JSON/骨架sensor、可选camera/arm_control及model类型、骨架格式和URDF名称，验证生命周期及关节接收时间更新。它不调用move/pose/reset，也不验证真实运动。

升级后刷新Motus，移除画布上的旧robot_status、reset_pose、joint_angles实例，重新添加joint_state以更新缓存的渲染格式。使用joint_state、battery_state、camera、camera_left、camera_right、arm_control六张业务卡片；model由骨架组件自动读取，不需要手动执行或连接。开启后从监控查看joint_state的厂家双臂/头部骨架。右侧骨架详情面板的旧平台模型请求问题未在本驱动修复，查看骨架优先使用监控。断连/过期发布空joints列表和不可用诊断，原始info缓存data为null。恢复并收到新关节回复后更新骨架。已保存画布不会随tools/list改变而自动删除旧实例；部署工具不得在未确认时覆写整张画布。

`GET http://127.0.0.1:15791/health` 返回卡片状态：启用关节/电池时，机器人连接且这些已启用传感器全部有最近接收数据为HTTP 200。可选camera的idle/断流不会阻塞它们或机械臂任务。只启用相机时，所有已启用相机都有最近成功发布的JPEG才就绪，否则为503。配置必须启用至少一张传感器，禁止仅开启arm_control却将空样本集合当成就绪。`registered_mcp_id` 单独表明 Core 注册是否完成；机器人可读与 Core 已注册是两项独立条件。

## 开发测试与验证范围

```bash
python3 -m pip install -r limx/tron2/requirements.txt
python3 -m unittest discover -s limx/tron2/tests -v
python3 -m pytest tests/test_dockerfile_copies.py -q
```

测试包含真实本机 HTTP / WebSocket 服务、请求关联及超时、序列号约束、重连、断连清空、过期数据、卡片生命周期、ROS2 JSON 发布的模拟验证和镜像入口布局。测试不驱动实体机器人。

此前现场验证了16项关节值和运动ACK；0.5.0的骨架网页效果和新增pose/reset操作仍需部署后单独验收。0.5.5新增按需头部RGB摄像头，平台画面仍需部署后验收。

## arm_control 动作

0.5.1修正骨架数据兼容：缺少真实IMU四元数时不再发送imu_quat空数组。升级后重新加载网页/骨架组件，清除旧数据留下的无效根旋转；后续关节反馈仍按原名称应用到URDF。发布回归验证两帧肘角变化，检查脚本同时显示左右肘角，便于区分真实反馈与网页显示问题。电池和机械臂动作保持0.5.0实现。

参考现有宇树arm卡片的命名姿态列表与x-action-params参数分组；实际执行仍使用TRON2 MoveJ，不复制宇树动作ID或硬件命令。

| ACTION | 参数 | 行为 |
| --- | --- | --- |
| move | joint、time（默认8秒） | 14项绝对目标，左7右7，rad，不包含头部 |
| pose | pose_name、time（省略则自动计算） | 执行以本机保存home为基准的命名姿态 |
| reset | 无 | 回到自然垂下目标，并自动检查连续反馈是否正确复位 |

pose_name选项：arms_down、left_elbow_20、right_elbow_20、both_elbows_20。后3项分别将home的左肘索引3、右肘索引10或两者减去20度（转成rad），其余关节保持home目标。它们是绝对目标，不会重复点击累积增加角度。自定义move继续接受数组或≤4096字符的JSON数组文本，不接受NaN/Infinity、bool、数字字符串、头部角度或超限值。

reset及pose使用现有`/config/tron2/home.json`（宿主`/opt/phanthy-motus/data/tron2`只读目录挂载）。沿用tron2-arms-reset-pose-v1、14项rad、标准顺序、序列号与固件绑定。升级不重新采集或覆盖home，也不从历史home文件回拷。文件缺失/不匹配时info报告未配置、实际运动拒绝，但平台仍可启动。

pose/reset时长至少max(8,ceil(最大关节变化度数/5))秒，超过60秒拒绝；pose显式填写time时不得短于此值。所有目标仍校验[厂家SDK限位](https://www.limxdynamics.com/zh/documents/847884267345285120)。头部、夹爪不发命令，不进行路径碰撞规划。

ACTION仅公开move、pose、reset。info/start/stop由平台内部生命周期和诊断使用，不显示为可选动作；start返回ready，stop返回idle，均不发送运动，也不是机器人急停。preview/list/verify_at_home已删除，旧调用会拒绝。只有显式move/pose/reset发送一次request_movej并等待GUID/序列号匹配的ACK；初始accepted响应不表示到达，arrival_verified为false，并提供唯一action_id。三个动作声明x-completion（120秒上限）和共享资源arm_l/arm_r；动作完成由后台反馈验证和ACP回调跟踪，不以ACK代替完成。

check.py只调用生命周期/信息和model资源，不执行运动。源码和模拟接口通过不代表网页渲染、实际运动到位或PR审批已经通过。正式镜像由上游PR Review Agent的/request_bot_review发布；部署后仍需现场验收。

## 动作完成协议（ACP）

MCP用于卡片发现与tools/call请求，ACP（Action Completion Protocol）用于异步动作完成跟踪。
这是仓库README_dev.md规定的配合方式：保留/mcp、x-completion及唯一action_id，在实际验证结束后POST /api/acp/complete，不能将MCP路由简单改名为ACP。
平台日志中的MCP_CALL finish是Agent结束当前推理轮次的工具调用，不代表机械臂跳过ACP。三种动作的受理/终态结果均标明completion_protocol=acp。

reset自动复位检查与动作完成跟踪为同一个过程，无需另调verify_at_home。reset的初始结果为reset_verified=false、at_home=null；运动结束后的连续新反馈通过后，ACP completed结果携带reset_verified=true、at_home=true、最大角度误差、容差、样本数和本次home摘要。
未通过验证时通过ACP报告error、reset_verified=false、at_home=null，并保留错误原因；不能把失联或缺少反馈时的未知位置声称为已复位。复位目标固定为发送时已验证的home，不在动作途中重新加载或覆盖文件。

- move、pose、reset返回唯一action_id；awaiting_ack/moving期间同一个驱动实例保留双臂所有权，第二个动作拒绝，不依赖Core的资源标签作为唯一锁。HTTP调用方断开不会取消正在发送的物理命令，也不会释放所有权。
- 从ACK结束后保守等待完整time/duration_seconds，再检查至少3个不同的新关节回复，跨度至少0.5秒；14项误差均≤1度、相对首个样本变化≤0.2度，有dq时额外要求每项原始值绝对值≤0.05（未将dq单位认证为rad/s）。位置稳定性直接由q判断。厂家名称重排时先映射至官方索引；不能用同一缓存样本或命令目标伪造到位。
- 到位后POST /api/acp/complete：action_id、status=completed、result.arrival_verified=true及误差/样本数，tool=arm_control。初始响应仍是受理；通过arm_control.info或/health的arm_motion可查看最终状态。该验证基于连续新收到的关节回复，厂家测量时间戳仍未验证，不代表路径碰撞规划或独立运动安全认证。
- 超出预计时长再加10秒仍未满足条件、反馈过期、断连或连接更换，报告status=error、arrival_verified=false。发送/ACK结果不明时仍返回跟踪ID并检查反馈，不重发MoveJ；厂家明确拒绝或发送前拒绝不生成已受理动作。
- ACP使用与注册相同的已验证HTTPS证书上下文，禁止重定向、不读取ACCESS_TOKEN、不关闭TLS验证；同一终态payload最多尝试3次，每次最多3秒，退避0.5/1秒。最坏发送/ACK、60秒动作、验证和回调仍小于声明的120秒上限。
- 只有验证成功且回调送达才释放本地所有权。反馈验证失败或回调无法送达会保留blocked_requires_operator_check，阻止新的运动。失败通知和Core超时不会物理停止机械臂；现场确认停止、处理原因后再重启驱动。内部start/stop不会清除该状态，stop仍不是急停。
- 进程正常关闭时终止验证任务并报告error，说明运动可能继续；不发送停机、复位或第二条运动命令。进程强制退出无法保证回调，Core保留120秒超时兜底。

Dockerfile按dpkg实际安装状态检查python3-pip和ca-certificates，仅缺少时执行apt更新/安装；基础镜像已提供的包不重复安装。aiohttp/PyYAML仍由requirements.txt管理，是既有HTTP/WebSocket和配置解析依赖；完成机制只增加Python标准库源码，没有新增pip包、设备权限或资源资产。


## 头部与左右腕RGB摄像头（0.5.6）

camera保留0.5.5的头部卡片名称与topic；新增camera_left、camera_right，使用三路独立中继和DDS topic。默认均可发现但进程启动不打开视频连接。刷新Motus，把需要的相机卡片拖到画布，开启智能控制，从监控或查看数据流观看，无需与arm_control连线或填写URL。Core显式start连接对应厂家源，stop只释放该相机连接，info只读，不影响其他相机或关节/电池后台采样。不自动执行任何机械臂、头部、夹爪运动。

| 卡片 | 厂家源topic | Motus输出topic | 只读现场尺寸 |
| --- | --- | --- | --- |
| camera（头部） | /camera/top/color/image_raw/compressed | /limx_tron2/camera | 640×480 |
| camera_left（左腕） | /camera/left/color/image_rect_raw/compressed | /limx_tron2/camera_left | 848×480 |
| camera_right（右腕） | /camera/right/color/image_rect_raw/compressed | /limx_tron2/camera_right | 848×480 |

固定读取同一TRON2_ROBOT_URL主机HTTP59001：`/stream?topic=<表中厂家源>&quality=80&rotate=0`。左右腕使用现场ROS2发布并有连续有效帧的image_rect_raw，不使用当前无有效帧的image_resized或套用头部image_raw名称。厂家的camera_service状态不是每路实时帧证据；话题存在/有发布者也不能代替实际帧验收。保留厂家left/right标识，现场可分别在对应镜头前放纸核对左右画面，不发运动命令。

只接入三路RGB，不发布depth/标定/metadata，不支持PTZ、录制或调用者指定URL/topic。原始JPEG不解码转码，复用既有aiohttp与ROS sensor_msgs/CompressedImage，声明image/jpeg供平台现有CameraRenderer显示。每路header.frame_id分别为tron2_head_camera、tron2_left_wrist_camera、tron2_right_wrist_camera；stamp是中继接收/发布时间，不是已验证的厂家测量时钟。原有Core和loopback domain42总线配置不变。

每路限10fps（实际受厂家源影响）、4MiB/帧、宽高各≤4096；三路各自depth=1、best-effort/volatile，独立任务/连接/锁/代际标记。校验JPEG封装、头部尺寸、SOF/SOS分段，不宣称完整熵解码。拒绝非200、重定向、非MJPEG、压缩HTTP编码及非法帧。整帧限3秒，断流清空available/尺寸/接收时间并按1秒间隔重连；首次start限8秒且要求成功发布一帧，并发start共用该路连接，stop能中断该路等待。一路断流不会停止另外两路或机械臂任务。health.cameras独立列出各路状态，旧health.camera保留头部；普通关节/电池就绪不受可选相机idle/断流影响。只启用相机时则要求所有已启用源可用。

旧平台图像组件可能保留最后一帧；静止图像不能证明连接正常，需查该路info.available与接收年龄或重新打开面板。普通check.py只验证发现/协议/info，不改变既有视频启停。显式视频验收开启全部已启用源：

```bash
sudo docker exec embodied-limx-tron2 /bin/bash -c 'source /opt/ros/humble/setup.bash && exec python3 /work/check.py --camera'
```

检查每路计数持续增长，并用实际本机DDS订阅逐topic收到至少两帧合法JPEG（每topic只缓存两帧，整体限5秒），打印各路名称/尺寸，保留连接供平台查看；不保存图像或发送运动。完成后停止对应卡片/画布释放连接。Mac诊断SSH隧道仅用于开发读取，不是Jetson平台正常视频链路的一部分。

测试使用合成MJPEG，覆盖启停/并发/中断等待、断流清空/重连、无效响应/畸形/截断/超限/停滞帧、RPC元数据与禁止运动；左右腕分开输出/独立停止、失败不影响其他源/JSON就绪，以及三路CompressedImage原始二进制、独立frame_id/单帧队列。只读隧道现场诊断确认三路同时连续视频，左停止后头部/右继续发布；实际图像/现场标识不进入仓库或公开PR。源/中继测试不能代替新镜像部署后的DDS/Core/平台验收。
