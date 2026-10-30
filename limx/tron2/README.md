# LimX TRON2 传感器、双臂、头部与夹爪控制驱动

为 Phanthy Motus 提供关节骨架及头部/左右腕RGB摄像头四张传感器卡片、双臂/头部控制卡片、预设pose动作卡片和一张可选择左右的夹爪控制卡片，新增一张基于 Perception Stack VOP 的人物头部视觉跟随卡片，另有骨架渲染必需的model资源。适用于通过以太网连接DACH_TRON2A双臂/头部主控制器的Jetson / ARM64主机。驱动版本：`0.15.0`；MCP仅监听`127.0.0.1:15791`。

| 卡片 | 类型 | 内容 | 数据流 |
| --- | --- | --- | --- |
| `joint_state` | sensor | 左右臂及头部16关节URDF实时骨架，附随腕运动的左右夹爪示意；info保留原始rad/deg、dq/tau | `/limx_tron2/joint_state`（sensor/skeleton） |
| `pose` | actuator | salute/wave/welcome预设轨迹；挥手含右夹爪一次开合；整套ACP完成 | 无；详情显示共享双臂任务 |
| `gripper` | actuator | 选择左/右夹爪并设置目标开口，0mm闭合 | 无；详情显示所选侧反馈 |
| `camera` | sensor | 按需读取头部RGB，原始JPEG限10fps发布，不录像 | `/limx_tron2/camera`（image/jpeg） |
| `camera_left` | sensor | 左腕RGB校正图像，独立启停，限10fps | `/limx_tron2/camera_left`（image/jpeg） |
| `camera_right` | sensor | 右腕RGB校正图像，独立启停，限10fps | `/limx_tron2/camera_right`（image/jpeg） |
| `arm_control` | actuator | move自定义目标、reset复位并自动校验；ACP完成通知 | 无 |
| `move_head` | actuator | 俯仰-60～45°（上正）、偏航-88～88°（自身左正）绝对目标；ACP确认到位 | 无 |
| `head_visual_follow` | actuator | 平滑人物跟随；不跟随模式含暂停与头部复位；ACP会话跟踪 | 左侧输入 `/limx_tron2/camera/objects`（data/json） |
| `model` | resource | 厂家DACH_TRON2A骨架URDF，供平台自动加载 | 无 |

夹爪卡片说明见 [夹爪卡片使用说明](GRIPPER.md)。只填写side（left/right）、opening_mm（0至85mm）、speed、force；没有action/close/recover选项。驱动按底层值=毫米数×94/85名义换算，0mm闭合。非法输入在占用任务和发送前拒绝，两侧独立管理；合法指令仍保留未到位或断连保护。

头部卡片使用步骤、坐标系和完成判定见 [头部控制说明](HEAD.md)。pitch_deg和yaw_deg均须明确填写，单位为度；驱动仅在显式move时发送两轴MoveH。头部与双臂/夹爪互斥，不改变原arm_control的14项目标或自然垂下home。

传感器通过Motus本机ROS2总线发布JSON：joint_state声明sensor/skeleton。默认每0.5秒查询关节并发布缓存，实际频率受厂家响应影响。robot_status不再作为卡片或ROS2 topic公开，但通知中的连接、固件、模式等仍供内部运动校验；值不变不表示这些信息没有用途。

battery_state卡片、配置、topic和电池计算/显示代码已从本版本移除；当前双臂版本不收录电池功能，后续有明确双腿需求再设计。

## 人物头部视觉跟随

同一个 TRON2 驱动和 `127.0.0.1:15791/mcp` 提供 `head_visual_follow`。
使用现有 Perception Stack VOP，不增加自建 YOLO、独立容器或额外端口；不修改 Core、VOP 或公共前端。

### 画布与操作

1. 连线为 TRON2 头部 `camera` → `vop` → `head_visual_follow` 左侧输入端。
   静态声明和 info 都返回解析后的 `/limx_tron2/camera/objects`、`data/json` 输入。
   开启项目只建立只读订阅，不启用头部运动；跟随执行前要求连线注入该头部对象话题并有新鲜检测。
   不接受左右腕相机或其它对象话题。
2. ACTION 选择 `Follow`（`follow`），在 `Speed` 选择 `0.5×`、`1×`、`2×` 或 `4×`，点击“执行”启用。默认 `1×` 保持原跟随行为。
   运行中换档后再次点击“执行”，在同一ACP会话内平滑调整速度，不重建控制器或重置目标。
   不传 `speed` 的重复调用保留当前档位，新会话省略时使用 `1×`。优先选择靠近图像中心的人，
   至少两个不同的新鲜结果确认后跟随；空间关联不等于身份识别。
3. ACTION 选择“不跟随模式”（`not_follow`）时显示“不跟随操作”：
   - “暂停跟随”（`pause`）停止发送新目标，等待最后受理目标取得持续新反馈的静止验证。
   - “头部复位”（`reset`）先暂停并确认前一会话停稳、完成回调，再经原 `move_head` 路径转至 pitch=0°、yaw=0°，验证到位后报告复位结果。
   暂停不依赖仍然有效的 VOP 连线；断流时也能请求暂停。暂停不是硬件急停。
4. 跟随过程中，至少3个有效新检测结果且持续1.5秒确认无人后平滑回正，随后继续等待人物。
   检测错误、过期、重复结果和短暂遮挡不会被当作“无人”。

### 平滑闭环与边界

仓库宇树实现中未找到可直接复用的独立头部跟随；G1 手臂伺服、R1 底盘伺服不属于这个执行器。
头部调度借鉴官方 [TRON2 head_scheduler.py](https://github.com/limxdynamics/tron2_env/blob/ec84cead9c4dab658b7253f94a2eab73d5e9b62d/src/tron2_env/motion/head_scheduler.py)：
只使用两轴 `request_moveh`，按最新视觉结果更新目标，每次命令等待其匹配ACK，连续跟随时不再等待每个小目标完全静止。
这是视觉闭环、短时 MoveH 目标流；不宣称厂商提供了原生连续电机 ServoH，也不使用全身16轴 ServoJ或向双臂发送保持目标。

控制计算周期20ms，头部命令不高于10Hz；实际频率受ACK和模式核对耗时限制。
`0.5×`/`1×` 的单个目标时间窗仍为250ms，`2×` 为125ms，`4×` 为100ms（原传输校验的下限）。
高档位的新目标相对当前反馈最多领先3.5°，限制时将规划位置同步到受限目标，防止规划积累；
不会为追求倍速而放宽原4°反馈误差保护。状态字段 `speed_limited_by_feedback` 表示最近目标是否因此受限。
人物中心观测滤波与角速度/加速度/加加速度限制分开处理。`Speed` 同时缩放期望角速度与平滑器的速度、加速度、加加速度上限，
包括跟随会话内确认无人后的自动回正；不改变暂停、手动 `Reset head` 或检测/发送频率。
`1×` 俯仰指令速度最高8°/s、偏航最高12°/s；`0.5×` 为4/6°/s，`2×` 为16/24°/s，`4×` 为32/48°/s。
这些是软件期望速度上限，不是测得的硬件速度；实际速度仍取决于厂家MoveH执行、反馈及ACK，保护触发时停止而不绕过。
降档保留平滑器状态，逐步减速，不瞬间截断已有速度。下拉框使用字符串 `"0.5"`、`"1"`、`"2"`、`"4"`，兼容现有画布枚举提交方式；
默认1倍排在首项，避免旧画布忽略默认值而自动选中慢速。直接调用也兼容对应数值；非法跟随倍率在绑定或变更运动前拒绝。
其它模式忽略隐藏的速度字段，避免过期表单值阻止暂停/复位。所有档位均保持原
角度范围 pitch=-55°至30°、yaw=-45°至45°。循环延迟后不追发积压目标；反馈落后已发送目标超过4°即停止累积新目标，
这4°是反馈误差保护，不是单次转头幅度上限。

VOP 使用原生 `objects[].name/position/confidence`，只接受置信度至少0.55的人物。
`position` 是−1至1的二维中心位置，向右/下为正，不据此编造人脸框、躯干或标定。
VOP发布时间、可选处理耗时、本地接收时间只用于新鲜度/重复检查，不证明硬件时钟同步；`measurement_time_verified=false`。

跟随全程占用原头部动作所有权，与双臂、预设动作和夹爪互斥。每次发送前仍检查运行模式、连接、
控制代际、头部所有权、机械臂参考姿态和观测有效性。机械臂改变、其它动作、反馈或ACK不明时不重试、不自动复位。
暂停/复位的受理响应不等于物理停止或复位成功。最后跟随目标要求至少3个推进的新反馈、持续0.5秒静止并到位，
复位使用原有到位验证器；失败或未知发送保留保护。

### 完成协议与验证

持续跟随使用既有后台 ACP：`background=true, timeout=0, silent_completion=true`，
返回唯一会话 `action_id`；“不跟随”控制操作也有独立ID及预计时长，实际终态通过原验证TLS的ACP回调报告。
回正/等待人物不是整次会话完成。`head_motion` 只显示当前所有者，`last_head_motion` 保留历史结果，二者不混同。
回调失败保留 `blocked_reporting`；不能把请求受理、ACK或缓存结果当作到位。

主对话已完成组件296项离线回归，另有真实本地MCP/ACP HTTP端到端场景覆盖连线跟随、连续目标、暂停和复位。
这些都是模拟机器人数据的离线验证；没有发送实机运动，不能替代现场平滑性、连线和模式操作验收。
配置仍需启用 `joint_state`、`camera`、`move_head`、`head_visual_follow`。

构建仅在原 TRON2 镜像中复制五个 head_follow Python 模块及导入检查，不增加模型、apt/pip依赖或其它容器。
运行离线回归：`python3 -m unittest discover -s tests -v`。

## 读取边界与字段说明

传感器后台仅发送request_get_joint_state。arm_control仅在显式move/reset调用时发送一次request_movej；move_head在显式move时发送一次request_moveh；显式启用人物跟随后，跟随模块占用同一头部通道发送带ACK关联的两轴MoveH最新目标流；夹爪卡片仅在显式读取状态或跟踪开合动作时查询夹爪，显式开合只发送一次所选侧的指令。pose仅在显式选择英文动作后自检、必要时先回原home并确认，再回正头部，以ServoJ执行预设轨迹；挥手在右肩旋转三轮后保持手臂、右夹爪开合一次，再回原home。没有模式切换、拖动示教或自动演示循环。

- 名称由机器人返回。`names` 缺失或为空时，`labels` 使用 `joint_index_0` 等索引，`labels_are_indices: true`；不推测关节名称及身体部位对应关系。
- `q` 按厂家关节接口的弧度值保留；`positions_deg` 仅进行弧度到度的换算。`dq` / `tau` 原样保留，缺失时为 `[]`。
- 左右夹爪通过安装点上的固定双指轮廓显示，跟随对应手腕运动。默认显示闭合，状态是静态示意，不表示当前实际夹持状态；夹爪卡片查询的相对开口度未接入骨架动画。附加display_hand_gripper_*节点只供渲染，全部fixed，不增加16关节反馈数量，不改变原始运动学或控制限位。显示间距/指长为示意尺寸，不是标定的实物外形、TCP或碰撞模型。
- 骨架使用Motus既有joints协议和model资源。名称缺失时按官方SDK索引顺序映射到URDF：左臂0–6、右臂7–13、头部俯仰14/偏航15；URDF文本中头部关节顺序相反，不使用文件遍历顺序。若厂家提供名称则按名称匹配，不支持的关节数或名称返回空骨架和不可用诊断。没有可用IMU姿态时省略可选imu_quat；不能发送空数组，因为平台会将其解构成四个undefined值、破坏根节点旋转。模型来源和Apache-2.0归属见resource/SOURCE.md；只保留厂家运动学，不包含网格资产。
- 厂家的 `UNKNOW` 等状态字符串原样保留；缺失字段为 `null`。
- 厂家关节时间戳的时钟及单位尚未验证，因此 `robot_timestamp` 仅作为原始值保留，`measurement_time_verified` 为 `false`。接收年龄不能证明测量数据的真实年龄。
- 默认超过5秒没有对应数据则`available: false`、`sample_status: stale`、缓存`data: null`。骨架发布空joints与不可用状态。断连时立即清空。

关节传感器start/stop兼容单实例生命周期，不启动/停止机器人，也不停止后台采样。info读取缓存，不额外请求机器人。camera的start/stop实际开启/释放只读视频连接。公开四张sensor、arm_control/pose/move_head/gripper/head_visual_follow五张actuator及model resource；不提供独立robot_status/reset_pose卡片或助手卡片。

兼容Agent Core自动注入的instance_id和_trace_id：最长128字符、无控制字符，不参与机器人请求或回显。关节传感器通过内部info查看当前原始缓存。

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
2. 挂载当前Core生成的实际DDS文件，不能把缺失文件误挂为目录。驱动使用FastDDS和loopback数据总线，默认只接受ROS_DOMAIN_ID=42。已有Core采用非标准域的专用现场，部署须先核对实际Core，再显式设置相同的ROS_DOMAIN_ID及TRON2_CORE_DOMAIN_ID（0至232）；只修改ROS_DOMAIN_ID会被拒绝。文件缺失或域号不匹配则拒绝创建participant。
3. Core 信任证书是实际文件。HTTPS 保持证书及主机名验证；`TRON2_AGENT_CORE_URL` 的主机名必须与证书 SAN 匹配。需要时使用证书包含的主机名，并相应调整 Compose 的 `extra_hosts`；不要关闭 TLS 验证。
4. MCP 端口 15791 未被其他本地原型占用。

通过 WebSocket 采样和显式 MoveJ 调用，仅将本机保存的home目录只读挂载到`/config/tron2`；不挂载 `/dev`、Docker socket或可写控制目录，也不需要 privileged 或 GPU runtime。容器使用只读根文件系统和临时 `/tmp`；ROS2 日志显式写入 `/tmp/ros-log`；`restart: always`。

如果同机部署过早期复位原型，先备份其文件，再停止旧服务以释放 15791，并在 Motus 设置中移除旧复位服务注册及画布卡片。本驱动注册名称为 **LimX TRON2 Sensors and Arm Control**；不会自动删除旧容器、旧注册或已保存的姿态文件。

## 网页查看与验收

在 Jetson 宿主运行只读检查，或从仓库运行同一脚本：

```bash
sudo docker exec embodied-limx-tron2 python3 /work/check.py
# 或：python3 limx/tron2/check.py
```

脚本等待就绪，检查Core注册、joint_state骨架sensor、可选camera/arm_control及model类型、骨架格式和URDF名称，验证生命周期及关节接收时间更新。它不调用move/reset，也不验证真实运动。

升级后刷新Motus，移除画布上的旧robot_status、reset_pose、joint_angles、battery_state实例，重新添加joint_state以更新缓存的渲染格式。相机、骨架与运动卡片按需添加；视觉跟随卡片单独放置，只需camera与VOP连线；model由骨架组件自动读取，不需要手动执行或连接。开启后从监控查看joint_state的厂家双臂/头部骨架。监控和查看数据流均加载同一model资源；升级后应重新加载组件以更新缓存。断连/过期发布空joints列表和不可用诊断，原始info缓存data为null。恢复并收到新关节回复后更新骨架。已保存画布不会随tools/list改变而自动删除旧实例；部署工具不得在未确认时覆写整张画布。

`GET http://127.0.0.1:15791/health` 返回卡片状态：启用关节时，机器人连接且这些已启用传感器全部有最近接收数据为HTTP 200。可选camera的idle/断流不会阻塞它们或机械臂任务。只启用相机时，所有已启用相机都有最近成功发布的JPEG才就绪，否则为503。配置必须启用至少一张传感器，禁止仅开启arm_control却将空样本集合当成就绪。`registered_mcp_id` 单独表明 Core 注册是否完成；机器人可读与 Core 已注册是两项独立条件。

## 开发测试与验证范围

```bash
python3 -m pip install -r limx/tron2/requirements.txt
python3 -m unittest discover -s limx/tron2/tests -v
python3 -m pytest tests/test_dockerfile_copies.py -q
```

测试包含真实本机 HTTP / WebSocket 服务、请求关联及超时、序列号约束、重连、断连清空、过期数据、卡片生命周期、ROS2 JSON 发布的模拟验证和镜像入口布局。测试不驱动实体机器人。

发布后使用check.py核对当前源码版本、骨架与动作声明、Core注册及实时关节；使用--camera核对三路实际DDS图像。该检查不会执行运动，不能代替move/reset的实际动作验收。

## arm_control 动作

按x-action-params显示每个动作所需参数；实际运动使用厂家TRON2 MoveJ接口。

| ACTION | 参数 | 行为 |
| --- | --- | --- |
| move | joint、time（默认8秒） | 14项绝对目标，左7右7，rad，不包含头部 |
| reset | 无 | 回到自然垂下目标，并自动检查连续反馈是否正确复位 |

自定义move继续接受数组或≤4096字符的JSON数组文本，不接受NaN/Infinity、bool、数字字符串、头部角度或超限值。

reset使用现有`/config/tron2/home.json`（宿主`/opt/phanthy-motus/data/tron2`只读目录挂载）。沿用tron2-arms-reset-pose-v1、14项rad、标准顺序、序列号与固件绑定。升级不重新采集或覆盖home，也不从历史home文件回拷。文件缺失/不匹配时info报告未配置、实际运动拒绝，但平台仍可启动。

发送任何move/reset前，除连接、近期状态、目标身份/限位与互斥检查外，还要求厂家SDK 3.1要求的上层developer_mode、working_mode_valid=true、collision_enable_valid=true且collision_enable=true、compliant_enable_valid=true且compliant_enable=false。compliant_enable表示柔顺/拖动状态，本驱动不在该状态下发位置MoveJ；不是要求把柔顺模式开启。字段缺失、无效、非严格布尔值或其他模式全部拒绝，返回motion_sent=false。*_valid作为厂家字段可用性/有效性门槛，不被当作独立安全认证。驱动只拒绝不合适的运动，不自动切换模式、开启保护或关闭拖动；内部info.motion_readiness给出拒绝原因。模式要求见[厂家SDK 3.1](https://www.limxdynamics.com/zh/documents/847884267345285120)，controller_mode等其他模式拒绝，不宣称穷举厂家所有可执行模式。

pose执行前自动回home和arm_control.reset共用时长max(8,ceil(最大关节变化度数/5))/1.2秒，相比旧复位规则提速20%，最短约6.67秒，超过60秒拒绝。所有目标仍校验[厂家SDK限位](https://www.limxdynamics.com/zh/documents/847884267345285120)。头部、夹爪不发命令，不进行路径碰撞规划。

ACTION仅公开move、reset。info/start/stop由平台内部生命周期和诊断使用，不显示为可选动作；start返回ready，stop返回idle，均不发送运动，也不是机器人急停。pose/pose_name/preview/list/verify_at_home已删除，旧调用会拒绝。只有显式move/reset发送一次request_movej并等待GUID/序列号匹配的ACK；初始accepted响应不表示到达，arrival_verified为false，并提供唯一action_id。两个动作声明x-completion（120秒上限）和共享资源arm_l/arm_r；动作完成由后台反馈验证和ACP回调跟踪，不以ACK代替完成。

check.py只调用生命周期/信息和model资源，不执行运动。源码和模拟接口通过不代表网页渲染、实际运动到位或PR审批已经通过。正式镜像由上游PR Review Agent的/request_bot_review发布；部署后仍需现场验收。

## 动作完成协议（ACP）

MCP用于卡片发现与tools/call请求，ACP（Action Completion Protocol）用于异步动作完成跟踪。
这是仓库README_dev.md规定的配合方式：保留/mcp、x-completion及唯一action_id，在实际验证结束后POST /api/acp/complete，不能将MCP路由简单改名为ACP。
平台日志中的MCP_CALL finish是Agent结束当前推理轮次的工具调用，不代表机械臂跳过ACP。move和reset的受理/终态结果均标明completion_protocol=acp。

reset自动复位检查与动作完成跟踪为同一个过程，无需另调verify_at_home。reset的初始结果为reset_verified=false、at_home=null；运动结束后的连续新反馈通过后，ACP completed结果携带reset_verified=true、at_home=true、最大角度误差、容差、样本数和本次home摘要。
未通过验证时通过ACP报告error、reset_verified=false、at_home=null，并保留错误原因；不能把失联或缺少反馈时的未知位置声称为已复位。复位目标固定为发送时已验证的home，不在动作途中重新加载或覆盖文件。

- move、reset返回唯一action_id；awaiting_ack/moving期间同一个驱动实例保留双臂所有权，第二个动作拒绝，不依赖Core的资源标签作为唯一锁。HTTP调用方断开不会取消正在发送的物理命令，也不会释放所有权。
- 从ACK结束后保守等待完整time/duration_seconds，再检查至少3个不同的新关节回复，跨度至少0.5秒；14项误差均≤1度、相对首个样本变化≤0.2度，有dq时额外要求每项原始值绝对值≤0.05（未将dq单位认证为rad/s）。位置稳定性直接由q判断。厂家名称重排时先映射至官方索引；不能用同一缓存样本或命令目标伪造到位。
- 到位后POST /api/acp/complete：action_id、status=completed、result.arrival_verified=true及误差/样本数，tool=arm_control。初始响应仍是受理；通过arm_control.info或/health的arm_motion可查看最终状态。该验证基于连续新收到的关节回复，厂家测量时间戳仍未验证，不代表路径碰撞规划或独立运动安全认证。
- 超出预计时长再加10秒仍未满足条件、反馈过期、断连或连接更换，报告status=error、arrival_verified=false。发送/ACK结果不明时仍返回跟踪ID并检查反馈，不重发MoveJ；厂家明确拒绝或发送前拒绝不生成已受理动作。
- ACP使用与注册相同的已验证HTTPS证书上下文，禁止重定向、不读取ACCESS_TOKEN、不关闭TLS验证；同一终态payload最多尝试3次，每次最多3秒，退避0.5/1秒。最坏发送/ACK、60秒动作、验证和回调仍小于声明的120秒上限。
- 只有验证成功且回调送达才释放本地所有权。反馈验证失败或回调无法送达会保留blocked_requires_operator_check，阻止新的运动。失败通知和Core超时不会物理停止机械臂；现场确认停止、处理原因后再重启驱动。内部start/stop不会清除该状态，stop仍不是急停。
- 进程正常关闭时终止验证任务并报告error，说明运动可能继续；不发送停机、复位或第二条运动命令。进程强制退出无法保证回调，Core保留120秒超时兜底。

Dockerfile按dpkg实际安装状态检查python3-pip和ca-certificates，仅缺少时执行apt更新/安装；基础镜像已提供的包不重复安装。aiohttp/PyYAML仍由requirements.txt管理，是既有HTTP/WebSocket和配置解析依赖；完成机制只增加Python标准库源码，没有新增pip包、设备权限或资源资产。

## 头部与左右腕RGB摄像头

camera、camera_left、camera_right使用三路独立中继和DDS topic。默认均可发现但进程启动不打开视频连接。刷新Motus，把需要的相机卡片拖到画布，开启智能控制，从监控或查看数据流观看，无需与arm_control连线或填写URL。Core显式start连接对应厂家源，stop只释放该相机连接，info只读，不影响其他相机或关节后台采样。不自动执行任何机械臂、头部、夹爪运动。

| 卡片 | 厂家源topic | Motus输出topic | 只读现场尺寸 |
| --- | --- | --- | --- |
| camera（头部） | /camera/top/color/image_raw/compressed | /limx_tron2/camera | 640×480 |
| camera_left（左腕） | /camera/left/color/image_rect_raw/compressed | /limx_tron2/camera_left | 848×480 |
| camera_right（右腕） | /camera/right/color/image_rect_raw/compressed | /limx_tron2/camera_right | 848×480 |

固定读取同一TRON2_ROBOT_URL主机HTTP59001：`/stream?topic=<表中厂家源>&quality=80&rotate=0`。左右腕使用现场ROS2发布并有连续有效帧的image_rect_raw，不将其他话题名称套用到这三路源。厂家的camera_service状态不是每路实时帧证据；话题存在/有发布者也不能代替实际帧验收。保留厂家left/right标识，现场可分别在对应镜头前放纸核对左右画面，不发运动命令。

只接入三路RGB，不发布depth/标定/metadata，不支持PTZ、录制或调用者指定URL/topic。原始JPEG不解码转码，复用既有aiohttp与ROS sensor_msgs/CompressedImage，声明image/jpeg供平台现有CameraRenderer显示。每路header.frame_id分别为tron2_head_camera、tron2_left_wrist_camera、tron2_right_wrist_camera；stamp是中继接收/发布时间，不是已验证的厂家测量时钟。原有Core和loopback domain42总线配置不变。

每路限10fps（实际受厂家源影响）、4MiB/帧、宽高各≤4096；三路各自depth=1、best-effort/volatile，独立任务/连接/锁/代际标记。校验JPEG封装、头部尺寸、SOF/SOS分段，不宣称完整熵解码。拒绝非200、重定向、非MJPEG、压缩HTTP编码及非法帧。整帧限3秒，断流清空available/尺寸/接收时间并按1秒间隔重连；首次start限8秒且要求成功发布一帧，并发start共用该路连接，stop能中断该路等待。一路断流不会停止另外两路或机械臂任务。health.cameras独立列出各路状态，旧health.camera保留头部；普通关节就绪不受可选相机idle/断流影响。只启用相机时则要求所有已启用源可用。

旧平台图像组件可能保留最后一帧；静止图像不能证明连接正常，需查该路info.available与接收年龄或重新打开面板。普通check.py只验证发现/协议/info，不改变既有视频启停。显式视频验收开启全部已启用源：

```bash
sudo docker exec embodied-limx-tron2 /bin/bash -c 'source /opt/ros/humble/setup.bash && exec python3 /work/check.py --camera'
```

检查每路计数持续增长，并用实际本机DDS订阅逐topic收到至少两帧合法JPEG（每topic只缓存两帧，整体限5秒），打印各路名称/尺寸，保留连接供平台查看；不保存图像或发送运动。完成后停止对应卡片/画布释放连接。Mac诊断SSH隧道仅用于开发读取，不是Jetson平台正常视频链路的一部分。

测试使用合成MJPEG，覆盖启停/并发/中断等待、断流清空/重连、无效响应/畸形/截断/超限/停滞帧、RPC元数据与禁止运动；左右腕分开输出/独立停止、失败不影响其他源/JSON就绪，以及三路CompressedImage原始二进制、独立frame_id/单帧队列。只读隧道现场诊断确认三路同时连续视频，左停止后头部/右继续发布；实际图像/现场标识不进入仓库或公开PR。源/中继测试不能代替新镜像部署后的DDS/Core/平台验收。

预设动作卡片的使用、起始姿态、停止与验证范围见 [POSE.md](POSE.md)。pose与arm_control复用同一双臂执行器；不扩展arm_control的公开动作，不更改原始home。
