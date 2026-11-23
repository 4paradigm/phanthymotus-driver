# PiPER 夹爪离桌距离

## Canvas 可视化配置和输出

卡片右上角“实例配置”齿轮提供机械臂 CAN 接口、显示名称、是否使用实测标定的下拉项。
当前为单实例卡片：一张卡一次监测一个接口。can0/can1 在本工位分别对应左右从臂，
其他工位需核对接线。选项保存在 Core 的卡片配置中，通过标准 config 操作应用；
停止或重启卡片仍可恢复，不需要修改镜像或配置文件。

- 不标定：选择“使用内置模型估算”，填写桌面相对安装面的高度（默认0）。
- 使用实测标定：选择“使用实测标定参数”，填写平面、几何、开合轴、开度范围、
  坐标系、误差余量及记录编号。数组使用 JSON 输入框。只有物理核验后才能确认 verified。
- 选择标定模式并不自动完成标定；未确认时清空距离。切换 CAN 接口会撤销旧标定确认，
  清空旧距离，避免把左臂标定应用到右臂。无效参数在更改前拒绝，原有有效配置保持运行。

当所选接口等于现有 runtime 接口时复用其反馈；另一个接口只创建本卡独立的
SocketCAN 接收器，使用 CAN_RAW_FILTER，只调用 recv、不发送 CAN 帧、不操作运动控制卡。
停止后释放接收器。另一个接口不存在、帧未到、过期或非宽度模式时不输出距离。
不支持同时双臂合并距离；不要将 CAN 接口等同于固定的物理左右标识。

生命周期顶层 state=running/idle 表示卡片是否运行；内层 result.state=estimated/ok/
unavailable/idle 表示测量结果状态。两者含义独立。

“查看数据流”首先显示中文“读数”：机械臂、接口、模式、数据状态、间隙、开度、
最近部件、反馈龄、状态说明以及“未启用自动防碰桌”。原机器字段保留，新增“字段说明”
为每个机器字段解释含义和单位；部件标识也给出中文对应。

配置仅属于这张测距卡，不改变 Arm control 的 can_interface。Core 的齿轮配置表单在
项目停止时编辑；不需要开启智能控制来查看已启动的测距数据。模型已内置，不增加新依赖。

卡片操作菜单提供“开始测距（仅监测）／暂停测距”，通过独立执行按钮操作，
不需要启动整个智能控制项目。暂停会发布最后一帧 idle，并清空距离，避免面板将历史
估算值误作实时值。停止项目后若需单独查看本卡，请重新执行“开始测距”。
标定参数集中在 Advanced settings（高级设置）；部分旧 Core 不支持条件隐藏，
因此高级字段即使在估算模式可展开，也不会参与估算或把 verified 改为 true。
查看数据流时将面板内部滚动到顶部，首先查看中文“读数”和实时更新时间。

2026-11-23：生命周期、配置、模型估算、标定、控制及部署回归合计 **122 passed**。
Canvas schema 和 config/start/stop 协议已验证；浏览器界面呈现需以现场观察为准。

## 模型估算模式（可选）

随附配置已启用内置模型估算，原厂 100 mm 夹爪、can0、桌面与模型安装平面重合的
现场无需下载模型、填写文件路径或增加挂载。默认配置为：

```yaml
table_clearance:
  enabled: true
  mode: model_estimate
  verified: false
  can_interface: can0
  estimate_profile_path: builtin:piper_100mm
  table_z_mm: 0
```

`builtin:piper_100mm` 解析为包内 resource/clearance/piper_100mm.json，并校验固定
SHA-256。现有 Dockerfile 已 COPY resource/，无需添加依赖或构建下载步骤。
资源目录保留来源记录、源文件哈希及 MIT 许可。新增模型 JSON 为 67,334 字节；
连同许可和说明会带来少量资源字节增长，未测量最终镜像层大小。
`table_z_mm: 0` 是桌面与模型安装平面重合的假设，不是自动检测或已完成实测标定。
不同工具、底座高度或 CAN 接口仍需选择匹配配置，不能将默认模型套用于不匹配硬件。

仍支持外部 profile：将 estimate_profile_path 换成容器内绝对路径，并由部署侧只读挂载。
外部模型模式和下文 calibrated 模式用于其他安装情况；默认用户无需这些额外步骤。

profile 的 `schema` 为 `piper.gripper_estimate/1`，包括 `root=world`、`source`、
`commit`、按父节点先行排序的 `joints`、`vertices_m`、`joint_bounds_deg` 和
`opening_range_mm`。每个 joint 包含 name、kind、parent、child、4×4 origin、axis
及可选 mimic（joint、multiplier、offset）。长度使用米，角度使用弧度。
vertices_m 包含 flange_link、gripper_base、gripper_link1、gripper_link2 四组
部件局部凸包点；其中最小桌面高度即输出估算值。运行时只需已有 NumPy，无需 SciPy。
loader 检查链路、有限坐标、刚体变换和覆盖范围，并记录实际文件 SHA-256；
哈希用于追溯，不能替代对模型来源及安装匹配关系的核验。

该模式直接使用现有六轴反馈和夹爪开度，不使用 SDK 末端位姿或额外 CAN 连接。
输出 `state=estimated`、`estimated=true`、`calibration_verified=false`，
`clearance_mm` 为估算高度，`conservative_clearance_mm=null`；并附模型版本、哈希、
最低部件和桌面假设。反馈过期或超出模型范围时清空距离。消费者必须显式接受
estimated 状态，不能按 calibrated 模式的 state=ok 使用。

本地与现有预览程序核对时，使用其固定模型版本
`680fa925cd00db7e2628563d2edc5540963045de`、同组关节角度及 99.7 mm 开度，
得到相同的 112.0788112340331 mm。此核对只证明计算复现一致，不是真机精度标定。

验证命令增加 `tests/test_piper_clearance_estimate.py`。2026-11-23 本次源码验证结果
记录在提交操作前的测试输出中。2026-11-23 内置模型版本回归合计 **116 passed**；
覆盖随附配置直接注册卡片并产生估算、固定模型校验和篡改拒绝。

2026-11-23 现场软件验证：使用独立命名的本地 AMD64 源码叠加测试镜像运行内置配置，
MCP 返回 state=estimated，真实反馈开度为 51.7 mm，模型估算间隙为约 155.7851 mm。
独立 DDS 订阅收到 10 帧，约 5.0009 Hz；motion_enabled=false。
这验证了配置、真实反馈读取与发布链路，不是量尺测量或距离精度验收。
此次是已有 AMD64 依赖镜像上的本地源码/资源叠加，不代表上游完整构建或云端 ARM64
镜像已在 AMD64 工位机原生运行。画布浏览器内的数值呈现尚未在本轮观察确认。

## 标定模式

`piper_table_clearance` 是只读 sensor，以 5 Hz 发布 `data/json`
到 `/{namespace}/piper/state/table_clearance`（ROS `std_msgs/String`）。
支持 MCP `start/stop/info`；只读取已有 runtime 缓存，不创建 CAN 连接、不发送控制指令。
Stop 释放本卡 timer/publisher，不影响现有控制和监控。`info` 不刷新硬件反馈。
下游须检查 state、timestamp 和 feedback_time，不得沿用历史有效值。

若需要使用实测标定模式，将 mode 改为 calibrated，并保持 enabled=true，重新初始化驱动。
verified 保持 false 时会持续输出不可用；完成以下物理标定后才能设为 true。
配置仅在初始化时读取。每个实例对应一个现有机械臂 runtime。


该卡根据实测几何计算模型距离，不从图像自动识别夹爪或拟合桌面。
两主两从中，每个实例只绑定其现有 runtime 的一条反馈源，不推测四臂对应关系。
标定模式在未提供经过验证的桌面平面和夹爪几何时输出
`unavailable/calibration_unverified`，不会填写任何猜测尺寸。

启用后先保持 `verified: false`，以下字段必须按该机械臂/工具实测填写：

| 字段 | 约定 |
| --- | --- |
| `calibration_id` | 可追溯的设备、工具及安装标定记录编号 |
| `can_interface` | 绑定的现有 runtime CAN 接口；反馈接口不匹配则拒绝计算 |
| `base_frame` | 该臂 SDK 末端反馈使用的基坐标系名称 |
| `plane_base_mm` | `[nx,ny,nz,d_mm]`，桌面满足 `n·p+d=0`；n 为朝上的单位向量，nz>0 |
| `body_vertices_mm` | 夹爪固定本体凸包顶点，SDK 末端反馈参考系，单位 mm |
| `left_finger_vertices_mm` | 左指在零开度下的凸包顶点，同一参考系 |
| `right_finger_vertices_mm` | 右指在零开度下的凸包顶点，同一参考系 |
| `opening_axis` | 该参考系的单位开合轴；左指随 width/2 沿正向平移，右指沿负向平移 |
| `opening_range_mm` | 该安装工具验证过的开度范围，超范围不计算 |
| `uncertainty_mm` | 实测总误差余量，含桌面拟合、安装、位姿反馈、开度、时间偏差及几何近似误差 |
| `verified` | 以上全部物理核验后才设为 true |

每组凸包须含 4–256 个非共面顶点，包含指尖、指背及可能最靠近桌面的突出结构。
可使用保守外包络；此时输出为模型间隙的保守估计，不声称是实际表面的精确值。
不计夹持物、软指变形、线缆和安装模型之外的部件。自适应、旋转或非对称夹爪不适用
当前平移开合模型，必须先更换几何模型。

计算过程：按当前开度平移两指顶点，使用现有 SDK 约定
`R=Rz(yaw)Ry(pitch)Rx(roll)` 将末端参考系顶点变换到基坐标系，
再对所有顶点求 `min(n·p+d)`。线性函数在凸包上的最小值由顶点取得。
末端反馈已是 SDK 的参考点，不能再叠加未经确认的 TCP 偏移。

输出 `clearance_mm` 为有符号法向间隙，负值保留，表示模型穿过桌面；
`lowest_point_base_mm` 为取得最小间隙的模型顶点；
`conservative_clearance_mm=clearance_mm-uncertainty_mm`。
水平桌面时即最低点的垂直离桌距离；倾斜桌面时为垂直于桌面的间隙，
并非沿世界 Z 的高度。输出还包含标定编号、基坐标系、开度和反馈时间。

位姿和开度均须在过去 0.5 秒内，且两者时间差不超过 0.1 秒。
缺失、断开、未来时间、超时、接口不匹配或读取异常均使距离及最低点为 `null`。
独立发布可持续报告失效，不会沿用上次的“安全距离”。


## 验收

1. 核对该臂 CAN 接口和 SDK 基坐标系，记录工具/安装状态及标定编号。
2. 在基坐标系实测桌面至少三个不共线点，核对拟合残差、法线方向。
3. 实测固定本体及两指几何，在多个开度和姿态验证覆盖最低结构和开合方向。
4. 用量块或尺核对至少三种离桌高度、不同姿态和开度，确定总误差余量。
5. 在有人监督且与桌面保有充分间距时核验实时结果和断流失效；负间隙算法使用离线数据测试。
6. 工具、底座、接口映射或桌面变更后撤销 verified 并重新标定。

本次尚未提供实测标定，未构建镜像、未部署、未移动机械臂，也未完成真机精度验收。
`advisory_only=true`；本卡仅观测，不执行避碰或证明安全。即使保守距离为正也不能单独用于安全联锁。

## 软件验证

Windows 11 使用 WSL Ubuntu，在仓库根目录执行：

```bash
python3 -m pytest -q tests/test_piper_table_clearance.py tests/test_piper_unified.py tests/test_piper_deployment.py
python3 scripts/check_service_yml.py agilex/piper
```

测试覆盖合成几何、旋转、开度、倾斜桌面、失效和生命周期、完整驱动注册及控制回归。
模拟反馈与离线测试不能代替物理标定和真机验收。复用已有 numpy/ROS 依赖，不添加包或设备权限。

2026-11-12：上述测试在 WSL Ubuntu 中实测 **97 passed**，service 检查 **1 checked, 0 failed, 0 known gap(s)**；`git diff --check` 通过。

## 官方接口来源与本卡新增计算

本卡没有调用官方的“夹爪离桌距离”接口，也不把末端 Z 直接当作离桌距离。
数据复用现有 runtime，从项目固定版本 SDK 取得：

- [GetArmEndPoseMsgs](https://github.com/agilexrobotics/piper_sdk/blob/8d4dc29bd6e753066dedee235c008e90ecb2ab4a/piper_sdk/interface/piper_interface_v2.py#L1003-L1023)：末端位置/欧拉角及时间戳，原始单位为 0.001 mm 和 0.001 度。
- [GetArmGripperMsgs](https://github.com/agilexrobotics/piper_sdk/blob/8d4dc29bd6e753066dedee235c008e90ecb2ab4a/piper_sdk/interface/piper_interface_v2.py#L1078-L1106)：夹爪行程及时间戳，原始行程单位为 0.001 mm。

现有驱动将单位转换成 mm/deg。本卡新增的是依据桌面平面、夹爪凸包、姿态和开度计算有符号法向间隙。
桌面和安装工具的标定仍必须现场完成，官方反馈接口不保证本卡模型的实际测量精度。

## 2026-11-13 审查修订验证

- `build_plugins()` 创建 verified=true 的真实卡片，绑定现有记录式模拟 SDK；没有替换 read_monitor_snapshot 或手工注入最终 snapshot。
- 验证实际绑定 UnifiedControlPlugin.read_monitor_snapshot；检查初始距离、姿态/开度变化、姿态过期失效和恢复，全程无 SDK 控制指令。
- 对有限输入运算溢出增加防护；返回 unavailable/nonfinite_geometry 并清空距离，结果通过严格 JSON 序列化检查。几何非共面验证在缩放后计算，避免极大坐标差导致 SVD 溢出。
- WSL Ubuntu 测量、统一控制和部署回归合计 **101 passed**；service 检查 **1 checked, 0 failed, 0 known gap(s)**；git diff --check 通过。
- requirements.txt 已有 numpy==1.26.4；Dockerfile 的 sed 仅删去 piper-sdk 源地址行，其余依赖仍通过 pip 安装。本次不改 requirements 或依赖安装层。
- 原 PR 的 Dockerfile COPY 仅打包该 Python 源码，driver.yaml 用于声明可选卡片；会增加少量源码字节，不声称镜像体积完全不变，也未测量实际镜像大小。
- 本轮没有构建镜像、部署或真机测距验收。
