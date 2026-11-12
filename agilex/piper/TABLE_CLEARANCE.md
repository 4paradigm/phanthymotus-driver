# PiPER 夹爪离桌距离

`piper_table_clearance` 是默认关闭的只读 sensor，以 5 Hz 发布 `data/json`
到 `/{namespace}/piper/state/table_clearance`（ROS `std_msgs/String`）。
支持 MCP `start/stop/info`；只读取已有 runtime 缓存，不创建 CAN 连接、不发送控制指令。
Stop 释放本卡 timer/publisher，不影响现有控制和监控。`info` 不刷新硬件反馈。
下游须检查 state、timestamp 和 feedback_time，不得沿用历史有效值。

启用方式：在部署配置中将 `table_clearance.enabled` 改为 true，重新初始化驱动。
verified 保持 false 时会持续输出不可用；完成以下物理标定后才能设为 true。
配置仅在初始化时读取，默认不在工具列表中注册。每个实例对应一个现有机械臂 runtime。


该卡根据实测几何计算模型距离，不从图像自动识别夹爪或拟合桌面。
两主两从中，每个实例只绑定其现有 runtime 的一条反馈源，不推测四臂对应关系。
当前现场未提供经过验证的桌面平面和夹爪几何，因此默认输出
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
