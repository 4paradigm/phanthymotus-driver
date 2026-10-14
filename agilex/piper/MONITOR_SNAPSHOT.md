# 共享监控快照接口

计划一与计划二从 PR #382 的提交 `f5ca598278a3f015cd9503d674a81d2d75b122fa` 建立独立分支。
计划一新增接口 `PiperRuntime.read_monitor_snapshot()`。计划二通过注入这个 bound method 读取同一 runtime；
不得创建新的 PiperDriver、SDK 实例或 CAN 连接。插件的具体构造函数由计划二定义，集成时再接线。

接口返回一个独立字典，可被监控插件转换为 `sensor/skeleton` 和参数数据流。角度单位为度；
骨架的 `q` 值由监控插件转换为弧度，并使用与 URDF 一致的关节名。

| 字段 | 类型与含义 |
|---|---|
| `device_name` | string，配置 name；缺省使用 CAN 标识 |
| `can_interface` | string，所属 CAN 设备 |
| `connected` | bool，共享 runtime 已持有连接且未关闭；不保证反馈新鲜 |
| `feedback_time` | float/null，反馈时间，Unix 秒；过期值可保留作为诊断 |
| `fresh` | bool，六轴反馈有效且不超过 0.5 秒 |
| `joints_deg` | 六项 number 数组/null，六轴实际角度（度） |
| `joint_limits_deg` | 六组 `[lower, upper]`/null，SDK 与部署限位交集（度） |
| `control_mode` | string/null，设备原始模式反馈 |
| `enabled` | 六项 bool 数组/null，各轴使能反馈 |
| `gripper` | dict/null：`width_mm`, `effort_nm`, `enabled`, `error_code`, `feedback_time` |
| `motion_enabled` | bool，runtime 当前操作许可；与使能/模式独立 |
| `faults` | dict/null：`arm_status`, `teach_status`, `error_code`, `motor_faults` |
| `busy` | bool，当前是否有独占动作 |
| `active_card`, `active_action`, `active_object` | string/null，正在执行的工具、操作、arm/gripper 对象 |
| `last_actions` | dict，按工具名记录最近完成结果，含 action_id/status/result |
| `reasons` | dict，`connection`, `feedback`, `joint_limits_deg`, `gripper` 对应不可用原因 |

接口不调用 `connected()`，不查询固件、不切换模式、不使能、不发运动指令，不修改预览或取消状态。
只读 SDK getter 与动作共用原连接。它不获取动作锁；连接锁采用非阻塞尝试，连接正在变更时立即返回空值及原因。
未连接、反馈过期、读取失败时保留 `null`，不得将缺失值替换成六个零或“无故障”。
连接变更导致本次无法读取时，`connected` 只表示调用开始时观察到的连接状态，数据仍为不可用。

两台机械臂分别注入各自的 runtime callback；设备名称、CAN 和话题沿用各自配置。
不可混用 callback，不使用模块级单例保存设备状态。

最终集成由计划一完成：注册计划二工具、打包模型与监控文件、核对元数据、备份并迁移画布。
当前控制分支只注册 `arm_control` 与可选 `piper_photo`，不声称监控或骨架已接入。
