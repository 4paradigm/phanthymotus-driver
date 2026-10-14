# 两卡集成与现场迁移

计划一与计划二各自提交独立分支。本分支交付控制卡和共享快照接口；监控/模型在计划二分支完成。
两分支均基于 `f5ca598278a3f015cd9503d674a81d2d75b122fa`。本分支不修改 agent core 或 Core PR #305。

## 合并监控代码

1. 核对计划二分支基线和新增文件，复查监控通过注入 `read_monitor_snapshot()` 读取同一 runtime。
2. 在 `build_plugins()` 中只构造一个控制 runtime，再把其 callback 注入监控插件；不另开 CAN。
3. 注册监控 sensor 与后台 model resource，加入 Docker COPY 和驱动元数据，验证 URDF 关节名与实时 q 弧度一致。
4. 两个实例分别使用原有 MCP、CAN、namespace/topic。运行两设备隔离、只读快照、骨架及控制回归。
5. 画布两张核心卡为 `arm_control` 和监控卡；`piper_photo` 可选，model 为后台渲染依赖，不摆成卡片。

## 替换现场版本之前

- 先读取现场两个驱动的工具清单、当前镜像和实际代码，核对额外“关节限位”、实时角度、空间移动等功能。
- 未读到或未比对的额外限位实现不得被本分支覆盖。新控制卡继续使用 SDK/部署限位交集。
- 导出当前布局及 MCP 配置，保留数据库、compose、镜像、代码和未提交文件的现场备份。
- 记录模式、六轴/夹爪使能、角度、故障、控制占用。部署须在核对后安排；真机动作另取得现场授权。

## 布局迁移对应关系

保持每台原 MCP 实例、CAN 和话题绑定，仅替换工具卡和动作参数；不要修改同事未备份的活动项目。
迁移前停止项目并确认设备占用，保存原布局用于回退。

| 旧工具与操作 | 新 arm_control 操作 |
|---|---|
| piper_status.read | read |
| piper_permission.allow / block | allow_motion / block_motion |
| piper_standby.switch / piper_can.switch | switch_mode，target_mode=standby/can |
| piper_enable.enable / piper_disable.disable | enable / disable，保留 object |
| piper_joint.move | move_joint，保留 joint 与 target_deg |
| piper_gripper.open / close / set_width | gripper_open / gripper_close / gripper_set_width |
| piper_stop.halt | halt，保留 target |

旧状态固件查询、重连、额外限位、零位与轨迹功能不自动转换。迁移不携带旧 execute、preview_id 或人工确认状态；
新卡重新预览确认。相机卡保留，旧十卡专用切换控件不参与新布局。

新版本“实际 Bot Version”在 Bot 构建报告生成后填写。当前：**尚未生成**。
Git提交号仅用于源码追踪，不能充当 Bot Version。现场部署、实际动作验收和老师验收应分别记录。
