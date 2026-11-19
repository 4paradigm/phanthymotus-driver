# Helios 电池读取接口

适配对象为安装版底盘 UI `v0.48.20261030-luoshi` 的只读状态流。当前接口来自厂家网页及实际报文核查，不保证所有底盘或后续版本具有相同合约。

## 传输与消息

连接 `ws://<chassis-host>/ws/AGVInfo` 即可被动接收二进制 Protobuf；不发送 online、heartbeat、模式、导航或充电命令。

- `UiDatagram` 字段 8：`robot_base`。
- `RobotBase` 字段 5：`robot_id`；字段 11：`data_pool`（JSON 字符串）。
- `data_pool.multi_battery_info` 是第二层 JSON 编码的电池数组。
- 厂家编号为字符串 `"1"` 和 `"2"`，物理左右尚未确认。重复或未知编号拒绝解析。

最小消息投影见 `resource/chassis_battery.proto`；使用 Protobuf 的未知字段兼容机制，不绑定完整厂家 SDK。

## 字段来源

| 原厂字段 | Driver 输出 | 处理 |
|---|---|---|
| `soc` | `soc_pct` | %，范围 0～100；0 有效 |
| `voltage` | `voltage_v` | 原始 mV / 1000 |
| `current` | `current_ma` | 原厂页面标注 mA，保留符号 |
| `min_temperature` / `max_temperature` | `temperature_min_c` / `temperature_max_c` | 当前 BMS 传感器温度范围，°C |
| `cycle_count` | `cycle_count` | 非负整数 |
| `min_cell_voltage` / `max_cell_voltage` | `cell_voltage_min_v` / `cell_voltage_max_v` | 原始值按 mV / 1000；拒绝倒置范围 |
| `rc` / `fcc` | `remaining_capacity_mah` / `full_charge_capacity_mah` | mAh，非负值 |
| `hardware_version` / `software_version` | BMS 版本字段 | 原样保留可打印字符串 |

单位核查来自原厂页面电池组件：总压除以 1000 显示 V，电流标注 mA，`max_temperature` 显示 °C，`fcc` 标注 mAh。`rc` 与 `fcc` 同尺度，其比例与实际 SOC 一致。单体电压按 mV 的尺度与原始数值和总压一致，尚缺厂家手册的显式定义。

## 有效性与显示

缺失、布尔值、非数值或非有限数值不补零。两个 SOC 均有效为 online，只有一个有效为 partial。默认 3 秒无有效电池报文即过期；其他报文不延长电池有效期。停止、断线或过期清空数据。

`received_at_ms` 记录 Driver 接收时间；底盘没有提供两块 BMS 的采集时刻。重复收到相同 SOC 不证明 BMS 内部更新正常。

原始电流曾持续报告为 0，只能说明报文如此，不证明物理电流为零。按当前 UI 要求隐藏电流；不依据其符号推断充电。`charge_arrive` 是充电感应光电，不作为正在充电的证据。没有验证过的充电状态、SOH 或设计容量不输出推断值。

前端展示每块电池当前帧最高传感器温度作为 `temperature`，并在 Details 标明来源。该值不是历史最高温度或整机温度。容量和 BMS 版本保留在 Details。Core 分支需单独评审。

## 验证

`tests/fixtures/helios/battery_records.json` 保留实际报文的电池字段，移除了无关底盘状态。测试运行时生成 Protobuf 信封并添加合成未知字段作为兼容性回归，不提交二进制采集文件。模拟 WebSocket 测试另核对只读行为、停流、断线重连及独立生命周期。

已有本机调试验证不代表 bot 镜像验收。正式提交、部署与验收要求见 README 和 VALIDATION.md。
