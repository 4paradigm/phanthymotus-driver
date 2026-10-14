# 机械臂控制卡 arm_control

中文“选择操作＋执行按钮”布局；动作及对应参数组织参考 Q5 `arm_control`。
复用 PiPER 既有 runtime、CAN 连接、速度/限位、一次性预览与 ACP 完成回报。

| 操作 | 参数与行为 |
|---|---|
| `read` | 读取实际状态，不发送控制指令；按需建立本 runtime 的连接 |
| `allow_motion` / `block_motion` | 开启/关闭本次操作许可，不使能；关闭可取消进行中的控制 |
| `switch_mode` | `target_mode=standby|can`；CAN 要求确认其他控制器已停止；仅切模式 |
| `enable` / `disable` | `object=arm|gripper`；两类电机分别操作；失能需 supported=true |
| `jog_positive` / `jog_negative` | `joint=1..6`，`step_deg=0.5|1|2`，默认0.5° |
| `move_joint` | `joint=1..6` 与绝对 `target_deg` |
| `gripper_open` / `gripper_close` / `gripper_set_width` | 夹爪须已使能；指定开度用 width_mm |
| `halt` | `target=all|arm|gripper`，绕过动作锁取消对应动作并尽力保持，不失能 |
| `cancel_preview` | 清除尚未执行的预览令牌，不移动 |
| `start` / `stop` / `info` | 被动生命周期；停止控制卡撤销本次人工许可并请求取消，不释放力矩 |

模式、使能和许可独立。控制卡不提供自动初始化、末端 XYZ 移动、软件示教。
默认关节速度1°/秒、SDK上限5%；保留0.5–5°/秒、SDK1–10%与单次12°校验。
夹爪力矩默认0.5 N·m，范围0.001–1 N·m；开度沿用SDK与部署限位交集。

模式、使能、关节和夹爪动作沿用现有 `x-confirmation`：先请求 `execute=false` 预览，
展示目标/变化量，人工确认后使用同一参数和 `preview_id` 执行。
微动预览保存六轴完整绝对目标；执行消费该目标，实际反馈轻微变化时也不会重新累计步长。
超过0.5°的姿态变化、模式/使能变化、60秒过期、取消、停止、重连、新预览和重复消费均使旧令牌失效。
直接 MCP 执行还需 `execute=true`, `workspace_clear=true`, `take_control=true`。
使能边界修正需单独确认 `allow_limit_adjustment`。关闭确认对话框不发送执行请求；
如需明确撤销手工预览，调用 `cancel_preview`。

长期动作返回 `action_id`，最终状态经 ACP 报 completed/error/cancelled，工具名为 `arm_control`。
重复点击在占用动作锁时拒绝，不排队。`info.snapshot.last_actions` 和 `read.completed_actions`
可查看最终结果；`running` 只表示已接受请求。

相机 `piper_photo` 保留，需本次拍摄确认。旧十卡实现与回归测试保留，对外注册已替换。
监控回调约定见 [MONITOR_SNAPSHOT.md](MONITOR_SNAPSHOT.md)，布局集成见 [INTEGRATION.md](INTEGRATION.md)。
