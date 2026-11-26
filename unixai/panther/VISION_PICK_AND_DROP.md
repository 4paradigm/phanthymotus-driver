# Panther `vision_pick_and_drop`

此执行卡使用独立名称，可与 `tabletop_pick` 并存。它移植了 [RealMan 视觉抓放卡片](https://github.com/4paradigm/phanthymotus-driver/tree/main/realman/rm75_6f_v/pick_place) 的桌面流程：移动到观察位、保存单次观察、抓起、水平搬运、放下并返回观察位。物体检测由 VOP 提供，Panther 的相机、机械臂和夹爪沿用现有驱动。

## 画布连线

```text
Panther camera_depth ──────────────────→ vision_pick_and_drop / 深度
Panther camera_color ────────────┬────→ vision_pick_and_drop / RGB
                                └─→ VOP ─→ vision_pick_and_drop / 物品列表
```

VOP 必须订阅同一张 Panther RGB 卡，并发布其 `/rgb/objects` 子主题。卡片直接复用现有 Gemini 2 卡片后的同光轴、已校正帧和内参，在测量时间通过 TF 把光轴坐标转换到配置的 `base_frame`。当前不使用 CAD 安装估算作为运动标定，也不使用最近一次 TF 代替测量时间 TF。VOP 没有原始帧号，静止窗口只能近似匹配 RGB、深度和检测结果。观察快照只存在内存中，30 秒后失效；每次接收搬运动作时消耗一次。

## 动作

| 动作 | 含义 |
|---|---|
| `start` / `info` | 绑定上图三路数据 / 读取状态与观察摘要 |
| `observe` | 到已标定的观察关节位，停稳后取得快照；返回物品列表 |
| `grab_to` | 使用同次观察的 `start_point_x/y` 和 `target_point_x/y`，完成抓、搬、放、返回 |
| `grab_by` | 使用同次观察的 `start_point_x/y`，沿图像右/下方向水平移动 `delta_x/y` 毫米 |
| `cancel` / `stop` | 取消当前动作 / 取消并停止输入订阅 |

照片坐标以图像中心为原点、范围 `[-1,1]`，X 向右、Y 向下，与 VOP `position` 对应。搬运可指定 `rotation_deg`，默认不转。三个运动动作均要求请求携带 `confirm_motion: true`。初始 `running` 只表示已接收；最终结果按动作 ID 通过 ACP 上报。`grasp_checked: false` 表示夹爪反馈稳定和搬运动作完成都不等于已验证物体抓取成功。

## 装机配置与验证

部署含此卡的新 Panther 驱动镜像后，画布会发现这张卡，默认 `allow_motion: false`。项目镜像包含隔离的 Pink/Pinocchio 环境。将实测参数写入 `/opt/phanthy-motus/data/panther/vision_pick_and_drop.json`，重启驱动后生效。参数如下：

| 参数 | 要填的值 |
|---|---|
| `side` / `base_frame` / `tcp_frame` | `left` 或 `right`；固定基座系（默认 `base_link`）；实机 URDF 中经校验的夹爪 TCP，观察位须使 TCP 的 +Z 向下 |
| `observation_joints_deg` | 对应侧按 SDK 的 0–7 顺序填写八个**实测**关节角，单位度 |
| `workspace_min_m` / `workspace_max_m` | 相对于固定基座轴的 TCP 容许范围，各三个值，单位米 |
| `pick_descent_m` / `place_descent_m` | 从观察高度出发的垂直下降距离，单位米 |
| `gripper_open_feedback` / `gripper_feedback_tolerance` | 实机夹爪全开时报告的位置及容差；ROS 反馈单位，非命令的归一化开度 |
| `xy_compensation_m` / `observe_after_transfer` | 可选的基座 XY 补偿（默认 `[0,0]`）；完成搬运后是否自动更新观察（默认否） |
| `allow_motion` | 标定和轨迹验证后才设置为 `true` |

规划用 Pink 和 `PositionBarrier`，分段检查 TCP 工作区与关节限位，使用已有关节 P2P 接口等待各段结束。屏障只限制 TCP，采样也不证明机器人全身或夹持物无碰撞；本卡按静态开阔桌面的垂直下探/水平搬运设计。货架的避障、侧向接近、盒体朝向与夹持宽度搜索另需规划。夹爪位置反馈用于判定稳定，对物体在爪中没有独立验证。报告 `recovery_required` 时，先处理可能持物或未停稳的设备状态，再重启驱动解除该卡的阻塞。

部署验收先核对 `info`、彩色与深度对齐、观测位、TCP 与相机 TF，再使用静态无遮挡桌面验证空载轨迹和放置。不要沿用 RealMan 的关节角、相机坐标补偿或夹爪参数。
