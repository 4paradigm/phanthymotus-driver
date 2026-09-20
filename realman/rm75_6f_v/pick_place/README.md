# vision_pick_and_drop

RealMan Driver ACTUATOR 卡片，提供 `observe` 观察拍照、`grab_to` 指定目标点搬运和
`grab_by` 指定距离搬运。
接收三个数据输入，运动与夹爪由本卡片独立执行。启动、配置和页面刷新不会触发运动。

## 画布数据流

```text
ext_camera (depth) ─────────────→ vision_pick_and_drop 输入 1：深度图像
ext_camera (rgb) ─┬─────────────→ vision_pick_and_drop 输入 2：RGB 图像
                 └→ VOP ───────→ vision_pick_and_drop 输入 3：物品列表
大模型 ── MCP / ACP ────────────↔ vision_pick_and_drop
```

两个 `ext_camera` 实例须选择同一台 RealSense，分别配置 `rgb`、`depth` channel。
VOP 订阅同一 RGB 通道，检测结果连入 `vision_pick_and_drop`，由 `observe` 完成结果交给大模型。
`vision_pick_and_drop` 没有图像输出端口。框架启动时传入三条 `input_topics`，
卡片按 `/rgb`、`/depth`、`/rgb/objects` 识别来源，不依赖连线创建顺序。
`info.topic_in` 返回三个实际绑定主题及格式；`info.inputs` 返回数据就绪和错误状态。

## 执行契约

三个运动动作 `observe`、`grab_to`、`grab_by` 每次都必须显式传入
`confirm_motion: true`。缺失、`false` 或非布尔值均返回 `CONFIRMATION_REQUIRED`，
不会占用机械臂或下发运动。该确认不跨请求复用；`RM_MOTION_ENABLED` 仅控制部署能力。
`cancel`、`stop`、`info`、启动和配置不需要运动确认，两个中断钩子始终可调用。

动作通过输入检查并取得设备互斥后，立即返回 `state: "running"` 和唯一 `action_id`，
后台线程执行完整动作。输入 Schema 声明 `x-completion.actions` 为三个运动动作、
`timeout: 210`，并保留 `type: actuator`、`x-is-dangerous: true`、`x-resource: arm`。
框架通过 ACP 等待完成，不要求模型轮询或重复下发动作。
单次观察及搬运后的返回观察各有 45 秒预算，抓放全流程有 120 秒预算，单段运动等待最多 45 秒。
210 秒 ACP 等待预算覆盖抓放、可选观察、停止确认与回调重试，不是对底层 SDK 阻塞时间的保证。
一次搬运请求的确认同时包含返回观察位的运动，整段流程共用设备互斥和取消令牌，只有一个完成回调。

后台执行结束后，卡片向 `${AGENT_CORE_URL}/api/acp/complete` 上报同一 `action_id`，
`status` 为 `completed`、`error` 或 `cancelled`，`result` 包含动作结果及
`observation_required`。只有 `status=completed` 且 `result.ok=true` 才表示动作完成；
最初的 `running` 响应仅表示已接收。动作接收前的拒绝同步返回，不产生 ACP 待完成记录。

回调使用 `AGENT_CORE_CA_CERT` 指定的 CA 并保留 TLS 证书与主机名校验，缺失或无效的 CA
在接收动作前拒绝请求。服务部署已提供 Core 地址和 CA 挂载。
回调要求 Core 返回 `ok: true` 及相同 `action_id`；未确认、连接失败或 Core 尚未登记
pending 时，最多尝试 3 次，每次请求超时 5 秒，间隔 0.5 秒。只重发同一完成通知，
不重发设备命令。`info.last_result.callback` 为 `pending`、`accepted` 或 `failed`，
失败原因保存在 `callback_error`，不会改写已完成的物理动作结果。
Driver 继续检查连接、只读模式、设备互斥、故障、反馈和照片有效期，并支持取消。

## 大模型调用

MCP 总描述及各动作描述包含调用顺序、坐标约定、动作选择和照片有效期。
用户可以用“把香蕉向右移动 3 厘米”这样的任务指令，由模型通过框架完成：

1. 调用 `observe(confirm_motion=true)`，等待框架 ACP 通知本次观察成功。
2. 从本次 ACP 完成结果 `result.objects` 读取物品名称、`position` 和 `confidence`。
   `count=0` 是有效的空检测结果。目标不存在或无法确定时报告或询问用户，不猜测坐标。
3. 选取香蕉检测中心的 `position[0]、position[1]`，调用 `grab_by`，
   填写 `delta_x=30、delta_y=0、confirm_motion=true`。放到照片中的指定位置时使用 `grab_to`；
   放在另一物体旁边时选择旁边空位，参照物中心不代表空位。
4. 等待框架 ACP 完成通知。`status=completed` 且 `result.ok=true` 表示配置要求的动作链完成。
   开启 `observe_after_transfer` 时，读取 `result.observation` 中的新照片和检测结果评估效果；
   关闭时结果明确跳过观察，如需验证效果或再次搬运则单独调用 `observe`。
   `grasp_checked=false` 表示卡片没有自动核验是否抓到物体，卡片不自行判断任务成功或自动重试。

当 `result.observation.ok=true` 时，后续搬运可直接使用其中 `objects` 的新位置，无须再单独调用 `observe`。
动作结果中的 `observation_required=true` 表示没有有效新照片，**再次搬运前**需要重新观察。
持物或停稳状态需要处理时（`recovery_required=true`），先人工处理，不直接从头重试或重新观察。
其他情况下由调用方根据最新结果决定后续任务，不能把后续观察失败解释为物体没有移动。
模型可直接使用结构化检测结果，无需读取本地照片文件、计算像素或查询异步任务。

## 配置

通过卡片 `configSchema` 声明配置，由 `config` 请求下发。
部分更新保留其他值，校验失败不修改任何配置；动作执行期间拒绝配置更新。配置保存在当前进程内，
重启后恢复默认值；使用已保存的卡片配置需要 Core 再次下发。

| 配置 | 默认值 | 范围与含义 |
| --- | --- | --- |
| `speed_percent` | 50 | 整数 1～100，全局运动速度百分比 |
| `observation_joints_deg` | `-90,0,0,90,0,90,0` | 英文逗号分隔的 J1～J7 关节角，单位 °，按设备各关节限位校验 |
| `x_compensation_mm` | 30 | 有限数值，基坐标 X 绝对目标补偿，单位 mm |
| `y_compensation_mm` | -75 | 有限数值，基坐标 Y 绝对目标补偿，单位 mm |
| `pick_grip_force` | 15 | 整数 0～100，设备原始夹持力度，非牛顿值 |
| `pick_descent_mm` | 91 | 有限正数，抓取下降距离，单位 mm |
| `place_descent_mm` | 60 | 有限正数，放置下降距离，单位 mm |
| `observe_after_transfer` | false | 搬运后拍照的布尔开关；搬运后始终返回观察位，开启时再保存新照片并返回物品列表 |

下降距离是从本次下降起点向下移动的距离。
X/Y 补偿是独立的水平定位参数，作用于基坐标系中的目标位置。

## observe

调用 `vision_pick_and_drop` 时传入 `{"action": "observe", "confirm_motion": true}`。
接收后返回 `running` 和 `action_id`，后台完成运动、同步观察和结果保存，再通过 ACP 上报结果。
Driver 必须已连接并以 `live` 模式部署；只读部署会明确提示当前模式不可执行动作。

动作检查关节故障、使能状态、控制器限位及三路输入可用性，然后以配置速度下发一条
完整目标的 `rm_movej`。通过新鲜 SDK 查询确认控制器空闲、关节到位及位姿稳定后，
从外部数据流保存一组 RGB-D 和物品列表，再核验采集期间的位姿与坐标系。
全过程不操作夹爪，也不打开物理相机。

成功时 ACP 上报 `status: "completed"`，`result` 包含照片路径、观察编号、拍摄时间、尺寸、
元数据路径、`objects`、`count`、`objects_timestamp` 和 `synchronization`。
ACP 结果中的 `observation_required: false` 表示本次照片可用于一次搬运。
后台执行失败或取消时由 ACP 上报对应终态；并发调用在设备忙时被拒绝。
`info.last_result` 和 `info.observation` 可查看最近结果；正常调用流程等待框架 ACP 通知，无需轮询。

## 位置坐标

两个搬运动作的位置参数 `start_point_x、start_point_y、target_point_x、target_point_y` 都使用最近一次成功观察照片的
**中心归一化坐标**，数值范围 `[-1, 1]`，支持小数，照片中心为 `(0, 0)`。

| 轴 | −1 | 0 | +1 | 正方向 |
| --- | --- | --- | --- | --- |
| X | 照片左边缘 | 水平中心 | 照片右边缘 | 向右 |
| Y | 照片上边缘 | 垂直中心 | 照片下边缘 | 向下 |

与 VOP 的 `position` 定义一致：将同次观察的检测结果 `position[0]` 传给 `start_point_x`，
`position[1]` 传给 `start_point_y` 即可，不需要大模型读取照片文件或换算像素。
卡片按保存照片的实际宽高在内部换算：`pixel_x = round((x+1)*width/2)`、
`pixel_y = round((y+1)*height/2)`；右/下边缘取整到尺寸边界时使用最后一个像素。
例如 `1280×720` 照片中的 `(0.08, -0.079)` 对应像素 `(691, 332)`。
输入超出 `[-1,1]` 或不是有限数值时拒绝执行，不自动猜测输入是否为像素坐标。
此坐标约定也可供符合该数据契约的检测模型或人工填写使用。

## grab_to：指定目标点

使用最近一次成功观察（独立 `observe` 或搬运返回的 `observation`）中的四个归一化坐标调用：

```json
{"action": "grab_to", "start_point_x": -0.3, "start_point_y": 0.2, "target_point_x": 0.3, "target_point_y": 0.2, "confirm_motion": true}
```

参数顺序为 `start_point_x、start_point_y、target_point_x、target_point_y、rotation_deg、confirm_motion`。

`(start_point_x, start_point_y)` 为抓取物体的中心，`(target_point_x, target_point_y)` 为放置位置，均遵循上述位置坐标约定。
调用传入这四个位置参数及 `confirm_motion=true`，接收后返回 `action_id`，整次搬运由后台完成并通过 ACP 上报。

两点均使用最近一次成功观察的原始对齐深度和内参计算，并分别叠加配置的 X/Y 补偿，
得到绝对基坐标水平目标；B 点不相对 A 点累加位移。任一点越界或深度无效，均在运动前
拒绝执行，不用邻点或新拍深度替代。沿用当前安装方向：相机右对应基坐标 −X、图像下
对应 +Y；这是固定安装约定，不能代替手眼标定，改变相机安装方向后需重新适配。

## grab_by：指定距离（mm）

参数顺序为 `start_point_x、start_point_y、delta_x、delta_y、rotation_deg、confirm_motion`；`delta_x/delta_y` 单位均为毫米。

使用最新观察照片中物体中心的归一化坐标 `(start_point_x, start_point_y)` 指定抓取对象，
再给出从该抓取点出发的桌面位移 `delta_x、delta_y`。两个位移都要填写，支持小数，
若两个位移都为 0，须设置非零 `rotation_deg`，表示原地抓起旋转再放下；无须填写 `target_point_x、target_point_y`。例如，将该物体向照片左侧移动 30 mm：

```json
{"action": "grab_by", "start_point_x": 0.08, "start_point_y": -0.079, "delta_x": -30, "delta_y": 0, "confirm_motion": true}
```

位移 `delta_x、delta_y` 的方向以本次 `observe` 的照片为准，单位为 mm；它们是实际位移量，不是归一化坐标。
**X 正方向向右，Y 正方向向下；正值沿正方向，负值反向，0 表示该方向不移动。**

| 需求 | `delta_x` | `delta_y` |
| --- | --- | --- |
| 向右移动 30 mm | 30 | 0 |
| 向左移动 30 mm | -30 | 0 |
| 向照片下方移动 20 mm | 0 | 20 |
| 向照片上方移动 20 mm | 0 | -20 |
| 向左 30 mm、向照片上方 20 mm | -30 | -20 |

这里的“上/下”是桌面平面内的方向，**不是机械臂 Z 轴升降**。位移相对于物体的抓取点，
不是当前机械臂位置、绝对基坐标或像素增量。当前安装映射为基坐标
`ΔX = -delta_x`、`ΔY = delta_y`，放置点直接由 `B = A + (ΔX, ΔY)` 计算。
抓取点 A 使用原始深度和配置的 X/Y 补偿定位，B 不重复添加补偿；位移按毫米计算，
不换算为目标像素，不使用另一点的深度。工作坐标有旋转时，仍保持上述基坐标方向。

MCP 调用选择：需要放到照片中某个位置时使用 `grab_to`；需要将选中物体沿照片方向
移动指定毫米距离时使用 `grab_by`。两者都由后台完成抓起、搬运、放下及回升；
随后始终返回观察位；`observe_after_transfer` 只决定是否采集新观察，整次动作只通过一次 ACP 通知框架。

## 可选夹爪 Rz 旋转

`grab_to` 和 `grab_by` 均接受可选 `rotation_deg`，单位为度，范围 `[-180,180]`，省略时为 0。
**常规搬运省略这个参数；只有用户明确要求旋转时才设置。** 正值表示俯视顺时针，负值表示逆时针。
旋转围绕夹爪自身向下的 Z 轴，保持抓起回升后的 TCP 位置与高度，再保持旋转后的朝向搬运和放下。
工作坐标有旋转时合成完整姿态，不直接把角度加到工作坐标的欧拉 Rz，也不改变图像 XY 的方向约定。

例如，向照片右侧搬运 30 mm，并将物体顺时针旋转 45°：

```json
{"action": "grab_by", "start_point_x": 0.08, "start_point_y": -0.079, "delta_x": 30, "delta_y": 0, "rotation_deg": 45, "confirm_motion": true}
```

旋转只在抓起并回升之后执行，到位并停稳后才移动到放置点。0 不下发旋转命令；旋转使用配置速度，
保留姿态、固定位置、夹爪、故障和取消检查。失败时返回 `stage=rotate_gripper` 及可能持物状态，不自动放下或重试。
结果包含请求角度 `rotation_deg` 和旋转到位标志 `rotation_completed`，省略/0 时后者为 false。
该标志表示机械臂姿态到位，不表示已验证物体实际旋转角度。

## 共用搬运流程

两种搬运动作使用相同的配置和抓放流程：

1. 保持观察高度和姿态，水平移动至 A。
2. 以力度 100 张爪到位，再设置配置的抓取力度。
3. 按配置的抓取下降距离下降，闭合后等待 1 秒，回到下降前保存的高度。
4. 仅在 `rotation_deg` 非零时绕夹爪 Rz 旋转并确认停稳，再保持高度、当前姿态和夹持状态水平移动至 B。
5. 按配置的放置下降距离下降，以力度 100 张爪到位，回到保存的高度，再闭合空爪。
6. 始终按配置的观察关节角和速度返回观察位并确认停稳。
7. 仅在 `observe_after_transfer=true` 时，保存一张新照片、深度及物品列表，刷新当前有效观察；关闭时不拍照。

抓放使用六段直线运动；请求旋转时增加一段固定 TCP 位置的姿态运动，每段仅下发一条 `rm_movel`；随后以一条 `rm_movej` 返回观察位，全部使用 `speed_percent`；
垂直距离直接使用 `pick_descent_mm` 和 `place_descent_mm`，工作坐标有旋转时仍按
基坐标水平/竖直方向计算。闭合不要求实际开度为 0，也不以检测到物体作为回升条件。
夹持力度只写寄存器 1220，以交替长度的回读确认，不写驱动力寄存器或 Flash。

全过程检查新鲜反馈、故障、坐标系、姿态、行程及夹爪状态，并确认每段到位和停稳。
成功时 ACP 上报 `status: "completed"`，结果包含使用的观察编号、内部实际读取深度的抓取像素
`pick_pixel` 和两处基坐标水平目标、最终位姿。`grab_to` 还返回放置像素 `place_pixel`；
这些结果字段仍是原图整数像素，与动作输入的归一化坐标区分。`grab_by` 返回 `delta_x、delta_y` 和方向基准
`direction_reference: "observation_image"`。`grasp_checked: false` 表示未判断是否实际
抓到物体。`final_pose` 是整次动作经核验的最终位姿：两种开关状态下都在观察位。

搬运结果还包含 `transfer_completed` 和 `observation`：

- `transfer_completed=true`：抓放及回升动作链完成，不自动代表物体已成功搬运。
- `return_completed=true`：已按配置返回观察位并确认停稳。回程不依赖相机或 VOP 输入是否可用；开启拍照而输入不可用时，先回到观察位，再报告观察失败。
- `observe_after_transfer`：本次实际使用的配置；关闭时返回 `observation={"skipped":true,"reason":"disabled"}`、
  `observation_required=true`，不是拍照失败。下一次搬运前须先 `observe`。
- `observation.ok=true`：已返回观察位并刷新观察，包含 `observation_id`、`captured_at`、
  `file_path`、`metadata_path`、`pose`、`objects/count` 和同步信息；结构与独立 `observe` 的结果一致。
- 顶层 `observation_id` 标识本次搬运使用的旧照片；`observation.observation_id` 标识动作后的新照片。
  新照片同时成为 `info.observation`，`observation_required=false`，可供下一次搬运。
- 抓放已完成而返回观察位或采集失败时，终态为 `error/cancelled`、`ok=false`、
  `transfer_completed=true`、`observation.ok=false`、`observation_required=true`，保留抓放结果及失败原因，
  不返回未经确认的最终位姿。抓放中途失败或取消时不再执行返回与拍照，`transfer_completed=false`。
- `stage/last_completed_stage`：当前或失败阶段、最近已完成阶段。`holding_object_possible=true` 表示抓取闭合可能已执行，
  放置张开尚未确认；`release_completed=true` 表示已在放置点确认张爪，不表示视觉核验成功。
  两字段用于区分尚未抓取、可能持物、已放下但回升或观察失败等情况。
- `recovery_required=true`：可能仍持物或无法确认停稳，须人工确认并安全处理后再开始任务，
  大模型不要直接重新观察或从头搬运。结果中的 `recovery_message` 解释待处理状态。

抓放全流程有 120 秒预算，每段运动仍最多等待 45 秒，单次观察或搬运后观察另有 45 秒预算；
ACP 等待预算为 210 秒，包含停止和通知余量。各预算均有限，持续异常不会无限等待或触发动作重试。
后台通过一次 ACP 上报整段流程的终态，不占用原 MCP 请求等待。

## 扰动与流程连续性

SDK 的分次读取短暂不一致、单次慢反馈，以及轻微偏移不会直接结束抓放。
这类条件只重读反馈，最多等待 2 秒，恢复后须连续通过 0.3 秒核验，再继续当前步骤和后续放下流程。
已提交的运动、夹爪开合和力度写入不重发，不从头重新抓取；每段绝对目标保持原值，不把扰动累加进下一步。
夹爪动作和下一段运动都必须等到原有到位条件重新满足后才能下发。

停稳位置的到位容差仍为 1 mm；1～2 mm 的短暂偏移只允许等待自行恢复，不视为到位。
水平高度和垂直高度区间超出 2 mm、垂直运动 XY 超出 2 mm、姿态超出 1°、
坐标系变化、SDK 命令失败、关节/夹爪故障及失能不作为可恢复噪声。
持续偏移超过等待预算、总动作超时或取消同样停止后续动作。
`feedback_recoveries` 记录抓放阶段成功恢复的反馈等待次数，便于定位实际表现。

抓取闭合下发前保守标记可能持物，只有放置点张开到位确认后才清除此标记。
因此故障结果不会把“物体可能已拿起”误报为“尚未移动”，放下后的回升失败也不会误报为仍持物。
真实故障或未知位置下不能无条件继续放下，也不会任意张爪丢落物体；此时明确交付中间状态和人工处理提示。

## RGB-D 对齐与单次观察

`ext_camera` 的 RGB 输出仍为 JPEG，深度仍为 `16UC1; compressedDepth zlib`：
640×480、小端 uint16、毫米单位，0 表示无效。USB 3 配置下彩色原生采集为 1280×720，
深度原生采集为 640×480；USB 2 下两者均为 640×480。两者由不同成像器采集，
分辨率相同也不代表像素对应。没有把深度简单缩放到 RGB 尺寸。

深度主题附带 `<depth_topic>/metadata` 的 `std_msgs/String` JSON，包含
`version=2`、相机序列号、采集会话 ID、RGB 来源主题、两帧采集时间（ns）、
两组内参、深度到彩色的外参和线上深度单位 `depth_scale_m=0.001`。
RGB、深度和红外图像保持发布时钟 `header.stamp` 和 `{namespace}_{stream}_optical` 的 `frame_id`。
旁路消息中的 `rgb_header_stamp_ns/depth_header_stamp_ns`、`rgb_frame_id/depth_frame_id`
精确关联对应图像头；`rgb_stamp_ns/depth_stamp_ns` 是同步到主机时钟的真实采集时间，
用于停稳后的新鲜度检查及结果中的 `captured_at`，不能用发布或接收时间替代。
相机会话及标定仅随旁路消息提供，不增加画布端口，不改变现有图像主题、编码、尺寸或头字段语义。
标定、时间或来源不匹配时拒绝观察；相机预览仍可独立使用。

采集端通过独立的 `realsense_metadata.py` 发布通用元数据，不依赖任何抓取卡片。
元数据在图像发布后处理；发布器创建、销毁、标定读取、序列化或发送失败仅记录
`rgbd_error`，不触发相机重连或停止图像。旁路发布器创建/销毁失败每秒重试一次，
其他正常旁路继续发布。没有旁路订阅者或没有同时发布 RGB/depth 时，不读取标定或序列化元数据。
标定按采集会话与 SDK 流配置缓存；重连或配置变化后重新读取，采集时间与图像头仍逐帧提供。
时间同步在每个采集会话中尽力设置一次；采集端只检查时间域和数值，照片新鲜度由本卡片判断。
原有图像主题、编码、分辨率、帧率、QoS 和图像头语义保持不变。

`vision_pick_and_drop` 根据上述标定，用官方 RealSense SDK 的 `software_device` 接收数据，
再执行 `align(color)`，得到与 RGB 同尺寸、同坐标的深度。该操作不打开物理相机。
保留原始有效/无效测量，不填洞、不取邻点替代。已有深度编码以毫米量化；
SDK 设备单位为 0.001 m 时无需单位量化转换，其他设备单位遵循毫米编码精度。

同步过程要求机械臂始终停稳：运动完成后保留 0.15 秒缓冲，
使用不少于 0.6 秒的静止窗口；RGB 和深度通过各自消息头与标定精确匹配，并校验来源及会话，
两者采集时差不超过 0.2 秒。VOP 提供完成时间和推理耗时，尚未提供原始图像帧号，
因此排掉第一份停稳后的在途结果，再使用后续推理结果，并选择推理时刻附近的 RGB-D。
窗口内持续比较图像；画面变化、轻微位姿扰动或短暂输入延迟会废弃本轮候选 RGB-D/VOP，
重新等待稳定窗口和新的检测结果，不复用变化前的数据。来源/标定错误立即拒绝；持续变化或无新输入超过 12 秒则观察失败。
`synchronization.window_restarts` 记录本轮同步窗口重置次数，观察整体仍受 45 秒预算约束。
结果用 `synchronization.mode=stationary_window` 明确表示静止窗口关联，不声称 VOP 与 RGB
严格同帧。若要支持运动场景中的严格逐帧关联，检测输出还需携带原始图像时间戳/帧号。

每次成功观察只保存一张接收到的 JPEG、一份对齐深度及一个物品列表快照。
照片、深度、彩色内参、源相机标定和时间、拍照位姿、工作/工具坐标系及配置一起保存在
持久目录的 `<观察编号>/` 中。后台输入流不会覆盖有效照片，也不会连续向大模型发送图像或物品列表。
持久目录由 `config.yaml` 的 `vision_pick_and_drop.output_dir` 设置，默认路径为
`/opt/phanthy-motus/data/pick_place/realman`，使用独立数据卷。

## 生命周期与设备协调

`cancel` 请求中止，`stop` 中止并释放本卡片的资源；不自动回程、释放夹爪或重试动作。
失败或取消不产生可搬运的观察结果。若无法确认机械臂已停稳，会保留设备互斥并报告
`motion_blocked`；检查设备后重启 Driver 才能解除该阻塞。
重新观察、配置变化、输入重新绑定、相机采集会话变化、停止或进程重启会使当前观察结果失效；已保存文件仍保留。
任一搬运动作首次下发设备命令即使当前照片失效，命令报错或取消也不恢复旧照片。
开启搬运后观察且新观察成功时，以新观察替换当前有效照片；关闭或新观察未完成时，下一次搬运须先 `observe`。
若失败结果同时返回 `recovery_required=true`，先人工确认并安全处理，再重新观察。
旧照片在同一设备互斥下被消费，不能通过切换 `grab_to` / `grab_by` 复用，
也不会从已保存文件恢复。历史文件保留，当前有效照片仅指向最新成功观察。
ACP 终态结果及 `info` 中的 `observation_required` 表示下一次搬运是否需要重新观察。
无有效照片时，搬运直接返回 `state: "error"`、`code: "OBSERVATION_REQUIRED"`、
`observation_required: true`，不会下发设备命令。
参数错误且尚未下发设备命令时保留照片，可修正参数；外部移动物体后也应重新观察。
有效期保障针对卡片持有的照片；重新观察后，调用者仍须使用新照片的检测坐标，
卡片无法仅凭四个数值判断调用者是否沿用了历史检测结果。

本卡片不导入或调用关节控制、夹爪、servo、ext_camera、vision_capture 或 VOP 的功能实现。
相机采集由 `ext_camera` 管理；`vision_pick_and_drop` 仅依赖三路消息及深度附带标定的公开数据契约，
自行完成同步、深度对齐、坐标转换和运动。停用 `vision_pick_and_drop` 只停止自己的订阅，
不会停用相机或影响其他消费者。相机或 VOP 未提供可用输入时，观察在运动前拒绝；
搬运使用已保存的 RGB-D，不因外部输入短暂延迟中止抓放；开启的后续观察会等待新输入，
持续缺失则独立报告观察失败并保留已完成的抓放结果。

机械臂 SDK 连接由公共 `hardware.py` 管理，生命周期独立于所有卡片。
本卡片持有设备独占权限期间，SDK 层拒绝其他卡片的控制命令。
其他机械臂卡片、VOP 与 Core 的实现不需要修改。

启动输入订阅时不持有动作锁，停止可以取消仍在初始化的订阅；迟到的节点会被销毁，
旧订阅回调按生命周期编号丢弃。启动或停止期间拒绝新的动作及重复启动，不改变其他卡片的订阅。
