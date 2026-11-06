# foot_pose：脚步基础 Card

提供 `read_feet`（读取双脚位置估计）、`check_pose`（比对已有脚姿）、`reload_assets`（重载动作清单）以及 `info`、`status`、`preview`、`stop`。
保留 `feet_together`（并脚）、`feet_apart`（分脚）、`feet_staggered`（双脚前后错开）、`neutral`（中立）的录制动作适配接口。
游戏中的石头、布、剪刀映射由 Skill 处理。

**已完成可独立测试的读取、几何估计、脚姿比对与逐个动作载入。当前没有已验证的腿部动作资产，三个真机脚姿动作尚未完成；动作请求默认仅预览。**

## 现在就能在电脑测试

无需连接机器人、安装 ROS 或加载 SDK。在仓库根目录运行：

```powershell
cd D:\phanthymotus-driver
python -X utf8 noetix/bumi/foot_pose_state.py --demo
```

输出带有 `input_kind: synthetic_demo`、`offline: true`、`command_sent: false`。
这是零关节角的合成输入，用于检查几何计算；计算出的横向中心间距为 0.199 m，不能作为机器人当前脚距或实际姿态。

也可对已导出的 `motion_state` JSON 快照运行 `--sample <JSON文件路径>`。文件应包含全部十二个腿部关节的 `motor_id`、`position`（弧度）；顺序不限，可同时包含手臂关节。
离线文件始终不作为实时遥测证据。缺失、重复或无效数值会报错。

## 前端基本操作

安装新驱动镜像并刷新工具列表后，把 `foot_pose` 拖入画布：

1. 保持“只预览，不执行动作”为 **Yes**，`confirm` 为 **No**，资产路径保留默认值。
2. ACTION 选 `read_feet`，点击执行。不需要动作资产，也不需要关闭空跑。
3. 查看 `feet.left/right.sole_center_m`、`separation.lateral_width_m`（横向间距）、`fore_aft_offset_m`（前后差）和 `height_difference_m`（高差），单位均为米。
4. `fore_aft_aligned` 仅按主仓库文档的 4 cm 前后差判断对齐，不表示双脚靠拢、着地、稳定或适合执行动作。
5. 已有经过验证的资产时，执行 `reload_assets`，再选 `check_pose` 并填写 `pose`，可只读比对关节终态。`joint_signature_matches` 不会被报告为真机动作完成。

位置来自主仓库 `leg_odometry.py` 的 URDF 正向运动学，坐标系为 `base_link`，属于模型估计。
SDK 读取可能返回缓存；输出如实给出 `accuracy_validated: false`、`floor_contact_verified: false` 和来源新鲜度，不能据此自动认定平衡或安全。

## 接入位置

- `foot_pose.py`：插件、资产校验、预览与原厂示教播放适配器。
- `foot_pose_state.py`：无 ROS/SDK/网络依赖的几何计算与离线命令。
- `main.py`：在现有 `BumiDeviceBundle` 中注册，复用 `loco` 的身体互斥与 `motion_state` 读数。
- `config.yaml`：默认启用卡片、`dry_run: true`。
- `driver.yaml`：卡片目录声明。
- `Dockerfile`：将插件复制到现有驱动镜像；复用仓库的 ROS 基础镜像和 SDK 构建流程。
- `resource/foot_pose/assets.example.json`：有意留空的资产清单模板。
- `tests/test_foot_pose.py`：无硬件行为检查和当前 Bundle 注册检查。
- `tests/test_foot_pose_state.py`：坐标、输入校验、来源时效和离线计算检查。

插件不提供裸关节命令、猜测关节角或低层运控备用路径。
只允许播放操作者独立验证过的原厂录制动作；不会因为存在录制编号，就认定它是指定脚姿。

## 姿态动作预览

部署后，将 `foot_pose` 拖入画布，先执行 `info`。没有有效资产时，应看到：

```json
{"state":"preview_only","dry_run":true,"physical_execution_ready":false,"available_poses":[]}
```

选择四个姿态动作或 `preview`，结果必须明确包含：

```json
{"state":"preview","command_sent":false,"pose_confirmed":false}
```

这是动作定义预览，不是物理仿真或真机动作成功。

## 真实执行边界

默认资产路径为 `/opt/phanthy-motus/data/foot_pose/assets.json`，由现有部署的数据卷保存。
目前模板中的 `poses` 为空；无需创建资产文件也能运行预览。

要使用真实模式，仍需确认本机原厂示教能录入并稳定回放目标腿部动作，并取得现场验证记录。
清单现在允许逐个载入姿态，不再要求四个一起提供。每个条目仍需独立录制编号、全部十二个腿部关节的终态、容差、最短观察时间、超时与停止验证记录。
真实执行还要求存在已验证的 `neutral` 资产，供动作开发完成后恢复使用。这些是本原型的执行门槛，不是平台统一规定。

仅切换 `dry_run` 不会解锁：还需要有效资产、明确 `confirm`、可验证的新鲜遥测及静止状态。容器重启或 `reload_assets` 不保留真实输出授权；重载文件无效时也保持关闭。
核对主仓库后确认，当前厂商 Python SDK 没有旧原型依赖的 `subscribe_robot_hardware_status`，也没有暴露源接收时间。
已移除这个不存在的调用。只读观察使用真实 getter；缓存轮询不充当源新鲜度证明，当前生产接入继续阻止真实播放。
播放模式只证明命令启动；插件持续观察腿部关节终态与静止状态后才确认姿态，且不声称直接测量了脚的位置。
`stop` 只处理本卡拥有的播放；无法确认退出时继续保留身体占用，不抢占其他任务。

## 开发与测试

从仓库根目录运行：

```bash
python -m pytest noetix/bumi/tests/test_foot_pose.py noetix/bumi/tests/test_foot_pose_state.py -q
```

测试使用明确标记的替身编号和遥测，不连接机器人。
早期电脑仿真仅通过了分别初始化后的静态姿态检查；迈步过渡、回中立与停止未通过。仿真控制器未接入此卡片。

后续按团队流程从功能分支开 PR，评论 `/request_bot_review` 获得镜像及部署指令。
真机预览接入与新腿部动作验收应分别记录，预览通过不能作为脚步能力验收通过。

复用依据与厂商接口边界见 [FOOT_POSE_REUSE.md](FOOT_POSE_REUSE.md)。
