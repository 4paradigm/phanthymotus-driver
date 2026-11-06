# foot_pose：脚部姿态 Card 原型

提供 `feet_together`（并脚）、`feet_apart`（分脚）、`feet_staggered`（双脚前后错开）、`neutral`（中立）以及 `info`、`status`、`preview`、`stop`。
游戏中的石头、布、剪刀映射由 Skill 处理。

**当前没有已验证的腿部动作资产，真机脚姿功能尚未完成。默认仅返回预览，不下发动作。**

## 接入位置

- `foot_pose.py`：插件、资产校验、预览与原厂示教播放适配器。
- `main.py`：在现有 `BumiDeviceBundle` 中注册，复用 `loco` 的身体互斥与 `motion_state` 读数。
- `config.yaml`：默认启用卡片、`dry_run: true`。
- `driver.yaml`：卡片目录声明。
- `Dockerfile`：将插件复制到现有驱动镜像；复用仓库的 ROS 基础镜像和 SDK 构建流程。
- `resource/foot_pose/assets.example.json`：有意留空的资产清单模板。
- `tests/test_foot_pose.py`：无硬件行为检查和当前 Bundle 注册检查。

插件不提供裸关节命令、猜测关节角或低层运控备用路径。
只允许播放操作者独立验证过的原厂录制动作；不会因为存在录制编号，就认定它是指定脚姿。

## 当前可测试的行为

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
当前实现要求四种姿态使用独立录制编号，记录全部十二个腿部关节的终态、容差、最短观察时间和超时，且验证停止行为。这些是本原型的执行门槛，不是平台统一规定。

仅切换 `dry_run` 不会解锁：还需要有效资产、明确 `confirm`、新鲜遥测及静止状态。容器重启不保留真实输出授权。
播放模式只证明命令启动；插件持续观察腿部关节终态与静止状态后才确认姿态，且不声称直接测量了脚的位置。
`stop` 只处理本卡拥有的播放；无法确认退出时继续保留身体占用，不抢占其他任务。

## 开发与测试

从仓库根目录运行：

```bash
python -m pytest noetix/bumi/tests/test_foot_pose.py -q
```

测试使用明确标记的替身编号和遥测，不连接机器人。
早期电脑仿真仅通过了分别初始化后的静态姿态检查；迈步过渡、回中立与停止未通过。仿真控制器未接入此卡片。

后续按团队流程从功能分支开 PR，评论 `/request_bot_review` 获得镜像及部署指令。
真机预览接入与新腿部动作验收应分别记录，预览通过不能作为脚步能力验收通过。
