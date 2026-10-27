# S10 姿态和定时移动卡

本 PR 一并引入同事交接的 S10 基础驱动和两张运动卡，因为主仓库尚无 S10 目录。原有监控模块作为基础依赖保留；本次新增功能是以下两张运动卡。顶部四张汇总监控卡和画布清理不属于本次提交。

## 两张卡的使用方式

| 卡片 | 操作 | 前置状态 | 结果依据 |
| --- | --- | --- | --- |
| `motion` | `prepare` | 状态 0、1、17，连续静止且关节和故障检查通过 | 建立最长 60 分钟的内存会话，准备命令通道，不发送姿态或速度 |
| `motion` | `stand` | 静止且允许姿态转换 | 单次发送 `/MOTION_STATE` 1，等待新鲜状态 1 |
| `motion` | `lie` | 允许姿态转换 | 单次发送 `/MOTION_STATE` 4，等待完整卧倒后的最终状态 0 |
| `motion` | `move_mode` | 静止站定，状态 1 | 单次发送 `/MOTION_STATE` 17，等待状态 17 |
| `timed_move` | 仅选择 `direction=forward/backward` | 遥控器已切换导航使用模式，状态 17，起步前静止 | 以 `vx=±0.08m/s`、10Hz 发送 2 秒，再重复零速并等待停止反馈 |

`timed_move` 不显示 ACTION 字段。画布执行时只发送 direction，驱动按工具名确定本次操作；内部 info/start/stop/config 仍供框架使用。两个工具共享同一个控制器、任务历史、授权和互斥锁，不能同时发布物理控制。移动正在执行时，姿态请求返回 busy。

厂家状态机对 state17 → stand 的直接转换不响应，因此需要先 lie 完成到 state0，再 stand。进入 move_mode 不自动切换遥控器的常规／导航使用模式；当前没有可信的自动模式检测，导航模式是现场前提。

定时移动按运动速度反馈确认方向响应，并按新鲜、不同序列的静止帧确认停止。它不使用 /ODOM 计距，不承诺固定距离。非零命令发送成功本身不算动作完成，未观测到响应或停止时返回错误并闭锁。没有实现横移、转向、打招呼、SDK 模式或固件升级。

## 独立输出

默认命名空间 `s10_pro` 下：

| 卡片 | 状态输出 | 事件输出 |
| --- | --- | --- |
| `motion` | `/s10_pro/s10_pro/posture_status` | `/s10_pro/s10_pro/posture_events` |
| `timed_move` | `/s10_pro/s10_pro/timed_move_status` | `/s10_pro/s10_pro/timed_move_events` |

两张卡均没有输入 topic。状态按卡片过滤动作、事件和结果；共享占用由 shared_controller_busy 表示。不要把两张卡都接到原 motion_status/motion_events 地址。已有画布需要刷新工具定义并重新核对端口；旧 topic 的消费者需要迁移。

## 用 PR 镜像复现

1. 使用 review bot 为当前 PR 提交构建的镜像。先遵循原 README 部署 DDS 配置、厂商消息、驱动 config.yaml 和持久化 state 目录；机器人和 Core 的网络隔离配置保持原要求。
2. 在 NOS 主机运行下列脚本，生成本机独有的 prepare 策略和身份副本。只在现场条件属实时使用三个确认开关。默认不带参数为 dry run；脚本拒绝覆盖现有文件。现有实机策略须单独保留和核对，不复制到 Git。

   ```bash
   sudo python3 deep_robotics/s10_pro/deploy/init_prepare_policy.py \
     --apply --operator-present --handheld-estop-verified --no-competing-commands
   ```

3. 将 `deploy/service.supervised.yml` 的 volumes/environment 合并到 bot 生成的 `deep-robotics-s10-pro` 服务。两份 YAML 使用的是服务片段格式；若现有 Compose 顶层有 services，则把片段放到 services 下。默认主机文件路径为 `/opt/phanthy-motus/s10-pro`，其他安装根目录要同步调整挂载源路径。不得同时配置 S10_EXECUTION_AUTHORIZATION_FILE。
4. ACP 保留原已验证的 HTTPS 配置：凭据文件独立只读挂载到 `/opt/phanthy-motus/.env`，或以 S10_ACP_ENV_FILE 指定；使用系统信任链、S10_ACP_CA_FILE 或已核对的 S10_ACP_PINNED_LEAF_FILE。凭据、证书私钥和实机策略不进镜像或 PR。不要以禁用 TLS 校验代替配置。
5. 重建驱动容器后，确认 tools/list 同时包含 motion 和 timed_move，且输出 topic 不交叉。保存卡片配置（enabled=true），将两张卡加入画布并开启智能控制。新进程初始没有活动会话；选择 motion 的 prepare 后才激活授权。只在现场确认后执行物理动作。
6. 按 stand → move_mode → 前进2秒 → 确认停止 → 后退2秒 → 确认停止 → lie 的顺序逐项验收，保留遥控器接管能力。记录画布终态和现场表现；不自动重放失败动作。部署验收可先只做 tools/list/info 和 prepare，这些操作不发送运动指令。

当前使用的 forward_30cm/backward_30cm 是历史内部授权 scope 名称，保留是为了兼容已有策略与方向检查。对外只提供固定2秒动作，不能根据 scope 名称推断30厘米完成。

## 兼容与回滚

原有授权 v1、持久授权 v2 保留。v2 仍只允许原 stand/lie 范围，新功能推荐使用显式 prepare 策略。没有策略挂载时沿用原有默认关闭路径，不自动建立控制会话。

回滚时恢复上一镜像、服务挂载与对应画布端口，保留 motion-tasks.sqlite3/acp-outbox.sqlite3。重启不恢复物理动作，未完成请求会记录为失败。同一请求 ID 不得用于另一方向。撤销时需修改已挂载策略文件内容；单纯更换或删除宿主文件路径不保证正在运行的单文件 bind mount 失效。

## 验收状态

此前现场已确认站立、卧倒及 move_mode；导航模式下独立探针获得前进／后退运动反馈，用户报告画布站立和卧倒成功。定时卡界面和独立端口已经部署核对。本次 PR 整理保留了仓库旧授权兼容逻辑，产生了区别于当前实机容器的新候选版本。必须使用本 PR bot 镜像重新验收；不把本地测试或之前容器补丁测试当成新镜像已通过。
