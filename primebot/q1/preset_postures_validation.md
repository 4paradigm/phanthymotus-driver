# preset_postures 开发与部署记录

日期：2026-11-27。开发基线：PR #584 的 `16e578f7`，分支 `feat/q1-extension`。

## 接口与卡片

- 对照 Bumi `semantic_action` 的 `actuator`、单实例、`action` 枚举和
  `x-action-params`，提供标准动作下拉框与执行按钮。
- 接入 16 个公开 Q1 上肢动作、16 个公开全身动作、`reset` 和 `status`，
  共 34 个选项。动作名称与编号已与官方接口说明、v1.1.8.0 消息包及
  机身 Q1 运控配置交叉核对。
- 不开放任意模型、轨迹路径或未经公开文档确认的动作。
- 启动/停止画布不下发运动命令。执行前检查模式、切换就绪和新鲜的直立 IMU；
  切换后再次核对状态才下发动作。并发请求不排队；丢失响应不重试。
- 服务回执显示 `accepted`，不伪造物理完成。`reset` 不是急停。
- 系统 `info` 立即返回有年龄标记的缓存；用户显式选择 `status` 刷新真机状态。

## 自动验证

在机身 ARM64 / Ubuntu 22.04 / ROS 2 Humble 容器内执行：

```text
python3 -m unittest discover -s primebot/q1/tests -p 'test_*.py' -v
Ran 20 tests in 25.398s
OK
```

运行前加载 ROS 和消息包环境，设置 `Q1_RUN_DDS_TESTS=1`，并将仓库加入
`PYTHONPATH`（保留已加载的 ROS 路径）。完整命令见 README。

覆盖范围：

- 32 个动作的接口映射、模式切换顺序、复位和生命周期行为。
- 非站立/保护/切换中拒绝、取消后不再发动作、模式被改变时中止后续调度。
- `header.code` 与 `state` 联合判断、超时不重试、接受回执不表示动作完成。
- Core 高频 `info` 查询不创建 DDS 进程。
- Docker 构建上下文包含全部 `COPY` 所需文件。
- 真实 `aimdk_msgs` 编解码与 RPC 往返：使用固定隔离域 **93** 的模拟服务，
  核对挥手、复位、全身动作的字段、触发源和模式命令。
- 原电池卡片回归：隔离域 **87/88** 的启动、数据转换、过期与停止/恢复。

这些运动 RPC 测试均在模拟 DDS 域执行，没有向真机域 0 下发运动命令。

## 机身部署与只读检查

机身镜像：`primebot-q1:preset-postures-20261127`。
镜像 ID：`ec174eb16e2669adc5df615f160d2ec535282262b36856d5477d6e40dc6a7895`。

此次部署在机身原有定制镜像上叠加新卡片，保留原有
`battery/imu/joints/model/mic/asr`，沿用网络、挂载、环境变量与重启策略。
原容器已停止并保留用于回退。标准仓库构建沿用 #584 基线，包含
`battery` 与 `preset_postures`；机身已有的其他定制卡片由原镜像提供。

- `tools/list` 返回 7 张卡片，其中 `preset_postures` 为 actuator，34 个动作选项。
- 驱动日志确认注册到 Core；最终检查期间无 Traceback。
- 20 次 `info` 查询均成功，最大耗时约 111 ms。
- 电池读数为 `live`。
- 真机只读 `status` 查询：四项服务均可访问；当前模式为 `BIPED_WALK_RUN`，
  `status=100`、`switch_ready=false`、`standing=false`，卡片准确返回 `not_ready`。
  两个请求发送标志均为 false，上次运动请求为空。

部署文件与分支源文件的 SHA-256 一致：

| 文件 | SHA-256 |
| --- | --- |
| preset_postures.py | `7e8be8b77cdd22681f8e097e704a09e51310ce2e84726a8511d606eafda7036e` |
| preset_robot.py | `090c42537b5ff4d0696e96558ae60c3e12cacdd5acedf520a0a1b058b0e6311d` |

## 待现场验收

按用户要求，未执行真机动作。尚未验证每个动作的实际轨迹、完成时机及固件复位表现，
也没有登录用户的画布进行视觉验收。操作者需先确认稳定站立、动作空间和官方急停条件，
通过 `status` 查看前置状态后，再逐项验收。

本阶段仅推送个人 fork，主仓库 PR 待用户验收后另行提交。
