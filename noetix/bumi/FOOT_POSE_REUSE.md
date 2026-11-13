# 主仓库复用核查

核查日期：2026-11-06。通过 GitHub 核对主仓库 `main`：`489fe58cd68b5229b1cf90b4b836bbe9d4e79ffe`，与本地分支的主仓库基线相同。

| 已查看的实现 | 可复用部分 | 边界 |
|---|---|---|
| [Bumi leg_odometry.py](https://github.com/4paradigm/phanthymotus-driver/blob/489fe58cd68b5229b1cf90b4b836bbe9d4e79ffe/noetix/bumi/leg_odometry.py) | `LEG_CHAINS`、`JOINT_INDEX`、`foot_geometry`；直接 import，无复制参数表 | 从关节位置估算脚底中心、脚跟与脚尖；不是接触或平衡测量 |
| [Bumi device.py](https://github.com/4paradigm/phanthymotus-driver/blob/489fe58cd68b5229b1cf90b4b836bbe9d4e79ffe/noetix/bumi/device.py) | 当前 `motion_state` 读取、`loco` 身体占用、单次原厂播放与停止、ACP 通知 | 预设动作与速度控制不等于脚位目标控制；播放开始不等于姿态到位 |
| [TRON2 reset.py](https://github.com/4paradigm/phanthymotus-driver/blob/489fe58cd68b5229b1cf90b4b836bbe9d4e79ffe/limx/tron2/reset.py) | 参考“缺资产明确报错，配置和执行分离”的设计 | 它使用另一种机器人的机械臂 MoveJ，不能用于 Bumi 的腿部平衡 |
| [common/motion/worker.py](https://github.com/4paradigm/phanthymotus-driver/blob/489fe58cd68b5229b1cf90b4b836bbe9d4e79ffe/common/motion/worker.py) | 已检查公共数值求解进程及超时隔离 | 仅数值计算，不提供 Bumi 的行走或平衡策略；本次无需引入 |

## 收步文档与实际实现的区别

`IMU_DISTANCE_TEST.md` 记录过“前后差不超过 0.04 m 时收步”的实验方案。
当前 `leg_odometry.py` 能计算 `foot_gap` 和运动学双支撑估计；当前 `imu_distance_control.py` 没有可独立调用的 `parallel_finish` 动作。
高层控制器收到零速后也可能继续完成步态。不能把历史收步说明直接当作可靠的并脚、分脚或前后错脚动作。
本次只将 0.04 m 用作只读几何输出中的前后对齐参考值，不启动收步或行走。

## 修正旧原型中的 SDK 假设

[highcontroller.h](https://github.com/4paradigm/phanthymotus-driver/blob/489fe58cd68b5229b1cf90b4b836bbe9d4e79ffe/noetix/bumi/noetix_sdk_bumi/include/highcontroller.h) 与 [highcontrol_pybind.cpp](https://github.com/4paradigm/phanthymotus-driver/blob/489fe58cd68b5229b1cf90b4b836bbe9d4e79ffe/noetix/bumi/noetix_sdk_bumi/src/highcontrol_pybind.cpp) 提供 `get_joint_state`、`get_imu_data`、`get_mode` 等 getter，但未提供 `subscribe_robot_hardware_status`。
旧测试替身曾提供该不存在的回调，掩盖了真实环境永远无法就绪的问题。本次删除这一依赖，测试改为符合厂商 getter 接口。

当前厂商 Python getter 也未暴露源消息接收时间。轮询成功与数值不变均不能证明底层 DDS 正在更新。
基础观察会返回模型估计并注明未验证来源时效；真实动作适配器只接受带实际源接收证据的内部读数。
当前生产 SDK 没有该适配，继续返回明确的执行阻塞原因；没有伪造新鲜时间戳来绕过。

## 示教边界

主仓库支持原厂 `STARTTEACH`、`SAVETEACH`、`PLAYTEACH` 命令，但没有这三种脚姿的现成资产。
[原厂 App 使用说明](https://app-api.noetixrobotics.com/infoToUse.html) 明确演示的是手臂示教，不能据此认定支持拖动腿部并稳定回放。
用户确认暂时没有腿部动作源，因此本次完成可独立测试的脚步基础能力，保留真实动作接入边界。

## 本次检查结果

- `python -X utf8 -m pytest noetix/bumi/tests -q`：173 项通过，包含 31 项脚步卡专项检查。
- 专项覆盖：对称关节几何、单腿角度变化、关节 ID 排序、无效/缺失/重复数据、只读比对、局部资产载入、重载失败关闭输出、没有源时效证据时拒绝动作，以及只启用脚步卡的真实 Bundle 注册路径。
- `python -X utf8 noetix/bumi/foot_pose_state.py --demo`：离线执行成功；结果标注合成输入、未发送命令、未确认物理姿态。
- 平台实际 MCP Schema 转换：`read_feet`、`check_pose`、`reload_assets` 和原有姿态动作均可见，没有结构错误或警告。
- Python 语法、YAML、Docker COPY 及 `git diff --check` 通过。

验证只使用电脑、合成数据及 SDK/遥测替身。未构建 ARM64 镜像，未连接机器人进行读数或运动测试，未宣称完成腿部动作验收。
