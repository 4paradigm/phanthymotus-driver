# 服务区域占用检测

`zone_presence` 是可复用的 processor Card，首版随 Bumi bundle 发布。算法与传输分为两个模块，算法不依赖 Bumi SDK，后续可按官方维护者意见迁移至 Perception。当前分支基于 fork 的 `main` 创建，仓库不存在 `master`。

机器人固定站立时，配置一个画面矩形区域，检测框中心落入区域的 person 计入人数。VOP 坐标左上为 (-1,-1)，中心为 (0,0)，右下为 (1,1)。这是画面区域占用，地面距离和真实空间区域需另行标定。

在画布中连接 VOP 的 objects 输出至本卡。启动时平台传入 `input_topic`。使用 `read` 获取最新状态。配置齿轮支持 `region=[left,top,right,bottom]`、`enter_s`、`leave_s`、`confidence`。修改设置前停止本卡，再重新启动。默认矩形为 [-0.6,-0.8,0.6,1]，进入持续 2 秒、离开持续 1 秒、检测置信度至少 0.5。

输入为标准 VOP JSON，示例为 `{"objects":[{"name":"person","position":[0.2,0.1],"confidence":0.9}]}`。有效的空数组表示无人。格式错误、缺少必要人物坐标或输入过期均输出 unknown。输入至少应稳定达到 2 Hz，超过 0.75 秒的帧间隔重新积累持续时间，2 秒未收到新数据则输出 unknown。时效以本进程接收时间判断，上游的采集时间及重复内容尚未纳入验证，部署时需确保输入来自实时 VOP。

持续状态发布到 `/<namespace>/zone_presence/state`，频率 5 Hz，包括 state、count、dwell_s、input_age_s 和 reason。状态为 unknown、empty 或 occupied。dwell_s 从确认 occupied 时开始计时，是区域连续占用时长。多人交替进入时仍按区域连续占用统计，不给个人分配身份。

事件发布到 `/<namespace>/zone_presence/events`，只在确认进入、离开或变为 unknown 时发送，带本次进程内递增 sequence 和 priority=1。将该事件输出接入 decision_core，持续状态输出用于画布查看。断流不会产生人员离开事件，恢复后重新等待持续时间。Skill 可用进入事件触发讲解，配合持续状态判断本轮区域是否仍有人。

本卡订阅既有 VOP 数据流，停止时销毁自己的订阅、定时器和发布器。Bumi 上的实际检测精度、画布自动接线与资源占用需要真机验收。相机移动后应重新设定区域。

测试命令为 `python -X utf8 -m pytest noetix/bumi/tests/test_zone_presence.py -q`。

PR #486 复审补充适配层测试。运行 `python -X utf8 -m pytest noetix/bumi/tests/test_zone_presence.py noetix/bumi/tests/test_zone_presence_card.py tests/test_bumi_bundle.py -q`，25 项通过。覆盖 PREFIX、完整动作参数声明、启停前后 info 话题发现、重复启停、配置与部分启动失败的资源回收。ROS 资源故障测试使用替身；新镜像上的真实画布接线与 Bumi 验收仍需执行。

## 本地 Gazebo 验证

仿真页面为 <http://127.0.0.1:15715/>，由 Ubuntu 中的 `zone_presence_sim.py` 提供。它读取已有 Gazebo 服务 15713 的真实 YOLO 检测框，仅转换到 VOP 的中心相对坐标，经隔离 ROS 域 78 发布，再交由生产 `ZonePresencePlugin` 订阅处理。只转发新的、有效的图像时间戳，重复读取同一帧不会刷新 Card 的输入时效。

2026-11-06 的运行验证通过区域外、单人进入、连续停留四秒不重复、双人计数、离开、暂停输入后 unknown、恢复输入重新确认七个场景，浏览器无脚本错误。场景切换是人物布局更新，需等待 Gazebo 当前操作结束后再切换。当前验证使用模拟相机和本地 YOLO，厂家 VOP 与 Bumi 真机的运行表现需后续验证。

重开预览时，在 Ubuntu 中加载 ROS Humble，设置 `ROS_DOMAIN_ID=78`、`ROS_LOCALHOST_ONLY=1`，使用已有 vision-venv Python 执行本文件同目录的 `zone_presence_sim.py`。Gazebo 服务需已在 15713 启动。页面提供人物布局切换、暂停输入和恢复输入按钮。

建议实机验收覆盖单人进入并停留、短暂经过、持续站立只触发一次、多人同区、边界进出、离开、VOP 断流和恢复。记录事件数量、人数误差、触发延迟与资源占用，交由 Master 核对。算法属于通用感知能力，正式 PR 的放置层级与维护者确认后确定。
