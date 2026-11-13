# P1-4：插件停止异常隔离与汇总

分支`feat/bumi-driver`，HEAD `814b3ca953d33eba0aca9a61dcefa773f5da483b`。基于510项主机回归，只修复Bumi bundle停止异常短路；主仓库只读，保留全部已有修改。

## 真实调用链与根因

源码路径均相对于Driver：

| 位置 | 原契约与风险 |
| --- | --- |
| `noetix/bumi/main.py::BumiDeviceBundle.__init__:120` | 按state、loco、servo、mic、sound_direction、speaker、camera、traffic_light、visual_odometry、motion_state、vision_capture顺序注册已启用插件 |
| `BumiDeviceBundle.start_all:218` | 同步按注册顺序start，已逐插件捕获Exception；本轮未修改 |
| `BumiDeviceBundle.stop_all:235` | 原为同步逆序直接调用stop，正常返回None；第一个异常直接传播，后续插件被跳过 |
| `main._shutdown:523` | SIGTERM/SIGINT处理函数调用stop_all后才启动HTTP server.shutdown线程；原先停止异常会跳过它 |
| `main finally:533` | stop_all后关闭executor和rclpy；原先停止异常会跳过它们 |

所有现有bundle插件stop入口都是同步方法，无需引入async运行器。两个生产stop_all调用者均不读取返回值，也没有依赖特定异常类型的分支；失败原本就会向外传播。

已在内存加载修改前后的真实bundle类复现：注册`other,loco,camera`，camera.stop抛OSError。原调用序列只有`camera`；修改后为`camera,loco,other`，全部尝试后抛出包含camera错误的RuntimeError。

## 各插件停止入口审查

| 入口 | 返回/异常特点 |
| --- | --- |
| `device.py::CameraPlugin.stop:2778`、`MicPlugin.stop:1948` | 调用子进程terminate，正常None；没有局部异常捕获，底层异常会传播 |
| `direction_card.py::SoundDirectionPlugin.stop:351` | terminate/wait，超时后kill并再次wait；第二次wait及发布路径的异常没有整体隔离 |
| `visual_odometry.py::VisualOdometryPlugin.stop:971` | 调用_node.stop，返回None；节点stop（419行）销毁订阅/publisher等异常可传播 |
| `device.py::StatePlugin.stop:360`、`MotionStatePlugin.stop:3149` | 委托停止轮询，正常None |
| `loco_servo.py::LocoServoPlugin.stop:583` | 调用原_stop并丢弃其返回值，正常None；本轮不修改servo协议或停止预算 |
| `common/traffic_light/plugin.py::TrafficLightPlugin.stop:123` | 返回shutdown/dispatch结果；清理未完成可返回state=error、cleanup_pending，不一定抛异常 |
| `device.py::VisionCapturePlugin.stop:3237` | 返回_stop_recording结果（3472行），可能仍为stopping；本轮不等待或修改P2预算 |
| `device.py::LocoPlugin.stop:686` | 原gesture.shutdown→_stop_move→等待旋转→结束观察预约；返回None。gesture若locked，沿原日志报告，不将它变成completed |

## 新stop_all语义

保持逆注册顺序，每轮对每个已注册插件调用一次stop；不重试、不重排，也不额外直接调用gesture.shutdown。

1. 每个插件独立try/except Exception。
2. 同步stop抛出的普通Exception、明确返回False、dict的state=error或ok=false均记录为失败。只读取错误/message字段，不打印整个返回对象或配置。
3. 日志采用main已有print风格：注册索引、插件类名、stop()阶段、异常类型及消息；每个失败都有自己的记录。
4. 所有插件尝试完后，若有失败，抛聚合RuntimeError，消息包含全部失败，并以首个原异常作为cause。Python 3.10不使用ExceptionGroup，不新增依赖。
5. 全部成功仍返回None，保留原正常日志。空bundle与重复调用保持原有语义；不调用start或业务动作。
6. 不捕获BaseException，KeyboardInterrupt/SystemExit继续传播。

聚合异常能继续让调用者感知失败，同时修复原先“首个异常阻断后续清理”。为了不在外层重现短路，signal handler用finally启动原HTTP关闭线程；main finally嵌套保证即使stop_all抛异常仍调用executor.shutdown，即使executor再抛异常也仍尝试rclpy.shutdown。异常继续传播，没有吞掉或伪造成功。原signal处理后再走main finally的两轮stop保持，没有添加第三轮。

## gesture安全链路与边界

Camera/其他前序插件失败后仍执行：

```text
BumiDeviceBundle.stop_all
  → LocoPlugin.stop
    → TrafficPoliceGesture.shutdown
      → 原cancel / stop_playback / 停稳检查 / watchdog join
```

`device.py`不需要修改，因为gesture.shutdown已经是LocoPlugin.stop的首个步骤。该链路在running、awaiting_verification及locked状态都获得执行机会。测试确认重复bundle停止没有新增PLAYTEACH、ACP终态仍至多一次；locked保持state=error、gesture_state=locked、busy和身体占用，physical_completion_confirmed始终false。

本轮保证软件停止路径得到机会，不证明机器人物理停止。stop返回stopping的异步清理、Loco内部的locked/超时确认、SDK反馈新鲜度和阻塞时长仍由原路径/后续P2及真机验收处理；没有改成新的bundle轮询、超时协议或“全部物理停稳”声明。

## 修改范围

- `noetix/bumi/main.py`：stop_all异常隔离/汇总；两个生产调用处的finally保护。
- `tests/test_bumi_bundle_shutdown.py`：新增27项主机测试。
- `tests/test_bumi_traffic_light_packaging.py::test_bundle_stop_cleans_other_instances_and_retains_failed_handle:320`：保留所有原清理和可重试断言，新增pytest.raises以适配现在会汇总上抛的明确失败回执。
- `docs/BUMI_SMART_TRAFFIC_POLICE_V1.md`：追加说明与最终测试命令/结果。
- 本文。

没有修改Core/主仓库、SDK/HighController、traffic_light、gesture业务/schema/execute/lifecycle、Skill、observation、loco/device、servo、ACP或P2。

## 测试口径

新增模块覆盖：单个/多个/最后一个插件失败、逆序且每轮恰好调用一次、正常None返回、空bundle、False/结构化错误、错误日志不打印配置、KeyboardInterrupt/SystemExit传播。

真实Camera.stop配假进程terminate异常，真实LocoPlugin/gesture配假Controller，覆盖running/awaiting_verification/locked与重复停止；使用假servo验证失败后仍继续调用。真实main函数的所有SDK/ROS/HTTP/线程/信号/注册边界都替换为测试对象，不导入真实SDK、不监听端口、不产生OS信号或机器人动作；验证正常finally、SIGTERM/SIGINT、聚合异常及executor异常后ROS关闭仍被尝试。

按顺序执行：新增shutdown测试27 passed（0.92s）；手势/bundle/Skill相关测试126 passed（4.28s）；最后完整回归。完整命令见[总说明](BUMI_SMART_TRAFFIC_POLICE_V1.md#主机测试与未覆盖项)。统一使用既有临时Python环境，禁用pytest自动插件、缓存与字节码，未安装依赖。

最终统一结果：**537 collected、537 passed、0 failed、0 skipped，59.49秒**；包含原510项及新增27项，原安全断言保留。

最终范围核对通过：除列出的5个文件外，所有Driver文件内容聚合哈希与开始一致；Driver索引未变。主仓库623个文件聚合SHA256仍为`c11944f4e940604a701e5c652e5cdf9c67f1e140df6e81594f56b1f5e982d645`，Git status及索引一致。Bumi bundle只有stop_all方法变化；main硬件初始化/注册前缀及其余入口保持。既有packaging测试仅增加异常断言，所有原资源清理、保留失败句柄和重试断言原样保留。

已执行git status --short、git diff --stat、git diff --check；check通过。tracked diff为8 files / +213 / -36，包含开始时既有改动；本轮tracked增量仅main.py的+29/-7。新增测试、文档及原本未跟踪的packaging测试不计入该stat。本轮P1-4修复通过。git add、commit、push、build、deploy、SSH、真机操作均未执行。
