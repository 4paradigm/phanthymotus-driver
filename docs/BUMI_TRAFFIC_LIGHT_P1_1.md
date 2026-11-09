# Bumi traffic_light：P1-1 标准发布接入

本轮只修复 traffic_light 的打包、注册和生命周期接入。生产实现位于 Driver 仓库 `common/traffic_light/`，由现有 Bumi bundle 在端口15704暴露，复用已经运行的 Camera、外部 VOP 和 ROS 数据。没有新增镜像、容器、服务、端口、模型或相机实现。

工作分支为 `feat/bumi-driver`，HEAD保持 `814b3ca953d33eba0aca9a61dcefa773f5da483b`。开始时已有未提交工作，全部保留。本轮没有执行 commit/push/build/deploy/SSH/真机测试；主仓库只读。

## 唯一生产实现和标准构建路径

| 文件 | 作用 |
| --- | --- |
| `common/traffic_light/schemas.py` | 原配置、阈值、证据结构、stamp/bbox校验，迁移时字节保持不变 |
| `common/traffic_light/color_classifier.py` | 原颜色和箭头分析，字节保持不变 |
| `common/traffic_light/processor.py` | 原精确匹配、有界缓存、tracking、多帧投票及stale处理，字节保持不变 |
| `common/traffic_light/plugin.py` | 原通用ROS runtime/MCP Card；增加bundle Python start/stop桥接，清理使用包内辅助模块 |
| `common/traffic_light/_ros_lifecycle.py` | 仅实现该插件所需的remove→destroy、失败保留进度、可重试及幂等语义 |
| `common/traffic_light/__init__.py` | 插件与算法导出 |
| `noetix/bumi/main.py::BumiDeviceBundle.__init__` | Bumi接入层：按配置注册共享插件，传入已有executor，不依赖HighController/MediaController/LocoPlugin |

无需额外的Bumi adapter文件：共享插件只处理ROS与视觉数据，Bumi接入逻辑只有bundle注册；Python生命周期桥接本身不含机器人专属逻辑。`common/__init__.py`不增加全局导入，其他未采用此功能的Driver不会因此加载OpenCV。

`build.sh`按 `driver.yaml` 复制 `noetix/bumi` 并加入 `build_context_extras: ../../common`；现有Dockerfile已经执行 `COPY common/ /work/common/` 并设置 `PYTHONPATH=/work:...`。因此新包直接进入标准Bumi源码COPY路径。本轮不修改build.sh、Dockerfile或deploy/service.yml，不把仓库根目录或主仓库作为运行时挂载/构建依赖。

`driver.yaml.cards`新增 `traffic_light: processor`；真正运行时定义仍来自共享插件get_tools，经BumiDeviceBundle.get_all_tools和原MCP handler的tools/list返回。默认配置注册一次，type=processor、multiInstance=true。

## 生命周期

| 调用 | 行为 |
| --- | --- |
| Python `plugin.start()` / bundle boot | 仅准备插件，无Node、instance、订阅、Camera/VOP启动或SDK调用 |
| Card `dispatch("start", args)` | 必须有明确输入（或此前保存的画布绑定）；创建一个runtime/node，订阅对应JPEG和objects，启动失效timer |
| Card `info/latest` | 返回本实例状态与原JSON契约；注册/订阅运行不等于交通灯结果有效 |
| Card `stop(instance_id)` | 只关闭该实例，停止回调，移除并销毁Node；清空frame/pending/history；保留画布绑定供observe复看 |
| Python `plugin.stop()` / bundle stop | 通过原shutdown路径关闭全部实例；重复调用幂等；失败返回error/cleanup_pending并保留清理句柄 |

节点清理不再导入主仓库 `utils.ros_lifecycle`。移除失败时禁止销毁仍注册的Node；销毁失败时不删除实例句柄，后续stop可以继续清理。现有bundle全局stop_all异常隔离问题P1-4不在本轮修改范围。

## ROS与同步契约

```text
Bumi Camera ── JPEG ──→ 现有 embodied-perception / VOP ── objects ─┐
       └──────────────────── 原JPEG Topic ──────────────────────┼→ Bumi traffic_light
```

输入为画布提供的绝对 `/.../objects` Topic。示例输入 `/ubuntu/camera/color/objects`，自动推导RGB `/ubuntu/camera/color` 和输出 `/ubuntu/camera/color/objects/traffic_light`；不硬编码ubuntu。RGB为sensor_msgs/CompressedImage，objects和输出为std_msgs/String JSON。插件两路订阅和输出QoS保持BEST_EFFORT、VOLATILE、KEEP_LAST depth=2。

仍严格要求 VOP.source_stamp == RGB.header.stamp 的纳秒值。Camera的stamp产生于JPEG编码后，VOP顶层timestamp是结果生成时间，均不得假称曝光时间。匹配不到、来源缺失、过期或回退时继续返回waiting/unknown/stale，不采用最近RGB。

原缓存64帧/16MiB、单JPEG1MiB、pending8份、尺寸保护、max_tracks、track TTL、HSV和箭头门限、3/5历史及JSON格式均不变。部署位置改变不改变这些算法。本地VOP源码有source_stamp，不代表现场官方镜像已经具备该字段，仍需在后续验收时核对版本。

## 测试覆盖与结果

原traffic_light算法断言保留；旧替代Perception handler测试迁为真实Bumi handler测试。ROS替身只替换中间件，OpenCV与processor使用真实实现；不监听端口，不启动SDK、相机或ROS服务。

新增 `tests/test_bumi_traffic_light_packaging.py`：

- 标准context按真实Bumi目录+extras复制；按Dockerfile最后阶段的源码COPY构造镜像等价目录，不执行Docker/RUN。
- 子进程以 `-I -B` 只加入等价 `/work` 路径，禁止perception/utils/SDK命名空间；真实Bumi入口导入、bundle、Card start/info/latest/stop在该目录运行。
- 真实默认YAML、真实Bumi bundle/tools/list（仅无关硬件插件使用替身），确认traffic_light恰好一次。
- Controller可用/不可用均不影响注册，SDK对象访问直接令测试失败。
- bundle boot不自动打开数据源，缺少输入的Card start拒绝。
- 真实Bumi MCP handler和消息类型、Topic、QoS验证。
- 精确stamp匹配、重复帧不增加票数、三帧稳定和stale降级。
- 多实例单独/全部停止、重启屏障、旧回调、清理失败重试。
- driver.yaml、默认配置和真实runtime元数据一致。
- 包内节点清理的remove失败/异常/仍注册、destroy失败/异常及重试幂等。

针对性测试（traffic_light、新增集成/打包、bundle、Skill）113项通过，1.43秒；退役旧目录后新增模块单独运行16项通过，0.77秒，无失败或跳过。最终完整回归结果见下方验收记录。隔离目录测试证明源码依赖与注册链路，不代表实际Docker镜像已构建、真实DDS已联通或Jetson性能已验收。

## 旧入口退役与画布引用

原Driver根目录perception是开发机替代入口，不属于标准Bumi产物。完成迁移、更新引用并通过完整440项回归后，删除以下9个文件；随后再对最终目录运行回归。相邻主仓库的同名原型未改动。

| 已退役文件 | 原因 |
| --- | --- |
| `perception/main.py` | 仅为load_shared开发机替代入口，真实Bumi入口已承接 |
| `perception/config.yaml` | 替代服务配置；traffic_light参数迁入Bumi配置，不迁移VOP模型/服务配置 |
| `perception/__init__.py`、`perception/plugins/__init__.py` | 只服务旧命名空间，已无生产或测试引用 |
| `perception/plugins/traffic_light/__init__.py` | 导出迁到common包 |
| `perception/plugins/traffic_light/schemas.py`、`color_classifier.py`、`processor.py` | 与common中对应文件字节相同，删除重复实现 |
| `perception/plugins/traffic_light/plugin.py` | 通用runtime迁到common，新增bundle桥接及包内清理适配 |

旧目录没有其他文件，删除源文件后只移除了空目录。没有为目录整洁删除未知用途文件。production现在不依赖load_shared、主仓库PerceptionBundle、主仓库utils或sys.path注入。

工具名仍为traffic_light；已有画布如果绑定旧Perception MCP，应重新连接到Bumi MCP下该工具。Skill.requiredTools不变；Skill的Core契约测试仍只读参考Core纯函数，这不属于production运行依赖。

## 明确保留的已知问题

本轮没有修复P1-2手势start/info握手、P1-3重复框tracking历史歧义、P1-4全局stop_all异常隔离，以及P2 ACP pending/stop budget。修复P1-1不代表完整智能交警V1已具备现场部署条件。动作、body互斥、servo、观察旋转10°/3次/30°预算和原厂SDK均未修改。

原手势测试唯一改动是元数据断言：由“driver.yaml不得包含traffic_light”改为“包含traffic_light processor”，以匹配本轮架构；该测试其余手势/安全断言及手势生产代码不变。Dockerfile、build.sh与deploy/service.yml本轮字节未改变，git中原有Dockerfile修改属于此前工作。

## 最终验收记录

删除旧目录后的最终完整回归：**collected=440，passed=440，failed=0，skipped=0，duration=57.11秒**。命令与范围见[总说明](BUMI_SMART_TRAFFIC_POLICE_V1.md#主机测试与未覆盖项)。相对422基线，增加16项新增测试，并扩展运行2项已有Bumi Dockerfile覆盖检查。

P1-1的源码打包/注册目标通过：标准context和源码COPY树包含共享包，隔离环境不依赖相邻checkout，真实Bumi tools/list返回一次traffic_light，bundle准备不订阅，Card生命周期与精确stamp匹配通过。没有执行实际镜像构建或现场验证，其余P1/P2保持待处理。
