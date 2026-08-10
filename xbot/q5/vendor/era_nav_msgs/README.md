

English version: [English](README_en.md)

## 1. 准备 ROS 工作空间

创建一个 ROS 工作空间，并把本仓库 clone 到其 `src` 目录下。

```bash
mkdir -p <your_ros_workspace>/src
cd <your_ros_workspace>/src
git clone https://github.com/roboterax/era_nav_msgs.git
```

## 2. 编译本包

```
cd <your_ros_workspace>
colcon build --packages-select era_nav_msgs
```

## 3. RobotEra 导航服务开发者指南

### 地图

RobotEra 导航服务中，地图分为两种类型： LiDAR 地图（点云地图）和导航地图。

- **LiDAR 地图**：用于 SLAM 定位，包含稠密的点云数据。
- **导航地图**：用于导航规划，包含站点、路径和禁行区域等信息。其中站点和路径用基于有向图（Directed Graph）的拓扑地图（Topological Map）表示，禁行区域用有向长方形或多边形（Polygon）表示。

每个场景中，都需要先创建 LiDAR 地图，再创建导航地图。

### ROS 子服务简介

RobotEra 导航服务中，包含多个 ROS 子服务，每个子服务对应一个 [srv](srv) 或 [action](action) 文件。具体如下：

|       子服务名称      |            子服务类型          |         对应 srv/action 文件              | 功能描述 |
| :---: | :---: | :---: | :---: |
| /slam/start_map     | std_srvs.srv.Trigger          | 标准 ROS 服务                             | 启动 LiDAR 建图 (启动后开始扫描环境) |
| /slam/cancel_map    | std_srvs.srv.Trigger          | 标准 ROS 服务                             | 取消 LiDAR 建图 |
| /slam/create_map    | era_nav_msgs.srv.CreateMap    | [CreateMap.srv](srv/CreateMap.srv)       | 优化并保存 LiDAR 地图 |
| /slam/load_map      | era_nav_msgs.srv.LoadMap      | [LoadMap.srv](srv/LoadMap.srv)           | 加载 LiDAR 地图 |
| /slam/init_pos      | era_nav_msgs.srv.InitPos      | [InitPos.srv](srv/InitPos.srv)           | 使用位置(xyz)初始化定位 |
| /slam/query_map     | era_nav_msgs.srv.QueryMap     | [QueryMap.srv](srv/QueryMap.srv)         | 查询当前 LiDAR 地图名 |
| /era_nav/nav_map_op | era_nav_msgs.srv.NavMapOp     | [NavMapOp.srv](srv/NavMapOp.srv)          | 用于操作导航地图（录制、编辑、保存、加载等） |
| /era_nav/nav_act    | era_nav_msgs.action.Navigate  | [Navigate.action](action/Navigate.action) | 用于执行、取消导航任务 |

开发者可查看这些 srv 或 action 文件，以及 [era_nav_pyclient](era_nav_pyclient) 中相应的 client 示例模块，了解每个子服务具体支持哪些操作，以及操作所需的参数和调用方式。


## 4. 运行 Demo

运行 Demo 前，首先 source 工作空间的 `setup.bash` 文件，确保本包的脚本可以正常调用。

```bash
cd <your_ros_workspace>
source install/setup.bash
```

此外，为了方便命令行输入，我们还准备了一些命令缩写，在开始下面的步骤之前，先应用这些缩写 [client_alias.sh](client_alias.sh)：

```bash
source src/era_nav_msgs/client_alias.sh
```

这些缩写命令都是对 [era_nav_pyclient](era_nav_pyclient) 中相应 client 模块的调用，开发者可在 `client_alias.sh` 文件中查看这些缩写命令的定义，了解每个命令具体调用了哪些子服务，再深入 `era_nav_pyclient` 中的 python 示例代码追溯每个操作所需的参数和调用方式。

### 4.1 LiDAR 建图

**开始扫图**：

```bash
StartLidarMapping
```

执行实例:
```
robot@le:era_nav_msgs$ StartLidarMapping 
Map started successfully
```

注意:
- 机器人保持静止时,运行此指令.
- 记录下扫图起始点位置，后续需要在定位初始化时将机器人移动到该位置.
- 执行完成后应该操控机器人扫描工作区域,扫描路径尽量先沿着主要路线绕一圈,回到原点,然后再扫描其他区域.回到原点后,建图状态会切换到Looped(参考监控定位和建图状态--建图执行实例)

**取消扫图**：

```bash
CancelLidarMapping
```

执行实例:
```
robot@le:era_nav_msgs$ CancelLidarMapping 
Map cancelled successfully
```

如果需要使机器人退出扫图状态,可以运行此指令.

**在线创建LiDAR地图**： 

```bash
CreateLidarMap --map_name <map_name>
```

执行实例:
```
robot@le:era_nav_msgs$ CreateLidarMap --map_name test1
Map created success, map_abs_path: maps/test1/lidar_map.pcd, data_abs_path: logs/lidar_mapping/rosbag2/rosbag2.20251202.193120.343
```

执行完成后会返回创建地图的路径,以及存储离线数据的路径.

路过建图区域较大,会花费一些时间来优化,请耐心等待.

**离线创建LiDAR地图**：

```bash
CreateLidarMap --map_name <map_name> --data_abs_path <data_abs_path> 
```

执行实例:
```
robot@le:era_nav_msgs$ CreateLidarMap --data_abs_path logs/lidar_mapping/rosbag2/rosbag2.20251202.193120.343 --map_name test_offline1
Map created success, map_abs_path: maps/test_offline1/lidar_map.pcd, data_abs_path: 
```

如果在线建图失败,需要使用不同的建图参数,可以使用此离线建图功能.

这里<data_abs_path>是在线建图时,返回的"data_abs_path"的参数(参考在线建图--执行实例)

### 4.2 LiDAR 定位

**加载LiDAR地图**:

```bash
LoadLidarMap --map_name <map_name>
```

执行实例:
```
robot@le:era_nav_msgs$ LoadLidarMap --map_name test1
Map loaded successfully
```

执行完成后会返回加载结果.

**Lidar定位的初始化**：

```bash
InitPos <x> <y> <z>
```

执行实例:
```
robot@le:era_nav_msgs$ InitLidarPos --position 0 0 0
Init pose: [[0.0, 0.0, 0.0]]
Localization initialized successfully
```

请确保执行此指令时,机器人一定处于静止状态.执行返回"successfully"后,代表机器人开始进行初始化,请监控到机器人定位状态从"Initializing"切换到"Run"(参考监控定位和建图状态--定位执行实例),代表定位初始化完成.

执行定位初始化,第一次定位时,应该将机器人移动到建图起始点位置,并执行此指令.定位初始化完成后,机器人开始输出当前坐标.
如果想要自定义初始化点,可以在定位初始化后,记下机器人实际位置和坐标(参考监控定位和建图状态--定位执行实例),后面将机器人移动记录的位置,并使用记录的坐标代替"0 0 0".

**查询当前地图名**：

```bash
QueryLidarMap
```

执行实例:
```
robot@le:era_nav_msgs$ QueryLidarMap 
Query map response: era_nav_msgs.srv.QueryMap_Response(map_name='test1')
```

执行完成后会返回当前加载的地图名.

**监控定位和建图状态**：

```bash
MonitorSlamState
```

在定位和建图过程中,可以运行此指令来查看当前的定位和建图状态.

定位执行实例:
```
robot@le:era_nav_msgs$ MonitorSlamState 
# 这是加载地图后,定位初始化前,由于没有初始化,导致激光匹配报错
Error happend 2: 陀螺仪估计异常
Error happend 3: 速度估计异常
Error happend 8: 特征点匹配率不足
# 可以看到有些错误即使没有初始化,也会反复恢复\报错,这是正常现象.
Error recover 9: 残差异常
# 执行了定位初始化指令,定位状态从 Idel 切换到 Initializing,代表正在初始化"定位系统"
Localization status changed: Idel -> Initializing
# 可以看到,定位初始化后,异常状态逐个恢复.
Error recover 2: 陀螺仪估计异常
Error recover 3: 速度估计异常
Error recover 8: 特征点匹配率不足
# 可以看到,定位初始化后,定位状态从 Initializing 切换到 Run,代表定位系统初始化完成,可以正常定位.
Localization status changed: Initializing -> Run
# 正常定位后,开始输出机器人当前坐标,每 1 秒输出一次.
Localization position: 0.624, -2.592, 0.330
Localization position: 0.630, -2.594, 0.335
Localization position: 0.632, -2.592, 0.336
Localization position: 0.633, -2.595, 0.336

# 手动构造错误,导致定位状态从 Run 切换到 Error,**正常情况下不应该发生.**
Error happend 8: 特征点匹配率不足
Localization status chagned: Run -> Error
```

建图执行实例:
```
robot@le:~/code/EraNav/era_nav_algo_ws/src/Nav/era_nav_msgs$ MonitorSlamState 
# 开始扫图(StartLidarMapping)后,建图状态会从Idel切换到Mapping
Mapping status changed: Idel -> Initializing
Mapping status changed: Initializing -> Mapping
# 扫图一圈后,建图状态会从Mapping切换到Looped,代表回环检测通过,后续可以继续对其他未见区域进行扫图.
# 如果刚回到原点后并没有切换到Looped状态,可以继续沿着刚出发的路径走一段,一般场景就会切换到Looped状态.
Mapping status changed: Mapping -> Looped
```


### 4.3 创建导航地图

不同于 LiDAR 点云地图（包含稠密点云，用于 SLAM 定位），导航地图只记录导航用的站点、路径和禁行区域等信息。其中 站点和路径 用基于有向图（Directed Graph）的拓扑地图（Topological Map）表示，禁行区域用有向长方形或多边形（Polygon）表示。

[NavMapOp.srv](srv/NavMapOp.srv) 中定义了导航地图支持的操作，包含录制地图、编辑地图、加载/保存地图等。

录制地图元素的操作包括：
- `StartRecordingPath`: 开始路径录制
- `MarkUserNodeOnNewPath`: 在路径上标记站点
- `FinishRecordingPath`: 完成路径录制
- `CancelRecordingPath`: 取消路径录制（丢弃本次录制的所有内容）
- `RecordUserNode`: 快捷录制单个站点
- `RecordForbiddenArea`: 录制禁行区域

编辑地图元素的操作包括：
- `OverrideMap`: 替换整个地图
- `UpdateMapElements`: 更新地图元素
- `RemoveMapElements`: 删除地图元素


导航站点和路径有两种创建方式：

1. **全录制方式**：遥控机器人沿预设路径行驶，并沿途记录站点和路径信息；
2. **站点录制 + 路径的离线编辑**： 导航站点依然靠录制，以保证其位置的精度（将机器人遥控到站点后记录其位置）； 路径则通过可视化界面来编辑（连线）。对于精度要求较低的站点，也可直接通过可视化界面来点击创建。

方式 1 主要是为了方便在没有离线编辑工具时快速运行导航 Demo 和测试。录制过程中，机器人会自动记录很多匿名站点以确保路径的连通性，生成的导航地图会比较复杂，不便于人工再编辑。

方式 2 则适用于实际部署，站点可以用录制（高精度站点）或点击（低精度站点）的方式创建，路径则通过可视化界面来连线编辑，生成的导航地图比较简洁，不会包含太多冗余站点，便于人工再编辑。

相应地，禁行区域也有两种创建方式：录制和可视化编辑。录制主要在没有可视化编辑工具时使用，仅支持录制长方形区域；可视化编辑时则支持任意多边形区域。

#### 录制路径和站点

> 本操作可用于创建初始导航地图（拓扑地图），也可用于在已有导航地图上添加新的站点和路径。

首先，执行

```bash
StartRecordingPath
```

开始路径录制。

然后，操控机器人沿预设路径行驶。期间，在需要标记导航站点时，将机器人停在站点处，并执行
```bash
MarkUserNodeOnNewPath <name>  # 参数为当前站点名称
```

路径和站点记录完成后，执行

```bash
FinishRecordingPath <auto_connection_radius> # 参数为地图节点间自动连接的半径，用于快捷生成拓扑地图中的边。一般设置为 2.0 米即可。
```

可将刚录制的路径和站点注册进导航地图中。

> 如果录制期间执行 `CancelRecordingPath`，则会丢弃本次录制的所有路径和站点。

**需要注意的是**，执行 `FinishRecordingPath` 完成路径和站点的记录后，需要再执行
```bash
SaveNavMap <map_name> 
```
才能将路径和站点保存进 **导航地图文件** 中！


#### 录制单个新站点时的快捷操作

当仅需要录制单个新站点时，可以先将机器人停到目标站点处，然后执行

```bash
RecordUserNode <name> <auto_connection_radius>
```

效果相当于顺序执行了 `StartRecordingPath`、`MarkUserNodeOnNewPath <name>` 和 `FinishRecordingPath <auto_connection_radius>`。（效果有细小差异，但用户一般不必关心）

同样地，在录制完成后执行

```bash
SaveNavMap <map_name> 
```

才能将站点保存进 **导航地图文件** 中！

#### 录制禁行区域

先将机器人停到禁行区域前方，面对禁行区域，然后执行

```bash
RecordForbiddenArea  <area_front_distance>  <area_length>  <area_width>

# area_front_distance: 机器人 base frame 中心（两主轮中心）到禁行区域前沿的距离
# area_length: 禁行区域的长度
# area_width: 禁行区域的宽度
```

同样注意，录制完成后需要执行

```bash
SaveNavMap <map_name> 
```
才能将禁行区域保存进 **导航地图文件** 中！


#### 编辑地图接口

编辑地图接口包括 `OverrideMap`、`UpdateMapElements` 和 `RemoveMapElements`。

这些接口的使用请参考 [python 示例](era_nav_pyclient/NavMapOpClient.py) 中的相关部分。


```bash

# 测试 OverrideMap 接口
python3 -m era_nav_pyclient.NavMapOpClient OverrideMap

# 测试 UpdateMapElements 接口
python3 -m era_nav_pyclient.NavMapOpClient UpdateMapElements

# 测试 RemoveMapElements 接口
python3 -m era_nav_pyclient.NavMapOpClient RemoveMapElements

```

同样注意，编辑完成后需要再调用 `NavMapOpClient` 的 `SaveMap` 操作，才能将编辑后的地图保存进 **导航地图文件** 中！


#### 站点录制 + 路径的离线编辑

"站点录制 + 路径的离线编辑" 是最推荐的实际部署方式。

- 录制站点可确保站点位置的精度。操作时，先将机器人停到站点处，然后调用 `RecordUserNode` 接口录制站点。站点的业务属性（如站点名称）可按需设置，auto_connection_radius 参数则设置为 0 （不自动连接站点）；
- 路径则通过在可视化界面上连线来编辑，按需连接已有节点、按需新增中间节点，避免全录制方式中自动生成的大量冗余节点；通过调用 `UpdateMapElements` 和 `RemoveMapElements` 可实现新增、删除节点和边（路径）。


### 4.4 加载导航地图

加载导航地图：

```bash
LoadNavMap <map_name>
```

### 4.5 导航

[Navigate.action](action/Navigate.action) 中定义了一系列导航操作。目前主要支持的操作包括：

- `NavToGlobalNode`: 导航至地图中的目标站点
- `CancelNav`: 取消进行中的导航（一般是用于取消由其他 client 发起的导航）


#### 导航至地图中的目标站点

```bash
NavToGlobalNode name <name-to-your-target>  # 用 name 检索目标节点。参数 2 为导航目标站点名称。
```

或

```bash
NavToGlobalNode <node-id>  # 当 node-id (一个数字) 已知时，可以直接写对应的站点 id;
```

其中，第一种执行方式（基于 属性键值对 来检索目标节点），只是为了方便测试，通过方便记忆的属性（如 name）来表示站点。但导航服务层不会检查节点的唯一性，如果存在多个同属性值（如, 相同name）的站点时，导航服务层会任意选择其中一个。

实际部署时，应该用第二种执行方式（基于站点 id），因为站点 id 是唯一的。name 等业务属性的存在与否、值是否唯一，由开发者根据实际业务需求自行定义和管理。

**导航控制指令转发**：

导航服务层会发布 `geometry_msgs.msg.TwistStamped` 类型的 topic `/era_nav/cmd_vel` 作为导航控制指令，控制机器人移动。

但机器人 locomotion 模块所订阅的控制指令通常并不是 `/era_nav/cmd_vel`，开发者一般需要将 `/era_nav/cmd_vel` 转发到机器人 locomotion 模块所订阅的 topic 上，才能使机器人按照导航服务层的指令进行导航。

例如，对于 Q5 机器人，locomotion 模块订阅的 topic 是 `/wr1_base_drive_controller/cmd_vel`，因此需要将 `/era_nav/cmd_vel` 的消息转发到 `/wr1_base_drive_controller/cmd_vel` 上。

为了方便开发者快速运行导航 Demo，我们提供了一些现成的脚本，用于自动转发 `/era_nav/cmd_vel` 指令给 locomotion 模块。比如，对于 Q5 机器人，开发者可以执行如下命令来启动导航控制指令的自动转发：

```bash
ros2 run era_nav_msgs q5_nav_cmd_relay
```

但生产环境中，我们建议开发者自己控制 `/era_nav/cmd_vel` 的转发，这样可以在必要时拦截 `/era_nav/cmd_vel` 指令，切断导航服务层对机器人的控制。比如，紧急情况下用户希望遥控介入时，可以拦截 `/era_nav/cmd_vel` 指令，把机器人控制权交给用户。

> 注意，机器人正常导航过程中，需要关闭机器人的遥控器，否则遥控器指令会与导航指令冲突，导致机器人无法正常导航。


#### 取消当前导航

导航过程中，保留开启导航时用的终端，按 Ctrl+C 可以取消当前导航。


#### 取消外部导航

如果当前正在导航，但启动导航的终端已丢失，可以在其他终端中执行

```bash
CancelNav
```

来强制取消进行中的导航（由其他 client 开启的导航）。


#### 查看导航状态字

导航状态字通过 topic `/era_nav/detailed_nav_fsm` 向外发布，topic 类型为 `std_msgs/String`。

通过如下命令可以查看导航状态：

```bash
ros2 topic echo /era_nav/detailed_nav_fsm
```

**导航状态字包括**：

- **`Navigating`** （或 `Navigating.<二级状态>`）: 导航中。其中 `<二级状态>` 只用于故障诊断和调试，开发者一般不必关心
- **`Idle`** （或 `Idle.<二级状态>`）: 空闲，此时可接受新的导航任务。
- **`WaitingForGoal`** （或 `WaitingForGoal.<二级状态>`）: 导航任务已开启，但用户尚未设置过导航目标。`NavToGlobalNode` 模式下不会触发此状态字（因为`NavToGlobalNode`时必须立即下发导航目标）。

注意，**只有空闲状态下，导航服务层才会接受新的导航任务**。其他状态下，导航服务层会拒绝新的导航请求。

> 唯一可能对开发者有用的二级状态是 `Arrived`，对应的完整状态字为 `Navigating.Arrived`。当机器人到达目标站点时，会发布 **一次** 此状态字，然后切入 `Idle` 状态。
