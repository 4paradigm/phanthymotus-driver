
中文版： [中文](README.md)

## 1. Prepare ROS Workspace

Create a ROS workspace and clone this repository into its src directory.

```bash
mkdir -p <your_ros_workspace>/src
cd <your_ros_workspace>/src
git clone https://github.com/roboterax/era_nav_msgs.git
```

## 2. Compile the Package

```
cd <your_ros_workspace>
colcon build --packages-select era_nav_msgs
```

## 3. RobotEra Navigation Service Developer Guide

### Map

In RobotEra's navigation service, there are two types of maps: LiDAR map (point cloud map) and navigation map.

- **LiDAR Map**: Used for SLAM localization, contains dense point cloud data.
- **Navigation Map**: Used for navigation planning, contains information such as stations, paths, and forbidden areas. Stations and paths are represented by a topological map based on directed graphs, and forbidden areas are represented by directed rectangles or polygons.

In each scenario, a LiDAR map must be created first, followed by the creation of a navigation map.

### Introduction to ROS Subservices

RobotEra's navigation service includes several ROS subservices, each corresponding to an [srv](srv) or [action](action) file. The details are as follows:

|       Subservice Name      |            Subservice Type          |         Corresponding srv/action File              | Description |
| :---: | :---: | :---: | :---: |
| /slam/start_map     | std_srvs.srv.Trigger          | Standard ROS service                             | Start LiDAR mapping (starts scanning the environment) |
| /slam/cancel_map    | std_srvs.srv.Trigger          | Standard ROS service                             | Cancel LiDAR mapping |
| /slam/create_map    | era_nav_msgs.srv.CreateMap    | [CreateMap.srv](srv/CreateMap.srv)       | Optimize and save the LiDAR map |
| /slam/load_map      | era_nav_msgs.srv.LoadMap      | [LoadMap.srv](srv/LoadMap.srv)           | Load the LiDAR map |
| /slam/init_pos      | era_nav_msgs.srv.InitPos      | [InitPos.srv](srv/InitPos.srv)           | Initialize localization with position (xyz) |
| /slam/query_map     | era_nav_msgs.srv.QueryMap     | [QueryMap.srv](srv/QueryMap.srv)         | Query the current LiDAR map name |
| /era_nav/nav_map_op | era_nav_msgs.srv.NavMapOp     | [NavMapOp.srv](srv/NavMapOp.srv)          | Operate on the navigation map (record, edit, save, load, etc.) |
| /era_nav/nav_act    | era_nav_msgs.action.Navigate  | [Navigate.action](action/Navigate.action) | Execute or cancel navigation tasks |

Developers can review these srv or action files and the corresponding client example modules in [era_nav_pyclient](era_nav_pyclient) to understand what operations each subservice supports, along with the necessary parameters and invocation methods.

## 4. Running the Demo

Before running the Demo, source the workspace's `setup.bash` file to ensure the package's scripts can be invoked properly.

```bash
cd <your_ros_workspace>
source install/setup.bash
```

Additionally, to make command line input easier, we have prepared some command aliases. Apply these aliases before proceeding to the following steps by sourcing the [client_alias.sh](client_alias.sh):

```bash
source src/era_nav_msgs/client_alias.sh
```

These alias commands correspond to calls to the respective client modules in [era_nav_pyclient](era_nav_pyclient). Developers can check the `client_alias.sh` file for these alias definitions and trace each operation's required parameters and invocation methods in the python example code from `era_nav_pyclient`.

### 4.1 LiDAR Mapping

**Start Scanning**:

```bash
StartLidarMapping
```

Execution example:
```
robot@le:era_nav_msgs$ StartLidarMapping 
Map started successfully
```

Note:
- Run this command while the robot is stationary.
- Record the starting position of the scanning, as the robot will need to be moved to this position during localization initialization.
- After execution, the robot should scan the work area, initially following the main path, returning to the origin before continuing to scan other areas. Once back at the origin, the mapping status will switch to 'Looped' (see monitoring localization and mapping status -- mapping execution example).

**Cancel Scanning**:

```bash
CancelLidarMapping
```

Execution example:
```
robot@le:era_nav_msgs$ CancelLidarMapping 
Map cancelled successfully
```

This command will exit the scanning state.

**Create LiDAR Map Online**: 

```bash
CreateLidarMap --map_name <map_name>
```

Execution example:
```
robot@le:era_nav_msgs$ CreateLidarMap --map_name test1
Map created success, map_abs_path: maps/test1/lidar_map.pcd, data_abs_path: logs/lidar_mapping/rosbag2/rosbag2.20251202.193120.343
```

After execution, it returns the created map path and the path where offline data is stored.

If the mapped area is large, the optimization will take some time, so please wait patiently.

**Create LiDAR Map Offline**:

```bash
CreateLidarMap --map_name <map_name> --data_abs_path <data_abs_path> 
```

Execution example:
```
robot@le:era_nav_msgs$ CreateLidarMap --data_abs_path logs/lidar_mapping/rosbag2/rosbag2.20251202.193120.343 --map_name test_offline1
Map created success, map_abs_path: maps/test_offline1/lidar_map.pcd, data_abs_path: 
```

If online mapping fails and different mapping parameters are needed, use this offline mapping functionality.

Here `<data_abs_path>` is the "data_abs_path" parameter returned by online mapping (see online mapping -- execution example).

### 4.2 LiDAR Localization

**Load LiDAR Map**:

```bash
LoadLidarMap --map_name <map_name>
```

Execution example:
```
robot@le:era_nav_msgs$ LoadLidarMap --map_name test1
Map loaded successfully
```

This command will return the result of loading the map.

**Initialize LiDAR Localization**:

```bash
InitPos <x> <y> <z>
```

Execution example:
```
robot@le:era_nav_msgs$ InitLidarPos --position 0 0 0
Init pose: [[0.0, 0.0, 0.0]]
Localization initialized successfully
```

Ensure the robot is stationary when executing this command. After the "successfully" response, the robot will start initialization. Monitor the robot's localization status to change from "Initializing" to "Run" (see monitoring localization and mapping status -- localization execution example), indicating that localization initialization is complete.

During the first localization, move the robot to the map's starting position before executing this command. After localization initialization, the robot will begin outputting current coordinates.
If you want to customize the initialization point, you can note the actual position and coordinates after localization initialization, and use the recorded coordinates to replace "0 0 0" later.

**Query Current Map Name**:

```bash
QueryLidarMap
```

Execution example:
```
robot@le:era_nav_msgs$ QueryLidarMap 
Query map response: era_nav_msgs.srv.QueryMap_Response(map_name='test1')
```

This will return the current loaded map's name.

**Monitor Localization and Mapping Status**:

```bash
MonitorSlamState
```

While performing localization and mapping, this command can be used to check the current status of localization and mapping.

Localization execution example:
```
robot@le:era_nav_msgs$ MonitorSlamState 
# This is before localization initialization after loading the map, errors due to lack of initialization
Error happend 2: Gyroscope estimate abnormal
Error happend 3: Speed estimate abnormal
Error happend 8: Feature point matching rate insufficient
# Some errors will recur and recover, which is normal.
Error recover 9: Residual abnormal
# After executing the localization initialization command, localization status changes from Idel to Initializing, indicating the "localization system" is initializing.
Localization status changed: Idel -> Initializing
# After localization initialization, errors gradually recover.
Error recover 2: Gyroscope estimate abnormal
Error recover 3: Speed estimate abnormal
Error recover 8: Feature point matching rate insufficient
# After localization initialization, status changes from Initializing to Run, indicating localization system initialization is complete, and the system is running normally.
Localization status changed: Initializing -> Run
# After successful localization, current coordinates are output every second.
Localization position: 0.624, -2.592, 0.330
Localization position: 0.630, -2.594, 0.335
Localization position: 0.632, -2.592, 0.336
Localization position: 0.633, -2.595, 0.336

# If errors are manually induced, the localization status changes from Run to Error, which **should not happen under normal circumstances.**
Error happend 8: Feature point matching rate insufficient
Localization status changed: Run -> Error
```

Mapping execution example:
```
robot@le:~/code/EraNav/era_nav_algo_ws/src/Nav/era_nav_msgs$ MonitorSlamState 
# After starting scanning (StartLidarMapping), mapping status changes from Idel to Mapping.
Mapping status changed: Idel -> Initializing
Mapping status changed: Initializing -> Mapping
# After scanning a full round, mapping status changes from Mapping to Looped, indicating that loop detection has passed and further mapping can continue in unexplored areas.
# If the status does not switch to Looped right after returning to the origin, continue along the starting path and the status will usually switch to Looped.
Mapping status changed: Mapping -> Looped
```

### 4.3 Creating Navigation Map

Unlike the LiDAR point cloud map (which contains dense point clouds for SLAM localization), the navigation map only records information for navigation, such as stations, paths, and forbidden areas. Stations and paths are represented by a topological map based on directed graphs, and forbidden areas are represented by directed rectangles or polygons.

[NavMapOp.srv](srv/NavMapOp.srv) defines the operations supported by the navigation map, including recording, editing, loading/saving maps, etc.

The operations for recording map elements include:
- `StartRecordingPath`: Start recording a path
- `MarkUserNodeOnNewPath`: Mark a station on the path
- `FinishRecordingPath`: Finish recording the path
- `CancelRecordingPath`: Cancel path recording (discard all recorded content)
- `RecordUserNode`: Quickly record a single station
- `RecordForbiddenArea`: Record a forbidden area

The operations for editing map elements include:
- `OverrideMap`: Replace the entire map
- `UpdateMapElements`: Update map elements
- `RemoveMapElements`: Remove map elements

Navigation stations and paths can be created in two ways:

1. **Full recording method**: Control the robot to travel along a predefined path and record stations and path information along the way.
2. **Station recording + offline editing of paths**: Navigation stations are recorded to ensure accuracy (the robot is moved to the station and its position recorded); paths are edited via a visual interface (connect the nodes). For stations with low accuracy requirements, they can be created directly by clicking on the visual interface.

Method 1 is intended for quickly running navigation demos and tests when there are no offline editing tools. During recording, the robot will automatically record many anonymous stations to ensure path connectivity, and the generated navigation map will be complex and difficult to edit manually.

Method 2 is suitable for actual deployment, where stations can be recorded (high-accuracy stations) or clicked (low-accuracy stations), and paths can be edited via a visual interface, creating a simpler and more editable navigation map.

Likewise, forbidden areas have two creation methods: recording and visual editing. Recording is mainly used when visual editing tools are unavailable and only supports recording rectangular areas; visual editing supports any polygonal area.

#### Recording Paths and Stations

> This operation can be used to create an initial navigation map (topological map) or add new stations and paths to an existing navigation map.

First, execute:

```bash
StartRecordingPath
```

Start path recording.

Then, control the robot to move along the predefined path. When a navigation station needs to be marked, stop the robot at the station and execute:
```bash
MarkUserNodeOnNewPath <name>  # The parameter is the name of the current station
```

After completing the path and station recording, execute:

```bash
FinishRecordingPath <auto_connection_radius> # Parameter is the radius for automatically connecting nodes in the map. Generally set to 2.0 meters.
```

This will register the recorded path and stations into the navigation map.

> If `CancelRecordingPath` is executed during recording, all recorded paths and stations will be discarded.

**Note**: After executing `FinishRecordingPath`, you need to execute:

```bash
SaveNavMap <map_name> 
```

To save the path and stations into the **navigation map file**!

#### Quick Operation for Recording a Single New Station

When only recording a single new station, stop the robot at the target station and then execute:

```bash
RecordUserNode <name> <auto_connection_radius>
```

This is equivalent to sequentially executing `StartRecordingPath`, `MarkUserNodeOnNewPath <name>`, and `FinishRecordingPath <auto_connection_radius>` (there are slight differences, but users generally don't need to worry about it).

Similarly, after recording, execute:

```bash
SaveNavMap <map_name> 
```

To save the station into the **navigation map file**!

#### Recording Forbidden Area

Stop the robot in front of the forbidden area, facing it, and then execute:

```bash
RecordForbiddenArea  <area_front_distance>  <area_length>  <area_width>

# area_front_distance: Distance from the robot's base frame center (center of the two main wheels) to the front edge of the forbidden area
# area_length: Length of the forbidden area
# area_width: Width of the forbidden area
```

Again, remember to execute:

```bash
SaveNavMap <map_name> 
```

To save the forbidden area into the **navigation map file**!

#### Editing Map Interface

The map editing interfaces include `OverrideMap`, `UpdateMapElements`, and `RemoveMapElements`.

For usage of these interfaces, refer to the relevant sections in the [python example](era_nav_pyclient/NavMapOpClient.py).

```bash

# Test OverrideMap interface
python3 -m era_nav_pyclient.NavMapOpClient OverrideMap

# Test UpdateMapElements interface
python3 -m era_nav_pyclient.NavMapOpClient UpdateMapElements

# Test RemoveMapElements interface
python3 -m era_nav_pyclient.NavMapOpClient RemoveMapElements

```

Again, after editing, you need to invoke the `NavMapOpClient`'s `SaveMap` operation to save the edited map into the **navigation map file**!

#### Station Recording + Offline Path Editing

"Station recording + offline path editing" is the recommended method for actual deployment.

- Recording stations ensures the accuracy of station positions. To do this, stop the robot at the station and use the `RecordUserNode` interface. The business attributes (like station names) can be set as needed, and the auto_connection_radius parameter should be set to 0 (no automatic connection of nodes).
- Paths are edited by connecting nodes via a visual interface, and unnecessary intermediate nodes are added only as needed. You can use `UpdateMapElements` and `RemoveMapElements` to add or delete nodes and edges (paths).

### 4.4 Loading Navigation Map

Load the navigation map:

```bash
LoadNavMap <map_name>
```

### 4.5 Navigation

[Navigate.action](action/Navigate.action) defines a series of navigation operations. The currently supported operations include:

- `NavToGlobalNode`: Navigate to a target station in the map
- `CancelNav`: Cancel the ongoing navigation (usually used to cancel navigation initiated by another client)

#### Navigate to a Target Station in the Map

```bash
NavToGlobalNode name <name-to-your-target>  # Use the name to retrieve the target node. Parameter 2 is the name of the target station.
```

or

```bash
NavToGlobalNode <node-id>  # When node-id (a number) is known, you can directly use the corresponding station id;
```

The first execution method (based on attribute key-value pairs) is just for convenience during testing, using memorable attributes (such as name) to represent stations. However, the navigation service layer does not check the uniqueness of nodes. If multiple stations share the same attribute (e.g., same name), the service will arbitrarily choose one.

In actual deployment, the second method (using station id) should be used, as station ids are unique. The existence and uniqueness of attributes like name are managed by the developer as per the business requirements.


**Navigation Command Forwarding**:

The navigation service layer will publish navigation control commands using the `geometry_msgs.msg.TwistStamped` type topic `/era_nav/cmd_vel`, which controls the robot's movement.

However, the robot's locomotion module typically subscribes to a topic other than `/era_nav/cmd_vel`. Developers generally need to forward the `/era_nav/cmd_vel` message to the topic subscribed to by the robot's locomotion module to allow the robot to navigate according to the navigation service layer's commands.

For example, for the Q5 robot, the locomotion module subscribes to the topic `/wr1_base_drive_controller/cmd_vel`, so the messages from `/era_nav/cmd_vel` need to be forwarded to `/wr1_base_drive_controller/cmd_vel`.

To help developers quickly run the navigation demo, we provide ready-made scripts for automatically forwarding the `/era_nav/cmd_vel` control command to the locomotion module. For instance, for the Q5 robot, developers can run the following command to start the automatic forwarding of the navigation control command:

```bash
ros2 run era_nav_msgs q5_nav_cmd_relay
```

However, in production environments, we recommend that developers control the forwarding of `/era_nav/cmd_vel` themselves. This allows for the interception of `/era_nav/cmd_vel` commands when necessary, cutting off the navigation service layer's control over the robot. For example, in an emergency, if a user wishes to take manual control, the `/era_nav/cmd_vel` commands can be intercepted, and the control of the robot can be handed over to the user.

> Note: During normal navigation, ensure that the robot's remote controller is turned off, as remote controller commands will conflict with navigation commands, preventing the robot from navigating properly.


#### Cancel Current Navigation

During navigation, keep the terminal where navigation was initiated open. Press Ctrl+C to cancel the current navigation.

#### Cancel External Navigation

If navigation is ongoing but the terminal that started it is lost, you can execute:

```bash
CancelNav
```

To force cancel the ongoing navigation (initiated by another client).

#### View Navigation Status Word

The navigation status word is published through the topic `/era_nav/detailed_nav_fsm` as a `std_msgs/String`. 

To view the navigation status, use the following command:

```bash
ros2 topic echo /era_nav/detailed_nav_fsm
```

**Navigation status words include**:

- **`Navigating`** (or `Navigating.<secondary status>`): Navigating. The `<secondary status>` is only for troubleshooting and debugging, and developers generally don't need to worry about it.
- **`Idle`** (or `Idle.<secondary status>`): Idle, new navigation tasks can be accepted at this time.
- **`WaitingForGoal`** (or `WaitingForGoal.<secondary status>`): The navigation task has been started, but the user has not yet set the navigation goal. This status is not triggered in `NavToGlobalNode` mode (because a navigation goal must be immediately provided in this mode).

Note that **only in Idle status** will the navigation service accept new tasks. In other statuses, it will reject new navigation requests.

> The only secondary status that might be useful to the developer is `Arrived`, corresponding to the full status word `Navigating.Arrived`. When the robot reaches the target station, this status word is published **once** and the system returns to `Idle` status.
