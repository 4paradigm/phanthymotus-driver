# 1. 系统简介
## 1.1 功能概述
RobotEraSDKAPI 是一个基于ROS2的Python库，提供完整的机器人控制功能，包括：

- 基础控制: 机器人初始化、关节状态监控、生命周期管理
- 轨迹控制: 预定义轨迹执行、自定义轨迹规划、同步/异步控制
- MPC控制: 模型预测控制、高级算法控制、ServoPose发布
- 手部控制: XHand/XHand Lite关节控制、抓取动作、自定义手部运动
- 底盘控制: 线速度和角速度控制、前进后退、转向运动

## 1.2 适用场景

- 机器人研究和开发
- 原型验证和测试

# 2. 环境要求

- 星动纪元提供的 developer 环境里面使用
- 在带有 ros2-humble + cyclonedds 环境中，需要执行如下操作：
```shell
# 下载对应的消息定义，并 source
git clone https://github.com/roboterax/teleop_client.git
cd teleop_client
colcon build
source install/setup.bash

# 设置环境变量
export ROS_DOMAIN_ID=211
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

# 3. 快速上手
## 3.1 基础使用示例
```python
#!/usr/bin/env python3
import rclpy
from robot_controller import RobotController, TrajectoryController

def main():
    try:
        # 创建机器人控制器
        robot = RobotController("my_robot")
        
        # 创建轨迹控制器，默认机型为 Q5
        # 如使用 L3 或 L7，请将 "Q5" 修改为 "L3" 或 "L7"
        trajectory = TrajectoryController(robot, "Q5")
        
        # 启动关节服务
        robot.start_joint_service()
        
        # 初始化关节（约20秒）
        robot.initialize_joints()
        
        # 设置零位
        trajectory.set_zero_position()
        
        # 移动到目标位置
        target_positions = {
            "left_wrist_pitch_joint": 0.5,
            "left_wrist_roll_joint": -0.3
        }
        trajectory.move_to_joint_positions(target_positions, duration=3.0)
        
        print("机器人控制完成！")
        
    finally:
        # 清理资源
        robot.shutdown()

if __name__ == "__main__":
    main()
```

## 3.2 MPC控制示例
```python
import rclpy
from robot_controller import RobotController, MPCController
from xbot_common_interfaces.msg import ServoPose
from geometry_msgs.msg import PoseStamped

def main():
    try:
        # 创建控制器
        robot = RobotController("mpc_robot")
        mpc = MPCController(robot)
        
        # 激活算法控制
        robot.activate_algorithm_control()
        
        # 启动MPC
        mpc.start_mpc()
        
        # 创建ServoPose消息
        msg = ServoPose()
        # 设置左臂位置
        msg.left_pose = PoseStamped()
        msg.left_pose.pose.position.x = 0.425
        msg.left_pose.pose.position.y = 0.3
        msg.left_pose.pose.position.z = 0.292
        
        # 发布ServoPose
        mpc.publish_servo_pose(msg, rate_hz=50.0, duration=3.0)
        
        # 停止MPC
        mpc.stop_mpc()
        
    finally:
        robot.shutdown()

if __name__ == "__main__":
    main()
```

## 3.3 手部和底盘控制示例
```python
#!/usr/bin/env python3
import rclpy
import time
from robot_controller import RobotController, TrajectoryController

def main():
    try:
        # 创建机器人控制器
        robot = RobotController("hand_and_base_demo")
        # 创建轨迹控制器，默认机型为 Q5
        # 如使用 L3 或 L7，请将 "Q5" 修改为 "L3" 或 "L7"
        trajectory = TrajectoryController(robot, "Q5")
        
        # 初始化机器人
        robot.start_joint_service()
        robot.initialize_joints()
        trajectory.set_zero_position()
        trajectory.set_lift_up_position()
        
        # 手部控制示例
        print("=== 手部控制示例 ===")
        
        # XHand Lite抓取
        robot.control_xhand_lite_grasp()
        time.sleep(3)
        
        # XHand抓取
        robot.control_xhand_grasp()
        time.sleep(3)
        
        # 自定义手部控制
        custom_joints = ['left_hand_thumb_bend_joint', 'left_hand_thumb_rota_joint1']
        custom_positions = [0.8, 0.8]
        robot.control_hand_joints(custom_joints, custom_positions)
        time.sleep(3)
        
        # 底盘控制示例
        print("=== 底盘控制示例 ===")
        
        # 前进（10Hz频率，持续1秒）
        robot.control_base_drive(linear_x=0.2, angular_z=0.0, frequency=10.0, duration=1.0)
        
        # 左转（10Hz频率，持续1秒）
        robot.control_base_drive(linear_x=0.0, angular_z=0.5, frequency=10.0, duration=1.0)
        
        # 前进并右转（10Hz频率，持续1秒）
        robot.control_base_drive(linear_x=0.1, angular_z=-0.3, frequency=10.0, duration=1.0)
        
        # 停止
        robot.stop_base_drive()
        
        print("手部和底盘控制完成！")
        
    finally:
        robot.shutdown()

if __name__ == "__main__":
    main()
```

## 3.4 独立控制示例

### 3.4.1 手部控制示例
```python
#!/usr/bin/env python3
# examples/hand_control_demo.py
import rclpy
from robot_controller import RobotController, TrajectoryController

def main():
    try:
        robot = RobotController("hand_control_demo")
        # 创建轨迹控制器，默认机型为 Q5
        # 如使用 L3 或 L7，请将 "Q5" 修改为 "L3" 或 "L7"
        trajectory = TrajectoryController(robot, "Q5")
        
        # 初始化机器人
        robot.start_joint_service()
        robot.initialize_joints()
        trajectory.set_zero_position()
        trajectory.set_lift_up_position()
        
        # XHand Lite抓取
        robot.control_xhand_lite_grasp()
        time.sleep(3)
        
        # XHand抓取
        robot.control_xhand_grasp()
        time.sleep(3)
        
        # 自定义手部控制
        custom_joints = ['left_hand_thumb_bend_joint', 'left_hand_thumb_rota_joint1']
        custom_positions = [0.8, 0.8]
        robot.control_hand_joints(custom_joints, custom_positions)
        time.sleep(3)
        
        print("手部控制完成！")
        
    finally:
        robot.shutdown()

if __name__ == "__main__":
    main()
```

### 3.4.2 底盘控制示例
```python
#!/usr/bin/env python3
# examples/base_control_demo.py
import rclpy
import time
from robot_controller import RobotController, TrajectoryController

def main():
    try:
        robot = RobotController("base_control_demo")
        # 创建轨迹控制器，默认机型为 Q5
        # 如使用 L3 或 L7，请将 "Q5" 修改为 "L3" 或 "L7"
        trajectory = TrajectoryController(robot, "Q5")
        
        # 初始化机器人
        robot.start_joint_service()
        robot.initialize_joints()
        trajectory.set_zero_position()
        trajectory.set_lift_up_position()
        
        # 前进并转向（10Hz频率，持续1秒）
        robot.control_base_drive(linear_x=0.2, angular_z=0.5, frequency=10.0, duration=1.0)
        
        # 停止
        robot.stop_base_drive()
        
        print("底盘控制完成！")
        
    finally:
        robot.shutdown()

if __name__ == "__main__":
    main()
```

# 4. 详细使用说明
## 4.1 机器人初始化流程
**步骤1: 创建控制器**
```python
# 创建基础控制器
robot = RobotController("robot_name")

# 创建轨迹控制器，默认机型为 Q5
# 如使用 L3 或 L7，请将 "Q5" 修改为 "L3" 或 "L7"
trajectory = TrajectoryController(robot, "Q5")

# 创建MPC控制器
mpc = MPCController(robot)
```
**步骤2: 启动关节服务**
```python
# 启动关节服务
success = robot.start_joint_service(
    app_name="my_application",  # 应用名称
    sync_control=False,         # 是否使用同步控制
    launch_mode="pos"          # 启动模式（位置控制）
)

if not success:
    print("启动关节服务失败")
    return
```
**步骤3: 初始化关节**
```python
# 初始化关节（这个过程需要约20秒）
success = robot.initialize_joints(timeout=25.0)

if not success:
    print("关节初始化失败")
    return
```
**步骤4: 验证系统状态**
```python
# 检查机器人是否就绪
if robot.is_ready():
    print("机器人已就绪")
else:
    print("机器人未就绪")
    return

# 获取关节信息
joints_info = robot.get_all_joints_info()
print(f"可用关节: {[joint['name'] for joint in joints_info]}")
```
## 4.2 轨迹控制使用
**预定义轨迹**
```
# 设置零位（所有关节回到零位置）
success = trajectory.set_zero_position(duration=4.0)

# 设置抬升位（机器人抬升到安全位置）
success = trajectory.set_lift_up_position(duration=3.0)
```
**自定义轨迹控制**
```python
# 定义目标位置
target_positions = {
    "left_shoulder_pitch_joint": 0.5,    # 左肩俯仰
    "left_shoulder_roll_joint": -0.3,    # 左肩横滚
    "right_shoulder_pitch_joint": -0.5,  # 右肩俯仰
    "right_shoulder_roll_joint": 0.3,    # 右肩横滚
}

# 同步移动到目标位置
success = trajectory.move_to_joint_positions(
    target_positions=target_positions,
)
异步轨迹控制
def progress_callback(progress):
    """进度回调函数"""
    print(f"执行进度: {progress*100:.1f}%")

def completion_callback(success, result):
    """完成回调函数"""
    if success:
        print(f"轨迹执行完成: {result}")
    else:
        print(f"轨迹执行失败: {result}")

# 异步执行轨迹
success = trajectory.move_to_joint_positions(
    target_positions=target_positions,
    duration=5.0,
    async_execution=True,
    progress_callback=progress_callback,
    completion_callback=completion_callback
)
```
## 4.3 MPC控制使用
**MPC控制流程**
```python
# 1. 激活算法控制权限
if not robot.activate_algorithm_control():
    print("激活算法控制失败")
    return

# 2. 启动MPC
if not mpc.start_mpc():
    print("启动MPC失败")
    return

# 3. 查询MPC状态
status = mpc.query_mpc()
if status == 1:
    print("MPC已启动")
elif status == 0:
    print("MPC已停止")
else:
    print("MPC状态未知")

# 4. 发布ServoPose
def create_pose_stamped(x, y, z, qx, qy, qz, qw, frame_id='base_link'):
    """创建PoseStamped消息"""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position = Point(x=x, y=y, z=z)
    pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return pose

# 创建ServoPose消息
msg = ServoPose()

# 设置左臂位置和姿态
msg.left_pose = create_pose_stamped(
    x=0.425, y=0.3, z=0.292,
    qx=0.612, qy=-0.016, qz=0.79, qw=-0.006
)

# 设置右臂位置和姿态
msg.right_pose = create_pose_stamped(
    x=0.417, y=-0.146, z=0.382,
    qx=0.521, qy=0.1, qz=0.842, qw=-0.096
)

# 设置头部位置和姿态
msg.head_pose = create_pose_stamped(
    x=0.0, y=0.0, z=0.0,
    qx=0.0, qy=0.0, qz=0.0, qw=1.0
)

# 发布ServoPose（50Hz频率，持续3秒）
mpc.publish_servo_pose(msg, rate_hz=50.0, duration=3.0)

# 5. 停止MPC
mpc.stop_mpc()

# 6. 停用算法控制权限
robot.deactivate_algorithm_control()
```

## 4.4 状态监控
**获取关节状态**
```python
# 获取所有关节信息
joints_info = robot.get_all_joints_info()
for joint in joints_info:
    print(f"关节 {joint['name']}:")
    print(f"  位置: {joint['position']:.3f} rad")
    print(f"  速度: {joint['velocity']:.3f} rad/s")
    print(f"  力矩: {joint['effort']:.3f} Nm")

# 获取特定关节信息
joint_info = robot.get_joint_info("left_shoulder_pitch")
if joint_info:
    print(f"左肩俯仰关节: {joint_info}")
```

# 5. API详细说明

## 5.1 RobotController类
|函数|功能|
|---|---|
|RobotController|构造函数|
|start_joint_service|启动关节服务|
|initialize_joints|初始化关节模块|
|get_joint_states|获取当前关节状态|
|get_joint_info|获取特定关节信息|
|get_all_joints_info|获取所有关节信息|
|is_ready|检查机器人是否就绪|
|activate_algorithm_control|激活算法控制权限|
|deactivate_algorithm_control|停用算法控制权限|
|control_hand_joints|控制手部关节|
|control_xhand_lite_grasp|控制XHand Lite抓取动作|
|control_xhand_grasp|控制XHand抓取动作|
|control_base_drive|控制底盘运动|
|stop_base_drive|停止底盘运动|


**构造函数**
```python
RobotController(node_name: str = "robot_controller")
```
**功能**：创建机器人控制器实例。
**参数**:
- node_name: ROS2节点名称

**start_joint_service()**
```python
start_joint_service(app_name: str = "", sync_control: bool = False, launch_mode: str = "pos") -> bool
```
**功能**：启动关节服务。
**参数**:
- app_name: 应用名称
- sync_control: 是否使用同步控制
- launch_mode: 启动模式
**返回**: 
- 成功返回True
- 失败返回False

**initialize_joints()**
```python
initialize_joints(timeout: float = 25.0) -> bool
```
**功能**：初始化关节模块。
**参数**:
- timeout: 超时时间（秒）
**返回**: 成功返回True，失败返回False

**get_joint_states()**
```python
get_joint_states() -> Optional[JointState]
```
**功能**：获取当前关节状态。
**返回**: JointState消息或None

**get_joint_info()**
```python
get_joint_info(joint_name: str) -> Optional[Dict[str, Any]]
```
**功能**：获取特定关节信息。
**参数**:
- joint_name: 关节名称
**返回**: 包含关节信息的字典或None

**get_all_joints_info()**
```python
get_all_joints_info() -> List[Dict[str, Any]]
```
**功能**：获取所有关节信息。

**返回**: 关节信息列表

**is_ready()**
```python
is_ready() -> bool
```
**功能**：检查机器人是否就绪。

**返回**: 就绪返回True，否则返回False

**activate_algorithm_control()**
```python
activate_algorithm_control() -> bool
```
**功能**：激活算法控制权限。

**返回**: 成功返回True，失败返回False

**deactivate_algorithm_control()**
```python
deactivate_algorithm_control() -> bool
```
**功能**：停用算法控制权限。

**返回**: 成功返回True，失败返回False

**control_hand_joints()**
```python
control_hand_joints(joint_names: List[str], positions: List[float], 
                   velocities: Optional[List[float]] = None,
                   feedforward: Optional[List[float]] = None,
                   kp: Optional[List[float]] = None,
                   kd: Optional[List[float]] = None) -> bool
```
**功能**：控制手部关节运动。
**参数**:
- joint_names: 关节名称列表
- positions: 目标位置列表
- velocities: 目标速度列表（可选，默认为0.0）
- feedforward: 前馈力/力矩列表（可选，默认为350.0）
- kp: 比例增益列表（可选，默认为100.0）
- kd: 微分增益列表（可选，默认为0.0）
**返回**: 成功返回True，失败返回False

**control_xhand_lite_grasp()**
```python
control_xhand_lite_grasp() -> bool
```
**功能**：控制XHand Lite执行轻柔抓取动作。
**返回**: 成功返回True，失败返回False

**control_xhand_grasp()**
```python
control_xhand_grasp() -> bool
```
**功能**：控制XHand执行轻柔抓取动作。
**返回**: 成功返回True，失败返回False

**control_base_drive()**
```python
control_base_drive(linear_x: float = 0.0, angular_z: float = 0.0, 
                   frequency: float = None, duration: float = None) -> bool
```
**功能**：控制底盘运动。
**参数**:
- linear_x: X方向线速度（m/s）
- angular_z: Z方向角速度（rad/s）
- frequency: 发布频率（Hz）。如果提供此参数，必须同时提供duration参数
- duration: 发布持续时间（秒）。如果提供此参数，必须同时提供frequency参数
**返回**: 成功返回True，失败返回False

**注意**：
- 如果不提供frequency和duration参数，命令只会发送一次
- 如果提供frequency和duration参数，会在指定时间内以指定频率持续发布命令

**stop_base_drive()**
```python
stop_base_drive() -> bool
```
**功能**：停止底盘运动。
**返回**: 成功返回True，失败返回False

## 5.2 TrajectoryController类
|函数|功能|
|---|---|
|TrajectoryController|构造函数|
|set_zero_position|设置关节到零位|
|set_lift_up_position|设置关节到抬升位|
|move_to_joint_positions|移动关节到目标位置|


**构造函数**
```python
TrajectoryController(robot_controller, model: str)
```
**功能**：创建轨迹控制器实例。

**参数**:
- robot_controller: RobotController实例
- model: 机器人机型名称，当前支持 "Q5"、"L3"、"L7"。

**set_zero_position()**
```python
set_zero_position(duration: float = 4.0) -> bool
```
**功能**：设置关节到零位。

**参数**: 
- duration: 执行时间（秒）
**返回**: 成功返回True，失败返回False

**set_lift_up_position()**
```python
set_lift_up_position(duration: float = 4.0) -> bool
```
**功能**：设置关节到抬升位。

**参数**:
- duration: 执行时间（秒）
**返回**: 成功返回True，失败返回False

**move_to_joint_positions**
```python
move_to_joint_positions(target_positions: Dict[str, float], duration: float = 3.0, 
                       kp: Optional[List[float]] = None, kd: Optional[List[float]] = None, 
                       rate_hz: float = 100.0, async_execution: bool = False, 
                       progress_callback: Optional[Callable] = None, 
                       completion_callback: Optional[Callable] = None) -> bool
```

**功能**：移动关节到目标位置。

**参数**:
- target_positions: 目标位置字典
- duration: 执行时间（秒）
- kp: 比例增益列表
- kd: 微分增益列表
- rate_hz: 发布频率
- async_execution: 是否异步执行
- progress_callback: 进度回调函数
- completion_callback: 完成回调函数
**返回**: 成功返回True，失败返回False

## 5.3 MPCController类
|函数|功能|
|---|---|
|MPCController|构造函数|
|start_mpc|启动MPC控制|
|stop_mpc|停止MPC控制|
|query_mpc|查询MPC状态|
|publish_servo_pose|移动到某个位置|
|||

**构造函数**
```python
MPCController(robot_controller: Node)
```
**功能**：创建MPC控制器实例。

**参数**:
- robot_controller: RobotController节点实例

**start_mpc()**
```python
start_mpc() -> bool
```
**功能**：启动MPC控制。
**返回**: 成功返回True，失败返回False

**stop_mpc()**
```python
stop_mpc() -> bool
```
**功能**：停止MPC控制。

**返回**: 成功返回True，失败返回False

**query_mpc()**
```python
query_mpc() -> Optional[int]
```
**功能**：查询MPC状态。

**返回**: 1表示已启动，0表示已停止，None表示查询失败

**publish_servo_pose()**
```python
publish_servo_pose(msg: ServoPose, rate_hz: float = 10.0, duration: float = 2.0)
```
**
**功能**：移动到某个位置

**参数**:
- msg: ServoPose消息
- rate_hz: 发布频率
- duration: 发布持续时间（秒）
