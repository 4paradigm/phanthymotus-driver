"""Send a bounded joint-goal request to the controller host's MoveIt action."""

from hardware import JOINT_LOWER, JOINT_MARGIN, JOINT_UPPER

JOINT_PATH_BOUNDS = [(lo + .05, hi - .05) for lo, hi in zip(JOINT_LOWER, JOINT_UPPER)]


_PREPARE_CLIENTS = '''
import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetPositionIK

node = rclpy.create_node("fr3_pick_planning")
clients = {
    "fk": node.create_client(GetPositionFK, "/compute_fk"),
    "ik": node.create_client(GetPositionIK, "/compute_ik"),
    "cartesian": node.create_client(GetCartesianPath, "/compute_cartesian_path"),
    "plan": ActionClient(node, MoveGroup, "/move_action"),
}
for name, timeout in (("fk", 4), ("ik", 4), ("cartesian", 5)):
    if not clients[name].wait_for_service(timeout_sec=timeout):
        raise RuntimeError("MoveIt service unavailable: " + name)
if not clients["plan"].wait_for_server(timeout_sec=8):
    raise RuntimeError("MoveIt planning action is unavailable")
worker_state["node"] = node
worker_state["clients"] = clients
print("FR3_RESULT {}")
'''


_FK = '''
import math
import rclpy
from moveit_msgs.srv import GetPositionFK

node = worker_state["node"]
client = worker_state["clients"]["fk"]
if not client.wait_for_service(timeout_sec=4):
    raise RuntimeError("MoveIt FK service is unavailable")
query = GetPositionFK.Request()
query.header.frame_id = request["base_frame"]
query.fk_link_names = [request["link"]]
query.robot_state.joint_state.name = [f"fr3_joint{i}" for i in range(1, 8)]
query.robot_state.joint_state.position = request["joints"]
future = client.call_async(query)
rclpy.spin_until_future_complete(node, future, timeout_sec=4)
if not future.done():
    raise TimeoutError("MoveIt FK did not respond")
result = future.result()
if result.error_code.val != 1 or len(result.pose_stamped) != 1:
    raise RuntimeError("MoveIt FK failed")
pose = result.pose_stamped[0]
if pose.header.frame_id != request["base_frame"]:
    raise RuntimeError("MoveIt FK returned a different reference frame")
p, q = pose.pose.position, pose.pose.orientation
values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
if not all(math.isfinite(value) for value in values):
    raise RuntimeError("MoveIt FK returned a non-finite pose")
print("FR3_RESULT " + json.dumps({"pose": values}), flush=True)
'''


_PLAN = '''
import rclpy
from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

def wait(node, future, timeout):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise TimeoutError("MoveIt did not respond")
    return future.result()

node = worker_state["node"]
client = worker_state["clients"]["plan"]
if not client.wait_for_server(timeout_sec=8):
    raise RuntimeError("MoveIt planning action is unavailable")
goal = MoveGroup.Goal()
goal.request.group_name = "fr3_arm"
goal.request.path_constraints.joint_constraints = [
    JointConstraint(joint_name=f"fr3_joint{i}", position=(lo + hi) / 2,
                    tolerance_above=(hi - lo) / 2, tolerance_below=(hi - lo) / 2, weight=1.)
    for i, (lo, hi) in enumerate(request.get("joint_bounds", []), 1)]

goal.request.allowed_planning_time = 5.0
goal.request.start_state.is_diff = True
if "start" in request:
    goal.request.start_state.joint_state.name = [f"fr3_joint{i}" for i in range(1, 8)]
    goal.request.start_state.joint_state.position = request["start"]
goal.request.max_velocity_scaling_factor = 0.1
goal.request.max_acceleration_scaling_factor = 0.1
goal.planning_options.plan_only = True
goal.planning_options.planning_scene_diff.is_diff = True
goal.planning_options.planning_scene_diff.robot_state.is_diff = True
goal.request.goal_constraints = [Constraints(joint_constraints=[
    JointConstraint(joint_name=f"fr3_joint{i}", position=position,
                    tolerance_above=0.001, tolerance_below=0.001, weight=1.0)
    for i, position in enumerate(request["joints"], 1)])]
handle = wait(node, client.send_goal_async(goal), 5)
if not handle.accepted:
    raise RuntimeError("MoveIt rejected the planning goal")
terminal = handle.get_result_async()
try:
    outcome = wait(node, terminal, 10)
except TimeoutError:
    cancel = wait(node, handle.cancel_goal_async(), 3)
    if not cancel.goals_canceling and not terminal.done():
        raise RuntimeError("MoveIt did not acknowledge cancellation")
    outcome = wait(node, terminal, 4)
response = outcome.result
trajectory = response.planned_trajectory.joint_trajectory
print("FR3_RESULT " + json.dumps({
    "code": response.error_code.val,
    "succeeded": outcome.status == GoalStatus.STATUS_SUCCEEDED,
    "planning_s": response.planning_time,
    "joint_names": list(trajectory.joint_names),
    "points": [{
        "positions": list(point.positions),
        "velocities": list(point.velocities),
        "accelerations": list(point.accelerations),
        "time_ns": point.time_from_start.sec * 1000000000 + point.time_from_start.nanosec,
    } for point in trajectory.points],
}), flush=True)
'''


_CARTESIAN = '''
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import JointConstraint

node = worker_state["node"]
client = worker_state["clients"]["cartesian"]
if not client.wait_for_service(timeout_sec=5):
    raise RuntimeError("MoveIt Cartesian path service is unavailable")
query = GetCartesianPath.Request()
query.header.frame_id = request["frame"]
query.group_name = "fr3_arm"
query.link_name = request["link"]
query.start_state.is_diff = True
if "start" in request:
    query.start_state.joint_state.name = [f"fr3_joint{i}" for i in range(1, 8)]
    query.start_state.joint_state.position = request["start"]
query.max_step = 0.005
query.jump_threshold = 1.5
query.avoid_collisions = True
query.path_constraints.joint_constraints = [
    JointConstraint(joint_name=f"fr3_joint{i}", position=(lo + hi) / 2,
                    tolerance_above=(hi - lo) / 2, tolerance_below=(hi - lo) / 2, weight=1.)
    for i, (lo, hi) in enumerate(request.get("joint_bounds", []), 1)]
query.max_velocity_scaling_factor = request["speed_scale"]
query.max_acceleration_scaling_factor = request["speed_scale"]
for x, y, z, qx, qy, qz, qw in request["poses"]:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
    query.waypoints.append(pose)
future = client.call_async(query)
rclpy.spin_until_future_complete(node, future, timeout_sec=10)
if not future.done():
    raise TimeoutError("MoveIt Cartesian planning timed out")
response = future.result()
trajectory = response.solution.joint_trajectory
print("FR3_RESULT " + json.dumps({
    "code": response.error_code.val,
    "fraction": response.fraction,
    "joint_names": list(trajectory.joint_names),
    "points": [{
        "positions": list(point.positions),
        "velocities": list(point.velocities),
        "accelerations": list(point.accelerations),
        "time_ns": point.time_from_start.sec * 1000000000 + point.time_from_start.nanosec,
    } for point in trajectory.points],
}), flush=True)
'''


_IK = '''
import rclpy
from moveit_msgs.srv import GetPositionIK

node = worker_state["node"]
client = worker_state["clients"]["ik"]
if not client.wait_for_service(timeout_sec=4):
    raise RuntimeError("MoveIt IK service unavailable")
query = GetPositionIK.Request()
ik = query.ik_request
ik.group_name, ik.ik_link_name = "fr3_arm", "fr3_link8"
ik.pose_stamped.header.frame_id = "fr3_link0"
x, y, z, qx, qy, qz, qw = request["pose"]
p, q = ik.pose_stamped.pose.position, ik.pose_stamped.pose.orientation
p.x, p.y, p.z = x, y, z
q.x, q.y, q.z, q.w = qx, qy, qz, qw
ik.avoid_collisions = True
ik.timeout.sec = 1
ik.robot_state.joint_state.name = [f"fr3_joint{i}" for i in range(1, 8)]
ik.robot_state.joint_state.position = request["start"]
future = client.call_async(query)
rclpy.spin_until_future_complete(node, future, timeout_sec=4)
if not future.done():
    raise TimeoutError("MoveIt IK timed out")
result = future.result()
joints = result.solution.joint_state
print("FR3_RESULT " + json.dumps(dict(code=result.error_code.val,
      joints=dict(zip(joints.name, joints.position)))), flush=True)
'''
