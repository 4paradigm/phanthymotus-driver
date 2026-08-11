#!/usr/bin/env python3
import sys
import random
import signal
import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import (PoseStamped, Pose, Point)
from era_nav_msgs.action import Navigate


class NavActionClient(Node):

  def __init__(self, node_name="nav_action_client"):
    super().__init__('nav_action_client')
    self._client = ActionClient(self, Navigate, '/era_nav/nav_act')
    self.goal_handle = None  # keep the handle for cancel
    self.result_future = None
    self._interrupt_flag = False

  def is_busy(self):
    return (self.goal_handle is not None)

  def req_nav_to_global_node(self,
                             node_id=None,
                             attr=None,
                             value=None,
                             feedback_cb=None,
                             done_cb=None,
                             advanced_params=None):
    nav_req = self._new_nav_req()

    if (node_id is not None) and (node_id >= 0):
      nav_req.nav_type = "NavToGlobalNode"
      nav_req.goal_node_id = int(node_id)
      self.get_logger().info(
          f"Sending request to nav to the global node with id={node_id}")
    elif attr is not None and value is not None:
      nav_req.nav_type = "NavToGlobalNode"
      nav_req.goal_node_id = -1
      nav_req.goal_attr = attr
      nav_req.goal_attr_value = value
      self.get_logger().info(
          f"Sending request to nav to the global node with {attr} = {value}")
    else:
      self.get_logger().error(
          "Invalid request to nav to global node. Please provide node_id or attr:value pair to specify the goal node."
      )
      return False

    goal_handle = self._send_action_goal(nav_req,
                                         feedback_cb=feedback_cb,
                                         done_cb=done_cb)
    return (goal_handle and goal_handle.accepted)

  def req_nav_to_global_pose(self,
                             goal_pose: Pose = None,
                             ref_path=None,
                             feedback_cb=None,
                             done_cb=None,
                             advanced_params=None):
    nav_req = self._new_nav_req()

    if goal_pose is not None:
      nav_req.nav_type = "NavToGlobalPose"
      nav_req.goal = goal_pose
      nav_req.ref_path = ref_path
      nav_req.goal_ready = True
      self.get_logger().info("Sending request to nav to global pose.")
    else:
      nav_req.nav_type = "NavToGlobalPose"
      nav_req.goal_ready = False
      self.get_logger().info(
          "Sending request to nav to global pose (the pose will be provided later)."
      )

    goal_handle = self._send_action_goal(nav_req,
                                         feedback_cb=feedback_cb,
                                         done_cb=done_cb)
    return (goal_handle and goal_handle.accepted)

  def req_nav_to_local_pose(self,
                            goal_pose: Pose = None,
                            ref_path=None,
                            feedback_cb=None,
                            done_cb=None,
                            advanced_params=None):
    nav_req = self._new_nav_req()
    nav_req.nav_type = "NavToLocalPose"

    if goal_pose is not None:
      nav_req.goal = goal_pose
      nav_req.ref_path = ref_path
      nav_req.goal_ready = True
      self.get_logger().info("Sending request to nav to local pose.")
    else:
      nav_req.goal_ready = False
      self.get_logger().info(
          "Sending request to nav to local pose (the pose will be provided later)."
      )

    goal_handle = self._send_action_goal(nav_req,
                                         feedback_cb=feedback_cb,
                                         done_cb=done_cb)
    return (goal_handle and goal_handle.accepted)

  def req_follow_object(self,
                        object_type="",
                        object_id=0,
                        feedback_cb=None,
                        done_cb=None,
                        advanced_params=None):
    nav_req = self._new_nav_req()
    nav_req.nav_type = "FollowObject"
    nav_req.object_type = object_type
    nav_req.object_id = object_id
    goal_handle = self._send_action_goal(nav_req,
                                         feedback_cb=feedback_cb,
                                         done_cb=done_cb)
    return (goal_handle and goal_handle.accepted)

  def req_dock_to_station(self,
                          station_type="",
                          station_id=0,
                          feedback_cb=None,
                          done_cb=None,
                          advanced_params=None):

    nav_req = self._new_nav_req()
    nav_req.nav_type = "DockToStation"
    nav_req.station_type = station_type
    nav_req.station_id = station_id
    goal_handle = self._send_action_goal(nav_req,
                                         feedback_cb=feedback_cb,
                                         done_cb=done_cb)
    return (goal_handle and goal_handle.accepted)

  def req_cancel_external_nav(self, feedback_cb=None, done_cb=None):
    # Cancel nav requests started by other clients
    nav_req = self._new_nav_req()
    nav_req.request_id = -1
    nav_req.nav_type = "CancelNav"
    goal_handle = self._send_action_goal(nav_req,
                                         feedback_cb=feedback_cb,
                                         done_cb=done_cb)
    return (goal_handle and goal_handle.accepted)

  def cancel_req(self):
    # cancel the current request if any
    if self.goal_handle:
      self.get_logger().info("Cancelling the current request...")
      cancel_future = self.goal_handle.cancel_goal_async()
      if not self._wait_future(cancel_future):
        self.get_logger().error("Failed to get cancel result!")
        return False

      cancel_response = cancel_future.result()
      self.get_logger().info("Cancel-Response: {cancel_response}")
      if cancel_response.return_code != 0:
        self.get_logger().warn(
            "Cancel-request was rejected by the action server.")
        return False
      else:
        self.get_logger().info("Cancel-request was accepted.")
        return True

    else:
      self.get_logger().warn("No active request to cancel.")
      return False

  def get_latest_result(self, print_result=False):
    if (not self.result_future) or (not self.result_future.done()):
      return None
    result_msg = self.result_future.result()
    self._reset(
    )  # We've finish the action, reset the client so that we can send a new request.
    return self._return_and_log_result(result_msg, print_result=print_result)

  def wait_for_result(self, timeout_sec=-1, poll=0.05, print_result=False):
    if self.result_future and self._wait_future(self.result_future,
                                                timeout_sec, poll):
      result_msg = self.result_future.result()
      self._reset(
      )  # We've finish the action, reset the client so that we can send a new request.
      return self._return_and_log_result(result_msg, print_result=print_result)
    else:
      return None

  def interrupt(self):
    self._interrupt_flag = True

  def clear_interrupt(self):
    self._interrupt_flag = False

  def has_interrupt(self):
    return self._interrupt_flag

  @staticmethod
  def is_req_success(res):
    if not res:
      return False
    return res.code == 0

  @staticmethod
  def is_req_cancelled(res):
    if not res:
      return False
    return res.code == 1

  @staticmethod
  def _new_nav_req(advanced_params=None):
    nav_req = Navigate.Goal()
    nav_req.request_id = random.randint(1, 2**63 - 1)
    NavActionClient._set_advanced_params(nav_req, advanced_params)
    return nav_req
  
  def _internal_done_cb(self, result_future):
    assert(self.result_future is result_future)
    self._reset()  # Auto reset the client when the action is done.
    self._extern_done_cb(result_future)  # Call the external done_cb

  def _send_action_goal(self, nav_req, feedback_cb=None, done_cb=None):
    if not feedback_cb:
      feedback_cb = NavActionClient._default_feedback_cb
    if not done_cb:
      done_cb = NavActionClient._default_done_cb
    self._extern_done_cb = done_cb

    # wait for action server
    if not self._wait_for_server():
      self.get_logger().error("Action server /era_nav/nav_act not available.")
      return None

    if self.is_busy():
      self.get_logger().warn("Action client is busy!")
      return None

    # 1) send nav_req (async) and wait for goal_handle
    send_future = self._client.send_goal_async(nav_req,
                                               feedback_callback=feedback_cb)
    if not self._wait_future(send_future):
      self.get_logger().error("Failed to get goal_handle!")
      return None

    goal_handle = send_future.result()
    if not goal_handle.accepted:
      self.get_logger().warn("Request was rejected by the action server.")
      return None

    self.goal_handle = goal_handle
    self.result_future = self.goal_handle.get_result_async()
    self.result_future.add_done_callback(self._internal_done_cb)
    return goal_handle

  def _reset(self):
    self.goal_handle = None

  def _return_and_log_result(self, result_msg, print_result=True):
    # if result_msg is None:
    #   # This should not happen.
    #   self.get_logger().error("Action call returned no result!")
    #   return None
    res = result_msg.result
    if print_result:
      self.get_logger().info(
          f"Result: request_id={res.request_id}, code={res.code}, message='{res.message}'"
      )
      # check if the current request is to cancel external nav tasks (started by other clients)
      is_cancel_external_req = (res.request_id < 0)

      # if not, it's a normal nav request, log the result.
      if not is_cancel_external_req:
        if self.is_req_success(res):
          self.get_logger().info("Navigation succeeded.")
        elif self.is_req_cancelled(res):
          self.get_logger().info("Navigation canceled.")
        else:
          self.get_logger().warn(f"Navigation failed!")

    return res

  def _wait_for_server(self, timeout_sec=-1, poll=0.05):
    start_time = time.time()
    while rclpy.ok() and (not self.has_interrupt()):
      if self._client.wait_for_server(timeout_sec=0):
        return True
      rclpy.spin_once(self, timeout_sec=poll)
      if timeout_sec > 0 and (time.time() - start_time) > timeout_sec:
        return False
    return False

  def _wait_future(self, future, timeout_sec=-1, poll=0.05):
    # rclpy.spin_until_future_complete(self, future)  # blocking
    start_time = time.time()
    while rclpy.ok() and (not future.done()) and (not self.has_interrupt()):
      if timeout_sec > 0 and (time.time() - start_time) > timeout_sec:
        break
      rclpy.spin_once(self, timeout_sec=poll)

    rclpy.spin_once(
        self, timeout_sec=poll
    )  # Wait a bit more to make sure the done callback is called
    return future.done()

  @staticmethod
  def _default_feedback_cb(feedback_msg):
    fb = feedback_msg.feedback
    # Print feedback
    print(f"Navigation Feedback callback: {fb}")

  @staticmethod
  def _default_done_cb(result_future):
    result_msg = result_future.result()
    res = result_msg.result
    print(
        f"Navigation Done callback: Result: request_id={res.request_id}, code={res.code}, message='{res.message}'"
    )

    # check if the current request is to cancel external nav tasks (started by other clients)
    is_cancel_external_req = (res.request_id < 0)

    # if not, it's a normal nav request, log the result.
    if not is_cancel_external_req:
      if NavActionClient.is_req_success(res):
        print("Navigation succeeded.")
      elif NavActionClient.is_req_cancelled(res):
        print("Navigation canceled.")
      else:
        print(f"Navigation failed!")

  @staticmethod
  def _set_advanced_params(nav_req, advanced_params):
    # First set default values
    nav_req.parking_position_tolerance = -1.0
    nav_req.parking_angle_tolerance = -1.0
    nav_req.max_speed = -1.0
    nav_req.max_acceleration = -1.0
    nav_req.max_angular_speed = -1.0
    nav_req.max_angular_acceleration = -1.0
    nav_req.max_centrifugal = -1.0

    # Then override with user-provided values
    if advanced_params is not None:
      if "parking_position_tolerance" in advanced_params:
        nav_req.parking_position_tolerance = advanced_params[
            "parking_position_tolerance"]
      if "parking_angle_tolerance" in advanced_params:
        nav_req.parking_angle_tolerance = advanced_params[
            "parking_angle_tolerance"]
      if "max_speed" in advanced_params:
        nav_req.max_speed = advanced_params["max_speed"]
      if "max_acceleration" in advanced_params:
        nav_req.max_acceleration = advanced_params["max_acceleration"]
      if "max_angular_speed" in advanced_params:
        nav_req.max_angular_speed = advanced_params["max_angular_speed"]
      if "max_angular_acceleration" in advanced_params:
        nav_req.max_angular_acceleration = advanced_params[
            "max_angular_acceleration"]
      if "max_centrifugal" in advanced_params:
        nav_req.max_centrifugal = advanced_params["max_centrifugal"]


########### Demos ###########


def print_usage():
  print("Usage:  (start-cmd = 'python[3] -m era_nav_pyclient.NavActionClient')")
  print("    <start-cmd> NavToGlobalNode <node_id>  OR <attr> <value>")
  print(
      "    <start-cmd> NavToGlobalPose  (The pose needs to be provided externally, e.g. from rviz)"
  )
  print(
      "    <start-cmd> NavToLocalPose   (The pose needs to be provided externally, e.g. from rviz)"
  )
  print("    <start-cmd> FollowObject  [object_type='']  [object_id=0]")
  print(
      "    <start-cmd> DockToStation [station_type=''] [station_id=0]")
  print(
      "    <start-cmd> CancelNav        (To cancel a nav task started by other client)"
  )


def wait_req_success(client: NavActionClient,
                     accepted=True,
                     cancel_if_interrupted=True):
  if (not accepted):
    print("Navigation request was rejected or interrupted!")
    return False

  result_ok = (client.wait_for_result() is not None)
  if (not result_ok) and client.has_interrupt() and cancel_if_interrupted:
    print("Cancelling navigation... (Press Ctrl+C again to force shutdown)")
    client.clear_interrupt(
    )  # Reset interrupt flag before canceling. Ctrl+C again during canceling to force shutdown.
    if client.cancel_req() and (client.wait_for_result() is not None):
      print("Navigation request cancelled.")
    else:
      print("Navigation request may not have been cancelled! Shutting down.")

  return client.is_req_success(client.get_latest_result())


def nav_to_global_node(client: NavActionClient,
                       node_id=None,
                       attr=None,
                       value=None):
  return wait_req_success(
      client,
      client.req_nav_to_global_node(node_id=node_id, attr=attr, value=value))


def nav_to_global_node_demo(client: NavActionClient, args):
  if len(args) < 1:
    print_usage()
    return 1

  if args[0].lstrip('-').isdigit():
    ok = nav_to_global_node(client, node_id=int(args[0]))
  else:
    if len(args) < 2:
      print_usage()
      return 1
    ok = nav_to_global_node(client, attr=args[0], value=args[1])

  if not ok:
    return 2
  return 0


def nav_to_global_pose_demo(client: NavActionClient, args):
  # The goal pose needs to be provided externally (e.g. from rviz)
  accepted = client.req_nav_to_global_pose()
  if accepted:
    print("Please provide a goal pose externally (e.g. from rviz) ...")
    ok = wait_req_success(client)
    if not ok:
      return 2
  else:
    print("Navigation request was rejected or interrupted!")
    return 2

  return 0

def nav_to_local_pose_demo(client: NavActionClient, args):
  # The goal pose needs to be provided externally (e.g. from rviz)
  accepted = client.req_nav_to_local_pose()
  if accepted:
    print("Please provide a goal pose externally (e.g. from rviz) ...")
    ok = wait_req_success(client)
    if not ok:
      return 2
  else:
    print("Navigation request was rejected or interrupted!")
    return 2

  return 0


def follow_object_demo(client: NavActionClient, args):
  object_type = ""
  object_id = 0
  if len(args) > 0:
    object_type = args[0]
  if len(args) > 1:
    object_id = int(args[1])
  accepted = client.req_follow_object(object_type, object_id)
  if accepted:
    print(
        "Please provide the object pose or the goal pose externally (e.g. from rviz) ..."
    )
    ok = wait_req_success(client)
    if not ok:
      return 2
  else:
    print("Navigation request was rejected or interrupted!")
    return 2
  
  return 0

def dock_to_station_demo(client: NavActionClient, args):
  station_type = ""
  station_id = 0
  if len(args) > 0:
    station_type = args[0]
  if len(args) > 1:
    station_id = int(args[1])
  accepted = client.req_dock_to_station(station_type, station_id)
  if accepted:
    print(
        "Please provide the station pose or the goal pose externally (e.g. from rviz) ..."
    )
    ok = wait_req_success(client)
    if not ok:
      return 2
  else:
    print("Navigation request was rejected or interrupted!")
    return 2
  
  return 0

def cancel_external_nav_demo(client: NavActionClient, args):
  if wait_req_success(client, client.req_cancel_external_nav()):
    print("External navigation cancelled.")
  else:
    print("External navigation may not have been cancelled!")
    return 2

  return 0


def main():
  rclpy.init()

  if len(sys.argv) < 2:
    print_usage()
    sys.exit(1)

  client = NavActionClient("nav_action_pydemo")

  # Create the executor and the spinning thread
  executor = SingleThreadedExecutor()
  executor.add_node(client)
  executor_thread = threading.Thread(target=executor.spin)
  executor_thread.start()

  # Disable the default Ctrl+C shutdown behavior by setting a custom signal handler
  def handle_shutdown_signal(signum, frame):
    """Handle Ctrl+C (SIGINT) signal"""
    print("Ctrl+C detected. Cancelling navigation and shutting down.")
    client.interrupt()  # Send interrupt signal to cancel navigation. See wait_req_success() for details.

  rclpy.signals.uninstall_signal_handlers()  # prevent ROS default shutdown so that we can cancel navigation if Ctrl+C is pressed
  signal.signal(signal.SIGINT, handle_shutdown_signal)

  if sys.argv[1] == "NavToGlobalNode":
    exit_code = nav_to_global_node_demo(client, sys.argv[2:])
  elif sys.argv[1] == "NavToGlobalPose":
    exit_code = nav_to_global_pose_demo(client, sys.argv[2:])
  elif sys.argv[1] == "NavToLocalPose":
    exit_code = nav_to_local_pose_demo(client, sys.argv[2:])
  elif sys.argv[1] == "FollowObject":
    exit_code = follow_object_demo(client, sys.argv[2:])
  elif sys.argv[1] == "DockToStation":
    exit_code = dock_to_station_demo(client, sys.argv[2:])
  elif sys.argv[1] == "CancelNav":
    exit_code = cancel_external_nav_demo(client, sys.argv[2:])
  else:
    print("Unknown command: ", sys.argv[1])
    exit_code = 1

  client.destroy_node()
  executor.shutdown()
  executor_thread.join()
  rclpy.shutdown()

  sys.exit(exit_code)

if __name__ == "__main__":
  main()
