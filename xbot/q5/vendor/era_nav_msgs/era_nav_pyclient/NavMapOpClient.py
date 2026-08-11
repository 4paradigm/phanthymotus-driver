#!/usr/bin/env python3
import sys
import random
import rclpy
import time
import math
import threading
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Point
from era_nav_msgs.srv import NavMapOp
from era_nav_msgs.msg import (TopoNode, TopoEdge, ForbiddenArea, Shape2d)

class NavMapOpClient(Node):

  def __init__(self, node_name="nav_map_op_client"):
    super().__init__(node_name)
    self._client = self.create_client(NavMapOp, '/era_nav/nav_map_op')
    self._interrupt_flag = False

  def req_load_map(self,
                   map_name: str,
                   force_reload: bool = True,
                   done_cb=None):
    req = self.new_req_()
    req.op_type = "LoadMap"
    req.map_name = map_name
    req.force_reload = force_reload
    return self._send_req(req, done_cb=done_cb)

  def req_save_map(self, map_name: str, done_cb=None):
    req = self.new_req_()
    req.op_type = "SaveMap"
    req.map_name = map_name
    return self._send_req(req, done_cb=done_cb)

  def req_clear_map(self, done_cb=None):
    req = self.new_req_()
    req.op_type = "ClearMap"
    return self._send_req(req, done_cb=done_cb)

  def req_read_map(self, map_name: str = "", done_cb=None):
    req = self.new_req_()
    req.op_type = "ReadMap"
    req.map_name = map_name
    return self._send_req(req, done_cb=done_cb)

  def req_start_recording_path(self, done_cb=None):
    req = self.new_req_()
    req.op_type = "StartRecordingPath"
    return self._send_req(req, done_cb=done_cb)

  def req_mark_user_node_on_new_path(self,
                                     record_orientation: bool = True,
                                     business_attr_dict={},
                                     done_cb=None):
    req = self.new_req_()
    req.op_type = "MarkUserNodeOnNewPath"
    req.record_orientation = record_orientation
    req.business_attr_keys = list(business_attr_dict.keys())
    req.business_attr_values = list(business_attr_dict.values())
    return self._send_req(req, done_cb=done_cb)

  def req_finish_recording_path(self,
                                auto_connection_radius=0.0,
                                done_cb=None):
    req = self.new_req_()
    req.op_type = "FinishRecordingPath"
    req.auto_connection_radius = auto_connection_radius
    return self._send_req(req, done_cb=done_cb)

  def req_cancel_recording_path(self, done_cb=None):
    req = self.new_req_()
    req.op_type = "CancelRecordingPath"
    return self._send_req(req, done_cb=done_cb)

  def req_record_user_node(self,
                           record_orientation: bool = True,
                           business_attr_dict={},
                           auto_connection_radius=0.0,
                           done_cb=None):
    req = self.new_req_()
    req.op_type = "RecordUserNode"
    req.record_orientation = record_orientation
    req.auto_connection_radius = auto_connection_radius
    req.business_attr_keys = list(business_attr_dict.keys())
    req.business_attr_values = list(business_attr_dict.values())
    return self._send_req(req, done_cb=done_cb)

  def req_record_forbidden_area(self,
                                area_front_distance,
                                area_length,
                                area_width,
                                area_min_h=-2.0,
                                area_max_h=2.0,
                                business_attr_dict={},
                                done_cb=None):
    req = self.new_req_()
    req.op_type = "RecordForbiddenArea"
    req.area_front_distance = area_front_distance
    req.area_length = area_length
    req.area_width = area_width
    req.area_min_h = area_min_h
    req.area_max_h = area_max_h
    req.business_attr_keys = list(business_attr_dict.keys())
    req.business_attr_values = list(business_attr_dict.values())
    return self._send_req(req, done_cb=done_cb)

  def req_override_map(self,
                       nodes=[],
                       edges=[],
                       forbidden_areas=[],
                       done_cb=None):
    req = self.new_req_()
    req.op_type = "OverrideMap"
    req.nodes = nodes
    req.edges = edges
    req.forbidden_areas = forbidden_areas
    return self._send_req(req, done_cb=done_cb)

  def req_update_map_elements(self,
                              nodes=[],
                              edges=[],
                              forbidden_areas=[],
                              done_cb=None):
    req = self.new_req_()
    req.op_type = "UpdateMapElements"
    req.nodes = nodes
    req.edges = edges
    req.forbidden_areas = forbidden_areas
    return self._send_req(req, done_cb=done_cb)

  def req_remove_map_elements(self,
                              nodes=[],
                              edges=[],
                              forbidden_areas=[],
                              done_cb=None):
    req = self.new_req_()
    req.op_type = "RemoveMapElements"
    req.nodes = nodes
    req.edges = edges
    req.forbidden_areas = forbidden_areas
    return self._send_req(req, done_cb=done_cb)

  def wait_for_response(self,
                        resp_future,
                        timeout_sec=-1,
                        poll=0.05,
                        print_response=False):
    if self._wait_future(resp_future, timeout_sec, poll):
      return self._return_and_log_response(resp_future.result())
    else:
      return None

  def interrupt(self):
    self._interrupt_flag = True

  def clear_interrupt(self):
    self._interrupt_flag = False

  def has_interrupt(self):
    return self._interrupt_flag

  @staticmethod
  def new_req_():
    req = NavMapOp.Request()
    req.request_id = random.randint(1, 2**63 - 1)
    return req

  def _return_and_log_response(self, resp, print_response=True):
    if resp is None:
      self.get_logger().error("Service call returned no result!")
      return None
    if print_response:
      self.get_logger().info(
          f"Response: request_id={resp.request_id}, code={resp.code}, message='{resp.message}'"
      )
    return resp

  def _send_req(self, req, done_cb=None):
    if done_cb is None:
      done_cb = NavMapOpClient._default_done_cb

    if not self._wait_for_server():
      self.get_logger().error("Service /era_nav/nav_map_op not available.")
      return None
    resp_future = self._client.call_async(req)
    if done_cb is not None:
      resp_future.add_done_callback(done_cb)
    return resp_future

  def _wait_for_server(self, timeout_sec=-1, poll=0.05):
    start_time = time.time()
    while rclpy.ok() and (not self.has_interrupt()):
      if self._client.wait_for_service(timeout_sec=poll):
        return True
      rclpy.spin_once(self, timeout_sec=0)
      if timeout_sec > 0 and (time.time() - start_time) > timeout_sec:
        break
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
  def _default_done_cb(resp_future):
    resp = resp_future.result()
    print(
        f"NavMapOp Done callback: Response: request_id={resp.request_id}, code={resp.code}, message='{resp.message}'"
    )


########### Demos ###########
def print_usage():
  print("Usage:  (start-cmd = 'python[3] -m era_nav_pyclient.NavMapOpClient')")
  print("    <start-cmd> LoadMap <map_name> [force_reload=true|false]")
  print("    <start-cmd> SaveMap <map_name>")
  print("    <start-cmd> ClearMap")
  print("    <start-cmd> ReadMap")
  print("    <start-cmd> StartRecordingPath")
  print(
      "    <start-cmd> MarkUserNodeOnNewPath <node_name> [record_orientation=true|false]"
  )
  print("    <start-cmd> FinishRecordingPath <auto_connection_radius>")
  print("    <start-cmd> CancelRecordingPath")
  print(
      "    <start-cmd> RecordUserNode <node_name> <auto_connection_radius> [record_orientation=true|false]"
  )
  print(
      "    <start-cmd> RecordForbiddenArea <area_front_distance> <area_length> <area_width> [area_min_h=-2.0] [area_max_h=2.0]"
  )
  print("    <start-cmd> OverrideMap             (use testing data)")
  print("    <start-cmd> UpdateMapElements       (use testing data)")
  print("    <start-cmd> RemoveMapElements       (use testing data)")

def load_map_demo(client, args):
  if len(args) < 1:
    print_usage()
    return 1
  map_name = args[0]
  force_reload = True
  if len(args) >= 2:
    force_reload = args[1].lower() == 'true'
  resp = client.wait_for_response(client.req_load_map(map_name, force_reload))
  if resp is None:
    return 2
  if resp.code < 0:  # The operation failed
    return 3
  
  return 0


def save_map_demo(client, args):
  if len(args) < 1:
    print_usage()
    return 1
  map_name = args[0]
  resp = client.wait_for_response(client.req_save_map(map_name))
  if resp is None:
    return 2
  if resp.code < 0:  # The operation failed
    return 3
  
  return 0

def clear_map_demo(client, args):
  resp = client.wait_for_response(client.req_clear_map())
  if resp is None:
    return 2
  if resp.code < 0:  # The operation failed
    return 3
  
  return 0


def read_map_demo(client, args):
  map_name = ""
  if len(args) > 0:
    map_name = args[0]
  resp = client.wait_for_response(client.req_read_map(map_name))

  if resp is None:
    return 2
  if resp.code < 0:  # The operation failed
    return 3

  # Print the map content
  print(f"Map: {resp.map}")
  return 0


def start_recording_path_demo(client, args):
  resp = client.wait_for_response(client.req_start_recording_path())
  if resp is None:
    return 2
  if resp.code < 0:  # The operation failed
    return 3
  
  return 0


def mark_user_node_on_new_path_demo(client, args):
  if len(args) < 1:
    print_usage()
    return 1
  node_name = args[0]
  record_orientation = True
  if len(args) >= 2:
    record_orientation = args[1].lower() == 'true'
  resp = client.wait_for_response(
      client.req_mark_user_node_on_new_path(
          record_orientation=record_orientation,
          business_attr_dict={"name": node_name}))
  if resp is None:
    return 2
  if resp.code < 0:  # The operation failed
    return 3
  
  return 0

def finish_recording_path_demo(client, args):
  if len(args) < 1:
    print_usage()
    return 1
  auto_connection_radius = float(args[0])
  resp = client.wait_for_response(
      client.req_finish_recording_path(auto_connection_radius))
  if resp is None:
    return 2
  if resp.code < 0:  # The operation failed
    return 3
  
  return 0


def cancel_recording_path_demo(client, args):
  resp = client.wait_for_response(client.req_cancel_recording_path())
  if resp is None:
    return 2
  if resp.code < 0:  # The operation failed
    return 3
  
  return 0


def record_user_node_demo(client, args):
  if len(args) < 2:
    print_usage()
    return 1
  node_name = args[0]
  auto_connection_radius = float(args[1])
  record_orientation = True
  if len(args) >= 3:
    record_orientation = args[2].lower() == 'true'
  resp = client.wait_for_response(
      client.req_record_user_node(
          record_orientation=record_orientation,
          business_attr_dict={"name": node_name},
          auto_connection_radius=auto_connection_radius))
  if resp is None:
    return 2
  if resp.code < 0:  # The operation failed
    return 3
  
  return 0

def record_forbidden_area_demo(client, args):
  if len(args) < 3:
    print_usage()
    return 1
  area_front_distance = float(args[0])
  area_length = float(args[1])
  area_width = float(args[2])
  area_min_h = -2.0
  area_max_h = 2.0
  if len(args) >= 4:
    area_min_h = float(args[3])
  if len(args) >= 5:
    area_max_h = float(args[4])
  resp = client.wait_for_response(
      client.req_record_forbidden_area(area_front_distance, area_length,
                                       area_width, area_min_h, area_max_h))
  if resp is None:
    return 2
  if resp.code < 0:  # The operation failed
    return 3
  
  return 0

def gen_map_editing_test_data():
  # Generate testing nodes
  node1 = TopoNode()
  node1.id = 1
  node1.pos = Point(x=0.0, y=0.0, z=0.0)
  node1.has_orientation = True
  node1.orientation = 0.0

  # Set the business attributes of the node if needed.
  # Example:
  #    name: office_1
  #    indoor: yes
  node1.business_attr_keys = ["name", "indoor"]
  node1.business_attr_values = ["office_1", "yes"]

  node2 = TopoNode()
  node2.id = 2
  node2.pos = Point(x=10.0, y=0.0, z=0.0)
  node2.has_orientation = False
  node2.orientation = 0.0

  node3 = TopoNode()
  node3.id = 3
  node3.pos = Point(x=0.0, y=10.0, z=0.0)
  node3.has_orientation = True
  node3.orientation = math.pi / 2.0  
  node3.business_attr_keys = ["name"]
  node3.business_attr_values = ["hall"]

  node4 = TopoNode()
  node4.id = 4
  node4.pos = Point(x=10.0, y=10.0, z=0.0)
  node4.has_orientation = False
  node4.orientation = 0.0

  nodes = [node1, node2, node3, node4]

  # Generate testing edges

  edge12 = TopoEdge()  # Edge 1 -> 2:  Path from node 1 to node 2 is available.
  edge12.from_node = 1
  edge12.to_node = 2
  edge12.cost = -1.0   # Always set to -1.0

  edge21 = TopoEdge()  # Edge 2 -> 1:  Path from node 2 to node 1 is available.
  edge21.from_node = 2
  edge21.to_node = 1
  edge21.cost = -1.0   # Always set to -1.0

  edge23 = TopoEdge()  # Edge 2 -> 3:  Path from node 2 to node 3 is available.
  edge23.from_node = 2
  edge23.to_node = 3
  edge23.cost = -1.0   # Always set to -1.0

  edge32 = TopoEdge()  # Edge 3 -> 2:  Path from node 3 to node 2 is available.
  edge32.from_node = 3
  edge32.to_node = 2
  edge32.cost = -1.0   # Always set to -1.0

  edges = [edge12, edge21, edge23, edge32]

  # Generate testing forbidden areas
  shape = Shape2d()
  shape.shape_type = "Polygon"
  shape.vertices = [
      Point(x=0.0, y=1.0, z=0.0),
      Point(x=1.0, y=1.0, z=0.0),
      Point(x=1.0, y=2.0, z=0.0),
      Point(x=0.0, y=2.0, z=0.0)
  ]
  forbidden_area1 = ForbiddenArea()
  forbidden_area1.id = 1
  forbidden_area1.shape = shape
  forbidden_area1.z_min = -2.0  # [z_min, z_max] are not used in 2D navigation.
  forbidden_area1.z_max = 2.0

  forbidden_areas = [forbidden_area1]

  # Return the generated map elements
  return nodes, edges, forbidden_areas

def override_nav_map_demo(client, args):
  nodes, edges, forbidden_areas = gen_map_editing_test_data()

  resp = client.wait_for_response(client.req_override_map(nodes=nodes,
                       edges=edges,
                       forbidden_areas=forbidden_areas))
  if resp is None:
    return 2
  if resp.code < 0:  # The operation failed
    return 3
  
  return 0

def update_nav_map_elements_demo(client, args):
  nodes, edges, forbidden_areas = gen_map_editing_test_data()

  resp = client.wait_for_response(client.req_update_map_elements(nodes=nodes,
                       edges=edges,
                       forbidden_areas=forbidden_areas))
  if resp is None:
    return 2
  if resp.code < 0:  # The operation failed
    return 3

  return 0

def remove_nav_map_elements_demo(client, args):
  nodes_to_remove = [
      TopoNode(id=1),
      TopoNode(id=4)
  ]
  edges_to_remove = [
      TopoEdge(from_node=2, to_node=3),
  ]
  forbidden_areas_to_remove = [
      ForbiddenArea(id=1)
  ]

  resp = client.wait_for_response(client.req_remove_map_elements(nodes=nodes_to_remove,
                       edges=edges_to_remove,
                       forbidden_areas=forbidden_areas_to_remove))
  if resp is None:
    return 2
  if resp.code < 0:  # The operation failed
    return 3

  return 0

def main():
  rclpy.init()
  if len(sys.argv) < 2:
    print_usage()
    sys.exit(1)

  op_type = sys.argv[1]
  client = NavMapOpClient("nav_map_op_pydemo")

  # Create the executor and the spinning thread
  executor = SingleThreadedExecutor()
  executor.add_node(client)
  executor_thread = threading.Thread(target=executor.spin)
  executor_thread.start()

  if op_type == "LoadMap":
    exit_code = load_map_demo(client, sys.argv[2:])
  elif op_type == "SaveMap":
    exit_code = save_map_demo(client, sys.argv[2:])
  elif op_type == "ClearMap":
    exit_code = clear_map_demo(client, sys.argv[2:])
  elif op_type == "ReadMap":
    exit_code = read_map_demo(client, sys.argv[2:])
  elif op_type == "StartRecordingPath":
    exit_code = start_recording_path_demo(client, sys.argv[2:])
  elif op_type == "MarkUserNodeOnNewPath":
    exit_code = mark_user_node_on_new_path_demo(client, sys.argv[2:])
  elif op_type == "FinishRecordingPath":
    exit_code = finish_recording_path_demo(client, sys.argv[2:])
  elif op_type == "CancelRecordingPath":
    exit_code = cancel_recording_path_demo(client, sys.argv[2:])
  elif op_type == "RecordUserNode":
    exit_code = record_user_node_demo(client, sys.argv[2:])
  elif op_type == "RecordForbiddenArea":
    exit_code = record_forbidden_area_demo(client, sys.argv[2:])
  elif op_type == "OverrideMap":
    exit_code = override_nav_map_demo(client, sys.argv[2:])
  elif op_type == "UpdateMapElements":
    exit_code = update_nav_map_elements_demo(client, sys.argv[2:])
  elif op_type == "RemoveMapElements":
    exit_code = remove_nav_map_elements_demo(client, sys.argv[2:])
  else:
    print_usage()
    exit_code = 1

  client.destroy_node()
  executor.shutdown()
  executor_thread.join()
  rclpy.shutdown()

  sys.exit(exit_code)

if __name__ == "__main__":
  main()
