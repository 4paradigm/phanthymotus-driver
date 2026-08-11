#!/usr/bin/env python3

import argparse

import rclpy
from era_nav_msgs.msg._glio_state import GlioState
from era_nav_msgs.srv._create_map import CreateMap
from era_nav_msgs.srv._init_pose import InitPose
from era_nav_msgs.srv._load_map import LoadMap
from era_nav_msgs.srv._query_map import QueryMap
from geometry_msgs.msg import Point, Pose, PoseStamped
from rclpy.node import Node
from std_msgs.msg import Int32, UInt8
from std_srvs.srv import Trigger
from enum import Enum

map_state = {
    0: "Idel",
    1: "Initializing",
    2: "Mapping",
    3: "Looped",
    4: "Optimizing",
    5: "Success",
    6: "Failed",
}
loc_state = {0: "Idel", 1: "Initializing", 2: "Run", 3: "Error"}


class GlioErrorIndex(Enum):
    kConditionNumberSick = "激光雷达观测退化1"
    kBaLarge = ("加速度计估计异常",)
    kBgLarge = ("陀螺仪估计异常",)
    kVelLarge = ("速度估计异常",)
    kPosJump = ("位置估计发生跳变",)
    kRotJump = ("姿态估计发生跳变",)
    kGravJump = ("重力加速度计估计异常",)
    kFeatNumLimited = ("特征点数量不足",)
    kFeatRatioLimited = ("特征点匹配率不足",)
    kResLarge = ("残差异常",)
    kLidarEmpty = ("激光雷达数据为空",)
    kLidarDisonncect = ("激光雷达连接异常",)
    kLidarImuDisconnect = ("激光内置IMU连接异常",)
    kWheelDisconnect = ("轮速计连接异常",)
    kWheelImuDisconnect = ("机身IMU连接异常",)
    kMovingInStatic = ("机器人静止漂移",)
    kLidarObsDegraded = ("激光雷达观测退化2",)
    kGlioWioVelMismatch = ("GLIO与WIO速度不一致",)


class NavStatus:
    def __init__(self):
        self.pre_map_state = 0
        self.map_state = 0
        self.pre_loc_state = 0
        self.loc_state = 0
        self.pre_error_code = 0
        self.error_code = 0
        self.pre_pose_ts = 0.0

    def map_status_callback(self, msg: UInt8):
        self.map_state = msg.data
        if self.pre_map_state != self.map_state:
            print(
                f"Mapping status changed: {map_state[self.pre_map_state]} -> {map_state[self.map_state]}"
            )
            self.pre_map_state = self.map_state

    def localization_status_callback(self, msg: GlioState):
        self.loc_state = msg.glio_state
        self.error_code = msg.glio_error_code
        if self.pre_loc_state != self.loc_state:
            print(
                f"Localization status changed: {loc_state[self.pre_loc_state]} -> {loc_state[self.loc_state]}"
            )
            self.pre_loc_state = self.loc_state
        if self.pre_error_code != self.error_code:
            changed = self.pre_error_code ^ self.error_code
            error_list = list(GlioErrorIndex)
            for i in range(len(error_list)):
                if (changed & (1 << i)) == 0:
                    continue
                if self.error_code & (1 << i):
                    print(f"Error happend {i}: {error_list[i].value[0]}")
                if self.pre_error_code & (1 << i):
                    print(f"Error recover {i}: {error_list[i].value[0]}")
            self.pre_error_code = self.error_code

    def map2lidar_callback(self, msg: PoseStamped):
        pose_ts = msg.header.stamp.sec
        if pose_ts != self.pre_pose_ts:
            print(
                f"Localization position: {msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}"
            )
        self.pre_pose_ts = msg.header.stamp.sec


class SlamClient(Node):
    def __init__(self):
        super().__init__("slam_client")

        # Initialize ROS2 clients and subscribers
        self.start_map_client = self.create_client(Trigger, "/slam/start_map")
        self.cancel_map_client = self.create_client(Trigger, "/slam/cancel_map")
        self.create_map_client = self.create_client(CreateMap, "/slam/create_map")
        self.load_map_client = self.create_client(LoadMap, "/slam/load_map")
        self.init_pos_client = self.create_client(InitPose, "/slam/init_pose")
        self.query_map_client = self.create_client(QueryMap, "/slam/query_map")

        # Status monitor.
        self.nav_status: NavStatus = NavStatus()
        self.map_status_sub = self.create_subscription(
            UInt8, "/slam/mapping_state", self.nav_status.map_status_callback, 10
        )
        self.localization_status_sub = self.create_subscription(
            GlioState,
            "/localization_state",
            self.nav_status.localization_status_callback,
            10,
        )
        self.loc_pose_sub = self.create_subscription(
            PoseStamped, "/map_to_lidar", self.nav_status.map2lidar_callback, 10
        )

    def start_map(self):
        if not self.start_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Start service not available")
            return False

        req = Trigger.Request()
        future = self.start_map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        response: Trigger.Response = future.result()

        if response.success:
            print("Map started successfully")
        else:
            self.get_logger().error(f"Map start failed: {response.message}")
        return response.success

    def cancel_map(self):
        if not self.cancel_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Cancel service not available")
            return False

        req = Trigger.Request()
        future = self.cancel_map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        response: Trigger.Response = future.result()

        if response.success:
            print("Map cancelled successfully")
        else:
            self.get_logger().error(f"Map cancel failed: {response.message}")
        return response.success

    def create_map(self, _resolution, _map_name, _data_abs_path):
        if not self.create_map_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Create service not available")
            return False

        req = CreateMap.Request()
        req.resolution = _resolution
        # TODO: change this msg define.
        req.map_name = _map_name
        req.data_abs_path = _data_abs_path

        future = self.create_map_client.call_async(req)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=100.0 * 60.0
        )  # 100 min.

        response: CreateMap.Response = future.result()
        if response.success:
            print(
                f"Map created success, map_abs_path: {response.map_abs_path}, data_abs_path: {response.data_abs_path}"
            )
        else:
            self.get_logger().error(
                f"Map creation failed, map_abs_path: {response.map_abs_path}, data_abs_path: {response.data_abs_path}"
            )
        return response.success

    def load_map(self, _map_name):
        if not self.load_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Load map service not available")
            return False

        req = LoadMap.Request()
        req.map_name = _map_name

        future = self.load_map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        response: LoadMap.Response = future.result()
        if response.success:
            print("Map loaded successfully")
            return True
        else:
            self.get_logger().error("Map load failed")
            return False

    def query_map(self):
        if not self.query_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Query map service not available")
            return False

        req = QueryMap.Request()

        future = self.query_map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        response: QueryMap.Response = future.result()
        print(f"Query map response: {response}")

    def init_pos(self, _pos_array):
        if not self.init_pos_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Init pose service not available")
            return False

        req = InitPose.Request()
        req.poses = [
            Pose(position=Point(x=_pos[0], y=_pos[1], z=_pos[2])) for _pos in _pos_array
        ]
        future = self.init_pos_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        response: InitPose.Response = future.result()
        if response.success:
            print("Localization initialized successfully")
            return True
        else:
            self.get_logger().error("Localization initialization failed")
            return False


def main(args=None):
    rclpy.init(args=args)

    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Navigation API Test")
    subparsers = parser.add_subparsers(dest="command", help="Available commands")

    start_map_parser = subparsers.add_parser("start_map", help="Start map")
    cancel_map_parser = subparsers.add_parser("cancel_map", help="Cancel map")

    create_map_parser = subparsers.add_parser("create_map", help="Create map")
    create_map_parser.add_argument(
        "--resolution", type=float, help="Resolution of the map", default=0.2
    )
    create_map_parser.add_argument(
        "--map_name",
        type=str,
        help="Directory to store the map file",
        default="default",
    )
    create_map_parser.add_argument(
        "--data_abs_path", type=str, help="Data path to store the map", default=""
    )

    load_map_parser = subparsers.add_parser("load_map", help="Load map")
    load_map_parser.add_argument(
        "--map_name", type=str, help="Absolute path to the map file", default="default"
    )

    query_map_parser = subparsers.add_parser("query_map", help="Query map")

    init_pos_parser = subparsers.add_parser(
        "init_pos", help="Initialize localization by position"
    )
    init_pos_parser.add_argument(
        "--position",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        help="Position as three floats: x y z",
        default=[0.0, 0.0, 0.0],
    )

    args = parser.parse_args()

    nav_api_test = SlamClient()

    while rclpy.ok():
        if args.command == "create_map":
            nav_api_test.create_map(args.resolution, args.map_name, args.data_abs_path)
            break
        elif args.command == "start_map":
            nav_api_test.start_map()
            break
        elif args.command == "cancel_map":
            nav_api_test.cancel_map()
            break
        elif args.command == "load_map":
            nav_api_test.load_map(args.map_name)
            break
        elif args.command == "query_map":
            nav_api_test.query_map()
            break
        elif args.command == "init_pos":
            init_pos = []
            init_pos.append([args.position[0], args.position[1], args.position[2]])
            print(f"Init pose: {init_pos}")
            nav_api_test.init_pos(init_pos)
            break

        rclpy.spin_once(nav_api_test, timeout_sec=0.1)

    nav_api_test.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
