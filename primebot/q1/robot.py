"""Read Q1 telemetry on the vendor Fast DDS network."""
import os
import time
from pathlib import Path


def battery_state(msg):
    return {
        "percentage": msg.remaining_capacity_percentage,
        "voltage_v": msg.voltage / 1000,
        "current_a": msg.current / 1000,
        "power_w": msg.power / 1000,
        "temperature_c": round(msg.temperature, 2),
        "remaining_capacity_mah": msg.remaining_capacity,
        "cycle_count": msg.cycle_count,
        "cycle_total_capacity_mah": msg.cycle_total_capacity,
        "status_bits": msg.status,
    }


def configure_robot_dds():
    from ament_index_python.packages import get_package_share_directory

    profile = Path(get_package_share_directory("aimdk_msgs")) / "config/fastdds_profiles.xml"
    for key in ("FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE"):
        os.environ[key] = os.environ.get("Q1_FASTDDS_PROFILE", str(profile))


def read_robot(domain, output, stopping):
    from common import logsafe
    logsafe.install(check_fd=False)

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from aimdk_msgs.msg import Bms

    configure_robot_dds()
    rclpy.init(domain_id=domain)
    node = rclpy.create_node("primebot_q1_battery_reader", start_parameter_services=False)
    node.create_subscription(
        Bms, "/aima/hal/bms/state",
        lambda msg: output.send((time.monotonic(), battery_state(msg))),
        qos_profile_sensor_data,
    )
    try:
        while not stopping.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        output.close()
        node.destroy_node()
        rclpy.try_shutdown()
