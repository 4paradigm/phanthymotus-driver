"""Optional RGB-D acquisition metadata, independent of image consumers."""

import json
import math
import time
from uuid import uuid4


def frame_time_ns(frame, rs):
    if frame.get_frame_timestamp_domain() not in (
            rs.timestamp_domain.system_time, rs.timestamp_domain.global_time):
        raise ValueError("RGB-D requires host-synchronized camera timestamps")
    seconds = frame.get_timestamp() / 1000.0
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError("Invalid RGB-D acquisition timestamp")
    # Freshness requirements belong to consumers, not the shared camera.
    return round(seconds * 1_000_000_000)


def calibration(profiles):
    def intrinsics(profile):
        intr = profile.get_intrinsics()
        return {key: getattr(intr, key) for key in ("width", "height", "fx", "fy", "ppx", "ppy")} | {
            "model": str(intr.model), "coeffs": list(intr.coeffs)}
    extr = profiles["depth"].get_extrinsics_to(profiles["rgb"])
    return {"rgb_intrinsics": intrinsics(profiles["rgb"]),
            "depth_intrinsics": intrinsics(profiles["depth"]),
            "depth_to_color": {"rotation": list(extr.rotation), "translation": list(extr.translation)},
            "depth_scale_m": 0.001, "depth_aligned_to": "depth"}


class RGBDMetadata:
    """Best-effort sideband for one pipeline session; never owns the pipeline."""

    def __init__(self, namespace, serial_number, rs):
        self.namespace, self.serial_number, self.rs = namespace, serial_number, rs
        self.session_id = uuid4().hex
        self.publishers = {}
        self._retry_at = 0.0
        self._route_error = None
        self._profile_key = self._calibration = None
        self.error = None

    def configure_clock(self, device):
        try:
            sensors = device.query_sensors()
        except Exception as exc:
            self.error = str(exc)
            return
        for sensor in sensors:
            try:
                if sensor.supports(self.rs.option.global_time_enabled):
                    sensor.set_option(self.rs.option.global_time_enabled, 1)
            except Exception as exc:
                self.error = str(exc)

    def _routes(self, node, topics, message_type, qos):
        if time.monotonic() < self._retry_at:
            return self._route_error
        self._route_error = None
        for topic in self.publishers.keys() - topics:
            try:
                node.destroy_publisher(self.publishers[topic])
                del self.publishers[topic]
            except Exception as exc:
                self._route_error = str(exc)
                self._retry_at = time.monotonic() + 1.0
        for topic in topics - self.publishers.keys():
            try:
                self.publishers[topic] = node.create_publisher(message_type, topic, qos)
            except Exception as exc:
                self._route_error = str(exc)
                self._retry_at = time.monotonic() + 1.0
        return self._route_error

    def publish(self, node, routes, frames, headers, status):
        # No sideband failure may escape into the capture/reconnect lifecycle.
        self.error = None
        try:
            topics = {f"/{self.namespace}/ext_camera/{key.replace('-', '_')}/depth/metadata"
                      for key, channel in routes.items() if channel == "depth"}
            if not topics and not self.publishers:
                return
            from rclpy.qos import qos_profile_sensor_data
            from std_msgs.msg import String

            self.error = self._routes(node, topics, String, qos_profile_sensor_data)
            listeners = []
            for topic in topics & self.publishers.keys():
                try:
                    publisher = self.publishers[topic]
                    if publisher.get_subscription_count():
                        listeners.append(publisher)
                except Exception as exc:
                    self.error = str(exc)
            if not listeners or not all(frames.get(name) and name in headers for name in ("rgb", "depth")):
                return
            profiles = {name: frames[name].profile.as_video_stream_profile() for name in ("rgb", "depth")}
            key = tuple((p.unique_id(), p.width(), p.height(), p.fps(), p.format()) for p in profiles.values())
            if key != self._profile_key:
                self._calibration = calibration(profiles)
                self._profile_key = key
            metadata = {"version": 2, "serial_number": self.serial_number, "session_id": self.session_id,
                        "rgb_topics": [f"/{self.namespace}/ext_camera/{key.replace('-', '_')}/rgb"
                                       for key, channel in routes.items() if channel == "rgb"],
                        **self._calibration}
            for name in ("rgb", "depth"):
                metadata[name + "_stamp_ns"] = frame_time_ns(frames[name], self.rs)
                header = headers[name]
                metadata[name + "_header_stamp_ns"] = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
                metadata[name + "_frame_id"] = header.frame_id
            msg = String()
            msg.data = json.dumps(metadata)
            for publisher in listeners:
                try:
                    publisher.publish(msg)
                except Exception as exc:
                    self.error = str(exc)
        except Exception as exc:
            self.error = str(exc)
        finally:
            if self.error:
                status["rgbd_error"] = self.error
            else:
                status.pop("rgbd_error", None)
