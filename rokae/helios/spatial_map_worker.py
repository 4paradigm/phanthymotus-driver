"""Read the chassis map in the vendor SDK's CPython 3.12 process."""

import asyncio
import base64
import json
import math
import os
import struct
import sys
import time


def read_stations(manager, reader):
    name = manager.get_current_map_name()
    if not name:
        raise ValueError("VMR did not report a current map")
    stations = [
        {"id": goal.id, "name": goal.name, "type": kind}
        for kind, goals in (("work", reader.get_work_points()), ("charge", reader.get_charge_points()))
        for goal in goals
    ]
    if manager.get_current_map_name() != name:
        raise ValueError("VMR changed maps while reading stations")
    return name, stations


class LocationFeedback:
    def __init__(self):
        self.latest = None

    def receive(self, info):
        stamp = info.timestamp_ns
        if stamp <= 0 or not all(math.isfinite(v) for v in (info.confidence, info.x, info.y, info.theta)):
            self.latest = None
            return
        if self.latest is None or self.latest[0] != stamp:
            # Copy the native values before the SDK releases the callback object.
            self.latest = (stamp, time.monotonic(), {
                "status_code": info.status, "confidence": info.confidence,
                "x": info.x, "y": info.y, "theta": info.theta,
                "timestamp_ns": str(stamp),
            })

    def fresh(self):
        sample = self.latest
        if sample and time.monotonic() - sample[1] < 3:
            return sample[2]
        return None


class MappingScan:
    """Sample both native lidar streams at a pose close to the scan timestamp."""
    def __init__(self, location):
        self.location = location
        self.frames = {}
        self.enabled = False

    def receive(self, cloud):
        pose = self.location.fresh()
        if not self.enabled or not pose or abs(cloud.timestamp_ns - int(pose["timestamp_ns"])) > 350_000_000:
            return
        old = self.frames.get(cloud.location)
        if old and cloud.timestamp_ns - old[0] < 250_000_000:
            return
        points = cloud.points
        c, s = math.cos(pose["theta"]), math.sin(pose["theta"])
        packed = bytearray()
        for p in points[::max(1, math.ceil(len(points)/1600))]:
            if all(map(math.isfinite, (p.x, p.y, p.z))):
                packed.extend(struct.pack("<fff", pose["x"] + c*p.x - s*p.y,
                                          pose["y"] + s*p.x + c*p.y, p.z))
        self.frames[cloud.location] = (cloud.timestamp_ns, packed)


async def serve_commands(manager, location, scans, snapshots, resume_mapping):
    import nav
    import point_cloud

    loop = asyncio.get_running_loop()
    reader = asyncio.StreamReader()
    transport, _ = await loop.connect_read_pipe(lambda: asyncio.StreamReaderProtocol(reader), sys.stdin)
    handle = None
    navigation = {"state": "idle"}
    subscribed = False
    if resume_mapping and point_cloud.has_point_cloud_dev():
        point_cloud.point_cloud_pub(scans.receive)
        subscribed = scans.enabled = True

    def finished(command):
        nonlocal handle
        try:
            result = command.future.result()
            state = "completed" if result.err_code == 0 else "error"
            navigation.update(state=state, code=result.err_code, message=result.err_message)
        except Exception as exc:
            navigation.update(state="error", message=str(exc))
        finally:
            handle = None

    try:
        while raw := await reader.readline():
            req = json.loads(raw)
            action = req["action"]
            try:
                name = req.get("map_name")
                if action == "list_maps":
                    data = {"maps": manager.get_map_list()}
                elif action == "start_mapping":
                    if handle or name in manager.get_map_list():
                        raise ValueError("Navigation active or map name already in use")
                    if not subscribed and point_cloud.has_point_cloud_dev():
                        point_cloud.point_cloud_pub(scans.receive)
                        subscribed = True
                    info = manager.MapInfo()
                    info.name = name
                    error = manager.start_mapping(info)
                    if error:
                        raise ValueError(error)
                    scans.frames.clear()
                    scans.enabled = True
                    data = {"state": "mapping", "map_name": name}
                elif action == "stop_mapping":
                    error = manager.stop_mapping()
                    if error:
                        raise ValueError(error)
                    scans.enabled = False
                    scans.frames.clear()
                    data = {"state": "mapping_stopped", "map_name": name}
                elif action == "load_map":
                    if handle or name not in manager.get_map_list():
                        raise ValueError("Navigation active or unknown map")
                    cmd = manager.async_change_map(name)
                    result = await cmd.future
                    if result.err_code or result.data:
                        raise ValueError(result.err_message or str(result.data))
                    location.latest = None
                    data = {"state": "map_loaded", "map_name": name}
                elif action == "navigate":
                    if handle or not location.fresh() or manager.get_current_map_name() != name:
                        raise ValueError("Navigation active, missing localization or active map changed")
                    params = nav.NavigateParams()
                    target = nav.Pose()
                    target.x, target.y, target.theta = req["x"], req["y"], req["yaw"]
                    params.target_position = target
                    handle = nav.async_NavigateTo(params)
                    command = handle
                    navigation = {"state": "navigating", "id": str(command.id), "map_name": name}
                    command.future.add_done_callback(lambda _, cmd=command: finished(cmd))
                    data = dict(navigation)
                elif action == "stop_nav":
                    if handle:
                        handle.cancel()
                        navigation["state"] = "cancel_requested"
                    data = dict(navigation)
                elif action == "nav_status":
                    data = dict(navigation)
                else:
                    raise ValueError("Unsupported chassis command")
                emit(snapshots, {"reply": req["id"], "result": data})
            except Exception as exc:
                emit(snapshots, {"reply": req["id"], "error": str(exc)})
    finally:
        if handle:
            handle.cancel()
        transport.close()


async def poll(manager, reader, localization, location, scans, snapshots):
    subscribed = False
    while True:
        if not subscribed and localization.has_location_dev():
            localization.loc_state(location.receive)
            subscribed = True
        try:
            name, stations = read_stations(manager, reader)
            feedback = location.fresh()
            data = {"state": "online" if feedback else "partial", "map_name": name,
                    "stations": stations, "localization": feedback}
        except Exception as exc:
            data = {"state": "unavailable", "message": str(exc)}
        if scans.enabled:
            batch = list(scans.frames.values())
            data["scan"] = base64.b64encode(b"".join(points for _, points in batch)).decode()
        emit(snapshots, data)
        await asyncio.sleep(1)


async def run(host, port, snapshots, resume_mapping):
    import localization
    import map_manager
    import map_reader
    import py_client_common as rpc

    config = rpc.ClientConfig()
    config.ip, config.port = host, port
    config.connect_timeout_ms = config.call_timeout_ms = 2000
    if not rpc.init_client(config):
        raise ConnectionError("VMR connection failed")
    try:
        if not (map_manager.has_map_manager() and map_reader.has_map_reader()):
            raise ConnectionError("VMR map service unavailable")
        location = LocationFeedback()
        scans = MappingScan(location)
        capture = asyncio.create_task(poll(map_manager, map_reader, localization, location, scans, snapshots))
        try:
            await serve_commands(map_manager, location, scans, snapshots, resume_mapping)
        finally:
            capture.cancel()
            await asyncio.gather(capture, return_exceptions=True)
    finally:
        rpc.close_client()


def emit(snapshots, data):
    print(json.dumps(data, ensure_ascii=False, allow_nan=False), file=snapshots, flush=True)


if __name__ == "__main__":
    from common import logsafe

    logsafe.install(check_fd=False)
    with os.fdopen(int(sys.argv[3]), "w", encoding="utf-8") as snapshots:
        try:
            asyncio.run(run(sys.argv[1], int(sys.argv[2]), snapshots, bool(sys.argv[4])))
        except Exception as exc:
            emit(snapshots, {"state": "unavailable", "message": str(exc)})
            sys.exit(1)
