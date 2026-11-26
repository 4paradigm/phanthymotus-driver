"""Read the chassis map in the vendor SDK's CPython 3.12 process."""

import json
import math
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


def run(host, port):
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
        subscribed = False
        while True:
            if not subscribed and localization.has_location_dev():
                localization.loc_state(location.receive)
                subscribed = True
            map_name, stations = read_stations(map_manager, map_reader)
            feedback = location.fresh()
            emit({"state": "online" if feedback is not None else "partial",
                  "map_name": map_name, "stations": stations, "localization": feedback})
            time.sleep(2)
    finally:
        rpc.close_client()


def emit(data):
    print("SPATIAL_MAP " + json.dumps(data, ensure_ascii=False, allow_nan=False), flush=True)


if __name__ == "__main__":
    try:
        run(sys.argv[1], int(sys.argv[2]))
    except Exception as exc:
        emit({"state": "unavailable", "message": str(exc)})
        sys.exit(1)
