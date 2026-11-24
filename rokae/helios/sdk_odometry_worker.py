"""Read vendor odometry using the SDK's separate Python runtime."""

import json
import math
import sys
import threading
import time


def main(host, port):
    import odometry
    import py_client_common as rpc

    config = rpc.ClientConfig()
    config.ip = host
    config.port = int(port)
    config.connect_timeout_ms = config.call_timeout_ms = 3000
    if not rpc.init_client(config) or not odometry.has_odom_dev():
        return 1

    lock = threading.Lock()
    last_emit = -math.inf

    def receive(sample):
        nonlocal last_emit
        now = time.monotonic()
        with lock:
            if now - last_emit < 0.04:
                return
            x, y, yaw = sample.x, sample.y, sample.theta
            if not all(math.isfinite(v) for v in (x, y, yaw)):
                return
            reading = {"x": x, "y": y, "yaw": yaw, "source_stamp_ns": sample.timestamp_ns}
            print("MOTUS_ODOM " + json.dumps(reading, allow_nan=False), flush=True)
            last_emit = now

    odometry.odom_state(receive)
    threading.Event().wait()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(*sys.argv[1:]))
