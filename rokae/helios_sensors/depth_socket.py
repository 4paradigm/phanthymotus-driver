"""Local-only depth transport for Jetson GMSL setups that capture on the host."""

import argparse
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import queue
import select
import socket
from socketserver import BaseRequestHandler
import struct
import time

from common import logsafe

from .orbbec import HEIGHT, WIDTH, capture


HEADER = struct.Struct("!QII")
MAX_FRAME = WIDTH * HEIGHT * 2 + 1024
MAX_METADATA = 4096
SOCKET_TIMEOUT = 9


def read_exact(link, size, deadline):
    data = bytearray(size)
    remaining = memoryview(data)
    while remaining:
        timeout = deadline - time.monotonic()
        if timeout <= 0:
            raise TimeoutError("timed out receiving a depth message")
        link.settimeout(timeout)
        count = link.recv_into(remaining)
        if count == 0:
            raise OSError("depth source disconnected during a message")
        remaining = remaining[count:]
    return data


def read_depth_frame(link):
    deadline = time.monotonic() + SOCKET_TIMEOUT
    stamp, meta_size, frame_size = HEADER.unpack(read_exact(link, HEADER.size, deadline))
    if not 0 < meta_size <= MAX_METADATA or frame_size > MAX_FRAME:
        raise ValueError("invalid depth socket message length")
    metadata = json.loads(read_exact(link, meta_size, deadline))
    if not isinstance(metadata, dict):
        raise ValueError("invalid depth socket metadata")
    if not stamp:
        raise RuntimeError(str(metadata.get("error", "host depth capture failed"))[:300])
    if not frame_size:
        raise ValueError("empty depth frame")
    payload = read_exact(link, frame_size, deadline)
    now = time.monotonic()
    received = metadata.get("captured_monotonic", now)
    if not isinstance(received, (int, float)) or not math.isfinite(received) or not 0 <= received <= now:
        raise ValueError("invalid depth capture timestamp")
    return "frame", received, stamp, payload, metadata.get("K")


def from_depth_socket(path, stop, output):
    logsafe.install(check_fd=False)
    output.cancel_join_thread()
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as link:
            link.settimeout(SOCKET_TIMEOUT)
            link.connect(path)
            while not stop.is_set():
                frame = read_depth_frame(link)
                try:
                    output.put_nowait(frame)
                except queue.Full:
                    pass
    except (OSError, RuntimeError, ValueError) as exc:
        try:
            output.put(("error", str(exc)[:300]), timeout=.4)
            stop.wait(.1)
        except queue.Full:
            pass


class DepthConnection(BaseRequestHandler):
    def handle(self):
        context = mp.get_context("spawn")
        stop = context.Event()
        output = context.Queue(maxsize=1)
        child = context.Process(target=capture, args=(
            "depth", self.server.sdk_dir, self.server.serial, stop, output), daemon=True)
        try:
            child.start()
            self.request.settimeout(2)
            while not select.select([self.request], [], [], 0)[0]:
                try:
                    item = output.get(timeout=.2)
                except queue.Empty:
                    if child.is_alive():
                        continue
                    item = ("error", f"depth source exited ({child.exitcode})")
                if item[0] == "error":
                    stamp, meta, payload = 0, {"error": item[1]}, b""
                else:
                    _, captured, stamp, payload, intrinsic = item
                    meta = {"K": intrinsic, "captured_monotonic": captured}
                metadata = json.dumps(meta, allow_nan=False).encode("utf-8")
                self.request.sendall(HEADER.pack(stamp, len(metadata), len(payload)) + metadata + payload)
                if not stamp:
                    break
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass
        finally:
            stop.set()
            if child.pid is not None:
                child.join(timeout=1.5)
                if child.is_alive():
                    child.kill()
                    child.join(timeout=1)
            output.close()


def main():
    import grp
    from socketserver import UnixStreamServer

    parser = argparse.ArgumentParser()
    parser.add_argument("--sdk-dir", required=True)
    parser.add_argument("--serial", required=True)
    parser.add_argument("--socket", required=True, type=Path)
    parser.add_argument("--group", default="video")
    args = parser.parse_args()
    gid = grp.getgrnam(args.group).gr_gid
    os.umask(0o117)
    server = UnixStreamServer(str(args.socket), DepthConnection)
    try:
        os.chown(args.socket, -1, gid)
        os.chmod(args.socket, 0o660)
        server.sdk_dir, server.serial = args.sdk_dir, args.serial
        server.serve_forever()
    finally:
        server.server_close()
        args.socket.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
