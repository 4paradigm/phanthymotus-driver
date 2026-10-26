"""ROS/MCP facade for an isolated instance of the existing velocity controller.

Only reception and lifecycle RPC remain in the camera bundle. The child owns
ControlSink, the watchdog, acceleration limiting and the SDK connection. IPC
never renews a producer timestamp or acknowledges an SDK write on its behalf.
"""
from __future__ import annotations

import sys
from pathlib import Path


def _install_child_logging():
    # A fresh interpreter inherits file descriptors, not the parent's Python
    # stream wrappers. Resolve common/ before importing any controller code.
    source_root = Path(__file__).resolve().parents[2]
    if (source_root / "common").is_dir() and str(source_root) not in sys.path:
        sys.path.insert(0, str(source_root))
    try:
        from common import logsafe
        logsafe.install(check_fd=False)
    except ImportError as exc:
        sys.stderr.write(f"[loco-servo-worker] logsafe unavailable ({exc}); stdout unprotected\n")


if __name__ == "__main__":
    _install_child_logging()

import copy
import json
import math
import os
import signal
import socket
import struct
import subprocess
import threading
import time
from types import SimpleNamespace
import uuid

from loco_servo import LocoServoPlugin as _Core, TOGGLES, _validated_min_vx
from slamtec_velocity import SlamtecVelocityBackend

_MAX_PACKET = 65536
_MAX_RPC = 262144
_UNCERTAIN_STOP_S = 1.5  # Includes the native firmware heartbeat timeout.


def _send_packet(channel, value):
    data = json.dumps(value, allow_nan=False).encode()
    if len(data) > _MAX_RPC:
        raise ValueError("control RPC exceeds size limit")
    channel.sendall(struct.pack("!I", len(data)) + data)


def _recv_exact(channel, count):
    chunks = bytearray()
    while len(chunks) < count:
        part = channel.recv(count - len(chunks))
        if not part:
            raise EOFError("control channel closed")
        chunks.extend(part)
    return bytes(chunks)


def _recv_packet(channel):
    size, = struct.unpack("!I", _recv_exact(channel, 4))
    if size > _MAX_RPC:
        raise ValueError("control RPC exceeds size limit")
    return json.loads(_recv_exact(channel, size))


class _ControlOutbox:
    """Bounded, nonblocking sender: a priority zero plus the latest motion."""
    def __init__(self):
        self._lock = threading.Lock()
        self._active = False
        self._generation = self._epoch = 0
        self._session = ""
        self._motion = self._zero = self._standing = None
        self._received = self._coalesced = 0

    def activate(self, generation, session, epoch, active):
        with self._lock:
            self._generation, self._session, self._epoch = generation, session, epoch
            self._active = active
            self._motion = self._zero = self._standing = None

    def _put(self, kind, data, generation, received_at):
        received_at = received_at or (time.monotonic(), int(time.time() * 1000))
        zero = False
        if kind == "motion":
            try:
                payload = json.loads(data)
                values = payload.get("values") if isinstance(payload, dict) else None
                zero = (isinstance(values, list) and len(values) == 6 and
                        all(type(v) in (int, float) and v == 0 for v in values))
            except (ValueError, TypeError):
                pass  # Leave command validation and counts to ControlSink.
        with self._lock:
            if not self._active or generation != self._generation:
                return False
            packet = json.dumps({"kind": kind, "data": data,
                                 "session": self._session, "epoch": self._epoch,
                                 "received_at": received_at}).encode()
            if len(packet) > _MAX_PACKET:
                raise ValueError("velocity input exceeds IPC size limit")
            self._received += 1
            if kind == "standing":
                self._coalesced += int(self._standing is not None)
                self._standing = packet
            elif zero:
                self._coalesced += int(self._zero is not None) + int(self._motion is not None)
                self._motion, self._zero = None, packet
            else:
                self._coalesced += int(self._motion is not None)
                self._motion = packet
            return True

    def put_motion(self, data, generation, received_at=None):
        return self._put("motion", data, generation, received_at)

    def put_standing(self, value, generation, received_at=None):
        return self._put("standing", value, generation, received_at)

    def flush(self, channel):
        sent = 0
        # send() is nonblocking and must stay under the slot lock: an EAGAIN
        # must not resurrect a motion replaced by a concurrent zero callback.
        with self._lock:
            for slot in ("_zero", "_standing", "_motion"):
                packet = getattr(self, slot)
                if packet is None:
                    continue
                try:
                    count = channel.send(packet)
                except BlockingIOError:
                    break
                if count != len(packet):
                    raise OSError("incomplete control datagram")
                setattr(self, slot, None)
                sent += 1
        return sent

    def info(self):
        with self._lock:
            return {"active": self._active, "received": self._received,
                    "coalesced": self._coalesced,
                    "pending_zero": self._zero is not None,
                    "pending_motion": self._motion is not None,
                    "pending_standing": self._standing is not None,
                    "pending": sum(x is not None for x in
                                   (self._zero, self._motion, self._standing))}


class _HeadlessController(_Core):
    def __init__(self, config, namespace):
        super().__init__(config, namespace, None)
        self._standing_lock = threading.Lock()
        self._pending_standing = None

    def _open(self, topic, generation):
        self._backend = SlamtecVelocityBackend(self._backend_config)
        if not self._settings["dry_run"]:
            self._backend.connect()

    def clear_standing_mailbox(self):
        with self._standing_lock:
            self._pending_standing = None

    def receive_standing(self, value, received_at):
        with self._standing_lock:
            self._pending_standing = (value, received_at[0])
        self._controller_wake.set()

    def _tick(self, generation=None):
        with self._lock:
            with self._standing_lock:
                pending, self._pending_standing = self._pending_standing, None
            if pending is not None:
                self._standing, self._standing_at = pending
            super()._tick(generation)


def _child(control_fd, data_fd):
    _install_child_logging()  # Also protect direct multiprocessing entry use.
    control = socket.socket(fileno=control_fd)
    data = socket.socket(fileno=data_fd)
    core = None
    receiver = None
    done = threading.Event()
    gate_lock = threading.Lock()
    gate = {"active": False, "session": "", "epoch": 0}

    def close_gate():
        with gate_lock:
            gate["active"] = False
            if core is not None:
                core._inbox.activate(core._generation, False)
                core.clear_standing_mailbox()

    def receive():
        try:
            while not done.is_set():
                raw = data.recv(_MAX_PACKET + 1)
                if len(raw) > _MAX_PACKET:
                    raise ValueError("oversized velocity datagram")
                packet = json.loads(raw)
                stamp = packet.get("received_at")
                if not (isinstance(stamp, list) and len(stamp) == 2 and
                        all(type(x) in (int, float) and math.isfinite(x) for x in stamp)):
                    raise ValueError("invalid original receive timestamp")
                with gate_lock:
                    if not gate["active"] or packet.get("session") != gate["session"] or packet.get("epoch") != gate["epoch"]:
                        continue
                    if packet.get("kind") == "motion" and isinstance(packet.get("data"), str):
                        core._on_message(SimpleNamespace(data=packet["data"]), core._generation, stamp)
                    elif packet.get("kind") == "standing" and type(packet.get("data")) is bool:
                        core.receive_standing(packet["data"], stamp)
                    else:
                        raise ValueError("invalid velocity datagram")
        except Exception:
            if not done.is_set():
                close_gate()
                # Wake the lifecycle loop; its finally block confirms STOP.
                control.shutdown(socket.SHUT_RDWR)

    try:
        while True:
            request = _recv_packet(control)
            if core is None:
                if request.get("op") != "init":
                    raise ValueError("controller must be initialized first")
                core = _HeadlessController(request["config"], request["namespace"])
                gate["session"] = request["session"]
                receiver = threading.Thread(target=receive, name="velocity-ipc", daemon=True)
                receiver.start()
                result = core.dispatch("info", {})
            else:
                if request.get("session") != gate["session"]:
                    raise ValueError("controller session mismatch")
                op = request.get("op")
                mutation = op != "info"
                if mutation:
                    close_gate()
                    # A paused transport does not deliver standing updates.
                    # Reopening must require new feedback, never reuse a true
                    # sample that could have changed while admission was shut.
                    with core._lock:
                        core.clear_standing_mailbox()
                        core._standing = core._standing_at = None
                        # Even an idempotent start/config clears the transport
                        # barrier. Confirm STOP so it cannot discard a pending
                        # priority zero while leaving old motion active.
                        if op in ("start", "config") and core._running:
                            core._hold()
                if op == "shutdown":
                    result = core.stop()
                elif op == "yield":
                    result = {"released": core.pause_for_explicit_command(request.get("reason", "")),
                              "info": core.dispatch("info", {})}
                elif op in ("start", "config") and core._zero_pending:
                    with core._lock:
                        core._paused = True
                    result = {"state": "error", "error": "SDK stop unconfirmed; settings unchanged"}
                else:
                    result = core.dispatch(op, request.get("args", {}))
                if mutation:
                    if isinstance(result, dict) and result.get("state") == "error":
                        core.dispatch("pause", {})
                    with gate_lock:
                        gate["epoch"] = request["epoch"]
                        gate["active"] = (op not in ("shutdown", "yield", "stop", "pause") and
                                          isinstance(result, dict) and result.get("state") == "running" and
                                          not core._yielded)
                        core._inbox.activate(core._generation, gate["active"])
                if op == "shutdown" and result.get("state") != "idle":
                    raise RuntimeError("controller shutdown stop unconfirmed")
            _send_packet(control, {"session": gate["session"], "id": request["id"], "result": result})
            if request.get("op") == "shutdown":
                break
    except (EOFError, OSError):
        pass
    finally:
        done.set()
        close_gate()
        if core is not None:
            core.stop()
            # A failed STOP kills/reaps the native helper; firmware heartbeat
            # remains the last resort. Never leave an orphan refreshing it.
            if core._backend is not None:
                core._backend.close()
        data.close()
        control.close()


class LocoServoPlugin:
    def __init__(self, config, namespace, ros2, conflict=None):
        self._config, self._namespace, self._ros2 = dict(config), namespace, ros2
        self._conflict = conflict or (lambda: "")
        self._model = _Core(config, namespace, ros2)
        self._rpc_lock = threading.RLock()
        self._state_lock = threading.RLock()
        self._process = self._control_socket = self._data_socket = None
        self._sender = None
        self._sender_stop, self._sender_wake = threading.Event(), threading.Event()
        self._nodes = []
        self._session = ""
        self._epoch = self._subscription_generation = self._request_id = 0
        self._outbox = _ControlOutbox()
        self._fault = ""
        self._unsafe_until = 0.0
        self._group_terminated = False
        self._stopping = False
        self._last_info = self._model.dispatch("info", {})

    def get_tool(self):
        return self._model.get_tool()

    def start(self):
        """Bundle lazy-start hook, intentionally inert."""

    def _activate(self, active):
        with self._state_lock:
            self._outbox.activate(self._subscription_generation, self._session, self._epoch,
                                  active and not self._fault and not self._stopping)

    def _spawn(self):
        parent_control, child_control = socket.socketpair()
        parent_data, child_data = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
        parent_control.settimeout(max(3.0, float(self._config.get("sdk_connect_timeout_ms", 1500)) / 1000 + 3))
        parent_data.setblocking(False)
        self._session = uuid.uuid4().hex
        try:
            self._process = subprocess.Popen(
                [sys.executable, "-u", str(Path(__file__).resolve()),
                 "--control-fd", str(child_control.fileno()), "--data-fd", str(child_data.fileno())],
                pass_fds=(child_control.fileno(), child_data.fileno()), stdin=subprocess.DEVNULL,
                close_fds=True, start_new_session=True)
        except Exception:
            parent_control.close()
            parent_data.close()
            raise
        finally:
            child_control.close()
            child_data.close()
        self._control_socket, self._data_socket = parent_control, parent_data
        self._sender_stop, self._sender_wake = threading.Event(), threading.Event()
        self._rpc("init", config=self._config, namespace=self._namespace)
        self._sender = threading.Thread(target=self._send_loop, name="tianyi-loco-ipc", daemon=True)
        self._sender.start()

    def _read_reply(self):
        return _recv_packet(self._control_socket)

    def _rpc(self, op, **extra):
        self._request_id += 1
        request_id = self._request_id
        _send_packet(self._control_socket, {"id": request_id, "session": self._session,
                                           "epoch": self._epoch, "op": op, **extra})
        reply = self._read_reply()
        if reply.get("id") != request_id or reply.get("session") != self._session:
            raise RuntimeError("invalid controller acknowledgement")
        result = reply["result"]
        snapshot = result.get("info", result) if isinstance(result, dict) else None
        if isinstance(snapshot, dict) and "counts" in snapshot:
            self._last_info = copy.deepcopy(snapshot)
        return result

    def _mark_fault(self, reason):
        with self._state_lock:
            if self._fault or self._stopping:
                return
            self._fault = str(reason)
            self._unsafe_until = time.monotonic() + _UNCERTAIN_STOP_S
            self._activate(False)
            self._sender_stop.set()
        self._terminate_group()
        if self._control_socket is not None:
            try:
                self._control_socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    def _terminate_group(self):
        process = self._process
        if process is not None:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass  # There is no remaining member of this owned group.
            except OSError:
                try:
                    process.kill()  # EOF also revokes the native helper lease.
                except OSError:
                    pass  # Keep the fault latched; live children block handover.
                return
        # Start the backup wait only once the whole owned group was targeted
        # successfully (or has vanished), not merely its Python parent. An
        # orphan blocked in an SDK call might otherwise refresh much later.
        self._group_terminated = True
        self._unsafe_until = time.monotonic() + _UNCERTAIN_STOP_S

    def _send_loop(self):
        try:
            while not self._sender_stop.is_set():
                self._sender_wake.clear()
                if self._process.poll() is not None:
                    raise RuntimeError("velocity control process exited; stop/start the card")
                self._outbox.flush(self._data_socket)
                self._sender_wake.wait(0.01)
        except Exception as exc:
            self._mark_fault(exc)

    def _on_message(self, message, generation=None):
        received_at = (time.monotonic(), int(time.time() * 1000))
        try:
            if self._outbox.put_motion(message.data, self._subscription_generation if generation is None else generation, received_at):
                self._sender_wake.set()
        except Exception as exc:
            self._mark_fault(exc)

    def _on_standing(self, message, generation=None):
        try:
            if type(message.data) is not bool:
                # Invalid feedback must revoke, rather than preserve, true.
                value = False
            else:
                value = message.data
            if self._outbox.put_standing(value, self._subscription_generation if generation is None else generation):
                self._sender_wake.set()
        except Exception as exc:
            self._mark_fault(exc)

    def _open_nodes(self, topic):
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from std_msgs.msg import String, Bool
        self._subscription_generation += 1
        generation = self._subscription_generation
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        node = Node("tianyi2_loco_servo_in", context=self._ros2.ctx_core)
        self._nodes.append((node, self._ros2.executor_core))
        node.create_subscription(String, topic, lambda msg: self._on_message(msg, generation), qos)
        self._ros2.executor_core.add_node(node)
        if self._model._standing_topic:
            node = Node("tianyi2_loco_servo_standing", context=self._ros2.ctx_tianyi)
            self._nodes.append((node, self._ros2.executor_tianyi))
            node.create_subscription(Bool, self._model._standing_topic,
                                     lambda msg: self._on_standing(msg, generation), qos)
            self._ros2.executor_tianyi.add_node(node)

    def _close_nodes(self):
        # Invalidate callbacks before removing nodes, including already queued
        # Bool callbacks which have no producer timestamp of their own.
        self._subscription_generation += 1
        nodes, self._nodes = self._nodes, []
        for node, executor in reversed(nodes):
            try:
                executor.remove_node(node)
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass

    def _decorate(self, result):
        result = copy.deepcopy(result)
        if not isinstance(result, dict) or "counts" not in result:
            return result
        dispatch = result["dispatch"]
        dispatch["controller_mode"] = dispatch["mode"]
        dispatch.update(mode="process_worker", worker_pid=self._process.pid if self._process else None,
                        worker_alive=bool(self._process and self._process.poll() is None and not self._fault),
                        worker_failed=bool(self._fault) or dispatch["worker_failed"])
        result["transport"] = self._outbox.info()
        if self._fault:
            result.update(state="error", error=self._fault, output=[0.0] * 6)
            result["last"] = {"verdict": "ERROR", "reason": self._fault}
            result["backend"].update(connected=False, state="error", error=self._fault,
                                     stop_confirmed=False,
                                     process_group_terminated=self._group_terminated,
                                     handover_wait_ms=max(0, math.ceil((self._unsafe_until - time.monotonic()) * 1000)))
        return result

    def dispatch(self, action, args):
        if action == "stop":
            return self.stop()
        with self._rpc_lock:
            # Reject local argument errors before touching a healthy process or
            # its admission epoch. They are not transport/SDK failures.
            try:
                json.dumps(args, allow_nan=False)
                if action == "config":
                    if any(key in args and type(args[key]) is not bool for key in TOGGLES):
                        raise ValueError("configuration toggles must be boolean")
                    _validated_min_vx(args.get("min_vx", self._model._min_vx),
                                      self._model._descriptor["limits"]["upper"][0])
                    for key, current in zip(("linear_acceleration", "angular_acceleration"),
                                            self._model._accel):
                        self._model._positive(args, key, current)
            except (ValueError, TypeError) as exc:
                return {"state": "error", "error": str(exc)}
            if self._process is not None and self._process.poll() is not None and not self._stopping:
                self._mark_fault("velocity control process exited; stop/start the card")
            if self._fault:
                return self._decorate(self._last_info)
            if action in ("start", "resume"):
                problem = self._conflict()
                if problem:
                    return {"state": "error", "error": problem}
            if self._process is None:
                if action != "start":
                    result = self._model.dispatch(action, args)
                    if action == "config" and result.get("ok"):
                        self._config.update({key: result[key] for key in
                                             (*TOGGLES, "min_vx", "linear_acceleration", "angular_acceleration")})
                    self._last_info = self._model.dispatch("info", {})
                    return self._decorate(result)
                try:
                    self._spawn()
                except Exception as exc:
                    self._mark_fault(f"velocity control process start failed: {exc}")
                    return self._decorate(self._last_info)
            if action not in ("start", "pause", "resume", "info", "config"):
                return None
            mutation = action != "info"
            if mutation:
                self._epoch += 1
                self._activate(False)
            try:
                result = self._rpc(action, args=args)
                if mutation and "counts" in result and result.get("state") in ("running", "paused"):
                    self._close_nodes()
                    self._open_nodes(result["input"])
                if action == "config" and result.get("ok"):
                    settings = {key: result[key] for key in
                                (*TOGGLES, "min_vx", "linear_acceleration", "angular_acceleration")}
                    self._model.dispatch("config", settings)
                    self._config.update(settings)
                if mutation:
                    self._activate(result.get("state") == "running" and not result.get("yielded_to_explicit_command"))
                return self._decorate(result)
            except Exception as exc:
                self._mark_fault(f"velocity controller RPC failed: {exc}")
                return self._decorate(self._last_info)

    def pause_for_explicit_command(self, reason=""):
        with self._rpc_lock:
            if self._process is None:
                return not self._fault
            if self._process.poll() is not None:
                self._mark_fault("velocity control process exited; stop/start the card")
            if self._fault:
                return (self._group_terminated and self._process.poll() is not None and
                        time.monotonic() >= self._unsafe_until)
            self._epoch += 1
            self._activate(False)
            try:
                return self._rpc("yield", reason=reason).get("released") is True
            except Exception as exc:
                self._mark_fault(f"velocity controller handover unconfirmed: {exc}")
                return False

    def _cleanup(self):
        self._sender_stop.set()
        self._sender_wake.set()
        if self._sender is not None and self._sender is not threading.current_thread():
            self._sender.join(timeout=1)
        for channel in (self._control_socket, self._data_socket):
            if channel is not None:
                channel.close()
        self._close_nodes()
        self._process = self._control_socket = self._data_socket = None
        self._sender = None

    def stop(self):
        with self._rpc_lock:
            self._epoch += 1
            self._activate(False)
            if self._fault and not self._group_terminated:
                self._terminate_group()
                if not self._group_terminated:
                    return {"state": "error", "error": "control process group termination unconfirmed; restart blocked"}
            if self._fault and time.monotonic() < self._unsafe_until:
                return {"state": "error", "error": "controller stop unconfirmed; waiting for firmware heartbeat expiry"}
            try:
                if self._process is not None:
                    if not self._fault:
                        result = self._rpc("stop", args={})
                        if result.get("state") != "idle":
                            return result
                        self._stopping = True
                        self._rpc("shutdown")
                    self._stopping = True
                    self._process.wait(timeout=2)
                self._cleanup()
                self._fault = ""
                self._group_terminated = False
                self._last_info = self._model.dispatch("info", {})
                return {"state": "idle"}
            except Exception as exc:
                self._stopping = False
                self._mark_fault(f"velocity controller stop unconfirmed: {exc}")
                return {"state": "error", "error": self._fault}
            finally:
                self._stopping = False


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--control-fd", required=True, type=int)
    parser.add_argument("--data-fd", required=True, type=int)
    options = parser.parse_args()
    _child(options.control_fd, options.data_fd)
