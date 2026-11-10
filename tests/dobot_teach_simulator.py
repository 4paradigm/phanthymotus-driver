"""Loopback-only Dashboard + 1440-byte feedback simulator (not firmware validation)."""

import re
import socket
import threading
import time

import numpy as np
from dobot_sdk.protocol.data_types import MyType, TEST_VALUE_MAGIC


class RobotSimulator:
    def __init__(self):
        self.lock = threading.RLock()
        self.done = threading.Event()
        self.commands = []
        self.connections = []
        self.threads = []
        self.mode = 5
        self.enabled = True
        self.joints = [0.0] * 6
        self.command_id = 0
        self.setting_until = 0
        self.motion = None
        self.hand_velocity = 0.0
        self.joint_speed = None  # overrides QDActual, e.g. an arm still coasting in mode 5
        self.drag_status = None  # overrides DragStatus, e.g. the field-observed mode 6 without drag
        self.running_packets = 0  # mode-5 packets reporting RunningStatus=1 (field: right after drag)
        self.entry_packets = 0  # mode-6 packets first reported as mode 0 (field: drag-button entry)
        self.failure_command = None
        self.feedback_enabled = True
        self.skin = 0
        self.skin_setting = [0, 0, 0, 0]
        self.alarm = False
        self.collision = False
        self.servo_hold_s = 0.06  # mode 7 persists this long after the last ServoJ target
        self.servo_gap = None  # stop following ServoJ targets (tracking loss)
        self.timestamp = 100000
        self.clock = time.monotonic()
        self.listeners = []
        for handler in (self._dashboard, self._feedback):
            listener = socket.socket()
            listener.bind(("127.0.0.1", 0))
            listener.listen()
            listener.settimeout(0.05)
            self.listeners.append(listener)
            thread = threading.Thread(target=self._accept, args=(listener, handler), daemon=True)
            thread.start()
            self.threads.append(thread)
        self.dashboard_port = self.listeners[0].getsockname()[1]
        self.feedback_port = self.listeners[1].getsockname()[1]

    def _accept(self, listener, handler):
        while not self.done.is_set():
            try:
                connection, _ = listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            self.connections.append(connection)
            thread = threading.Thread(target=handler, args=(connection,), daemon=True)
            thread.start()
            self.threads.append(thread)

    def _advance(self):
        now = time.monotonic()
        dt = now - self.clock
        self.clock = now
        if self.mode in (0, 6):
            self.joints[0] += self.hand_velocity * dt
        elif self.mode == 7 and self.motion is not None:
            motion = self.motion
            fraction = min(1.0, (now - motion["start"]) / motion["duration"])
            self.joints = [a + (b - a) * fraction for a, b in zip(motion["origin"], motion["target"])]
            if fraction >= 1 and now >= motion.get("hold_until", 0):
                self.mode, self.motion = 5, None
        elif self.mode == 7 and now >= self.setting_until:
            self.mode = 5

    def _move(self, target, duration):
        self.mode = 7
        self.command_id += 1
        self.motion = dict(start=time.monotonic(), duration=max(0.1, duration),
                           origin=list(self.joints), target=list(target))

    def handle(self, command):
        with self.lock:
            self._advance()
            self.commands.append(command)
            name = command.split("(", 1)[0]
            if self.failure_command == name:
                return f"-1,{{}},{command};"
            result = ""
            if name == "ClearError":
                self.alarm = self.collision = False
                self.mode = 5 if self.enabled else 4
            elif name == "EnableRobot":
                self.enabled, self.mode = True, 5
            elif name in ("EnableSafeSkin", "SetSafeSkin"):
                if name == "EnableSafeSkin":
                    self.skin = int(command[len(name) + 1:-1])
                else:
                    region, level = map(int, command[len(name) + 1:-1].split(","))
                    self.skin_setting[region - 3] = level
                self.mode = 7
                self.command_id += 1
                self.setting_until = time.monotonic() + 0.025
                result = str(self.command_id)
            elif name == "StartDrag":
                self.mode = 6
            elif name == "ServoJ":
                values = command[len(name) + 1:-1].split(",")
                target = [float(value) for value in values[:6]]
                if self.servo_gap is not None:
                    target = [value + self.servo_gap for value in target]
                period = float(re.search(r"t=([\d.]+)", command)[1])
                self._move(target, period)
                self.motion.update(duration=period, hold_until=time.monotonic() + period + self.servo_hold_s)
                result = str(self.command_id)
            elif name == "MovJ":
                target = [float(value) for value in re.search(r"joint=\{([^}]+)\}", command)[1].split(",")]
                self._move(target, 0.14)
                result = str(self.command_id)
            elif name == "Pause":
                if self.motion is None:
                    return f"-1,{{}},{command};"
                self.motion["duration"] = max(.1, self.motion["duration"] - (time.monotonic() - self.motion["start"]))
                self.motion["origin"] = list(self.joints)
                self.mode = 10
            elif name == "Continue":
                if self.mode != 10 or self.motion is None:
                    return f"-1,{{}},{command};"
                self.mode = 7
                self.motion["start"] = time.monotonic()
            elif name == "Stop":
                self.mode, self.motion = 5, None
            else:
                raise AssertionError(f"unexpected simulator command {command}")
            return f"0,{{{result}}},{command};"

    def _dashboard(self, connection):
        try:
            with connection.makefile("rb") as file:
                while not self.done.is_set():
                    data = file.readline()
                    if not data:
                        return
                    response = self.handle(data.decode().strip()).encode()
                    # Fragment acknowledgements to exercise framing.
                    connection.sendall(response[:4])
                    connection.sendall(response[4:])
        except (OSError, ValueError):
            return

    def _feedback(self, connection):
        try:
            while not self.done.wait(0.008):
                with self.lock:
                    self._advance()
                    if not self.feedback_enabled:
                        continue
                    record = np.zeros(1, dtype=MyType)
                    record["len"], record["TestValue"] = 1440, TEST_VALUE_MAGIC
                    self.timestamp += 8
                    record["TimeStamp"] = self.timestamp
                    reported = self.mode
                    if self.mode == 6 and self.entry_packets:
                        self.entry_packets -= 1
                        reported = 0
                    record["RobotMode"] = reported
                    record["CRRobotType"] = 113
                    record["EnableStatus"] = int(self.enabled)
                    record["DragStatus"] = (int(self.mode in (0, 6)) if self.drag_status is None
                                            else int(self.drag_status))
                    running = self.mode == 7 and self.motion is not None
                    if self.mode == 5 and self.running_packets:
                        self.running_packets -= 1
                        running = True
                    record["RunningStatus"] = int(running)
                    record["RunQueuedCmd"] = int(self.mode == 7)
                    record["PauseCmdFlag"] = 0  # Matches the actual controller observation.
                    record["ErrorStatus"] = int(self.alarm)
                    record["CollisionState"] = int(self.collision)
                    record["CurrentCommandId"] = self.command_id
                    record["QActual"] = self.joints
                    record["QDActual"] = (self.joint_speed if self.joint_speed is not None else
                                          [1 if self.mode == 7 and self.motion is not None else 0] * 6)
                    data = record.tobytes()
                connection.sendall(data[:37])
                connection.sendall(data[37:])
        except OSError:
            return

    def close(self):
        self.done.set()
        for connection in self.connections:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()
        for listener in self.listeners:
            listener.close()
        for thread in self.threads:
            thread.join(0.3)
