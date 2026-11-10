"""Loopback-only Dashboard + 1440-byte feedback simulator (not firmware validation).

Its CSV is intentionally synthetic. It tests the configurable bridge without
presenting an invented layout as a controller-native format.
"""

import csv
import json
from pathlib import Path
import re
import socket
import threading
import time

import numpy as np
from dobot_sdk.protocol.data_types import MyType, TEST_VALUE_MAGIC


class RobotSimulator:
    def __init__(self, directory):
        self.directory = Path(directory)
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
        self.failure_command = None
        self.feedback_enabled = True
        self.skin = 0
        self.skin_setting = [0, 0, 0, 0]
        self.alarm = False
        self.collision = False
        self.return_path_id = False
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
        if self.mode == 6:
            self.joints[0] += self.hand_velocity * dt
        elif self.mode == 7 and self.motion is not None:
            motion = self.motion
            fraction = min(1.0, (now - motion["start"]) / motion["duration"])
            self.joints = [a + (b - a) * fraction for a, b in zip(motion["origin"], motion["target"])]
            if fraction >= 1:
                self.mode, self.motion = 5, None
        elif self.mode == 7 and now >= self.setting_until:
            self.mode = 5

    def _move(self, target, duration):
        self.mode = 7
        self.command_id += 1
        self.motion = dict(start=time.monotonic(), duration=max(0.1, duration),
                           origin=list(self.joints), target=list(target))

    def _rows(self, filename):
        with (self.directory / filename).open() as file:
            rows = list(csv.reader(file))
        assert rows[0] == ["SIMULATOR_ONLY_NOT_A_DOBOT_FORMAT"]
        return [[float(value) for value in row] for row in rows[1:]]

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
            elif name == "StopDrag":
                self.mode, self.hand_velocity = 5, 0
            elif name == "GetStartPose":
                filename = re.search(r'"([^"\n]+)"', command)[1]
                values = ",".join(str(value) for value in self._rows(filename)[0])
                return f"0,{{1,{{{values}}}}},{command};"
            elif name == "MovJ":
                target = [float(value) for value in re.search(r"joint=\{([^}]+)\}", command)[1].split(",")]
                self._move(target, 0.14)
                result = str(self.command_id)
            elif name == "StartPath":
                filename = re.search(r'"([^"\n]+)"', command)[1]
                rows = self._rows(filename)
                speed = float(re.search(r"multi=([\d.]+)", command)[1])
                self._move(rows[-1], (len(rows) - 1) * 0.05 / speed)
                result = str(self.command_id) if self.return_path_id else ""
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
                    record["RobotMode"] = self.mode
                    record["CRRobotType"] = 113
                    record["EnableStatus"] = int(self.enabled)
                    record["DragStatus"] = int(self.mode == 6)
                    record["RunningStatus"] = int(self.mode == 7 and self.motion is not None)
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
