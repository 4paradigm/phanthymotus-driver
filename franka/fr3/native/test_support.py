"""Shared hardware-free fixtures for native SDK and Python pipe boundary tests."""
import json
import queue
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path


def eventually(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("Session condition did not become true")


class NativeProcessCase(unittest.TestCase):
    event_timeout = 8

    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("c++")
        if compiler is None:
            raise unittest.SkipTest("C++ compiler unavailable")
        temporary = tempfile.TemporaryDirectory(prefix="fr3-native-test-")
        cls.addClassCleanup(temporary.cleanup)
        cls.directory = Path(temporary.name)
        headers = cls.directory / "franka"
        headers.mkdir()
        (headers / "fake.h").write_text(cls.fake_sdk)
        for name in cls.headers:
            (headers / name).write_text('#include "fake.h"\n')
        cls.binary = cls.directory / "backend"
        built = subprocess.run(
            [compiler, "-std=c++17", "-pthread", "-Wall", "-Wextra", "-Wpedantic",
             "-I", str(cls.directory), str(Path(__file__).with_name(cls.source)),
             "-o", str(cls.binary)], check=False, capture_output=True, text=True)
        if built.returncode:
            raise RuntimeError(built.stderr)

    def setUp(self):
        self.processes = []
        self.lock_dir = tempfile.TemporaryDirectory(prefix="fr3-lock-test-")
        self.addCleanup(self.lock_dir.cleanup)

    def spawn(self, address, env, *, consume=True):
        process = subprocess.Popen([str(self.binary), address], stdin=subprocess.PIPE,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                   text=True, bufsize=1, env=env)
        self.addCleanup(self.reap, process)
        received, history = queue.Queue(), []

        def receive():
            for line in process.stdout:
                record = json.loads(line)
                history.append(record)
                received.put(record)

        if consume:
            threading.Thread(target=receive, daemon=True).start()
        item = process, received, history
        self.processes.append(item)
        return item

    @staticmethod
    def reap(process):
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
        for pipe in (process.stdin, process.stdout, process.stderr):
            if pipe and not pipe.closed:
                pipe.close()

    def wait_for(self, item, predicate, timeout=None):
        deadline = time.monotonic() + (self.event_timeout if timeout is None else timeout)
        while time.monotonic() < deadline:
            try:
                message = item[1].get(timeout=max(0.001, deadline - time.monotonic()))
            except queue.Empty:
                break
            if predicate(message):
                return message
        self.fail(f"Expected native event missing; observed {item[2][-8:]}")

    def send(self, item, line):
        item[0].stdin.write(line + "\n")
        item[0].stdin.flush()


class PipeFactory:
    def __init__(self, path, mode, backend, split_commands):
        self.path, self.mode, self.backend = path, mode, backend
        self.split_commands = split_commands
        self.processes, self.calls = [], []
        self.lock = threading.Lock()

    def __call__(self, args, **options):
        with self.lock:
            self.calls.append(args)
            process = subprocess.Popen(
                [sys.executable, "-u", "-c", self.backend, self.mode, self.path], **options)
            self.processes.append(process)
            return process

    def commands(self):
        lines = Path(self.path).read_text().splitlines()
        return [line.split() for line in lines] if self.split_commands else lines


class PipeSessionCase(unittest.TestCase):
    def session(self, mode="hold", **options):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = str(Path(directory.name) / "commands.log")
        Path(path).touch()
        factory = PipeFactory(path, mode, self.backend, self.split_commands)
        defaults = dict(connect_timeout=.3, stale_after=.15, retry_delay=.1,
                        retry_max=.2, stop_timeout=.1, motion_stop_timeout=.4)
        defaults.update(options)
        manager = self.hardware.SessionManager(
            session_factory=self.session_type, process_factory=factory, **defaults)
        self.addCleanup(manager.close)
        owner = (self.owner_tool, "first")
        session = manager.acquire("172.16.0.3", owner)
        eventually(lambda: session.snapshot()["connected"])
        return manager, session, factory, owner
