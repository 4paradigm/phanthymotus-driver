"""S10-only executor recovery and depth restart, without ROS or hardware."""
import queue
import sys
import threading
import time
from types import ModuleType, SimpleNamespace

import pytest

from common.vendor_runtime import DualDomainROS2
from test_s10_front_depth import rig, configure, cloud
from test_s10_pro_control import _load_device, FakeNative


class InvalidHandle(Exception):
    pass


class Executor:
    def __init__(self):
        self.work = queue.Queue()

    def spin_once(self, timeout_sec=.1):
        try:
            item = self.work.get(timeout=timeout_sec)
        except queue.Empty:
            return
        if isinstance(item, Exception):
            raise item
        item()


@pytest.fixture
def runtime(monkeypatch):
    exceptions = ModuleType('rclpy.exceptions')
    exceptions.InvalidHandle = InvalidHandle
    monkeypatch.setitem(sys.modules, 'rclpy.exceptions', exceptions)
    active = threading.Event()
    active.set()
    ros = object.__new__(DualDomainROS2)
    ros.rclpy = SimpleNamespace(ok=lambda **kwargs: active.is_set())
    ros.ctx_robot = object()
    ros.executor_robot = Executor()
    ros.executor_core = None
    ros._threads = []
    from deep_robotics.s10_pro.ros_runtime import install_robot_executor_guard
    guard = install_robot_executor_guard(ros)
    ros.start_spin()
    yield SimpleNamespace(ros=ros, guard=guard, active=active)
    active.clear()
    for thread in ros._threads:
        thread.join(timeout=1)
        assert not thread.is_alive()


def run_callback(runtime, callback):
    finished = threading.Event()
    runtime.ros.executor_robot.work.put(callback)
    runtime.ros.executor_robot.work.put(finished.set)
    assert finished.wait(1), 'robot executor stopped before the next callback'


def test_install_waits_for_inflight_spin_before_allowing_new_entities(runtime):
    from deep_robotics.s10_pro.ros_runtime import install_robot_executor_guard
    runtime.ros.executor_robot.spin_once = runtime.guard._spin_once
    del runtime.ros.s10_robot_executor
    run_callback(runtime, lambda: None)
    blocked, release, installed = threading.Event(), threading.Event(), threading.Event()
    def blocked_callback():
        blocked.set()
        release.wait(1)
    runtime.ros.executor_robot.work.put(blocked_callback)
    assert blocked.wait(1)
    def install():
        install_robot_executor_guard(runtime.ros)
        installed.set()
    installer = threading.Thread(target=install)
    installer.start()
    try:
        assert not installed.wait(.03), 'installation left an unguarded spin in flight'
    finally:
        release.set()
        installer.join(timeout=1)
    assert installed.is_set()


def test_s10_factory_installs_guard_before_creating_ros_entities(runtime, monkeypatch):
    device = _load_device(monkeypatch)
    runtime.ros.executor_robot.spin_once = runtime.guard._spin_once
    del runtime.ros.s10_robot_executor
    def create_nodes(config, namespace, ros2):
        ros2.executor_robot.work.put(InvalidHandle('entity destroyed during node construction'))
        run_callback(runtime, lambda: None)
        return None
    monkeypatch.setattr(device, 'BasicServerClient', FakeNative)
    monkeypatch.setattr(device, 'S10Nodes', create_nodes)
    device.build_plugins({'model_variant': 'standard'}, 's10', runtime.ros)
    assert runtime.ros.s10_robot_executor.status()['recoveries'] == 1


def test_stop_start_recovers_destroyed_handle_and_resumes_depth_and_ssh_watchdog(
        rig, runtime, monkeypatch):
    rig.nodes.robot_executor_status = runtime.guard.status
    configure(rig)
    assert rig.plugin.dispatch('start', {})['state'] == 'running'
    old_subscription, old_worker = rig.subscriptions[-1], rig.plugin._worker
    run_callback(runtime, lambda: old_subscription.callback(cloud()))
    run_callback(runtime, old_worker.complete)
    assert len(rig.publications) == 1

    destroy = rig.nodes.robot.destroy_timer
    def destroy_during_wait(timer):
        destroy(timer)
        runtime.ros.executor_robot.work.put(
            InvalidHandle('cannot use Destroyable because destruction was requested'))
    monkeypatch.setattr(rig.nodes.robot, 'destroy_timer', destroy_during_wait)
    assert rig.plugin.dispatch('stop', {})['state'] == 'idle'
    run_callback(runtime, lambda: None)
    assert runtime.guard.status()['recoveries'] == 1

    assert rig.plugin.dispatch('start', {})['state'] == 'running'
    run_callback(runtime, lambda: old_subscription.callback(cloud(sec=11)))
    run_callback(runtime, old_worker.complete)
    assert len(rig.publications) == 1
    future = time.monotonic() + 4
    monkeypatch.setattr(time, 'monotonic', lambda: future)
    run_callback(runtime, rig.timers[-1].callback)
    assert rig.plugin.dispatch('info', {})['input']['transport'] == 'ssh'
    run_callback(runtime, lambda: rig.relays[-1].on_message(cloud(sec=12)))
    run_callback(runtime, rig.plugin._worker.complete)
    assert len(rig.publications) == 2
    assert rig.publications[-1].header.stamp.sec == 12
    assert rig.plugin.dispatch('info', {})['state'] == 'running'
    rig.plugin.stop()


def test_guard_is_idempotent_and_does_not_change_other_executors(runtime):
    from deep_robotics.s10_pro.ros_runtime import install_robot_executor_guard
    assert install_robot_executor_guard(runtime.ros) is runtime.guard
    runtime.ros.executor_robot.work.put(InvalidHandle('destroyed'))
    run_callback(runtime, lambda: None)
    assert runtime.guard.status()['recoveries'] == 1
    other = Executor()
    other.work.put(InvalidHandle('unguarded'))
    with pytest.raises(InvalidHandle):
        other.spin_once()


def test_unknown_executor_failure_is_not_hidden_and_depth_cannot_restart(rig, runtime):
    rig.nodes.robot_executor_status = runtime.guard.status
    configure(rig)
    assert rig.plugin.dispatch('start', {})['state'] == 'running'
    runtime.ros.executor_robot.work.put(RuntimeError('executor failed'))
    runtime.ros._threads[0].join(timeout=1)
    assert not runtime.ros._threads[0].is_alive()
    info = rig.plugin.dispatch('info', {})
    assert info['state'] == 'error'
    assert info['error_code'] == 'ROS_EXECUTOR_UNAVAILABLE'
    assert 'executor failed' in info['last_error']
    assert info['input']['executor']['thread_alive'] is False
    assert rig.plugin.dispatch('start', {})['code'] == 'ROS_EXECUTOR_UNAVAILABLE'
    rig.plugin.stop()


def test_missing_executor_progress_is_reported_and_success_clears_it(rig, runtime, monkeypatch):
    rig.nodes.robot_executor_status = runtime.guard.status
    configure(rig)
    rig.plugin.dispatch('start', {})
    blocked, release = threading.Event(), threading.Event()
    def blocked_callback():
        blocked.set()
        release.wait(1)
    runtime.ros.executor_robot.work.put(blocked_callback)
    assert blocked.wait(1)
    future = time.monotonic() + 4
    monkeypatch.setattr(time, 'monotonic', lambda: future)
    try:
        info = rig.plugin.dispatch('info', {})
        assert info['state'] == 'error'
        assert info['error_code'] == 'ROS_EXECUTOR_UNAVAILABLE'
        assert info['input']['executor']['thread_alive'] is True
        assert rig.plugin.dispatch('start', {})['code'] == 'ROS_EXECUTOR_UNAVAILABLE'
    finally:
        release.set()
    run_callback(runtime, lambda: None)
    info = rig.plugin.dispatch('info', {})
    assert info['state'] == 'running' and info['error_code'] is None
    rig.plugin.stop()


def test_destroyed_handle_during_context_shutdown_does_not_retry(runtime):
    runtime.active.clear()
    runtime.ros._threads[0].join(timeout=1)
    runtime.ros.executor_robot.work.put(InvalidHandle('shutdown'))
    with pytest.raises(InvalidHandle):
        runtime.ros.executor_robot.spin_once()
    assert runtime.guard.status()['recoveries'] == 0
