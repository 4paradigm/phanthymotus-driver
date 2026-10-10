"""Thread and failed-spawn lifecycle checks; no ROS network or actuators."""
import threading
from types import SimpleNamespace
import pytest
from teleop_executor import TeleopExecutor


def executor():
    result=TeleopExecutor.__new__(TeleopExecutor)
    result._lifecycle_lock=threading.RLock()
    result._closed=threading.Event()
    result._thread=None;result._bus_socket=None;result._bus_process=None
    result._feedback_executor=None;result._feedback_thread=None
    result._feedback_closed=threading.Event();result._feedback_error=None
    result.node=None;result._subscribed=False
    result.ns='test'
    result.gate=SimpleNamespace(session_id=None,hold=lambda *a,**k:None)
    result.info=lambda:{'ownership_held':bool(result.gate.session_id)}
    return result


def test_repeated_start_stop_and_restart_are_idempotent():
    item=executor();starts=[]
    class Process:
        def poll(self):return None
        def terminate(self):pass
        def wait(self,timeout=None):return 0
    def start():
        starts.append(True);item._closed.clear();item._bus_process=Process()
        item._thread=threading.Thread(target=item._closed.wait)
        item._thread.start()
    item._start=start
    item.start();thread=item._thread
    item.start();assert item._thread is thread and len(starts)==1
    item.stop();item.stop()
    assert not thread.is_alive() and item._thread is None
    assert item._bus_process is None
    item.start();assert len(starts)==2 and item._thread is not thread
    item.stop()


def test_failed_spawn_closes_socket_and_allows_retry(monkeypatch):
    import socket
    import teleop_executor as module
    item=executor();item.subscribe_feedback=lambda:None
    pair=socket.socketpair(socket.AF_UNIX,socket.SOCK_DGRAM)
    monkeypatch.setattr(module.socket,'socketpair',lambda *a,**kw:pair)
    monkeypatch.setattr(module.subprocess,'Popen',lambda *a,**kw:(_ for _ in ()).throw(OSError('spawn_failed')))
    with pytest.raises(OSError,match='spawn_failed'):item.start()
    assert all(s.fileno()==-1 for s in pair)
    assert item._thread is None and item._bus_socket is None and item._bus_process is None
    # A later explicit start is not suppressed by a stale lifecycle flag.
    entered=[];item._start=lambda:entered.append(True)
    item.start();assert entered==[True]


def test_latched_ownership_prevents_silent_executor_restart():
    item=executor();item.gate.session_id='latched'
    item._start=lambda:pytest.fail('must not restart while held')
    with pytest.raises(RuntimeError,match='confirmed_release'):item.start()


def test_feedback_node_is_removed_and_destroyed_once():
    item=executor();removed=[];destroyed=[]
    node=SimpleNamespace(destroy_node=lambda:destroyed.append(True))
    item.node=node;item._subscribed=True
    shutdown=[]
    item._feedback_executor=SimpleNamespace(remove_node=removed.append,
        shutdown=lambda **kw:shutdown.append(True))
    item._feedback_thread=threading.Thread(target=item._feedback_closed.wait)
    item._feedback_thread.start();thread=item._feedback_thread
    item.stop();item.stop()
    assert removed==[node] and destroyed==[True]
    assert item.node is None and not item._subscribed
    assert not thread.is_alive() and item._feedback_executor is None
    assert shutdown==[True]


def test_stuck_feedback_thread_is_not_destroyed_under_it():
    item=executor();destroyed=[]
    item.node=SimpleNamespace(destroy_node=lambda:destroyed.append(True))
    item._feedback_thread=SimpleNamespace(join=lambda **kw:None,is_alive=lambda:True)
    with pytest.raises(RuntimeError,match='feedback_thread_stop_unconfirmed'):
        item.stop()
    assert item.node is not None and not destroyed


def test_always_ready_feedback_yields_and_shutdown_does_not_drain_backlog():
    item=executor();calls=[];waits=[]
    class Closed:
        done=False
        def is_set(self):return self.done
        def wait(self,timeout):
            waits.append((len(calls),timeout))
            if len(waits)==2:self.done=True
    item._feedback_closed=Closed()
    item._feedback_executor=SimpleNamespace(spin_once=lambda **kw:calls.append(kw))
    item._spin_feedback()
    assert waits==[(10,.01),(20,.01)] and len(calls)==20
    assert all(c['timeout_sec']==0. for c in calls)
    assert item._feedback_error is None
    # Closing in the middle of a permanently ready stream stops immediately.
    item._feedback_closed.done=False;calls.clear();waits.clear()
    def close_in_callback(**kw):
        calls.append(kw);item._feedback_closed.done=True
    item._feedback_executor.spin_once=close_in_callback
    item._spin_feedback()
    assert len(calls)==1 and waits==[]


def test_batched_feedback_exception_still_holds_and_reports_failure():
    item=executor();holds=[]
    item.gate.hold=holds.append
    def broken(**kw):raise RuntimeError('reader_failed')
    item._feedback_executor=SimpleNamespace(spin_once=broken)
    item._spin_feedback()
    assert holds==['feedback_executor_failed'] and item._feedback_error=='RuntimeError'


def test_watchdog_period_includes_work_without_overrun_catchup(monkeypatch):
    import teleop_executor as module
    item=executor();now=[0];starts=[];waits=[];durations=[5,35,5]
    monkeypatch.setattr(module.time,'monotonic_ns',lambda:now[0])
    class Closed:
        done=False
        def is_set(self):return self.done
        def wait(self,timeout):
            waits.append(timeout);now[0]+=round(timeout*1e9)
            self.done=len(waits)==3
    item._closed=Closed();item._watchdog_timing={};item._tick_timing={}
    item._bus_process=SimpleNamespace(poll=lambda:None)
    item._bus_socket=SimpleNamespace(send=lambda payload:len(payload))
    def receive():
        starts.append(now[0]);now[0]+=durations[len(starts)-1]*1_000_000
    item._receive_latest_command=receive;item.tick=lambda:None
    item._watchdog_loop()
    assert starts==[0,20_000_000,55_000_000]
    assert waits==pytest.approx([.015,0.,.015])
    assert item._watchdog_timing['max_cycle_ms']==35
