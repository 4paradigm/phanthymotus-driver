"""Independent UDP publisher; no robot, SDK output, or shared-process delivery."""
import json
from pathlib import Path
import subprocess
import sys
import time

DRIVER = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DRIVER))


def test_slow_decode_cannot_block_another_reader_or_hide_source_age():
    result = subprocess.run([sys.executable, __file__, 'reader'], capture_output=True,
                            text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr


def initialize():
    from unitree_sdk2py.core import channel as sdk
    sdk.ChannelConfigHasInterface = '''<CycloneDDS><Domain><General>
      <Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast>
      </General><Discovery><ParticipantIndex>auto</ParticipantIndex>
      <Peers><Peer Address="127.0.0.1"/></Peers></Discovery>
      <Tracing><Verbosity>none</Verbosity></Tracing></Domain></CycloneDDS>'''
    sdk.ChannelFactoryInitialize(97, 'lo')
    return sdk


def writer():
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
    sdk = initialize()
    publisher = sdk.ChannelPublisher('rt/lowstate', LowState_)
    publisher.Init()
    callbacks = sdk.ChannelPublisher('test/sdk_callback', String_)
    callbacks.Init()
    time.sleep(1)
    start = time.monotonic()
    msg = unitree_hg_msg_dds__LowState_()
    count = 0
    while time.monotonic()-start < 3:
        msg.tick = (time.monotonic_ns()//1_000_000) & 0xffffffff
        assert publisher.Write(msg)
        assert callbacks.Write(String_(str(msg.tick)))
        count += 1
        time.sleep(max(0., start+count*.001-time.monotonic()))
    print(json.dumps({'count':count, 'last_tick':msg.tick}), flush=True)
    time.sleep(.5)
    publisher.Close()
    callbacks.Close()


def reader():
    import threading
    import weakref
    from cyclonedds.qos import Policy
    from cyclonedds.sub import DataReader
    from sdk_adapter import FeedbackSubscriber
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
    from arm_stream import ArmStreamExecutor
    from low_state import LowStateSource
    sdk = initialize()
    original_init, original_take = DataReader.__init__, DataReader.take
    readers = []
    def track_reader(self, *args, **kwargs):
        original_init(self, *args, **kwargs)
        # Weak references must not keep a DDS reader alive after Driver Close.
        readers.append(weakref.ref(self))
    DataReader.__init__ = track_reader
    decode_threads = set()
    def slow_take(self, *args, **kwargs):
        # Only G1 LowState decoding is moved off the shared DDS receive thread.
        if getattr(self, 'slow_decode', False):
            decode_threads.add(threading.current_thread())
            time.sleep(.004)
        return original_take(self, *args, **kwargs)
    DataReader.take = slow_take
    source = LowStateSource()
    source.start()
    assert len(readers) == 1
    assert readers[0]().get_qos()[Policy.History] == Policy.History.KeepLast(1)
    readers[0]().slow_decode = True
    arm = ArmStreamExecutor({}, 'test', None, low_state=source)
    arm.start()
    callback_ages = []
    other = FeedbackSubscriber('test/sdk_callback', String_)
    other.Init(lambda msg: callback_ages.append((time.monotonic_ns()//1_000_000-int(msg.data)) & 0xffffffff))
    other_worker = other._thread
    assert readers[-1]().get_qos()[Policy.History] == Policy.History.KeepLast(1)
    # Also retain the original proof that LowState decode cannot block SDK RPC-style callbacks.
    sdk_callback_ages = []
    sdk_callback = sdk.ChannelSubscriber('test/sdk_callback', String_)
    threads_before = set(threading.enumerate())
    sdk_callback.Init(lambda msg: sdk_callback_ages.append(
        (time.monotonic_ns()//1_000_000-int(msg.data)) & 0xffffffff), 10)
    sdk_workers = [thread for thread in threading.enumerate()
                   if thread not in threads_before and thread.name == 'ch_reader']
    assert len(sdk_workers) == 1
    producer = subprocess.Popen([sys.executable, __file__, 'writer'], stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, text=True)
    samples = []
    last_tick = None
    try:
        deadline = time.monotonic()+7
        while producer.poll() is None and time.monotonic()<deadline:
            state = arm._snapshot()
            tick = state.get('source_tick')
            if tick is not None and tick != last_tick:
                samples.append(((time.monotonic_ns()//1_000_000-tick) & 0xffffffff,
                                (time.monotonic_ns()-state['arm_ns'])/1e6))
                last_tick = tick
            time.sleep(.01)
        stdout, stderr = producer.communicate(timeout=2)
        assert producer.returncode == 0, stderr
        assert json.loads(stdout)['count'] > 2000
        assert len(callback_ages) > 2000 and len(samples) > 100
        assert max(callback_ages) < 100, callback_ages
        assert decode_threads == {source._thread}
        assert len(sdk_callback_ages) > 2000
        assert max(sdk_callback_ages) < 100, sdk_callback_ages
        assert max(age for age, _ in samples) < 100, samples
        assert max(age for _, age in samples) < 100, samples
    finally:
        if producer.poll() is None: producer.kill(); producer.wait()
        arm.stop()
        worker = source._thread
        source.stop(); source.stop()
        assert not worker.is_alive() and source.snapshot() is None
        other.Close()
        other.Close()  # Driver teardown is idempotent even though SDK Close is not.
        assert not other_worker.is_alive() and other._channel is None
        sdk_callback.Close()  # Restored SDK readers are closed exactly once.
        assert all(not thread.is_alive() for thread in sdk_workers)
        DataReader.take = original_take
        DataReader.__init__ = original_init


if __name__ == '__main__':
    (writer if sys.argv[1] == 'writer' else reader)()
