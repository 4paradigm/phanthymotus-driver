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
    sdk = initialize()
    publisher = sdk.ChannelPublisher('rt/lowstate', LowState_)
    publisher.Init()
    time.sleep(1)
    start = time.monotonic()
    msg = unitree_hg_msg_dds__LowState_()
    count = 0
    while time.monotonic()-start < 3:
        msg.tick = (time.monotonic_ns()//1_000_000) & 0xffffffff
        assert publisher.Write(msg)
        count += 1
        time.sleep(max(0., start+count*.001-time.monotonic()))
    print(json.dumps({'count':count, 'last_tick':msg.tick}), flush=True)
    time.sleep(.5)
    publisher.Close()


def reader():
    from cyclonedds.qos import Qos, Policy
    from cyclonedds.sub import DataReader
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    from arm_stream import ArmStreamExecutor
    sdk = initialize()
    original_take = DataReader.take
    def slow_take(self, *args, **kwargs):
        # Model expensive decoding in a second sensor subscription, before its BQueue.
        if getattr(self, 'slow_decode', False): time.sleep(.004)
        return original_take(self, *args, **kwargs)
    DataReader.take = slow_take
    arm = ArmStreamExecutor({}, 'test', None)
    arm.start()
    slow_samples = []
    other = sdk.ChannelSubscriber('rt/lowstate', LowState_)
    other.Init(lambda msg: slow_samples.append(msg.tick), 10, qos=Qos(Policy.History.KeepLast(1)))
    other_reader = other._ChannelSubscriber__channel._Channel__reader._Reader__reader
    other_reader.slow_decode = True
    readers = [s._ChannelSubscriber__channel._Channel__reader
               for s in (arm._subscriber, arm._fsm_subscriber, arm._arm_observer, other)]
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
        assert slow_samples and len(samples) > 100
        assert max(age for age, _ in samples) < 100, samples
        assert max(age for _, age in samples) < 100, samples
        assert all(b >= a for a,b in zip(slow_samples,slow_samples[1:]))
    finally:
        if producer.poll() is None: producer.kill(); producer.wait()
        arm.stop()
        other.Close()
        other.Close()  # Repeated teardown must not leave a polling/queue thread alive.
        for r in readers:
            assert not r._Reader__readThread.is_alive()
        assert not readers[-1]._Reader__threadReader.is_alive()
        DataReader.take = original_take


if __name__ == '__main__':
    (writer if sys.argv[1] == 'writer' else reader)()
