"""G1-owned extensions composed from the bundled SDK's existing interfaces.

No global SDK patches or new DDS participant. A subscriber owns its channel and,
only when a handler is supplied, a bounded-read thread. LowStateSource keeps its
own reader thread. SDK RPC callbacks and other subscriptions remain unchanged.
"""
import json
import logging
import threading


_read_context = threading.local()


class _EmptyReadFilter(logging.Filter):
    def filter(self, record):
        # CycloneDDS 0.10.5 signals an empty timed read with StopIteration;
        # the unchanged SDK catches it as a generic error. Only normalize this
        # exact exception inside our reads, never other SDK callers/errors.
        return not (getattr(_read_context, 'active', False)
                    and record.exc_info and record.exc_info[0] is StopIteration)


logging.getLogger('unitree_sdk2py.core.channel').addFilter(_EmptyReadFilter())


def request_reset(client):
    """Send action 99 without awaiting an RPC reply; not physical completion."""
    from unitree_sdk2py.g1.arm.g1_arm_action_api import ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION
    return client._CallNoReply(ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION,
                               json.dumps({'data': 99}))


class FeedbackSubscriber:
    """Latest-state subscription using public ChannelFactory/Channel methods.

    Never install an SDK on_data_available callback: its empty-take indexing and
    deserialization run before a user handler could guard them. Read with a
    positive timeout instead, and close once after all owned reads have ended.
    Callers using synchronous Read must stop/join their reader before Close.
    """
    def __init__(self, topic, dtype):
        self._topic, self._dtype = topic, dtype
        self._channel = None
        self._thread = None
        self._closed = threading.Event()
        self._lifecycle_lock = threading.Lock()

    def Init(self, handler=None):
        from cyclonedds.qos import Qos, Policy
        from unitree_sdk2py.core.channel import ChannelFactory
        with self._lifecycle_lock:
            if self._channel is not None:
                return
            channel = ChannelFactory().CreateChannel(self._topic, self._dtype)
            try:
                channel.SetReader(Qos(Policy.History.KeepLast(1)))
                self._channel = channel
                self._closed.clear()
                if handler is not None:
                    thread = threading.Thread(target=self._read_loop, args=(handler,),
                                              name='g1-feedback', daemon=True)
                    thread.start()
                    self._thread = thread
            except Exception:
                self._closed.set()
                self._channel = None
                try:
                    channel.CloseReader()
                except Exception:
                    logging.getLogger(__name__).exception('Failed to close partial feedback reader')
                raise

    def Read(self, timeout=.1):
        from cyclonedds.internal import InvalidSample
        if timeout is None or not 0 < timeout <= .1:
            raise ValueError('feedback_read_timeout_must_be_positive_and_bounded')
        channel = self._channel
        if channel is None or self._closed.is_set():
            return None
        previous = getattr(_read_context, 'active', False)
        _read_context.active = True
        try:
            sample = channel.Read(timeout)
        finally:
            _read_context.active = previous
        return None if isinstance(sample, InvalidSample) else sample

    def _read_loop(self, handler):
        try:
            while not self._closed.is_set():
                sample = self.Read()
                if self._closed.is_set():
                    break
                if sample is None:
                    self._closed.wait(.001)
                else:
                    handler(sample)
        except Exception:
            # No fresh callback on failure; consumers retain their stale guards.
            logging.getLogger(__name__).exception('Feedback reader failed: %s', self._topic)

    def Close(self):
        with self._lifecycle_lock:
            self._closed.set()
            if self._thread is not None:
                self._thread.join()
                self._thread = None
            channel, self._channel = self._channel, None
            if channel is not None:
                channel.CloseReader()
