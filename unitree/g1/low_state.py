"""One bundle-owned LowState reader; consumers never mutate decoded messages."""
import logging
import threading
import time
from typing import NamedTuple


class LowStateSample(NamedTuple):
    message: object
    source_tick: int
    sampled_ns: int
    received_ns: int


class LowStateSource:
    def __init__(self, *, clock=time.monotonic_ns):
        self.clock = clock
        self._lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        self._subscriber = None
        self._active = False
        self._latest = None
        self._source_tick = self._source_time_ns = None

    def start(self):
        with self._lifecycle_lock:
            if self._subscriber is not None:
                return
            from cyclonedds.qos import Qos, Policy
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
            sub = ChannelSubscriber('rt/lowstate', LowState_)
            with self._lock:
                self._active = True
                self._latest = None
                self._source_tick = self._source_time_ns = None
            try:
                sub.Init(self._on_state, 0, qos=Qos(Policy.History.KeepLast(1)))
            except Exception:
                with self._lock:
                    self._active = False
                    self._latest = None
                try: sub.Close()
                except Exception: logging.getLogger(__name__).exception('Failed to close partial LowState reader')
                raise
            self._subscriber = sub

    def _on_state(self, msg):
        received_ns = self.clock()
        with self._lock:
            if not self._active:
                return
            try:
                tick = int(msg.tick)
                if not 0 <= tick <= 0xffffffff:
                    raise ValueError('invalid_lowstate_tick')
            except (AttributeError, TypeError, ValueError):
                self._latest = None
                return
            delta = None if self._source_tick is None else (tick-self._source_tick) & 0xffffffff
            if delta is not None and not 0 < delta < 0x80000000:
                return
            # Reads and late deliveries must not renew the source sample's age.
            sampled_ns = received_ns if delta is None else min(received_ns, self._source_time_ns+delta*1_000_000)
            self._source_tick, self._source_time_ns = tick, sampled_ns
            self._latest = LowStateSample(msg, tick, sampled_ns, received_ns)

    def snapshot(self):
        with self._lock:
            return self._latest

    def stop(self):
        with self._lifecycle_lock:
            with self._lock:
                self._active = False
                self._latest = None
                self._source_tick = self._source_time_ns = None
            sub, self._subscriber = self._subscriber, None
            if sub is not None:
                sub.Close()
