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
        self._thread = None
        self._closed = threading.Event()
        self._active = False
        self._latest = None
        self._source_tick = self._source_time_ns = None
        # Discontinuities (gap, invalid sample, reader failure, stop) that
        # invalidate a motion preview. Counted when the feedback is dropped,
        # not when it resumes; never reset, so consumers compare snapshots.
        self.gaps = 0
        self._last_sampled_ns = None

    def start(self):
        with self._lifecycle_lock:
            if self._subscriber is not None:
                return
            from cyclonedds.internal import InvalidSample
            from sdk_adapter import FeedbackSubscriber
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
            sub = FeedbackSubscriber('rt/lowstate', LowState_)
            with self._lock:
                self._active = True
                self._latest = None
                self._source_tick = self._source_time_ns = None
            try:
                sub.Init()
                self._closed.clear()
                thread = threading.Thread(target=self._read, args=(sub, InvalidSample),
                                          name='g1-lowstate', daemon=True)
                thread.start()
            except Exception:
                with self._lock:
                    self._active = False
                    self._latest = None
                try: sub.Close()
                except Exception: logging.getLogger(__name__).exception('Failed to close partial LowState reader')
                raise
            self._subscriber = sub
            self._thread = thread

    def _read(self, sub, invalid_sample):
        # Decode LowState outside the shared DDS callback thread used by RPC replies.
        try:
            while not self._closed.is_set():
                msg = sub.Read(timeout=.1)
                if msg is None:
                    self._closed.wait(.001)
                elif not isinstance(msg, invalid_sample):
                    self._on_state(msg)
        except Exception:
            logging.getLogger(__name__).exception('LowState reader failed')
            with self._lock:
                self._active = False
                self._invalidate()

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
                self._invalidate()
                return
            delta = None if self._source_tick is None else (tick-self._source_tick) & 0xffffffff
            if delta is not None and not 0 < delta < 0x80000000:
                return
            # Reads and late deliveries must not renew the source sample's age.
            sampled_ns = received_ns if delta is None else min(received_ns, self._source_time_ns+delta*1_000_000)
            if (self._latest is not None and self._last_sampled_ns is not None
                    and received_ns-self._last_sampled_ns > 100_000_000):
                self.gaps += 1
            self._last_sampled_ns = sampled_ns
            self._source_tick, self._source_time_ns = tick, sampled_ns
            self._latest = LowStateSample(msg, tick, sampled_ns, received_ns)

    def _invalidate(self):
        # Lock held. Fence previews the moment feedback stops being usable.
        if self._latest is not None:
            self.gaps += 1
        self._latest = None

    def snapshot(self):
        with self._lock:
            return self._latest

    def stop(self):
        with self._lifecycle_lock:
            with self._lock:
                self._active = False
                self._invalidate()
                self._source_tick = self._source_time_ns = None
            sub, self._subscriber = self._subscriber, None
            self._closed.set()
            if self._thread is not None:
                self._thread.join()
                self._thread = None
            if sub is not None:
                sub.Close()
