"""Dedicated velocity RPC transport for loco_point; does not patch the SDK."""
import json
import threading
import time


class PointRpc:
    def __init__(self, factory=None):
        from unitree_sdk2py.core.channel import ChannelFactory
        from unitree_sdk2py.idl.unitree_api.msg.dds_ import Request_, Response_
        from unitree_sdk2py.g1.loco.g1_loco_api import LOCO_SERVICE_NAME
        self.lock = threading.Lock()
        self.pending = {}
        self.closed = False
        factory = factory or ChannelFactory()
        self.writer = factory.CreateSendChannel(f"rt/api/{LOCO_SERVICE_NAME}/request", Request_)
        self.reader = factory.CreateRecvChannel(f"rt/api/{LOCO_SERVICE_NAME}/response", Response_,
                                               self._receive, 10)

    def _receive(self, response):
        with self.lock:
            future = self.pending.pop(response.header.identity.id, None)
        if future:
            future.Ready(response)

    def _velocity(self, vx, vy, vyaw, duration):
        from unitree_sdk2py.idl.unitree_api.msg.dds_ import (
            Request_, RequestHeader_, RequestIdentity_, RequestLease_, RequestPolicy_)
        from unitree_sdk2py.g1.loco.g1_loco_api import ROBOT_API_ID_LOCO_SET_VELOCITY
        from unitree_sdk2py.rpc.request_future import RequestFuture
        from unitree_sdk2py.utils.future import FutureResult
        from unitree_sdk2py.rpc.internal import (
            RPC_ERR_CLIENT_SEND, RPC_ERR_CLIENT_API_TIMEOUT,
            RPC_ERR_CLIENT_API_NOT_MATCH, RPC_ERR_UNKNOWN)
        api = ROBOT_API_ID_LOCO_SET_VELOCITY
        request_id = time.monotonic_ns()
        header = RequestHeader_(RequestIdentity_(request_id, api), RequestLease_(0), RequestPolicy_(0, False))
        request = Request_(header, json.dumps({"velocity": [vx, vy, vyaw], "duration": duration}), [])
        future = RequestFuture()
        future.SetRequestId(request_id)
        with self.lock:
            if getattr(self, "closed", False):
                return lambda: RPC_ERR_CLIENT_SEND
            self.pending[request_id] = future
        try:
            sent = self.writer.Write(request, 10.)
        except Exception:
            with self.lock:
                self.pending.pop(request_id, None)
            raise
        if not sent:
            with self.lock:
                self.pending.pop(request_id, None)
            return lambda: RPC_ERR_CLIENT_SEND

        def wait():
            try:
                result = future.GetResult(10.)
                if result.code != FutureResult.FUTURE_SUCC:
                    return (RPC_ERR_CLIENT_API_TIMEOUT if result.code == FutureResult.FUTUTE_ERR_TIMEOUT
                            else RPC_ERR_UNKNOWN)
                response = result.value
                if response.header.identity.api_id != api:
                    return RPC_ERR_CLIENT_API_NOT_MATCH
                return response.header.status.code
            finally:
                with self.lock:
                    self.pending.pop(request_id, None)
        return wait

    def BeginMove(self, vx, vy, vyaw, continuous=False):
        return self._velocity(vx, vy, vyaw, 864000. if continuous else 1.)

    def BeginStopMove(self):
        return self._velocity(0., 0., 0., 1.)

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            pending = list(self.pending.values())
            self.pending.clear()
        for future in pending:
            future.Fail("loco_point transport shut down")
        self.reader.Close()
        self.writer.Close()
