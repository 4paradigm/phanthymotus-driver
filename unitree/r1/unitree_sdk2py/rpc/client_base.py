import logging
import os
import time

from ..idl.unitree_api.msg.dds_ import Request_ as Request
from ..idl.unitree_api.msg.dds_ import RequestHeader_ as RequestHeader
from ..idl.unitree_api.msg.dds_ import RequestLease_ as RequestLease
from ..idl.unitree_api.msg.dds_ import RequestIdentity_ as RequestIdentity
from ..idl.unitree_api.msg.dds_ import RequestPolicy_ as RequestPolicy

from ..utils.future import FutureResult

from .client_stub import ClientStub
from .internal import *

_log = logging.getLogger(__name__)

# Locally-added RPC instrumentation. Every locomotion/arm/audio call goes through
# _CallBase, so printing on the success path costs 3 lines per RPC — enough to
# dominate the container log on its own. Off by default; set UNITREE_RPC_DEBUG=1
# to get it back when chasing RPC timeouts (e.g. error 3104).
RPC_DEBUG = bool(os.environ.get('UNITREE_RPC_DEBUG'))


"""
" class ClientBase
"""
class ClientBase:
    def __init__(self, serviceName: str):
        self.__timeout = 1.0
        # Repeated-failure counter: a peer service that is simply not running makes
        # every call fail, and logging each one buries the log. Warn on the first
        # and every 100th, and say so when it recovers.
        self.__fail_warns = 0
        self.__stub = ClientStub(serviceName)
        self.__stub.Init()

    def SetTimeout(self, timeout: float):
        self.__timeout = timeout

    def _CallBase(self, apiId: int, parameter: str, proirity: int = 0, leaseId: int = 0):
        header = self.__SetHeader(apiId, leaseId, proirity, False)
        request = Request(header, parameter, [])
        req_id = request.header.identity.id

        if RPC_DEBUG:
            _log.debug("[CallBase] sending apiId=%s, id=%s, timeout=%s", apiId, req_id, self.__timeout)

        t0 = time.monotonic()
        future = self.__stub.SendRequest(request, self.__timeout)
        if future is None:
            self.__fail_warns += 1
            if self.__fail_warns == 1 or self.__fail_warns % 100 == 0:
                _log.warning("[CallBase] SendRequest failed (send error), elapsed=%.3fs "
                             "(occurrence %d)", time.monotonic() - t0, self.__fail_warns)
            return RPC_ERR_CLIENT_SEND, None

        if RPC_DEBUG:
            _log.debug("[CallBase] sent ok, waiting for response...")
        result = future.GetResult(self.__timeout)
        elapsed = time.monotonic() - t0

        if result.code != FutureResult.FUTURE_SUCC:
            self.__stub.RemoveFuture(request.header.identity.id)
            code = RPC_ERR_CLIENT_API_TIMEOUT if result.code == FutureResult.FUTUTE_ERR_TIMEOUT else RPC_ERR_UNKNOWN
            self.__fail_warns += 1
            if self.__fail_warns == 1 or self.__fail_warns % 100 == 0:
                _log.warning("[CallBase] failed: result.code=%s, rpc_code=%s, elapsed=%.3fs "
                             "(occurrence %d)", result.code, code, elapsed, self.__fail_warns)
            return code, None

        response = result.value
        if self.__fail_warns:
            _log.warning("[CallBase] RPC recovered after %d consecutive failures",
                         self.__fail_warns)
            self.__fail_warns = 0
        if RPC_DEBUG:
            _log.debug("[CallBase] success: apiId=%s, status=%s, elapsed=%.3fs",
                       response.header.identity.api_id, response.header.status.code, elapsed)

        if response.header.identity.api_id != apiId:
            return RPC_ERR_CLIENT_API_NOT_MATCH, None
        else:
            return response.header.status.code, response.data

    def _CallNoReplyBase(self, apiId: int, parameter: str, proirity: int, leaseId: int):
        header = self.__SetHeader(apiId, leaseId, proirity, True)
        request = Request(header, parameter, [])

        if self.__stub.Send(request, self.__timeout):
            return 0
        else:
            return RPC_ERR_CLIENT_SEND

    def _CallRequestWithParamAndBinBase(self, apiId: int, requestParamter: str,
                                        requestBinary: list, proirity: int = 0,
                                        leaseId: int = 0):
        header = self.__SetHeader(apiId, leaseId, proirity, False)
        request = Request(header, requestParamter, requestBinary)

        future = self.__stub.SendRequest(request, self.__timeout)
        if future is None:
            return RPC_ERR_CLIENT_SEND, None

        result = future.GetResult(self.__timeout)

        if result.code != FutureResult.FUTURE_SUCC:
            self.__stub.RemoveFuture(request.header.identity.id)
            code = RPC_ERR_CLIENT_API_TIMEOUT if result.code == FutureResult.FUTUTE_ERR_TIMEOUT else RPC_ERR_UNKNOWN
            return code, None

        response = result.value

        if response.header.identity.api_id != apiId:
            return RPC_ERR_CLIENT_API_NOT_MATCH, None
        else:
            return response.header.status.code, response.data

    def _CallRequestWithParamAndBinNoReplyBase(self, apiId: int, requestParamter: str,
                                               requestBinary: list, proirity: int,
                                               leaseId: int):
        header = self.__SetHeader(apiId, leaseId, proirity, True)
        request = Request(header, requestParamter, request_binary)

        if self.__stub.Send(request, self.__timeout):
            return 0
        else:
            return RPC_ERR_CLIENT_SEND

    def _CallBinaryBase(self, apiId: int, parameter: list, proirity: int, leaseId: int):
        header = self.__SetHeader(apiId, leaseId, proirity, False)
        request = Request(header, "", parameter)
        
        future = self.__stub.SendRequest(request, self.__timeout)
        if future is None:
            return RPC_ERR_CLIENT_SEND, None

        result = future.GetResult(self.__timeout)
        if result.code != FutureResult.FUTURE_SUCC:
            self.__stub.RemoveFuture(request.header.identity.id)
            code = RPC_ERR_CLIENT_API_TIMEOUT if result.code == FutureResult.FUTUTE_ERR_TIMEOUT else RPC_ERR_UNKNOWN
            return code, None

        response = result.value

        if response.header.identity.api_id != apiId:
            return RPC_ERR_CLIENT_API_NOT_MATCH, None
        else:
            return response.header.status.code, response.binary

    def _CallBinaryNoReplyBase(self, apiId: int, parameter: list, proirity: int, leaseId: int):
        header = self.__SetHeader(apiId, leaseId, proirity, True)
        request = Request(header, "", parameter)

        if self.__stub.Send(request, self.__timeout):
            return 0
        else:
            return RPC_ERR_CLIENT_SEND
    
    def __SetHeader(self, apiId: int, leaseId: int, priority: int, noReply: bool):
        identity = RequestIdentity(time.monotonic_ns(), apiId)
        lease = RequestLease(leaseId)
        policy = RequestPolicy(priority, noReply)
        return RequestHeader(identity, lease, policy)
