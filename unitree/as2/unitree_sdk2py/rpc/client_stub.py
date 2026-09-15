import logging
import time

from enum import Enum
from threading import Thread, Condition

from ..idl.unitree_api.msg.dds_ import Request_ as Request
from ..idl.unitree_api.msg.dds_ import Response_ as Response

from ..core.channel import ChannelFactory
from ..core.channel_name import ChannelType, GetClientChannelName
from .request_future import RequestFuture, RequestFutureQueue

_log = logging.getLogger(__name__)


"""
" class ClientStub
"""
class ClientStub:
    def __init__(self, serviceName: str):
        self.__serviceName = serviceName
        self.__futureQueue = None

        self.__sendChannel = None
        self.__recvChannel = None

    def Init(self):
        factory = ChannelFactory()
        self.__futureQueue = RequestFutureQueue()
        self.__fail_warns = 0

        # create channel
        self.__sendChannel = factory.CreateSendChannel(GetClientChannelName(self.__serviceName, ChannelType.SEND), Request)
        self.__recvChannel = factory.CreateRecvChannel(GetClientChannelName(self.__serviceName, ChannelType.RECV), Response,
                                    self.__ResponseHandler,10)
        time.sleep(0.5)


    def Send(self, request: Request, timeout: float):
        if self.__sendChannel.Write(request, timeout):
            return True
        else:
            self.__fail_warns += 1
            if self.__fail_warns == 1 or self.__fail_warns % 100 == 0:
                _log.warning("[ClientStub] send error. id: %s (occurrence %d)",
                             request.header.identity.id, self.__fail_warns)
            return False

    def SendRequest(self, request: Request, timeout: float):
        id = request.header.identity.id

        future = RequestFuture()
        future.SetRequestId(id)
        self.__futureQueue.Set(id, future)

        if self.__sendChannel.Write(request, timeout):
            self.__fail_warns = 0
            return future
        else:
            self.__fail_warns += 1
            if self.__fail_warns == 1 or self.__fail_warns % 100 == 0:
                _log.warning("[ClientStub] send request error. id: %s (occurrence %d)",
                             request.header.identity.id, self.__fail_warns)
            self.__futureQueue.Remove(id)
            return None

    def RemoveFuture(self, requestId: int):
        self.__futureQueue.Remove(requestId)

    def __ResponseHandler(self, response: Response):
        id = response.header.identity.id
        apiId = response.header.identity.api_id
        future = self.__futureQueue.Get(id)
        if future is None:
            pass  # expected for fire-and-forget sport commands
        elif not future.Ready(response):
            _log.warning("[ClientStub] set future ready error.")
