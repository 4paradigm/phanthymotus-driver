import logging
import time
import socket
import os
import json

from threading import Thread, Lock

from .client_base import ClientBase
from .internal import *

_log = logging.getLogger(__name__)


"""
" class LeaseContext
"""
class LeaseContext:
    def __init__(self):
        self.id = 0
        self.term = RPC_LEASE_TERM

    def Update(self, id, term):
        self.id = id
        self.term = term

    def Reset(self):
        self.id = 0
        self.term = RPC_LEASE_TERM

    def Valid(self):
        return self.id != 0


"""
" class LeaseClient
"""
class LeaseClient(ClientBase):
    def __init__(self, name: str):
        self.__name = name + "_lease"
        self.__contextName = socket.gethostname() + "/" + name + "/" + str(os.getpid())
        self.__context = LeaseContext()
        self.__thread = None
        self.__lock = Lock()
        super().__init__(self.__name)
        _log.info("[LeaseClient] lease name: %s, context name: %s", self.__name, self.__contextName)
        self.__fail_warns = 0
    
    def Init(self):
        self.SetTimeout(1.0)
        self.__thread = Thread(target=self.__ThreadFunc, name=self.__name, daemon=True)
        self.__thread.start()

    def WaitApplied(self):
        while True:
            with self.__lock:
                if self.__context.Valid():
                    break
            time.sleep(0.1)            
    
    def GetId(self):
            with self.__lock:
                return self.__context.id
    
    def Applied(self):
            with self.__lock:
                return self.__context.Valid()
    
    def __Apply(self):
        parameter = {}
        parameter["name"] = self.__contextName
        p = json.dumps(parameter)

        c, d = self._CallBase(RPC_API_ID_LEASE_APPLY, p)
        if c != 0:
            # The keepalive thread retries forever, so an unavailable service would
            # otherwise emit this every cycle. Warn once, then every 100th.
            self.__fail_warns += 1
            if self.__fail_warns == 1 or self.__fail_warns % 100 == 0:
                _log.warning("[LeaseClient] apply lease error on %s, code=%s (occurrence %d) "
                             "— is the peer service running?",
                             self.__name, c, self.__fail_warns)
            return

        data = json.loads(d)
        
        id = data["id"]
        term = data["term"]

        if self.__fail_warns:
            _log.warning("[LeaseClient] %s acquired after %d failed attempts",
                         self.__name, self.__fail_warns)
            self.__fail_warns = 0
        _log.info("[LeaseClient] lease applied id: %s, term: %s", id, term)

        with self.__lock:
            self.__context.Update(id, float(term/1000000))
    
    def __Renewal(self):
        parameter = {}
        p = json.dumps(parameter)

        c, d = self._CallBase(RPC_API_ID_LEASE_RENEWAL, p, 0, self.__context.id)
        if c != 0:
            self.__fail_warns += 1
            if self.__fail_warns == 1 or self.__fail_warns % 100 == 0:
                _log.warning("[LeaseClient] renewal lease error on %s, code=%s (occurrence %d)",
                             self.__name, c, self.__fail_warns)
            if c == RPC_ERR_SERVER_LEASE_NOT_EXIST:
                with self.__lock:
                    self.__context.Reset()
    
    def __GetWaitSec(self):
        waitsec = 0.0
        if self.__context.Valid():
            waitsec = self.__context.term

        if waitsec <= 0:
            waitsec = RPC_LEASE_TERM

        return waitsec * 0.3

    def __ThreadFunc(self):
        while True:
            if self.__context.Valid():
                self.__Renewal()
            else:
                self.__Apply()
            # sleep waitsec 
            time.sleep(self.__GetWaitSec())
