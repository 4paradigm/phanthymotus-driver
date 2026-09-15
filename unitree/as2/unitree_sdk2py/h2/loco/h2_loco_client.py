import json

from ...rpc.client import Client
from .h2_loco_api import *

"""
" class SportClient — locomotion only (legs)
"""
class LocoClient(Client):
    def __init__(self):
        super().__init__(LOCO_SERVICE_NAME, False)

    def Init(self):
        self._SetApiVerson(LOCO_API_VERSION)
        self._RegistApi(ROBOT_API_ID_LOCO_GET_FSM_ID, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_SET_FSM_ID, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_SET_VELOCITY, 0)

    def SetFsmId(self, fsm_id: int):
        p = {"data": fsm_id}
        code, data = self._Call(ROBOT_API_ID_LOCO_SET_FSM_ID, json.dumps(p))
        return code

    def SetVelocity(self, vx: float, vy: float, omega: float, duration: float = 1.0):
        p = {"velocity": [vx, vy, omega], "duration": duration}
        code, data = self._Call(ROBOT_API_ID_LOCO_SET_VELOCITY, json.dumps(p))
        return code

    def Damp(self):
        return self.SetFsmId(1)

    def Stance(self):
        return self.SetFsmId(4)

    def Start(self):
        return self.SetFsmId(811)

    def Lie2StandUp(self):
        return self.SetFsmId(701)

    def StandUp2Lie(self):
        return self.SetFsmId(702)

    def ZeroTorque(self):
        return self.SetFsmId(0)

    def StopMove(self):
        return self.SetVelocity(0., 0., 0.)

    def Move(self, vx: float, vy: float, vyaw: float, continous_move: bool = False):
        duration = 864000.0 if continous_move else 1
        return self.SetVelocity(vx, vy, vyaw, duration)

    def GetFsmId(self):
        code, data = self._Call(ROBOT_API_ID_LOCO_GET_FSM_ID, json.dumps({}))
        if code != 0:
            return code, None
        js = json.loads(data)
        return code, js.get("data")
