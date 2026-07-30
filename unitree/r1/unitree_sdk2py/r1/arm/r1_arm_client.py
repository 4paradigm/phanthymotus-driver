import json

from ...rpc.client import Client
from .r1_arm_api import *


class ArmClient(Client):
    """R1 arm/hand action client (arm DDS service)."""

    def __init__(self):
        super().__init__(ARM_SERVICE_NAME, True)  # lease=True for arm control

    def Init(self):
        self._SetApiVerson(ARM_API_VERSION)
        self._RegistApi(ROBOT_API_ID_ARM_SET_TASK, 0)
        self._RegistApi(ROBOT_API_ID_ARM_LIST_ACTIONS, 0)
        self._RegistApi(ROBOT_API_ID_ARM_ENABLE, 0)
        self._RegistApi(ROBOT_API_ID_ARM_RELEASE, 0)
        self._RegistApi(ROBOT_API_ID_ARM_GET_STATUS, 0)
        self._RegistApi(ROBOT_API_ID_ARM_EXECUTE_NAME, 0)
        self._RegistApi(ROBOT_API_ID_ARM_STOP, 0)

    def Enable(self):
        """Acquire arm SDK control."""
        code, data = self._Call(ROBOT_API_ID_ARM_ENABLE, json.dumps({"data": True}))
        return code, data

    def Release(self):
        """Release arm SDK control."""
        code, data = self._Call(ROBOT_API_ID_ARM_RELEASE, json.dumps({"data": False}))
        return code, data

    def ListActions(self):
        """Get list of available arm actions."""
        code, data = self._Call(ROBOT_API_ID_ARM_LIST_ACTIONS, json.dumps({}))
        if code != 0:
            return code, []
        actions = json.loads(data)
        # Returns [[{id, name}, ...], []] — first element is the action list
        if isinstance(actions, list) and len(actions) > 0:
            return code, actions[0]
        return code, []

    def ExecuteById(self, action_id: int):
        """Execute arm action by numeric ID."""
        code, data = self._Call(ROBOT_API_ID_ARM_SET_TASK, json.dumps({"data": action_id}))
        return code, data

    def ExecuteByName(self, action_name: str):
        """Execute arm action by name."""
        code, data = self._Call(ROBOT_API_ID_ARM_EXECUTE_NAME, json.dumps({"action_name": action_name}))
        return code, data

    def Stop(self):
        """Stop current arm action."""
        code, data = self._Call(ROBOT_API_ID_ARM_STOP, json.dumps({}))
        return code, data

    def GetStatus(self):
        """Get arm SDK status."""
        code, data = self._Call(ROBOT_API_ID_ARM_GET_STATUS, json.dumps({}))
        return code, data
