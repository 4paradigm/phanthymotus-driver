import json
import logging
import os

from ...rpc.client import Client
from .g1_audio_api import *

_log = logging.getLogger(__name__)

# PlayStream is called once per PCM chunk of every TTS utterance, so anything
# logged here is a hot path. Off by default; UNITREE_RPC_DEBUG=1 re-enables.
_AUDIO_DEBUG = bool(os.environ.get('UNITREE_RPC_DEBUG'))

"""
" class SportClient
"""
class AudioClient(Client):
    def __init__(self):
        super().__init__(AUDIO_SERVICE_NAME, False)
        self.tts_index = 0

    def Init(self):
        # set api version
        self._SetApiVerson(AUDIO_API_VERSION)

        # regist api
        self._RegistApi(ROBOT_API_ID_AUDIO_TTS, 0)
        self._RegistApi(ROBOT_API_ID_AUDIO_ASR, 0)
        self._RegistApi(ROBOT_API_ID_AUDIO_START_PLAY, 0)
        self._RegistApi(ROBOT_API_ID_AUDIO_STOP_PLAY, 0)
        self._RegistApi(ROBOT_API_ID_AUDIO_GET_VOLUME, 0)
        self._RegistApi(ROBOT_API_ID_AUDIO_SET_VOLUME, 0) 
        self._RegistApi(ROBOT_API_ID_AUDIO_SET_RGB_LED, 0) 

    ## API Call ##
    def TtsMaker(self, text: str, speaker_id: int):
        self.tts_index += self.tts_index
        p = {}
        p["index"] = self.tts_index
        p["text"] = text
        p["speaker_id"] = speaker_id
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_AUDIO_TTS, parameter)
        return code

    def GetVolume(self):
        p = {}
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_AUDIO_GET_VOLUME, parameter)
        if code == 0:
            return code, json.loads(data)
        else:
            return code, None

    def SetVolume(self, volume: int):
        p = {}
        p["volume"] = volume
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_AUDIO_SET_VOLUME, parameter)
        return code

    def LedControl(self, R: int, G: int, B: int):
        p = {}
        p["R"] = R
        p["G"] = G
        p["B"] = B
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_AUDIO_SET_RGB_LED, parameter)
        return code
    
    def PlayStream(self, app_name: str, stream_id: str, pcm_data: bytes):
        param = json.dumps({"app_name": app_name, "stream_id": stream_id})
        pcm_list = list(pcm_data)
        if _AUDIO_DEBUG:
            _log.debug("[AudioClient] PlayStream app=%s id=%s pcm_bytes=%d",
                       app_name, stream_id, len(pcm_data))
        result = self._CallRequestWithParamAndBin(ROBOT_API_ID_AUDIO_START_PLAY, param, pcm_list)
        # Deliberately log only the status code, never `result` itself: it is
        # (code, response.data) and response.data is a DDS string field that can
        # carry non-UTF-8 bytes straight into the log framer.
        if _AUDIO_DEBUG:
            code = result[0] if isinstance(result, tuple) else result
            _log.debug("[AudioClient] PlayStream code=%s", code)
        return result
    
    def PlayStop(self, app_name: str):
        parameter = json.dumps({"app_name": app_name})
        self._Call(ROBOT_API_ID_AUDIO_STOP_PLAY, parameter)
        return 0
