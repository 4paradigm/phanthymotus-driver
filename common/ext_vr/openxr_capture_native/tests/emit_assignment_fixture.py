"""Production ext_vr assignment consumed by native host parser."""
import json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[4]))
from common.ext_vr.capture import CaptureManager
from common.ext_vr.runtime import DeviceRuntime
runtime=DeviceRuntime('fixture')
manager=CaptureManager(runtime,None,None,wall_clock=lambda:1000.)
assignment=manager._new_assignment({'session_id':'7ad3de66-64f2-4d47-89a2-c8da2940eb97'})
print(json.dumps({'type':'assignment','assignment':assignment}))
