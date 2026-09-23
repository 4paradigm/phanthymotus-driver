"""Device wire constants retained for native-client compatibility."""
import hashlib
import json
CAPTURE_PROTOCOL='motus.teleop.capture.v1'
RTC_FRAME_PROTOCOL='motus.teleop.rtc-frame.v1'
SIGNALING_AUDIENCE='motus-teleop-rtc'
CAPABILITIES={'profile_id':'ext_vr_dual_controller_v1','input_bindings':{'head':{'required':True,'role':'reference'},'left_controller':{'required':True,'role':'left_end_effector'},'right_controller':{'required':True,'role':'right_end_effector'}},'outputs':{'dual_arm':{'enabled':True,'joint_count':0},'base':{'enabled':False},'hands':{'enabled':False}},'effectors':['dual_arm']}
CAPABILITY_DIGEST=hashlib.sha256(json.dumps(CAPABILITIES,sort_keys=True).encode()).hexdigest()
