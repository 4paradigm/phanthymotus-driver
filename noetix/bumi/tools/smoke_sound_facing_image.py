"""Runs only through the build verifier's sandboxed container, never main.py."""
import base64
import importlib.abc
import json
from pathlib import Path
import sys
import tempfile


class NoSDK(importlib.abc.MetaPathFinder):
    def find_spec(self,fullname,path=None,target=None):
        if fullname.split('.')[0] in ('highcontrol_py','mediacontrol_py'):
            raise AssertionError('Smoke test must not import the real robot SDK')


sys.meta_path.insert(0,NoSDK())
sys.path.insert(0,'/work')
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from sound_facing_authority import OperatorAuthority,canonical
from sound_facing import FacingConfig,SoundFacing
from sound_facing_card import SoundFacingPlugin


def main():
    assert FacingConfig.schema_properties()['session_timeout_s']['maximum']==30
    class NoMotion:
        def reserve_relative_motion(self,*args,**kwargs):raise AssertionError('No motion in image import/auth smoke')
    no_pin=OperatorAuthority('image-test',public_key_path='')
    engine=SoundFacing(NoMotion(),authority=no_pin)
    engine.start()
    assert engine.enable()['error']=='operator_authorization_unavailable'
    key=Ed25519PrivateKey.generate()
    with tempfile.TemporaryDirectory() as directory:
        public=Path(directory)/'operator.pub.pem'
        public.write_bytes(key.public_key().public_bytes(serialization.Encoding.PEM,serialization.PublicFormat.SubjectPublicKeyInfo))
        public.chmod(0o400)
        authority=OperatorAuthority('image-test',public)
        payload={k:v for k,v in authority.snapshot()['request'].items() if k!='max_valid_for_s'}
        payload['valid_for_s']=30
        proof=json.dumps(dict(payload=payload,signature=base64.b64encode(key.sign(canonical(payload))).decode()))
        assert authority.authorize(proof)['state']=='approved'
        session=authority.claim();authority.deadline(session)
        authority.invalidate()
        try:authority.deadline(session)
        except ValueError:pass
        else:raise AssertionError('Revocation must deny old session')
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(domain_id=223)
    executor=MultiThreadedExecutor(num_threads=1)
    card=SoundFacingPlugin({},'isolated_image_smoke',executor,NoMotion())
    try:
        assert card.start()
        executor.spin_once(timeout_sec=.05)
        assert not card._engine.snapshot()['enabled']
        card.stop()
        assert card._node is None and not card._worker.is_alive()
    finally:
        card.stop();executor.shutdown();rclpy.shutdown()
    print(json.dumps(dict(plugin_start_stop=True,imports=True,signature=True,missing_pin_denied=True,revocation=True,sdk_imported=False,motion=False)))

if __name__=='__main__':main()
