"""Run the real HTTP dispatcher with a controlled peer address and body."""
import io
import json
from types import SimpleNamespace

import pytest

from test_motion_control_bundle import bundle_class  # noqa: F401
from test_servo import ros_stubs  # noqa: F401


@pytest.mark.parametrize('tool,peer,origin,rejected',[
    ('teleop_control','127.0.0.1',None,False),
    ('teleop_control','::1',None,False),
    ('teleop_control','127.0.0.1','http://untrusted.example',True),
    ('teleop_control','192.0.2.10',None,True),
    ('motion_control','192.0.2.10',None,True),
    ('teleop_executor','127.0.0.1','http://untrusted.example',True),
    ('arm','192.0.2.10','http://untrusted.example',False),
])
def test_management_boundary_leaves_ordinary_cards_unchanged(bundle_class,monkeypatch,tool,peer,origin,rejected):
    scope=bundle_class.__init__.__globals__;calls=[];responses=[]
    def dispatch(name,args):
        calls.append((name,args));return {'state':'idle','error':None}
    monkeypatch.setitem(scope,'_bundle',SimpleNamespace(dispatch=dispatch))
    handler_type=scope['make_handler']();handler=handler_type.__new__(handler_type)
    body=json.dumps({'jsonrpc':'2.0','id':1,'method':'tools/call',
        'params':{'name':tool,'arguments':{'action':'stop'}}}).encode()
    handler.headers={'Content-Length':str(len(body))}
    if origin:handler.headers['Origin']=origin
    handler.rfile=io.BytesIO(body);handler.client_address=(peer,1234)
    handler._send=lambda status,payload:responses.append((status,json.loads(payload)))
    handler.do_POST()
    response=responses[0][1]
    if rejected:
        assert response['error']['code']==-32600 and not calls
    else:
        assert calls==[(tool,{'action':'stop'})]
        assert not response['result'].get('isError')
