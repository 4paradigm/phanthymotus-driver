"""State serialization without gRPC, protobuf wheels or hardware access."""
import ast
import json
from pathlib import Path
import types
import unittest


class RepeatedScalarContainer:
    def __init__(self, values):
        self.values = values

    def __iter__(self):
        return iter(self.values)


class GrpcStateJsonTests(unittest.TestCase):
    def test_robot_state_repeated_fields_are_json_lists(self):
        source = ast.parse(Path(__file__).with_name('grpc_client.py').read_text())
        cls = next(n for n in source.body if isinstance(n, ast.ClassDef)
                   and n.name == 'AdamGrpcClient')
        module = ast.Module(body=[cls], type_ignores=[])
        scope = {'pb2': types.SimpleNamespace(GetRobotStateRequest=lambda: None)}
        exec(compile(module, 'grpc_client.py', 'exec'), scope)
        client = scope['AdamGrpcClient'].__new__(scope['AdamGrpcClient'])
        response = types.SimpleNamespace(
            success=True, fsm_state='STAND_WALK', vx=0, vy=0, vyaw=0, height=0,
            current_motion_file='', motion_playing=False,
            current_tracking_motion='', tracking_playing=False,
            switchable_states=RepeatedScalarContainer(['STAND_WALK', 'MULTI_AGENT']),
            available_actions=RepeatedScalarContainer(['SetMotion']))
        client._call = lambda method, request: response
        result = json.loads(json.dumps(client.get_robot_state()))
        self.assertEqual(['STAND_WALK', 'MULTI_AGENT'], result['switchable_states'])
        self.assertEqual(['SetMotion'], result['available_actions'])
        self.assertTrue(result['success'])


if __name__ == '__main__':
    unittest.main()
