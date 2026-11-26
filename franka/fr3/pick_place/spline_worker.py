"""Run the driver spline implementation in the existing PNP planning worker."""

from inspect import getsource
from pathlib import Path
import time

import numpy as np

from .hand_collision import TableClearanceError


_LOAD = '''
import sys
from types import ModuleType

package = ModuleType('_fr3_spline')
package.__path__ = []
sys.modules[package.__name__] = package
for name, source in request['modules'].items():
    module = ModuleType(package.__name__ + '.' + name)
    module.__package__ = package.__name__
    sys.modules[module.__name__] = module
    exec(compile(source, name + '.py', 'exec'), module.__dict__)
worker_state['resources'] = request['resources']
print('FR3_RESULT {}')
'''

_INITIALIZE = '''
import numpy as np
from _fr3_spline.remote_execution import RemoteExecution
from _fr3_spline.kinematics import Kinematics

execution = RemoteExecution.__new__(RemoteExecution)
execution.model = Kinematics(request['q'], np.asarray(request['actual']).reshape(4, 4, order='F'),
                             **worker_state['resources'])
execution.profile, execution.scene = request['profile'], request['scene']
execution.hand_points = np.asarray(request['hand_points'])
surface = request['visible_surface']
execution.visible_surface = np.asarray(surface) if surface is not None else None
execution.options = {'speed_deg_s': request['speed']}
execution.active = {'action_id': request['action_id']}
worker_state['execution'] = execution
print('FR3_RESULT {}')
'''

_BUILD = '''
import time
from _fr3_spline.hand_collision import TableClearanceError

execution = worker_state['execution']
execution.q = request['q']
deadline = time.monotonic() + request['budget']

def check():
    if time.monotonic() >= deadline:
        raise TimeoutError('MoveIt candidate planning deadline')

execution.check = check
try:
    _, plan = execution.build_trajectory(*request['arguments'])
    if plan and 'descent_planes' in plan:
        plan['descent_planes'] = [(point.tolist(), normal.tolist())
                                  for point, normal in plan['descent_planes']]
    result = dict(plan=plan, end=execution.q)
except (RuntimeError, ValueError, TimeoutError) as error:
    result = dict(error=str(error), error_type=type(error).__name__)
print('FR3_RESULT ' + json.dumps(result, allow_nan=False))
'''


def load(session):
    from hardware import JOINT_LOWER, JOINT_UPPER
    from .planning import rigid, rotation

    folder = Path(__file__).parent
    modules = {'planning': 'import math\nimport numpy as np\n' + getsource(rigid) + getsource(rotation)}
    for name in ('arm_collision', 'execution', 'hand_collision', 'kinematics', 'remote_execution'):
        modules[name] = (folder / (name + '.py')).read_text()
    resources = {'urdf': (folder.parent / 'resource/fr3.urdf').read_text(),
                 'limits': (JOINT_LOWER, JOINT_UPPER)}
    session.call(_LOAD, dict(modules=modules, resources=resources), 20)


def initialize(execution, state):
    scene = execution.scene
    scene = None if scene is None else {key: scene[key] for key in
                                       ('table_point_base_mm', 'table_normal_base') if key in scene}
    profile = {key: execution.profile[key] for key in
               ('table_plane', 'tool_frame', 'descent_clearance_allowance_mm',
                'descent_min_clearance_mm') if key in execution.profile}
    surface = execution.visible_surface
    execution.arm.call(_INITIALIZE, dict(
        q=execution.q, actual=state['O_T_EE'], profile=profile, scene=scene,
        hand_points=execution.hand_points.tolist(), visible_surface=None if surface is None else surface.tolist(),
        speed=execution.options['speed_deg_s'], action_id=execution.active.get('action_id')), 15)


def checked_spline(execution, stage, target, plan, cartesian, joint):
    deadline = min(execution.deadline, execution.planning_deadline or execution.deadline)
    budget = deadline - time.monotonic()
    result = execution.arm.call(_BUILD, dict(
        q=execution.q, budget=budget,
        arguments=[stage, np.asarray(target).tolist(), plan, cartesian, joint]), min(budget, 40))
    if 'error' in result:
        types = {'TableClearanceError': TableClearanceError, 'ValueError': ValueError,
                 'TimeoutError': TimeoutError}
        raise types.get(result['error_type'], RuntimeError)(result['error'])
    return result['plan'], result['end']
