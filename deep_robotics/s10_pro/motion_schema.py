"""Public base-motion schemas following README_dev's action/ACP contract."""
import copy

ACTIONS = ['info', 'start', 'stop', 'status', 'config', 'stand_up', 'sit_down', 'move', 'stop_move']
MOVE = {
    'direction': {'type': 'string', 'enum': ['forward', 'backward', 'turn_left', 'turn_right'],
                  'title': 'Movement Direction'},
    'speed': {'type': 'number', 'minimum': .06, 'maximum': 1., 'default': .08,
              'title': 'Speed', 'description': 'Translation in m/s; turning in rad/s (minimum 0.08).'},
    'duration': {'type': 'number', 'minimum': .5, 'maximum': 5., 'default': 2., 'title': 'Duration (s)'},
}
METADATA = {
    'request_id': {'type': 'string', 'minLength': 1, 'maxLength': 128},
    'issued_at_ms': {'type': 'integer', 'minimum': 0},
    'ttl_ms': {'type': 'integer', 'minimum': 100, 'maximum': 10000},
    'instance_id': {'type': 'string', 'minLength': 1},
    '_tool_name': {'type': 'string', 'minLength': 1},
}


def tools(config_schema, relative_topic):
    properties = {'action': {'type': 'string', 'enum': ACTIONS},
                  **copy.deepcopy(METADATA), **copy.deepcopy(MOVE),
                  **copy.deepcopy(config_schema['properties']), 'config': copy.deepcopy(config_schema)}
    groups = [(['move'], set(METADATA) | set(MOVE)),
              (['stand_up', 'sit_down', 'stop_move'], set(METADATA) | set(MOVE)),
              (['start', 'config'], {'instance_id', '_tool_name', 'config', *config_schema['properties'], *MOVE}),
              (['info', 'status', 'stop'], {'instance_id', '_tool_name', *MOVE})]
    rules = []
    for actions, allowed in groups:
        forbidden = sorted(set(properties) - allowed - {'action'})
        rule = {'not': {'anyOf': [{'required': [key]} for key in forbidden]}} if forbidden else {}
        if actions == ['move']:
            rule['required'] = ['direction', 'speed', 'duration']
        rules.append({'if': {'properties': {'action': {'enum': actions}}, 'required': ['action']}, 'then': rule})
    loco = {'name': 'loco_move', 'title': 'S10 Locomotion', 'type': 'actuator', 'multiInstance': False,
        'description': 'Bounded posture and locomotion. Agent Core automatically tracks action completion.',
        'inputSchema': {'type': 'object', 'properties': properties, 'required': ['action'],
            'additionalProperties': False, 'allOf': rules,
            'x-action-params': {
                'stand_up': {'params': [], 'description': 'Stand, then confirm move mode 17; no velocity sent.'},
                'sit_down': {'params': [], 'description': 'Lie down and confirm final idle state 0.'},
                'move': {'params': ['direction', 'speed', 'duration'], 'description': 'Move for a bounded duration, then confirm stopping.'},
                'stop_move': {'params': [], 'description': 'Cancel the current action and stop active velocity output.'}},
            'x-resource': 'base', 'x-completion': {
                'actions': ['stand_up', 'sit_down', 'move', 'stand_down', 'sitdown', 'start'], 'timeout': 20}},
        'configSchema': copy.deepcopy(config_schema), 'topic_in': [], 'topic_out': []}
    relative = {'name': 'relative_move', 'title': 'S10 Relative Movement', 'type': 'actuator', 'multiInstance': False,
        'description': 'Reach a relative distance or angle using odometry, then confirm stopping.',
        'inputSchema': {'type': 'object', 'properties': {
            'direction': {'type': 'string', 'enum': ['forward', 'backward', 'left', 'right', 'turn_left', 'turn_right']},
            'distance': {'type': 'number', 'minimum': .3, 'maximum': .6, 'default': .5},
            'angle': {'type': 'integer', 'enum': [30, 60], 'default': 60},
            'speed': {'type': 'number', 'minimum': .06, 'maximum': .20, 'default': .12}},
            'required': ['direction', 'speed'], 'additionalProperties': False,
            'allOf': [{'if': {'properties': {'direction': {'enum': ['turn_left', 'turn_right']}}, 'required': ['direction']},
                'then': {'required': ['angle'], 'not': {'required': ['distance']}},
                'else': {'required': ['distance'], 'not': {'required': ['angle']}}}],
            'x-resource': 'base', 'x-completion': {'actions': ['move', 'relative_move'], 'timeout': 20}},
        'configSchema': copy.deepcopy(config_schema), 'topic_in': [],
        'topic_out': [{'topic': relative_topic, 'format': 'data/json'}]}
    return [loco, relative]
