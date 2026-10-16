"""Operator-authorized one-award test; prepare and cleanup are separate phases."""
import json
import pathlib
import sys
import time
from skill_test_probe import request

BACKUP = pathlib.Path('/opt/phanthy-motus/data/g1-award-ceremony/skill-test-v31-layout.json')
SESSION = 'skill-single-test-supervised'

def save(layout):
    request('/api/canvas/claim-edit', {'session_id': SESSION})
    try:
        print('layout', request('/api/canvas/layout', dict(layout, session_id=SESSION)), flush=True)
    finally:
        request('/api/canvas/release-edit', {'session_id': SESSION})

phase = sys.argv[1]
if phase == 'prepare':
    if request('/api/config/project-running').get('running'):
        raise SystemExit('Project already running; will not interfere')
    snapshot = request('/api/canvas/layout')['data']
    with BACKUP.open('x') as f:
        json.dump(snapshot, f, ensure_ascii=False)
    allowed = {('agentcore', 'remote_message'), ('agentcore', 'decision_core')}
    allowed.update(('driver-unitree-g1', t) for t in ('tts', 'arm', 'loco', 'led'))
    cards = [c for c in snapshot['cards'] if (c['mcpId'], c['toolName']) in allowed]
    assert len(cards) == 6, 'Unexpected minimal layout'
    ids = {c['id'] for c in cards}
    layout = dict(snapshot, cards=cards)
    for key in ('connections', 'execConnections'):
        layout[key] = [e for e in snapshot.get(key, []) if e['fromCardId'] in ids and e['toCardId'] in ids]
    save(layout)
    print('start', request('/api/config/start-project', {}), flush=True)
    print('running', request('/api/config/project-running'), flush=True)
elif phase == 'send':
    marker = BACKUP.with_suffix('.sent')
    with marker.open('x') as f:
        f.write(str(time.time()))
    print('send', request('/api/mcp/agentcore/call', {'tool': 'remote_message', 'arguments': {
        'action': 'send_message',
        'text': '单项现场实测，请使用 g1-award-ceremony Skill，只执行一次本轮的一项名单，不补充其它奖项，不调用 award_ceremony 复合卡片。机器人已稳定站立、周围无遮挡，可以动作。一键颁奖：最佳创意奖=张三'
    }}), flush=True)
elif phase == 'cleanup':
    print('stop', request('/api/config/stop-project', {}), flush=True)
    print('running', request('/api/config/project-running'), flush=True)
    save(json.loads(BACKUP.read_text()))
else:
    raise SystemExit('Unknown phase')
