"""Model-only replay: tool results are simulated; no robot APIs invoked."""
import json
import pathlib
import sqlite3
import urllib.request

c = sqlite3.connect('file:/opt/phanthy-motus/data/data.db?mode=ro', uri=True)
cfg = json.loads(c.execute("SELECT value FROM config WHERE key='client'").fetchone()[0])['llm'][0]
log = pathlib.Path('/opt/phanthy-motus/data/llm_recent_request/main_agent/261016_101058_6a5c4471.txt')
d = json.loads(log.read_text().split('=' * 64)[0])
messages = [d['messages'][0], {'role': 'user', 'content': '机器人已稳定站立、周围无遮挡，可以动作。一键颁奖：最佳创意奖=张三'}]
url = cfg['url'].rstrip('/')
if not url.endswith('/chat/completions'):
    url += '/chat/completions'
calls = []
for round_index in range(24):
    body = {'model': cfg['model'], 'messages': messages, 'tools': d['tools'], 'stream': False}
    req = urllib.request.Request(url, data=json.dumps(body).encode(), headers={
        'Authorization': 'Bearer ' + cfg['key'], 'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=40) as r:
        msg = json.load(r)['choices'][0]['message']
    messages.append(msg)
    tcs = msg.get('tool_calls', [])
    if not tcs:
        print('NO_TOOL_RESPONSE', flush=True)
        break
    for tc in tcs:
        name, args = tc['function']['name'], tc['function']['arguments']
        calls.append({'name': name, 'arguments': args})
        print(round_index, name, args, flush=True)
        messages.append({'role': 'tool', 'tool_call_id': tc['id'], 'content': json.dumps({
            'ret': 0, 'status': 'completed', 'simulation': True})})
    if any(t['function']['name'] == 'finish' for t in tcs):
        break
pathlib.Path('/opt/phanthy-motus/data/g1-award-ceremony/model-replay.json').write_text(json.dumps(calls, ensure_ascii=False, indent=2))
