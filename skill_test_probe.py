"""Read-only preflight for a supervised Skill test. Does not start a project."""
import json
import pathlib
import ssl
import urllib.request


def request(path, body=None):
    token = next(line.split('=', 1)[1].strip() for line in
                 pathlib.Path('/opt/phanthy-motus/.env').read_text().splitlines()
                 if line.startswith('ACCESS_TOKEN='))
    req = urllib.request.Request('https://127.0.0.1:15678' + path,
        data=None if body is None else json.dumps(body).encode(),
        headers={'Authorization': 'Bearer ' + token, 'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, context=ssl._create_unverified_context(), timeout=15) as res:
        return json.load(res)


if __name__ == '__main__':
    for path, body in [
        ('/api/config/project-running', None),
        ('/api/mcp/driver-unitree-g1/call', {'tool': 'switch_mode', 'arguments': {'action': 'get_current_mode'}}),
        ('/api/mcp/driver-unitree-g1/call', {'tool': 'teleop_control', 'arguments': {'action': 'info'}}),
    ]:
        print(path, json.dumps(request(path, body), ensure_ascii=False), flush=True)
