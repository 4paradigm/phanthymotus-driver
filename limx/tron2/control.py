#!/usr/bin/env python3
"""Local operator client. Status/commission do not send motion; execute DOES."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import urllib.error
import urllib.request


def post(path, body):
    class NoRedirect(urllib.request.HTTPRedirectHandler):
        def redirect_request(self, *args, **kwargs):
            return None
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect())
    request = urllib.request.Request('http://127.0.0.1:15791/' + path,
        data=json.dumps(body).encode(), headers={'Content-Type': 'application/json'})
    try:
        with opener.open(request, timeout=180) as response:
            return json.load(response)
    except urllib.error.HTTPError as exc:
        # Surface preflight rejection instead of hiding it behind generic HTTP 409.
        raw = exc.read(16 * 1024)
        try:
            body = json.loads(raw)
            reason = body.get('error') if isinstance(body, dict) else None
        except (ValueError, UnicodeError):
            reason = None
        raise ValueError('HTTP ' + str(exc.code) + ': ' +
                         (str(reason)[:300] if reason else 'Service rejected the request')) from exc


def call(action):
    reply = post('mcp', {'jsonrpc': '2.0', 'id': 1, 'method': 'tools/call',
        'params': {'name': 'reset_pose', 'arguments': {'action': action}}})
    if 'error' in reply:
        raise ValueError(str(reply['error']))
    result = json.loads(reply['result']['content'][0]['text'])
    if reply['result'].get('isError'):
        raise ValueError(json.dumps(result, ensure_ascii=False))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    for name in ('status', 'preview', 'verify_at_home', 'execute'):
        sub.add_parser(name)
    clear = sub.add_parser('clear', help='Only after manually inspecting an uncertain result')
    clear.add_argument('--confirm-inspected', action='store_true', required=True)
    commissioning = sub.add_parser('commission', help='Configure verified firmware fields; no movement')
    commissioning.add_argument('--confirm-high-level', action='store_true', required=True)
    commissioning.add_argument('--teach-state-pointer', required=True)
    commissioning.add_argument('--teach-inactive-json', required=True)
    args = parser.parse_args()
    if args.action == 'clear':
        result = post('operator/clear', {'confirm_inspected': args.confirm_inspected})
    elif args.action == 'commission':
        # Import pure pose data only; host Python does not need aiohttp.
        import pose
        if not pose.SERIAL:
            raise ValueError('TRON2_SERIAL must be configured for commissioning')
        current = call('status')
        home = call('info')
        robot = current['robot']
        mode = robot.get('working_mode')
        if robot.get('working_mode_valid') is not True or not isinstance(mode, str) or not mode:
            raise ValueError('No valid working mode reported')
        # Do not infer High-Level Development from controller_mode or API success.
        # The flag records an operator observation; the teach field must be independently known.
        from teach import inactive_value
        pointer = args.teach_state_pointer
        expected = json.loads(args.teach_inactive_json)
        inactive_value(current['teach_status'], {'teach_state_pointer': pointer,
                       'teach_inactive_value': expected, 'expected_working_mode': mode})
        # Check recorded metadata and fresh stationary state through the existing preview.
        call('preview')
        target_path = Path(os.environ.get('TRON2_HOME_FILE', '/data/home.json'))
        raw = target_path.read_bytes()
        pose.validate_pose(json.loads(raw))
        if hashlib.sha256(raw).hexdigest() != home['pose_sha256']:
            raise ValueError('Host recorded pose differs from the container mount')
        config = {'commissioned': True, 'pose_sha256': home['pose_sha256'],
                  'expected_working_mode': mode, 'teach_state_pointer': pointer,
                  'teach_inactive_value': expected,
                  'high_level_confirmed_by_operator': True}
        path = Path(os.environ.get('TRON2_COMMISSION_FILE', '/data/commissioning.json'))
        # Replace contents in place: bind mount is the directory, not an individual file.
        path.write_text(json.dumps(config, ensure_ascii=False, indent=2) + '\n')
        result = {'commissioned': True, 'motion_sent': False,
                  'note': 'No automatic proof of High-Level mode; operator verified it.', **config}
    else:
        result = call(args.action)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise SystemExit('FAILED (do not automatically retry execute): ' + str(exc))
