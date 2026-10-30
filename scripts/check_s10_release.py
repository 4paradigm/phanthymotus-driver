#!/usr/bin/env python3
"""Check the S10 public contract and stage the exact Docker COPY inputs offline."""
from __future__ import annotations

import argparse
import ast
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import shlex
import shutil
import sys

import jsonschema
import yaml

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / 'deep_robotics/s10_pro'


def check_release(context: Path | None = None) -> dict:
    for path in DRIVER.glob('*.py'):
        ast.parse(path.read_text(), filename=str(path))
    manifest = yaml.safe_load((DRIVER / 'driver.yaml').read_text())
    config = yaml.safe_load((DRIVER / 'config.yaml').read_text())
    assert manifest['port'] == config['mcp_port'] == 15745
    assert manifest['build_context_extras'] == ['../../common']
    assert config['motion']['max_vx'] == config['motion']['max_yaw_rate'] == 1.
    assert config['motion']['max_vy'] == .1
    sys.path[:0] = [str(ROOT), str(DRIVER)]
    from operator_authorization import OperatorAuthorization
    from common.vendor_runtime import DriverBundle
    spec = importlib.util.spec_from_file_location('s10_release_main', DRIVER / 'main.py')
    main = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(main)

    class NoRobot:
        def acquire(self, *args):
            raise AssertionError('Release check must not start observation')
        def release(self, *args):
            pass
        def snapshot(self, *args):
            return {}
        def status(self):
            return {}

    plugins = main.build_runtime_plugins({}, 'release_check', NoRobot(), lambda *a: None,
                                         operator_authorization=OperatorAuthorization())
    try:
        tools = DriverBundle(plugins).get_all_tools()
        assert {c['name']: c['type'] for c in manifest['cards']} == {
            t['name']: t['type'] for t in tools}
        assert len(tools) == len(manifest['cards']) == 8
        for tool in tools:
            for key in ('name', 'type', 'title', 'description', 'inputSchema'):
                assert key in tool, (tool['name'], key)
            jsonschema.Draft202012Validator.check_schema(tool['inputSchema'])
            if 'configSchema' in tool:
                jsonschema.Draft202012Validator.check_schema(tool['configSchema'])
        loco = next(t for t in tools if t['name'] == 'loco_move')
        assert loco['topic_out'] == []
        schema = loco['inputSchema']
        assert next(iter(schema['properties'])) == 'action'
        assert schema['properties']['speed']['maximum'] == 1.
        assert schema['properties']['direction']['enum'] == [
            'forward', 'backward', 'turn_left', 'turn_right']
        jsonschema.validate({'action': 'stand_up', 'direction': 'forward'}, schema)
        assert schema['x-completion']['actions'] == ['stand_up', 'stand_down', 'move']
        assert not any(p._backend.transport.status().get('worker_running')
                       for p in plugins if hasattr(p, '_backend'))
    finally:
        for plugin in plugins:
            plugin.close() if hasattr(plugin, 'close') else plugin.stop()

    dockerfile = (DRIVER / 'Dockerfile').read_text()
    variables = dict(re.findall(r'^ARG (\w+)=(.+)$', dockerfile, re.M))
    inputs = set()
    for line in dockerfile.replace('\\\n', ' ').splitlines():
        if not line.startswith('COPY '):
            continue
        tokens = shlex.split(line)[1:]
        if tokens[0].startswith('--from='):
            continue
        for item in tokens[:-1]:
            item = re.sub(r'\$\{(\w+)\}', lambda m: variables[m[1]], item)
            path = ROOT / 'common' if item == 'common/' else DRIVER / item
            assert path.exists(), f'Missing Docker COPY input: {item}'
            inputs.add(item)
    assert {'loco_move.py', 'stationary_feedback.py', 'deploy/', 'common/'} <= inputs
    report = {'public_tools': [t['name'] for t in tools], 'copy_inputs': sorted(inputs),
              'robot_io': False, 'full_image_build_performed': False}
    if context is not None:
        context = context.resolve()
        assert not context.exists(), 'Context destination must not already exist'
        assert context != ROOT and ROOT not in context.parents, 'Stage outside the repository'
        context.mkdir(parents=True)
        for item in sorted(inputs):
            source = ROOT / 'common' if item == 'common/' else DRIVER / item
            dest = context / item
            if source.is_dir():
                shutil.copytree(source, dest, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
            else:
                dest.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source, dest)
        shutil.copy2(DRIVER / 'Dockerfile', context / 'Dockerfile')
        report['files'] = {str(p.relative_to(context)): hashlib.sha256(p.read_bytes()).hexdigest()
                           for p in sorted(context.rglob('*')) if p.is_file()}
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--context', type=Path)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    report = check_release(args.context)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + '\n')
    print('S10 catalogue, schemas and Docker inputs: OK (no robot I/O)', flush=True)


if __name__ == '__main__':
    main()
