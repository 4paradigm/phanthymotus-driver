"""ACP outcomes/delivery, with a recording SDK and loopback HTTP only."""
import ast
import asyncio
from contextvars import ContextVar
import json
import os
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from agilex.piper.completion import ActionCompletions
import test_piper_unified as fixtures


def eventually(test, predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate(): return
        time.sleep(.01)
    test.assertTrue(predicate(), 'Timed out waiting for completion delivery')


class CallbackServer:
    def __init__(self, reply=None):
        self.bodies = []
        self.reply = reply or (lambda body: {'ok': True, 'action_id': body['action_id']})
        owner = self
        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                owner.bodies.append(body)
                try:
                    result = owner.reply(body)
                except Exception:
                    self.send_error(503); return
                encoded = json.dumps(result).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(encoded)))
                self.end_headers(); self.wfile.write(encoded)
            def log_message(self, *args): pass
        self.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    @property
    def url(self): return f'http://127.0.0.1:{self.server.server_port}'

    def close(self):
        self.server.shutdown(); self.server.server_close(); self.thread.join(1)


class CoreFunctions:
    """Run the installed Core's unchanged ACP/register/sync function bodies.

    Optional source snapshot has no credentials and is supplied by the runner.
    Stubs replace only the unrelated event bus, logging and table cleanup.
    No requests are made to a production Core or a physical robot.
    """
    def __init__(self, root):
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self.thread.start()
        self.events = []
        ns = dict(asyncio=asyncio, time=time, current_agent_context=ContextVar('agent', default='test'),
                  _pending_timeout=lambda *args: 3, _notify_settled=lambda *args: None)
        for key in ('actions', 'tools', 'resources', 'started', 'owner', 'timeouts',
                    'background', 'silent', 'results'):
            ns['_pending_' + key] = {}
        def forget(ids, *args):
            for action_id in list(ids):
                for key in tuple(ns):
                    if key.startswith('_pending_') and isinstance(ns[key], dict): ns[key].pop(action_id, None)
        ns['_forget_pending'] = forget
        self.load(root / 'mcp_client.py', ns, {'mark_action_complete', 'completion_is_silent', '_register_pending', 'sync'})
        client = SimpleNamespace(**ns)
        async def enqueue(**kwargs): self.events.append(kwargs['payload'])
        self.module_patch = patch.dict('sys.modules', {
            'benchmark_facts': SimpleNamespace(note_acp_post=lambda body: None),
            'event_bus': SimpleNamespace(enqueue=enqueue)})
        self.module_patch.start()
        api = dict(json=json, fastapi=SimpleNamespace(Request=object), mcp_client=client)
        self.load(root / 'start.py', api, {'acp_complete'})
        self.ns, self.api = ns, api
        self.server = CallbackServer(self.complete)

    @staticmethod
    def load(path, namespace, names):
        tree = ast.parse(path.read_text())
        nodes = [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name in names]
        if {node.name for node in nodes} != names: raise AssertionError(f'Core interface changed: {path}')
        for node in nodes: node.decorator_list = []
        exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), 'exec'), namespace)

    def complete(self, body):
        async def read(): return body
        return asyncio.run_coroutine_threadsafe(self.api['acp_complete'](SimpleNamespace(json=read)), self.loop).result(2)

    def register(self, action_id):
        async def register():
            self.ns['_register_pending'](action_id, tool_name='arm_control',
                meta={'completion': {'timeout': 3}, 'resource': {'arm'}}, args={}, result={})
        asyncio.run_coroutine_threadsafe(register(), self.loop).result(1)

    def wait(self, action_id, timeout=2):
        return asyncio.run_coroutine_threadsafe(self.ns['sync']([action_id], timeout=timeout), self.loop)

    def close(self):
        self.server.close(); self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(1); self.loop.close(); self.module_patch.stop()


class CompletionTests(unittest.TestCase):
    setUp = fixtures.UnifiedTests.setUp
    tearDown = fixtures.UnifiedTests.tearDown
    call = fixtures.UnifiedTests.call

    def start_callback(self, reply=None):
        self.completion_mock.stop()
        server = CallbackServer(reply)
        self.addCleanup(server.close)
        env = patch.dict('os.environ', {'AGENT_CORE_URL': server.url})
        env.start(); self.addCleanup(env.stop)
        self.runtime.completions.RETRY_DELAYS = (.01, .01, .01)
        return server

    def test_real_http_acknowledgment_is_validated_and_retried(self):
        replies = iter([{'ok': False}, {'ok': True, 'action_id': 'wrong'}, {'ok': True, 'action_id': 'id1'}])
        server = self.start_callback(lambda body: next(replies))
        manager = self.runtime.completions
        manager.reserve('id1', 'arm_control', 'move')
        manager.finish('id1', 'completed', {'executed': True})
        manager.deliver('id1', 'completed', {'executed': True}, 'arm_control')
        record = manager.snapshot()[0]
        self.assertEqual(record['attempts'], 3)
        self.assertEqual(record['delivery'], 'acknowledged')
        self.assertEqual([body['action_id'] for body in server.bodies], ['id1'] * 3)
        self.assertEqual(self.sdk.calls, [])

    def test_exhausted_delivery_does_not_change_actual_robot_success(self):
        server = self.start_callback(lambda body: (_ for _ in ()).throw(RuntimeError('offline')))
        manager = self.runtime.completions
        manager.reserve('id1', 'arm_control', 'move')
        manager.finish('id1', 'completed', {'executed': True})
        manager.deliver('id1', 'completed', {'executed': True}, 'arm_control')
        self.assertEqual(len(server.bodies), 4)
        record = manager.snapshot()[0]
        self.assertEqual((record['status'], record['delivery']), ('completed', 'failed'))
        self.assertTrue(self.runtime.last_actions['arm_control']['completion_delivery_failed'])
        self.assertTrue(self.runtime.last_actions['arm_control']['result']['executed'])

    def test_old_delivery_cannot_overwrite_a_newer_action(self):
        manager = self.runtime.completions
        for action_id, action in [('old', 'move'), ('new', 'reset')]:
            manager.reserve(action_id, 'arm_control', action)
            manager.finish(action_id, 'completed', {'target': action_id})
        server = self.start_callback()
        manager.deliver('old', 'completed', {'target': 'old'}, 'arm_control')
        self.assertEqual(self.runtime.last_actions['arm_control']['action_id'], 'new')
        self.assertEqual(manager.snapshot()[0]['result']['target'], 'old')
        self.assertEqual(manager.snapshot()[0]['delivery'], 'acknowledged')
        self.assertEqual(len(server.bodies), 1)

    def test_record_bound_does_not_evict_pending_completions(self):
        manager = self.runtime.completions
        for i in range(manager.MAX_RECORDS): manager.reserve(str(i), 'arm_control', 'move')
        with self.assertRaisesRegex(RuntimeError, 'delivery is busy'): manager.reserve('new', 'arm_control', 'move')
        manager.update('0', delivery='acknowledged')
        manager.reserve('new', 'arm_control', 'reset')
        self.assertEqual(len(manager.snapshot()), manager.MAX_RECORDS)
        self.assertEqual(manager.snapshot()[0]['action_id'], '1')

    def test_fast_terminal_return_has_no_pending_id_or_spurious_callback(self):
        self.runtime.config['async_actions'] = True
        server = self.start_callback()
        result = self.call('move', translation_mm=[0, 0, 0])
        self.assertEqual(result['state'], 'completed')
        self.assertNotIn('action_id', result)
        self.assertEqual(self.runtime.completions.snapshot()[0]['delivery'], 'not_required')
        self.assertEqual(server.bodies, [])
        self.assertEqual(self.sdk.calls, [])

    def test_worker_start_failure_releases_reservation_without_a_robot_command(self):
        self.runtime.config['async_actions'] = True
        with patch('agilex.piper.completion.threading.Thread.start', side_effect=RuntimeError('Thread unavailable')):
            with self.assertRaisesRegex(RuntimeError, 'Thread unavailable'):
                self.call('move')
        self.assertFalse(self.runtime.gate.locked())
        self.assertEqual(self.runtime.task['state'], 'failed')
        self.assertEqual(self.runtime.completions.snapshot()[0]['status'], 'error')
        self.assertEqual(self.sdk.calls, [])

    def test_running_success_error_and_cancel_each_deliver_their_own_terminal(self):
        server = self.start_callback()
        self.runtime.config['async_actions'] = True
        for outcome in ('success', 'error', 'cancelled'):
            entered, release = threading.Event(), threading.Event()
            def execute(*args, **kwargs):
                entered.set(); release.wait(2)
                if outcome == 'error': raise RuntimeError('Simulated controller rejection')
                return fixtures.fast_execute(*args, **kwargs)
            with self.subTest(outcome=outcome), patch('agilex.piper.unified_control.execute_target', side_effect=execute):
                result = self.call('move')
                self.assertEqual(result['state'], 'running')
                self.assertTrue(entered.is_set())
                self.assertEqual(self.runtime.task['task_id'], result['action_id'])
                if outcome == 'cancelled': self.call('stop_motion')
                release.set()
                eventually(self, lambda: self.runtime.completions.snapshot()[-1]['delivery'] == 'acknowledged')
                body = server.bodies[-1]
                self.assertEqual(body['action_id'], result['action_id'])
                self.assertEqual(body['status'], {'success': 'completed', 'error': 'error', 'cancelled': 'cancelled'}[outcome])
                self.assertFalse(self.runtime.gate.locked())
        self.assertEqual(len(server.bodies), 3)

    @unittest.skipUnless(os.environ.get('PIPER_CORE_SRC'), 'Set PIPER_CORE_SRC to the unchanged installed Core snapshot')
    def test_actual_core_waiter_unblocks_only_after_each_terminal_callback(self):
        self.completion_mock.stop()
        core = CoreFunctions(Path(os.environ['PIPER_CORE_SRC']))
        self.addCleanup(core.close)
        with patch.dict('os.environ', {'AGENT_CORE_URL': core.server.url}):
            self.runtime.config['async_actions'] = True
            for outcome in ('success', 'error', 'cancelled'):
                release = threading.Event()
                def execute(*args, **kwargs):
                    release.wait(2)
                    if outcome == 'error': raise RuntimeError('Controller rejected target')
                    return fixtures.fast_execute(*args, **kwargs)
                with self.subTest(outcome=outcome), patch('agilex.piper.unified_control.execute_target', side_effect=execute):
                    result = self.call('move'); action_id = result['action_id']
                    core.register(action_id)
                    waiter = core.wait(action_id)
                    self.assertFalse(waiter.done())
                    if outcome == 'cancelled': self.call('stop_motion')
                    release.set()
                    payload = waiter.result(3)
                    self.assertEqual(payload['status'], 'completed')
                    self.assertEqual(payload['results'][action_id]['status'],
                                     {'success': 'completed', 'error': 'error', 'cancelled': 'cancelled'}[outcome])
                    eventually(self, lambda: self.runtime.completions.snapshot()[-1]['delivery'] == 'acknowledged')
            self.assertEqual(len(core.events), 3)

    @unittest.skipUnless(os.environ.get('PIPER_CORE_SRC'), 'Set PIPER_CORE_SRC to the unchanged installed Core snapshot')
    def test_current_core_receipt_does_not_prove_pending_registration(self):
        core = CoreFunctions(Path(os.environ['PIPER_CORE_SRC']))
        self.addCleanup(core.close)
        ack = core.complete({'action_id': 'early', 'status': 'completed', 'result': {}})
        self.assertTrue(ack['ok'])
        core.register('early')
        self.assertEqual(core.wait('early', timeout=.02).result(1)['status'], 'timeout')


if __name__ == '__main__': unittest.main()
