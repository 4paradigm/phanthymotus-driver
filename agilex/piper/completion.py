"""Bounded per-action ACP delivery; never owns or retries a robot command."""
from collections import OrderedDict
from copy import deepcopy
import json
import os
import ssl
import threading
import time
import urllib.request
from uuid import uuid4


class ActionCompletions:
    MAX_RECORDS = 64
    ATTEMPTS = 4
    RETRY_DELAYS = (.25, .5, 1.)
    RESPONSE_GRACE = .15

    def __init__(self, runtime):
        self.runtime = runtime
        self.lock = threading.Lock()
        self.records = OrderedDict()
        self.threads = set()

    def wait(self, timeout=2):
        deadline = time.monotonic() + timeout
        with self.lock:
            threads = list(self.threads)
        for thread in threads:
            if thread is not threading.current_thread():
                thread.join(max(0, deadline - time.monotonic()))

    def reserve(self, action_id, tool, action):
        with self.lock:
            if len(self.records) >= self.MAX_RECORDS:
                removable = next((key for key, value in self.records.items()
                                  if value['delivery'] in {'acknowledged', 'failed', 'not_required'}), None)
                if removable is None:
                    raise RuntimeError('Completion delivery is busy; wait before submitting another action.')
                self.records.pop(removable)
            self.records[action_id] = dict(action_id=action_id, tool=tool, action=action,
                status='running', delivery='waiting', attempts=0, created_at=time.time())

    def update(self, action_id, **values):
        with self.lock:
            record = self.records[action_id]
            record.update(deepcopy(values))
            # Keep the historical last-result API without letting a delayed ACP
            # retry overwrite a newer action's actual robot outcome.
            latest = self.runtime.last_actions.get(record['tool'])
            if latest and latest.get('action_id') == action_id:
                self.runtime.last_actions[record['tool']] = {**latest,
                    'completion_delivery': record['delivery'], 'completion_attempts': record['attempts'],
                    'completion_delivery_failed': record['delivery'] == 'failed'}
            return deepcopy(record)

    def finish(self, action_id, status, result):
        record = self.update(action_id, status=status, result=result, finished_at=time.time())
        with self.lock:
            self.runtime.last_actions[record['tool']] = dict(action_id=action_id, status=status,
                result=deepcopy(result), completion_delivery=record['delivery'])

    def snapshot(self):
        with self.lock:
            return deepcopy(list(self.records.values()))

    def send(self, action_id, status, result, tool):
        """Check the HTTP acknowledgment, not merely its status code.

        Core's current acknowledgment confirms receipt only: it does not expose
        whether the pending waiter was found. Integration tests exercise that
        distinction; no second successful POST is sent to manufacture a waiter.
        """
        url = os.environ.get('AGENT_CORE_URL', 'https://localhost:15678').rstrip('/')
        body = dict(action_id=action_id, status=status, result=result, tool=tool, ts=time.time())
        request = urllib.request.Request(url + '/api/acp/complete',
            data=json.dumps(body, allow_nan=False).encode(),
            headers={'Content-Type': 'application/json'}, method='POST')
        ca = os.environ.get('AGENT_CORE_CA_CERT')
        context = (ssl.create_default_context(cafile=ca) if ca else
                   ssl._create_unverified_context() if url.startswith('https://localhost:') else None)
        with urllib.request.urlopen(request, timeout=2, context=context) as response:
            ack = json.load(response)
        if not isinstance(ack, dict) or ack.get('ok') is not True or ack.get('action_id') != action_id:
            raise RuntimeError('Agent Core did not acknowledge this action_id.')
        return ack

    def deliver(self, action_id, status, result, tool):
        for attempt in range(1, self.ATTEMPTS + 1):
            self.update(action_id, delivery='sending', attempts=attempt)
            try:
                # Use the existing hook so offline SDK tests can record delivery.
                self.runtime.acp_complete(action_id, status, deepcopy(result), tool)
            except Exception as exc:
                error = str(exc).replace('\n', ' ')[:240] or type(exc).__name__
                self.update(action_id, delivery='failed' if attempt == self.ATTEMPTS else 'retrying',
                            delivery_error=error)
                print(f'[piper ACP] {action_id} {status}: attempt={attempt} failed: {error}', flush=True)
                if attempt < self.ATTEMPTS:
                    time.sleep(self.RETRY_DELAYS[attempt - 1])
            else:
                self.update(action_id, delivery='acknowledged', delivery_error=None, delivered_at=time.time())
                print(f'[piper ACP] {action_id} {status}: acknowledged attempt={attempt}', flush=True)
                return


def run_async(runtime, name, action, args, *, error_text=str):
    """Reserve once, publish one outcome, then deliver it outside the motion lock."""
    reservation = runtime.operation(name, action)
    reservation.__enter__()
    action_id = f'{name}_{uuid4().hex}'
    completions = runtime.completions
    try:
        completions.reserve(action_id, name, action)
    except BaseException:
        reservation.__exit__(None, None, None)
        if name == 'arm_control':
            runtime._phase('failed', error='Completion delivery is busy; no action was started.')
        raise
    if name == 'arm_control':
        runtime._phase('preparing', task_id=action_id)
    finished, published = threading.Event(), threading.Event()
    outcome = {}

    def perform():
        runtime.worker_context.reserved = (name, action)
        try:
            outcome.update(status='completed', result=runtime.dispatch_sync(action, dict(args)))
        except BaseException as exc:
            cancelled = runtime.cancel.is_set()
            outcome.update(status='cancelled' if cancelled else 'error',
                           result={'state': 'cancelled' if cancelled else 'failed', 'error': error_text(exc)})
        finally:
            try:
                completions.finish(action_id, outcome['status'], outcome['result'])
            finally:
                reservation.__exit__(None, None, None)
                runtime.worker_context.reserved = None
                finished.set()
        published.wait()
        if outcome.get('notify'):
            # Existing Core registers its pending action after reading the MCP
            # response. This grace reduces that race but is not an acknowledgment
            # of registration; do not claim arbitrary response delays are safe.
            time.sleep(completions.RESPONSE_GRACE)
            completions.deliver(action_id, outcome['status'], outcome['result'], name)

    def work():
        try:
            perform()
        finally:
            with completions.lock:
                completions.threads.discard(threading.current_thread())

    thread = threading.Thread(target=work, name=action_id, daemon=True)
    try:
        with completions.lock:
            completions.threads.add(thread)
            thread.start()
    except BaseException:
        with completions.lock:
            completions.threads.discard(thread)
        completions.update(action_id, status='error', delivery='not_required',
                           result={'error': 'The action worker could not start.'})
        reservation.__exit__(None, None, None)
        if name == 'arm_control':
            runtime._phase('failed', error='The action worker could not start.')
        raise
    if finished.wait(.05):
        completions.update(action_id, delivery='not_required')
        published.set()
        if outcome['status'] != 'completed':
            raise RuntimeError(outcome['result']['error'])
        return outcome['result']
    completions.update(action_id, delivery='pending')
    outcome['notify'] = True
    published.set()
    return {'state': 'running', 'action_id': action_id,
            'reminder': 'Task accepted. Wait for the completion result or use Get status.'}
