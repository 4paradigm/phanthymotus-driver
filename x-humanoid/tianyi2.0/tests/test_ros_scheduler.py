"""Exercise the actual bundle scheduler without importing robot integrations."""
import ast
from pathlib import Path
from types import SimpleNamespace


def scheduler(ok, sleep):
    tree = ast.parse((Path(__file__).parents[1] / 'main.py').read_text())
    ros = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'DualDomainROS2')
    start = next(n for n in ros.body if isinstance(n, ast.FunctionDef) and n.name == 'start_spin')
    spin = next(n for n in start.body if isinstance(n, ast.FunctionDef) and n.name == '_spin')
    scope = {'rclpy': SimpleNamespace(ok=ok), 'time': SimpleNamespace(sleep=sleep)}
    exec(compile(ast.Module(body=[spin], type_ignores=[]), 'main.py', 'exec'), scope)
    return scope['_spin']


def test_continuously_ready_body_yields_after_bounded_batch():
    calls, waits = [], []
    context = object()
    def ok(*, context):
        return len(waits) < 3
    spin = scheduler(ok, waits.append)
    spin(SimpleNamespace(context=context, spin_once=lambda **kw: calls.append(kw)), 'domain0')
    assert len(calls) == 30
    assert all(c == {'timeout_sec': 0.0} for c in calls)
    assert waits == [.005] * 3


def test_body_context_shutdown_interrupts_batch_without_extra_dispatch():
    calls, waits = [], []
    spin = scheduler(lambda **kw: len(calls) < 4, waits.append)
    spin(SimpleNamespace(context=object(), spin_once=lambda **kw: calls.append(kw)), 'domain0')
    assert len(calls) == 4 and waits == []


def test_core_retains_blocking_dispatch_without_batch_delay():
    calls, waits = [], []
    spin = scheduler(lambda **kw: len(calls) < 4, waits.append)
    spin(SimpleNamespace(context=object(), spin_once=lambda **kw: calls.append(kw)), 'domain42')
    assert calls == [{'timeout_sec': .1}] * 4 and waits == []
