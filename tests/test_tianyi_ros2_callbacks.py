"""A bad vendor callback must not stop delivery of later sensor updates."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS


def test_callback_error_does_not_stop_other_body_updates(capsys):
    path = Path(__file__).resolve().parents[1] / 'x-humanoid/tianyi2.0/main.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'DualDomainROS2')
    threads, received = [], []

    class Executor:
        context = None
        def __init__(self):
            self.pending = [ValueError('bad sensor frame'), 'next sensor frame']
        def spin_once(self, **kwargs):
            value = self.pending.pop(0)
            if isinstance(value, Exception):
                raise value
            received.append(value)

    class Thread:
        def __init__(self, target, args, **kwargs):
            self.target, self.args = target, args
            threads.append(self)
        def start(self): pass

    body, core = Executor(), Executor()
    body.context, core.context = body, core
    scope = dict(rclpy=NS(ok=lambda context: bool(context.pending)), threading=NS(Thread=Thread))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), str(path), 'exec'), scope)
    manager = scope['DualDomainROS2'].__new__(scope['DualDomainROS2'])
    manager.executor_tianyi, manager.executor_core = body, core
    manager.start_spin()
    threads[0].target(*threads[0].args)
    assert received == ['next sensor frame']
    assert 'bad sensor frame' in capsys.readouterr().out


def test_repeated_and_intermittent_callback_errors_have_bounded_logs(capsys):
    path = Path(__file__).resolve().parents[1] / 'x-humanoid/tianyi2.0/main.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'DualDomainROS2')
    threads, received = [], []
    class Executor:
        def __init__(self):
            self.context = self
            self.pending = [item for _ in range(250) for item in
                            (ValueError('bad\nframe\r\x1b[31m' + 'x' * 10000), 'good')]
        def spin_once(self, **kwargs):
            item = self.pending.pop(0)
            if isinstance(item, Exception):
                raise item
            received.append(item)
    class Thread:
        def __init__(self, target, args, **kwargs):
            self.target, self.args = target, args
            threads.append(self)
        def start(self): pass
    scope = dict(rclpy=NS(ok=lambda context: bool(context.pending)), threading=NS(Thread=Thread))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), str(path), 'exec'), scope)
    manager = scope['DualDomainROS2'].__new__(scope['DualDomainROS2'])
    manager.executor_tianyi, manager.executor_core = Executor(), Executor()
    manager.start_spin()
    for thread in threads:
        thread.target(*thread.args)
    output = capsys.readouterr().out
    errors = [line for line in output.splitlines() if 'callback failed' in line]
    assert len(errors) == 6  # first/100th/200th, independently for each executor
    for domain in ('domain0', 'domain42'):
        lines = [line for line in errors if domain in line]
        assert len(lines) == 3
        assert all(f'count={count}' in line for count, line in zip((1, 100, 200), lines))
    assert all(len(line) < 320 for line in errors)
    assert r'\n' in output and r'\r' in output and r'\x1b' in output
    assert '\r' not in output and '\x1b' not in output
    assert len(output.splitlines()) == 8  # six bounded records plus two exits
    assert received == ['good'] * 500
