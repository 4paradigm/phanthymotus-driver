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
