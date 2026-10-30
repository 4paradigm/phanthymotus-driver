"""Execute real scheduler method without model/ROS dependencies or hardware."""
import ast,threading,time,types,unittest
from pathlib import Path


def scheduler(path):
    tree=ast.parse(path.read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Engine')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='run')
    ns={'time':time}
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec'),ns)
    return ns['run']

class Event:
    def wait(self,*a):pass
    def clear(self):pass
    def set(self):pass

class Fake:
    def __init__(self,values):
        self.closed=False;self.active=True;self.wake=Event();self.lock=threading.RLock()
        self.settings=types.SimpleNamespace(fps=2);self.values=iter(values);self.steps=0
    def clock(self):return next(self.values,.51)
    def step(self):
        self.steps+=1
        if self.steps>=2:self.closed=True

class RegressionTests(unittest.TestCase):
    def test_deadline_crossing_does_not_kill_worker(self):
        f=Fake([0.,0.,.49,.51,.51,.51])
        scheduler(Path(__file__).resolve().parents[1]/'yolo_detect/engine.py')(f)
        self.assertEqual(f.steps,2)
    def test_already_due_does_not_sleep_negative(self):
        f=Fake([0.,0.,.5,.51])
        scheduler(Path(__file__).resolve().parents[1]/'yolo_detect/engine.py')(f)
        self.assertEqual(f.steps,2)

if __name__=='__main__':unittest.main(verbosity=2)
