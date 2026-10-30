import importlib.util
import threading
from pathlib import Path
from types import SimpleNamespace
import pytest

spec=importlib.util.spec_from_file_location('current_automatic_prepare',Path(__file__).resolve().parents[1] / 'deep_robotics/s10_pro/automatic_prepare.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class Auth:
    supports_prepare=True
    def __init__(self):self.valid=True;self.active=False;self.count=0
    def _policy_current(self):return self.valid
    def automatic_policy_enabled(self):return self.valid
    def automatic_prepare_allowed(self):return self.valid
    def authorized_now(self):return self.active and self.valid
    def public_status(self):return {'session_id':str(self.count)}
class Plugin:
    def __init__(self):
        self.auth=Auth();self._backend=SimpleNamespace(authorization=self.auth,safety_status=lambda:{'failed_closed':False})
        self._lock=threading.RLock();self._closed=threading.Event();self._settings={'enabled':True}
        self._started=False;self._active=None;self._worker=None;self._error=None;self.calls=[];self.blocked=[]
    def start(self):self._started=True
    def dispatch(self,action,args):
        assert action=='prepare_automatic' and args=={}
        self.calls.append(action)
        if self.blocked:return {'prepared':False,'blocked_by':self.blocked}
        self.auth.active=True;self.auth.count+=1
        return {'prepared':True}
@pytest.fixture
def p():
    p=Plugin();p.start();return p

def test_opt_in_required(monkeypatch,p):
    monkeypatch.setenv('S10_AUTO_PREPARE','0')
    assert m.start_automatic_preparation([p]) is None and not p.calls

def test_boot_starts_observation_and_prepares_without_action(monkeypatch):
    monkeypatch.setenv('S10_AUTO_PREPARE','1');p=Plugin();s=m.start_automatic_preparation([p])
    try:
        s._thread.join(.1)
        assert p._started and p.auth.active and p.calls==['prepare_automatic']
    finally:s.close()

@pytest.mark.parametrize('reason',['fresh_clear_fault_snapshot_required','joints_not_enabled','fresh_clear_hard_estop_required','stationary_idle_or_standing_required'])
def test_blockers_remain_and_recovery_requires_healthy_conditions(p,reason):
    p.blocked=[reason];s=m.AutomaticPreparation(p);s.tick()
    assert not p.auth.active and reason in s.status()['blocked_by']
    p.blocked=[];s.tick();assert p.auth.active and p.calls==['prepare_automatic','prepare_automatic']

def test_renew_only_after_expiry(p):
    s=m.AutomaticPreparation(p);s.tick();s.tick();assert p.calls==['prepare_automatic']
    p.auth.active=False;s.tick();assert p.calls==['prepare_automatic','prepare_automatic']

@pytest.mark.parametrize('case',['stopped','disabled','busy','failed','revoked'])
def test_never_overrides_explicit_stop_busy_failure_or_revocation(p,case):
    if case=='stopped':p._started=False
    if case=='disabled':p._settings['enabled']=False
    if case=='busy':p._active={'request_id':'active'}
    if case=='failed':p._backend.safety_status=lambda:{'failed_closed':True}
    if case=='revoked':p.auth.valid=False
    supervisor=m.AutomaticPreparation(p)
    supervisor._startup_observation_started=True
    supervisor.tick();assert not p.calls and not p.auth.active

def test_prepare_exception_latches_instead_of_looping(p):
    count=[]
    def fail(*args):count.append(1);raise RuntimeError()
    p.dispatch=fail;s=m.AutomaticPreparation(p);s.tick();s.tick()
    assert len(count)==1 and s.status()['state']=='needs_review'

def test_close_prevents_later_prepare(p):
    s=m.AutomaticPreparation(p);s.close();s.tick();assert not p.calls


def test_saved_configuration_can_enable_after_process_boot(p):
    p._started=False;p._settings['enabled']=False
    s=m.AutomaticPreparation(p);s.tick();assert not p.calls
    p._settings['enabled']=True;s.tick();assert p._started and p.auth.active
    p._started=False;s.tick();assert not p._started and p.calls==['prepare_automatic']


@pytest.mark.parametrize('case', ['legacy', 'startup_denied', 'renewal_denied', 'revoked', 'approved'])
def test_real_policy_controls_supervisor_without_motion(policy, case):
    import time
    from deep_robotics.s10_pro.loco_move import LocoMoveBackend, LocoMovePlugin
    from test_s10_timed_move import Bridge, Transport, Notifier
    from test_s10_prepare_authorization import automatic_manifest
    manifest, state, load = policy
    if case != 'legacy':
        automatic_manifest(manifest, state, startup=case != 'startup_denied', renewal=case != 'renewal_denied')
    grant, bridge, transports = load(real_clock=True), Bridge(), []
    def factory():
        transport = Transport(bridge)
        transports.append(transport)
        return transport
    backend = LocoMoveBackend(bridge, factory(), grant, transport_factory=factory)
    plugin = LocoMovePlugin({'enabled': True}, 'test', backend, notifier=Notifier())
    supervisor = m.AutomaticPreparation(plugin)
    try:
        supervisor.tick()
        expected = case not in ('legacy', 'startup_denied')
        assert grant.authorized_now() == expected
        if case == 'revoked':
            grant.revoke()
        if expected:
            if grant._session:
                grant._session['expires_monotonic'] = time.monotonic() - 1.
            assert not grant.authorized_now()
            supervisor.tick()
            assert grant.authorized_now() == (case == 'approved')
        else:
            # Repeated boot ticks cannot silently create the denied session.
            supervisor.tick()
            assert not grant.authorized_now()
        assert all(not t.states and not t.velocity_commands for t in transports)
    finally:
        supervisor.close()
        plugin.close()


from test_s10_prepare_authorization import policy
