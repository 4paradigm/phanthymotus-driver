"""Payload mass contracts, rejection cases and lifecycle without ROS or SDK."""
from __future__ import annotations

import copy
import importlib.util
import json
import random
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

BUMI=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(BUMI))
from payload_mass_sampling import SharedSampler, MassError, check_frame
from payload_mass_estimator import (validate_profile,summarize,fit_model,estimate,check_baseline)
from payload_mass import PayloadMassPlugin, Store
from payload_mass_cli import synthetic_profile, synthetic_frame, replay


def frames(mass=0, *, seed=4, count=25):
    rng=random.Random(seed)
    result=[]
    now=time.monotonic()
    for i in range(count):
        f=synthetic_frame(mass,i,rng)
        f.update(serial=i+1,host_monotonic_s=now-(count-1-i)*0.1,host_time_s=time.time(),error_epoch=0)
        result.append(f)
    return result


@pytest.fixture
def profile():
    return validate_profile("left_A",synthetic_profile(),"left",loading=True)


@pytest.fixture
def calibration(profile):
    baseline={"id":"tare_test","profile_fingerprint":profile["fingerprint"],"created_s":time.time(),
              "zero_trial_ids":["zero1","zero2"],"summary":summarize(frames(),profile)}
    trials=[]
    for i,(role,mass) in enumerate([("train",0.3),("train",0.3),("train",0.7),("train",0.7),("validation",0.3),("validation",0.7)]):
        trials.append({"id":f"independent_{i}","action":"calibrate","baseline_id":baseline["id"],"role":role,
                       "known_mass_kg":mass,"profile_fingerprint":profile["fingerprint"],
                       "summary":summarize(frames(mass,seed=10+i),profile)})
    model=fit_model(trials,baseline,profile)
    model["id"]="cal_test"
    return baseline,trials,model


def test_signed_multi_axis_net_mass_and_unresponsive_axis(profile,calibration):
    baseline,trials,model=calibration
    assert model["k"][0]>0 and model["k"][1]<0
    assert model["weights"][2]==0  # insensitive yaw is excluded, not averaged
    assert not set(model["training_trial_ids"]) & set(model["validation_trial_ids"])
    result=estimate(summarize(frames(0.5),profile),baseline,model,profile,time.time())
    assert result["mass_kg"]==pytest.approx(0.5,abs=0.01)
    assert model["validation"]["max_abs_error_kg"]<0.01


@pytest.mark.parametrize("mass,code",[(0,"BELOW_RESOLUTION"),(-0.001,"BELOW_RESOLUTION"),(-0.4,"OUT_OF_RANGE"),(0.9,"OUT_OF_RANGE")])
def test_no_clipping_or_extrapolation(mass,code,profile,calibration):
    baseline,_,model=calibration
    with pytest.raises(MassError,match="signal|range") as exc:
        estimate(summarize(frames(mass),profile),baseline,model,profile,time.time())
    assert exc.value.code==code


def test_wrong_load_vector_not_hidden_by_absolute_value(profile,calibration):
    baseline,_,model=calibration
    data=frames(0.5)
    for f in data:
        f["tau"][1]+=0.6
    with pytest.raises(MassError) as exc:
        estimate(summarize(data,profile),baseline,model,profile,time.time())
    assert exc.value.code=="RESIDUAL"


@pytest.mark.parametrize("mutation,code",[
    (lambda t,b: t.__setitem__(4,copy.deepcopy(t[0])),"TRIAL_LEAKAGE"),
    (lambda t,b: t.pop(),"INSUFFICIENT_VALIDATION"),
    (lambda t,b: b.__setitem__("zero_trial_ids",["only_one"]),"INSUFFICIENT_CALIBRATION"),
    (lambda t,b: t[4]["summary"]["tau"].__setitem__(0,8),"CALIBRATION_RESIDUAL"),
    (lambda t,b: t[0].__setitem__("profile_fingerprint","wrong"),"CONFIG_CHANGED"),
])
def test_calibration_rejects_leakage_bad_holdout_and_context(mutation,code,profile,calibration):
    baseline,trials,_=copy.deepcopy(calibration)
    mutation(trials,baseline)
    with pytest.raises(MassError) as exc:
        fit_model(trials,baseline,profile)
    # Duplicating a complete train trial can first invalidate endpoint coverage.
    assert exc.value.code in (code,"INSUFFICIENT_VALIDATION")


def test_zero_and_noisy_feedback_cannot_create_model(profile,calibration):
    baseline,trials,_=copy.deepcopy(calibration)
    for t in trials:
        t["summary"]["tau"]=baseline["summary"]["tau"][:]
    with pytest.raises(MassError) as exc:
        fit_model(trials,baseline,profile)
    assert exc.value.code=="UNIDENTIFIABLE"


@pytest.mark.parametrize("field,value,code",[
    ("arm","right","NOT_CALIBRATED"),("motor_ids",[10,11],"MAPPING"),
    ("joint_names",["arm_l1_joint"],"MAPPING"),("mapping_verified",False,"PROFILE_UNVERIFIED"),
    ("feedback_verified",False,"PROFILE_UNVERIFIED"),("max_mass_kg",float("nan"),"BAD_PARAMETER"),
    ("torque_units","Nm","UNITS_UNVERIFIED"),("accept_host_only_freshness","false","BAD_PARAMETER"),
])
def test_profile_gates(field,value,code):
    raw=synthetic_profile()
    raw[field]=value
    with pytest.raises(MassError) as exc:
        validate_profile("test",raw,"left",loading=True)
    assert exc.value.code==code


@pytest.mark.parametrize("mutate,code",[
    (lambda f:f["q"].__setitem__(20,0.3),"POSE_MISMATCH"),
    (lambda f:f.__setitem__("quaternion_xyzw",[0.2,0,0,0.98]),"BODY_MISMATCH"),
    (lambda f:f.__setitem__("workmode",26),"MODE_MISMATCH"),
    (lambda f:f["errors"].__setitem__(16,999),"MOTOR_FAULT"),
    (lambda f:f["temperature"].__setitem__(0,90),"OVER_TEMPERATURE"),
    (lambda f:f["tau"].__setitem__(0,25),"SATURATED"),
    (lambda f:f["dq"].__setitem__(4,0.3),"UNSTABLE"),
    (lambda f:f.__setitem__("source_seq",None),"FRESHNESS_UNVERIFIED"),
])
def test_frame_conditions(mutate,code,profile):
    frame=frames()[0]
    mutate(frame)
    with pytest.raises(MassError) as exc:
        check_frame(frame,profile)
    assert exc.value.code==code


def test_temperature_pose_and_time_relative_to_tare(profile,calibration):
    baseline,_,_=calibration
    for name,delta,code in (("temperature",5,"TEMPERATURE_DRIFT"),("pose_q",0.05,"POSE_MISMATCH")):
        summary=copy.deepcopy(baseline["summary"])
        summary[name][0]+=delta
        with pytest.raises(MassError) as exc:
            check_baseline(baseline,summary,profile,time.time())
        assert exc.value.code==code
    with pytest.raises(MassError) as exc:
        check_baseline(baseline,baseline["summary"],profile,baseline["created_s"]+3601)
    assert exc.value.code=="BASELINE_EXPIRED"


def test_sampler_reorders_by_id_and_rejects_bad_mapping_and_nan():
    sampler=SharedSampler(maxlen=3)
    frame=synthetic_frame(0.5,1)
    fields=("motor_ids","q","dq","tau","temperature","errors")
    for name in fields:
        frame[name].reverse()
    assert sampler.push(frame)
    assert sampler.diagnostics()["latest"]["tau"][0]>1.9
    bad=synthetic_frame(0,2)
    bad["motor_ids"][1]=0
    assert not sampler.push(bad)
    bad=synthetic_frame(0,2)
    bad["tau"][0]=float("nan")
    assert not sampler.push(bad)
    for seq in range(2,8):
        sampler.push(synthetic_frame(0,seq))
    assert sampler.diagnostics()["buffer_count"]==3
    assert sampler.diagnostics()["read_error_count"]==2


def test_identical_values_are_not_automatically_dead_frames():
    sampler=SharedSampler()
    assert sampler.push(synthetic_frame(0,None))
    assert sampler.push(synthetic_frame(0,None))
    diag=sampler.diagnostics()
    assert diag["received_count"]==2 and diag["torque_changed_observations"]==0
    assert diag["freshness_status"]=="host_receipt_only"
    assert diag["effective_sample_count"] is None


def test_sampler_does_not_invent_new_source_frames():
    sampler=SharedSampler()
    assert sampler.push(synthetic_frame(0,5))
    for seq in (5,4,3):
        assert not sampler.push(synthetic_frame(0,seq))
    assert sampler.diagnostics()["received_count"]==1
    assert sampler.diagnostics()["source_duplicate_count"]==3


def test_getter_adapter_preserves_precision_and_uses_no_sdk_init():
    sampler=SharedSampler()
    frame=synthetic_frame(0)
    frame["tau"][0]=1.123456789
    motors=[SimpleNamespace(motor_id=i,pos=frame["q"][i],vel=0,tau=frame["tau"][i],temperature=30,error=0) for i in range(21)]
    sampler.accept_getter(motors,SimpleNamespace(ori=[0,0,0,1],angular_vel=[0,0,0]),2)
    assert sampler.diagnostics()["latest"]["tau"][0]==1.123456789
    assert sampler.diagnostics()["latest"]["source_seq"] is None


@pytest.mark.parametrize("kind",["empty","old_source","oscillating","too_few"])
def test_window_rejects_missing_stale_oscillation_and_short_pause(kind,profile):
    p=dict(profile,settle_timeout_s=0.05,min_samples=5)
    sampler=SharedSampler()
    cancel=threading.Event()
    halted=threading.Event()
    def feed():
        seq=0
        while not halted.is_set():
            seq+=1
            f=synthetic_frame(0,1 if kind=="old_source" else seq)
            if kind=="oscillating":
                f["tau"][0]+=0.2 if seq%2 else -0.2
            if kind!="empty":
                sampler.push(f)
            halted.wait(0.15 if kind=="too_few" else 0.005)
    thread=threading.Thread(target=feed)
    thread.start()
    try:
        with pytest.raises(MassError) as exc:
            sampler.collect(0.12,p,cancel)
        assert exc.value.code in {"INSUFFICIENT_SAMPLES","STALE_SOURCE","UNSTABLE"}
    finally:
        halted.set()
        thread.join()


def test_window_accepts_full_stable_interval_and_cancel(profile):
    sampler=SharedSampler()
    stopped=threading.Event()
    def feed():
        seq=0
        while not stopped.is_set():
            seq+=1
            sampler.push(synthetic_frame(0.4,seq))
            stopped.wait(0.005)
    thread=threading.Thread(target=feed)
    thread.start()
    try:
        result=sampler.collect(0.12,dict(profile,min_samples=5),threading.Event())
        assert len(result)>=5
        assert result[-1]["host_monotonic_s"]-result[0]["host_monotonic_s"]>=0.12
        cancel=threading.Event()
        cancel.set()
        with pytest.raises(MassError) as exc:
            sampler.collect(0.1,profile,cancel)
        assert exc.value.code=="CANCELLED"
    finally:
        stopped.set()
        thread.join()


@pytest.fixture
def card(tmp_path,monkeypatch):
    engineering=tmp_path/"engineering.json"
    engineering.write_text(json.dumps({"profiles":{"left_A":synthetic_profile()}}))
    notifications=[]
    published=[]
    sampler=SharedSampler()
    plugin=PayloadMassPlugin({"data_dir":str(tmp_path/"data"),"profiles_file":str(engineering),
                              "hardware_id":"SIMULATION_ONLY","sdk_version":"synthetic/v1"},"test",
                             sampler=sampler,notify=lambda *a:notifications.append(a),publish=published.append)
    monkeypatch.setattr(sampler,"collect",lambda *args:frames(0))
    plugin.notifications=notifications
    plugin.published=published
    yield plugin
    plugin.stop()


def dispatch_wait(card,action,**args):
    args={"arm":"left","profile_id":"left_A","mode":"static",**args}
    result=card.dispatch(action,args)
    if result["state"]=="running":
        card._job["thread"].join(3)
        assert not card._job["thread"].is_alive()
        result=copy.deepcopy(card._last)
    return result


def seed_card(card,profile,calibration):
    baseline,trials,model=calibration
    card._store.write("profiles","left_A",{"baseline":baseline,"model":model["id"]})
    card._store.write("calibrations",model["id"],model,immutable=True)
    for trial in trials:
        card._store.write("trials",trial["id"],trial,immutable=True)


def test_schema_truth_not_in_estimate_and_single_json_port(card):
    tool=card.get_tool()
    assert tool["type"]=="processor" and not tool["multiInstance"]
    assert "known_mass_kg" not in tool["inputSchema"]["x-action-params"]["estimate"]["params"]
    assert {"start","stop","info"}<=set(tool["inputSchema"]["properties"]["action"]["enum"])
    assert tool["topic_out"]==[{"topic":"/test/payload_mass/result","format":"data/json"}]
    assert len(card.get_tools())==1
    assert card.dispatch("start",{})["state"]=="ready"
    assert card.dispatch("estimate",{"profile_id":"left_A","known_mass_kg":0.5})["error_code"]=="BAD_PARAMETER"
    assert card.dispatch("estimate",{"profile_id":"left_A","arm":"right"})["error_code"]=="NOT_CALIBRATED"
    assert card.dispatch("estimate",{"profile_id":"left_A"})["error_code"]=="NOT_CALIBRATED"
    assert card.dispatch("estimate",{"mode":"slow_probe"})["error_code"]=="UNSUPPORTED"


@pytest.mark.parametrize("mass",[-1,float("nan"),float("inf"),True,2])
def test_calibrate_rejects_bad_truth_and_range(card,mass):
    dispatch_wait(card,"tare",confirm_empty=True,trial_note="empty one")
    result=dispatch_wait(card,"calibrate",known_mass_kg=mass,independent_trial_confirmed=True,trial_note="independent")
    assert result["valid"] is False and result["mass_kg"] is None
    assert result["error_code"] in {"BAD_PARAMETER","OUT_OF_RANGE"}


def test_calibration_failure_and_corrupt_store_preserve_model(card,profile,calibration):
    seed_card(card,profile,calibration)
    previous=card._store.read("profiles","left_A")
    result=dispatch_wait(card,"fit")
    assert result["state"]=="error"  # seeded trial names deliberately don't belong to left_A prefix
    assert card._store.read("profiles","left_A")==previous
    path=card._store._path("calibrations","cal_test")
    path.write_text('{"bad":1}')
    result=dispatch_wait(card,"estimate")
    assert result["error_code"]=="STORAGE_CORRUPT"
    assert result["mass_kg"] is None
    assert card._store.read("profiles","left_A")==previous


def test_success_then_failed_estimate_does_not_show_old_mass(card,profile,calibration,monkeypatch):
    seed_card(card,profile,calibration)
    monkeypatch.setattr(card._sampler,"collect",lambda *a:frames(0.5))
    result=dispatch_wait(card,"estimate",instance_id="canvas-id",role="train",confirm_empty=True)
    assert result["valid"] and result["mass_kg"]==pytest.approx(0.5,abs=0.01)
    assert result["effective_sample_count"] is None
    trial=card._store.read("trials",result["trial_id"])
    assert "known_mass_kg" not in trial
    card.dispatch("estimate",{"profile_id":"left_A","known_mass_kg":0.5})
    assert card.info()["mass_kg"] is None
    assert not card.info()["valid"]


def test_tare_requires_confirmation_and_success_invalidates_model(card,profile,calibration):
    seed_card(card,profile,calibration)
    result=dispatch_wait(card,"tare",trial_note="empty")
    assert result["error_code"]=="CONFIRM_EMPTY"
    assert card._store.read("profiles","left_A")["model"]=="cal_test"
    result=dispatch_wait(card,"tare",confirm_empty=True,trial_note="independent unloaded fixture")
    assert result["state"]=="completed"
    assert card._store.read("profiles","left_A")["model"] is None


def test_expired_result_and_disconnect_publish_null(card,profile,calibration,monkeypatch):
    seed_card(card,profile,calibration)
    monkeypatch.setattr(card._sampler,"collect",lambda *a:frames(0.5))
    dispatch_wait(card,"estimate")
    card._last["completed_monotonic_s"]-=1000
    card._expire()
    assert card.published[-1]["error_code"]=="RESULT_EXPIRED"
    assert card.published[-1]["mass_kg"] is None
    dispatch_wait(card,"estimate")
    card._expire()  # no raw shared channel observations in this mocked fixture
    assert card.published[-1]["error_code"]=="STALE_SOURCE"


def test_config_changes_and_identity_are_not_shared_between_robots(card,profile,calibration):
    seed_card(card,profile,calibration)
    data=json.loads(Path(card._file).read_text())
    data["profiles"]["left_A"]["hook_id"]="moved_hook"
    Path(card._file).write_text(json.dumps(data))
    result=dispatch_wait(card,"estimate")
    assert result["error_code"]=="CONFIG_CHANGED"
    assert card.info()["profiles"]["left_A"]["calibration_id"] is None
    card._hardware_id="OTHER_ROBOT"
    assert dispatch_wait(card,"tare",confirm_empty=True,trial_note="empty")["error_code"]=="IDENTITY_MISMATCH"


def test_busy_stop_and_shutdown_do_not_resurrect_worker(card,monkeypatch):
    started=threading.Event()
    def collect(duration,p,cancel):
        started.set()
        assert cancel.wait(2)
        raise MassError("CANCELLED","cancelled")
    monkeypatch.setattr(card._sampler,"collect",collect)
    args={"profile_id":"left_A","confirm_empty":True,"trial_note":"independent empty"}
    run=card.dispatch("tare",args)
    assert started.wait(1)
    assert card.dispatch("tare",args)["error_code"]=="BUSY"
    assert card.dispatch("stop",{})["state"]=="stopping"
    card._job["thread"].join(1)
    assert card._last["state"]=="cancelled"
    assert card.notifications==[(run["action_id"],"cancelled",card._last)]
    assert not list((card._store.root/"trials").glob("*"))
    card.stop()
    assert card.dispatch("tare",args)["error_code"]=="CLOSED"


def test_acp_gate_and_failed_http_response(card):
    args={"profile_id":"left_A","confirm_empty":True,"trial_note":"independent empty","_await_response_publication":True}
    result=card.dispatch("tare",args)
    assert not card._job["publication"].is_set()
    assert card.notifications==[]
    card.response_published(result["action_id"],True)
    card.response_published(result["action_id"],True)
    card._job["thread"].join(1)
    assert len(card.notifications)==1
    result=card.dispatch("tare",args)
    card.response_published(result["action_id"],False)
    card._job["thread"].join(1)
    assert len(card.notifications)==1
    assert card._last["state"]=="cancelled"


def test_shutdown_releases_response_gate_without_callback(card):
    result=card.dispatch("tare",{"profile_id":"left_A","confirm_empty":True,"trial_note":"empty","_await_response_publication":True})
    assert result["state"]=="running"
    card.stop()
    assert not card._job["thread"].is_alive()
    assert card.notifications==[]


def test_immutable_trials_and_atomic_profile_integrity(tmp_path):
    store=Store(tmp_path)
    store.write("trials","trial_one",{"value":1},immutable=True)
    with pytest.raises(FileExistsError):
        store.write("trials","trial_one",{"value":2},immutable=True)
    store.write("profiles","left_A",{"value":1})
    store.write("profiles","left_A",{"value":2})
    assert store.read("profiles","left_A")=={"value":2}
    assert not list((tmp_path/"profiles").glob(".payload-*"))
    path=store._path("profiles","left_A")
    record=json.loads(path.read_text())
    record["data"]["value"]=99
    path.write_text(json.dumps(record))
    with pytest.raises(MassError) as exc:
        store.read("profiles","left_A")
    assert exc.value.code=="STORAGE_CORRUPT"


def test_static_modules_contain_no_control_calls():
    import ast
    forbidden={"publish_cmd","set_joint","init","enable","disable","move_arm","LowController"}
    for filename in ("payload_mass.py","payload_mass_sampling.py","payload_mass_estimator.py","payload_mass_cli.py"):
        tree=ast.parse((BUMI/filename).read_text())
        calls={n.func.attr if isinstance(n.func,ast.Attribute) else n.func.id if isinstance(n.func,ast.Name) else "" for n in ast.walk(tree) if isinstance(n,ast.Call)}
        assert not calls&forbidden


def test_profile_template_cannot_enable_hardware_measurement():
    raw=json.loads((BUMI/"resource/payload_mass_profiles.example.json").read_text())["profiles"]["left_hook_A"]
    with pytest.raises(MassError):
        validate_profile("left_hook_A",raw,"left",loading=True)


def test_full_card_workflow_restart_replay_and_failed_refit(card,monkeypatch):
    seed=[0]
    current_mass=[0.0]
    def collect(*args):
        seed[0]+=1
        return frames(current_mass[0],seed=seed[0])
    monkeypatch.setattr(card._sampler,"collect",collect)
    for i in range(2):
        assert dispatch_wait(card,"tare",confirm_empty=True,trial_note=f"independent empty {i}")["state"]=="completed"
    for role,masses in (("train",[0.3,0.3,0.7,0.7]),("validation",[0.3,0.7])):
        for i,m in enumerate(masses):
            current_mass[0]=m
            result=dispatch_wait(card,"calibrate",known_mass_kg=m,role=role,independent_trial_confirmed=True,trial_note=f"independent {role} {i}")
            assert result["state"]=="completed"
    fitted=dispatch_wait(card,"fit")
    assert fitted["state"]=="completed"
    current_mass[0]=0.5
    result=dispatch_wait(card,"estimate")
    assert result["valid"] and abs(result["mass_kg"]-0.5)<0.01
    offline=replay(card._store.root,"left_A",result["trial_id"])
    assert offline["mass_kg"]==result["mass_kg"] and offline["replay_only"]
    restarted=PayloadMassPlugin(card._cfg,"test",sampler=card._sampler,notify=lambda *a:None)
    try:
        assert restarted._store.read("profiles","left_A")["model"]==fitted["calibration_id"]
        measured=dispatch_wait(restarted,"estimate")
        assert measured["valid"] and abs(measured["mass_kg"]-0.5)<0.01
    finally:
        restarted.stop()
    # Fresh independent validation disagrees; a failed fit leaves the good pointer intact.
    current_mass[0]=0.6
    dispatch_wait(card,"calibrate",known_mass_kg=0.3,role="validation",independent_trial_confirmed=True,trial_note="bad independent holdout")
    result=dispatch_wait(card,"fit")
    assert result["error_code"]=="CALIBRATION_RESIDUAL"
    assert card._store.read("profiles","left_A")["model"]==fitted["calibration_id"]


def test_atomic_replace_failure_does_not_overwrite_old_pointer(tmp_path,monkeypatch):
    import payload_mass
    store=Store(tmp_path)
    store.write("profiles","left_A",{"old":True})
    def fail(*a):
        raise OSError("simulated disk/rename failure")
    monkeypatch.setattr(payload_mass.os,"replace",fail)
    with pytest.raises(OSError):
        store.write("profiles","left_A",{"old":False})
    assert store.read("profiles","left_A")=={"old":True}
    assert not list((tmp_path/"profiles").glob(".payload-*"))


def test_profile_changed_during_collection_cannot_commit(card,monkeypatch):
    def change_config(*a):
        data=json.loads(Path(card._file).read_text())
        data["profiles"]["left_A"]["hook_id"]="new_mount"
        Path(card._file).write_text(json.dumps(data))
        return frames()
    monkeypatch.setattr(card._sampler,"collect",change_config)
    result=dispatch_wait(card,"tare",confirm_empty=True,trial_note="empty")
    assert result["error_code"]=="CONFIG_CHANGED"
    assert not list((card._store.root/"trials").glob("*"))


def test_bundle_registers_shared_sampler_without_extra_controller(monkeypatch):
    # Real StatePlugin + real payload plugin; replace only ROS objects and SDK data.
    from test_mic_card import _load_device
    device,_=_load_device(monkeypatch)
    fake_executors=SimpleNamespace()
    monkeypatch.setitem(sys.modules,"rclpy.executors",fake_executors)
    sys.modules["rclpy"].executors=fake_executors
    monkeypatch.setitem(sys.modules,"device",device)
    executor=SimpleNamespace(add_node=lambda n:None,remove_node=lambda n:None)
    module=SimpleNamespace()
    source=(BUMI/"main.py").read_text()
    namespace={"__name__":"payload_bundle_test","__file__":str(BUMI/"main.py")}
    exec(compile(source,str(BUMI/"main.py"),"exec"),namespace)
    # HighController object has no writes or init; bundle receives it as-is.
    bundle=namespace["BumiDeviceBundle"]({"plugins":{"state":{"enabled":True},"payload_mass":{"enabled":True}}},"test",executor,object(),None)
    state,plugin=bundle._plugins
    assert state._node.payload_sampler is plugin._sampler
    assert "payload_mass" in {t["name"] for t in bundle.get_all_tools()}
    assert plugin.info()["diagnostics"]["received_count"]==0
    # Fake Node lacks destroy_node; real destruction is separately exercised below.
    plugin._node=None
    plugin.stop()


def test_ros_node_shutdown_destroys_and_detaches(monkeypatch):
    from test_mic_card import _load_device
    _load_device(monkeypatch)
    detached=[]
    card=PayloadMassPlugin({},"test",SimpleNamespace(add_node=lambda n:None,remove_node=detached.append))
    destroyed=[]
    node=card._node
    node.destroy_node=lambda:destroyed.append(True)
    card.stop()
    card.stop()
    assert detached==[node] and destroyed==[True]


def test_http_mcp_discovers_card_and_completion_follows_response(card,monkeypatch):
    from http.server import ThreadingHTTPServer
    import urllib.request
    from test_mic_card import _load_device
    _load_device(monkeypatch)
    monkeypatch.setitem(sys.modules,"rclpy.executors",SimpleNamespace())
    sys.modules["rclpy"].executors=sys.modules["rclpy.executors"]
    namespace={"__name__":"payload_http_test","__file__":str(BUMI/"main.py")}
    exec(compile((BUMI/"main.py").read_text(),str(BUMI/"main.py"),"exec"),namespace)
    bundle=namespace["BumiDeviceBundle"]({"plugins":{}},"test",object(),None,None)
    bundle._plugins=[card]
    namespace["_bundle"]=bundle
    server=ThreadingHTTPServer(("127.0.0.1",0),namespace["make_handler"]())
    thread=threading.Thread(target=server.serve_forever)
    thread.start()
    url=f"http://127.0.0.1:{server.server_port}/mcp"
    def request(method,params):
        req=urllib.request.Request(url,data=json.dumps({"jsonrpc":"2.0","id":1,"method":method,"params":params}).encode(),headers={"Content-Type":"application/json"})
        with urllib.request.urlopen(req,timeout=2) as response:
            return json.load(response)
    try:
        listed=request("tools/list",{})
        assert listed["result"]["tools"][0]["name"]=="payload_mass"
        running=request("tools/call",{"name":"payload_mass","arguments":{"action":"tare","profile_id":"left_A","confirm_empty":True,"trial_note":"independent empty"}})
        payload=json.loads(running["result"]["content"][0]["text"])
        assert payload["state"]=="running" and "action_id" in payload
        card._job["thread"].join(1)
        assert len(card.notifications)==1
        assert card.notifications[0][0]==payload["action_id"]
        assert card._job["publication"].is_set()
    finally:
        server.shutdown()
        thread.join(2)
        server.server_close()


def test_source_timestamp_only_is_deduplicated(profile):
    sampler=SharedSampler()
    frame=synthetic_frame(0,None)
    frame["source_timestamp"]=123.0
    assert sampler.push(frame)
    assert not sampler.push(frame)
    assert sampler.diagnostics()["freshness_status"]=="source_timestamp"
    check_frame(sampler.diagnostics()["latest"],profile)


def test_observed_load_change_invalidates_display(card,profile,calibration,monkeypatch):
    seed_card(card,profile,calibration)
    monkeypatch.setattr(card._sampler,"collect",lambda *a:frames(0.5))
    dispatch_wait(card,"estimate")
    card._sampler.push(synthetic_frame(0.5,1))
    card._expire()
    assert card._last["valid"]
    card._sampler.push(synthetic_frame(0.6,2))
    card._expire()
    assert card._last["mass_kg"] is None
    assert card._last["error_code"]=="CONDITIONS_CHANGED"


@pytest.mark.parametrize("value",[0,-1,31,float("nan"),float("inf"),True])
def test_bad_sampling_duration(card,value):
    result=dispatch_wait(card,"tare",confirm_empty=True,trial_note="empty",sample_duration_s=value)
    assert result["error_code"]=="BAD_PARAMETER"
    assert result["mass_kg"] is None


def test_non_monotonic_load_response_rejected(profile,calibration):
    baseline,trials,_=copy.deepcopy(calibration)
    for trial in trials:
        actual=0.6 if trial["known_mass_kg"]==0.3 else 0.2
        trial["summary"]=summarize(frames(actual),profile)
    with pytest.raises(MassError) as exc:
        fit_model(trials,baseline,profile)
    assert exc.value.code=="NON_MONOTONIC"


def test_duplicate_whole_trial_id_rejected(profile,calibration):
    baseline,trials,_=copy.deepcopy(calibration)
    trials[1]["id"]=trials[0]["id"]
    with pytest.raises(MassError) as exc:
        fit_model(trials,baseline,profile)
    assert exc.value.code=="TRIAL_LEAKAGE"


def test_schema_is_valid_draft7_and_truth_is_not_required_for_estimate(card):
    import jsonschema
    tool=card.get_tool()
    jsonschema.Draft7Validator.check_schema(tool["inputSchema"])
    jsonschema.Draft7Validator.check_schema(tool["configSchema"])
    jsonschema.validate({"action":"estimate","arm":"left","mode":"static","profile_id":"left_A","sample_duration_s":5},tool["inputSchema"])
