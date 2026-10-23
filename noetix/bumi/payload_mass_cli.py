#!/usr/bin/env python3
"""SDK-free mock, immutable trial replay, and read-only live MCP diagnostics."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import random
import tempfile
import threading
import time
import urllib.request

from payload_mass import PayloadMassPlugin, Store
from payload_mass_estimator import validate_profile, summarize, estimate, digest
from payload_mass_sampling import SharedSampler, check_frame


def synthetic_profile():
    """Simulation values ONLY. Not robot poses, safety limits or accuracy claims."""
    return {"arm":"left","motor_ids":[0,1,2,3],
            "joint_names":["l_arm_pitch_joint","l_arm_roll_joint","l_arm_yaw_joint","l_elbow_pitch_joint"],
            "hardware_id":"SIMULATION_ONLY","sdk_version":"synthetic/v1","hook_id":"sim_hook",
            "control_config":"synthetic_static","mapping_evidence":"synthetic array, not a hardware verification",
            "mapping_verified":True,"load_range_confirmed":True,"feedback_verified":True,
            "feedback_evidence":"known synthetic linear response","torque_units":"sdk_raw",
            "accept_host_only_freshness":False,"pose_q":[0.0]*21,"quaternion_xyzw":[0,0,0,1],"workmode":2,
            "pose_tolerance_rad":0.02,"body_tolerance_rad":0.02,"max_velocity_rad_s":0.02,"max_gyro_rad_s":0.02,
            "max_torque_span":0.04,"torque_abs_limit":20,"max_temperature_c":60,"max_temperature_drift_c":3,
            "max_mass_kg":1,"noise_floor_raw":0.005,"max_calibration_error_kg":0.03,
            "settle_timeout_s":0.5,"min_samples":20,"max_receive_gap_s":0.2}


def synthetic_frame(mass=0.0, seq=None, rng=None):
    rng=rng or random.Random(42)
    # Fixture/arm contribution is included in tare; signed load responses differ.
    tau=[1.0,-0.5,0.2,0.7]+[0.0]*17
    for j,k in enumerate([2.0,-1.0,0.001,0.6]):
        tau[j]+=k*mass+rng.uniform(-0.003,0.003)
    return {"motor_ids":list(range(21)),"q":[0.0]*21,"dq":[0.0]*21,"tau":tau,
            "temperature":[30.0]*21,"errors":[0]*21,"quaternion_xyzw":[0,0,0,1],"gyro":[0,0,0],
            "workmode":2,"source_seq":seq,"source_timestamp":None,"adapter":"SYNTHETIC_ONLY"}


def run_mock(root):
    root=Path(root)
    root.mkdir(parents=True,exist_ok=True)
    engineering=root/"engineering_profiles.json"
    engineering.write_text(json.dumps({"profiles":{"sim_left":synthetic_profile()}}))
    sampler=SharedSampler()
    notifications=[]
    card=PayloadMassPlugin({"data_dir":str(root),"profiles_file":str(engineering),
                            "hardware_id":"SIMULATION_ONLY","sdk_version":"synthetic/v1"},"sim",
                           sampler=sampler,notify=lambda *args:notifications.append(args))
    rng=random.Random(7)
    def task(action,mass,**extra):
        halted=threading.Event()
        # Synthetic getter-only observations require explicit policy, just like
        # hardware. Use source_seq instead here so old-frame tests stay separate.
        def sequenced_feed():
            while not halted.is_set():
                frame=synthetic_frame(mass,sampler._serial+1,rng)
                sampler.push(frame)
                halted.wait(0.01)
        thread=threading.Thread(target=sequenced_feed,name="synthetic_feedback")
        thread.start()
        try:
            args={"arm":"left","profile_id":"sim_left","mode":"static",**extra}
            if action!="fit":
                args["sample_duration_s"]=1
            result=card.dispatch(action,args)
            if result["state"]=="running":
                card._job["thread"].join(8)
                result=card.info()
            if result["state"]!="completed":
                raise RuntimeError(json.dumps(result))
            return result
        finally:
            halted.set()
            thread.join(1)
    try:
        for i in range(2):
            task("tare",0,confirm_empty=True,trial_note=f"synthetic independent empty {i}")
        for role,masses in (("train",[0.3,0.3,0.7,0.7]),("validation",[0.3,0.7])):
            for i,mass in enumerate(masses):
                task("calibrate",mass,known_mass_kg=mass,role=role,
                     independent_trial_confirmed=True,trial_note=f"synthetic {role} {i}")
        fitted=task("fit",0)
        measured=task("estimate",0.5)  # Ground truth only selects generated feedback, never tool args.
        if not measured["valid"] or abs(measured["mass_kg"]-0.5)>0.03:
            raise RuntimeError("synthetic recovery failed")
        return {"simulation_only":True,"mass_kg":measured["mass_kg"],"valid":True,
                "calibration_id":fitted["calibration_id"],"trial_id":measured["trial_id"],
                "validation":fitted["validation"],"terminal_notifications":len(notifications),
                "data_dir":str(root),"note":"Synthetic recovery is not Bumi hardware accuracy."}
    finally:
        card.stop()


def replay(root,profile_id,trial_id):
    store=Store(root)
    trial=store.read("trials",trial_id)
    raw=dict(trial["profile"])
    raw.pop("fingerprint",None)
    profile=validate_profile(profile_id,raw,raw["arm"],loading=True)
    if profile["fingerprint"]!=trial["profile_fingerprint"]:
        raise ValueError("trial profile fingerprint mismatch")
    for frame in trial["raw_frames"]:
        check_frame(frame,profile)
    model=store.read("calibrations",trial["calibration_id"])
    baseline=store.read("profiles",model["baseline_id"])
    result=estimate(summarize(trial["raw_frames"],profile),baseline,model,profile,trial["created_s"])
    result.update(replay_only=True,trial_id=trial_id)
    return result


def diagnose(url,seconds,interval):
    """Only tools/call(payload_mass, info); never imports/initializes SDK."""
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        body=json.dumps({"jsonrpc":"2.0","id":1,"method":"tools/call",
                         "params":{"name":"payload_mass","arguments":{"action":"info"}}}).encode()
        request=urllib.request.Request(url,data=body,headers={"Content-Type":"application/json"})
        with urllib.request.urlopen(request,timeout=3) as response:
            result=json.load(response)
        if "error" in result:
            raise RuntimeError(result["error"])
        payload=json.loads(result["result"]["content"][0]["text"])
        print(json.dumps({"host_time_s":time.time(),"diagnostics":payload["diagnostics"],
                          "deployment_identity":payload["deployment_identity"]},allow_nan=False),flush=True)
        time.sleep(min(interval,max(0,end-time.monotonic())))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest="command",required=True)
    mock=sub.add_parser("mock")
    mock.add_argument("--output-dir")
    live=sub.add_parser("diagnose",help="read-only MCP info, no SDK initialization or motor writes")
    live.add_argument("--url",default="http://127.0.0.1:15704/mcp")
    live.add_argument("--seconds",type=float,default=10)
    live.add_argument("--interval",type=float,default=0.2)
    replay_parser=sub.add_parser("replay")
    replay_parser.add_argument("--data-dir",required=True)
    replay_parser.add_argument("--profile-id",required=True)
    replay_parser.add_argument("--trial-id",required=True)
    args=parser.parse_args()
    if args.command=="mock":
        if args.output_dir:
            print(json.dumps(run_mock(args.output_dir),ensure_ascii=False,indent=2))
        else:
            with tempfile.TemporaryDirectory(prefix="bumi-payload-mock-") as root:
                print(json.dumps(run_mock(root),ensure_ascii=False,indent=2))
    elif args.command=="diagnose":
        if not 0<args.seconds<=120 or not 0.05<=args.interval<=5:
            parser.error("seconds must be (0,120], interval [0.05,5]")
        diagnose(args.url,args.seconds,args.interval)
    else:
        print(json.dumps(replay(args.data_dir,args.profile_id,args.trial_id),ensure_ascii=False,indent=2))


if __name__=="__main__":
    main()
