"""Optional adapter for the existing WSL Gazebo preview, not the robot image."""

import math
import time

from detaforce_motion import ARM_JOINTS, INVITE_TEXT, LABELS, build_motion
from detaforce_rps import TOOL, tool_spec


SPEC = tool_spec(simulation=True)


def call(simulator, arguments, *, invite_speech=False):
    if not isinstance(arguments, dict) or set(arguments) != {"action"}:
        raise ValueError("DetaForce basic actions accept only action")
    action = arguments["action"]
    if action in ("info", "start"):
        return {"state": "ready", "tool": TOOL, "simulation": True,
                "physical_interaction_validated": False,
                "actions": LABELS, "preview": "/detaforce",
                "tts_composed_by_preview": True}
    if action == "status":
        status = simulator.status()
        return status if status.get("action") == "rps_motion" else {
            "state": "idle", "simulation": True, "tool": TOOL}
    if action == "stop_action":
        with simulator.lock:
            if simulator.report.get("action") != "rps_motion":
                return {"state": "idle", "command_sent": False}
            return simulator.call({"action": "stop"})
    if action not in LABELS:
        raise ValueError("Unknown DetaForce action")
    return simulator.schedule("rps_motion", {"gesture": action,
                                            "invite_speech": bool(invite_speech)})


def run_motion(simulator, arguments):
    action = arguments["gesture"]
    motion = build_motion(action)
    base = simulator.base[:]
    if simulator.visitor_hands or any(abs(v) > 1e-4 for v in simulator.joints.values()):
        raise ValueError("Reset to standing before playing an arm-only clip")
    # Use a front three-quarter view so elbow bends and hand depth are visible.
    from bumi_model import transform
    camera_x, camera_y = 1.6, -1.2
    yaw = base[2]
    x = base[0]+math.cos(yaw)*camera_x-math.sin(yaw)*camera_y
    y = base[1]+math.sin(yaw)*camera_x+math.cos(yaw)*camera_y
    simulator.model.observer_override = transform(
        (x, y, 1.05), (0, math.atan2(.60, math.hypot(camera_x, camera_y)),
                      math.atan2(base[1]-y, base[0]-x)))
    with simulator.lock:
        simulator.report["rps"] = {
            "gesture": action, "label": LABELS[action], "tool": TOOL,
            "planned_duration_s": motion.duration_s, "sample_count": 0,
            "base_stationary": True, "legs_stationary": True,
            "simulation_only": True, "dynamics_validated": False,
            "physical_interaction_validated": False,
        }
    speech = None
    audio = None
    if action.startswith("invite") and arguments.get("invite_speech"):
        from local_speech import LocalSpeech
        speech = LocalSpeech(simulator.directory, phrases=(INVITE_TEXT,))
        audio = speech.prepare(INVITE_TEXT)
    simulator.event("rps_started", gesture=action, label=LABELS[action],
                    planned_duration_s=motion.duration_s)
    started = time.monotonic()
    steps = math.ceil(motion.duration_s/.10)
    previous_phase = None
    samples = 0
    next_capture_s = 0.0
    for index in range(steps+1):
        step_started = time.monotonic()
        simulator.check_cancel()
        elapsed = min(motion.duration_s, index*motion.duration_s/steps)
        joints, phase = motion.sample(elapsed)
        stamp = simulator.apply_pose(base[:], joints)
        if simulator.base != base or any(name not in ARM_JOINTS for name in joints):
            raise RuntimeError("Arm-only motion changed the robot base or legs")
        samples += 1
        if phase != previous_phase or elapsed >= next_capture_s or index == steps:
            simulator.event("rps_pose", gesture=action, pose_phase=phase,
                            clip_time_s=elapsed, joints_rad=joints,
                            feedback_stamp_ns=stamp, model_feedback_verified=True)
            previous_phase = phase
            next_capture_s = elapsed+.8
        if index < steps:
            simulator.cancel.wait(max(0, step_started+motion.duration_s/steps-time.monotonic()))
    with simulator.lock:
        simulator.report["rps"].update(sample_count=samples,
                                        pose_feedback_verified=True,
                                        actual_duration_s=time.monotonic()-started)
    if speech:
        path, duration = audio
        simulator.event("rps_invite_speech", text=INVITE_TEXT,
                        audio="/audio/"+path.name, audio_played=False,
                        duration_s=duration, composed_with_tts=True)
        speech.play(path, simulator.cancel)
        with simulator.lock:
            simulator.report["events"][-1].update(audio_played=True, playback_state="completed")
            simulator.report["actual_speech"] = True
    simulator.event("rps_completed", gesture=action, model_feedback_verified=True,
                    base_stationary=True, legs_stationary=True)
