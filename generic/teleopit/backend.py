"""Card-controlled adapter for the pinned Teleopit 0.5.0 simulation pipeline.

No imitation policy, IK solver or physics loop is implemented here. The real
``TeleopPipeline.run`` owns all of them. Hooks at its existing message-bus and
policy-step seams provide measurements, cancellation and card controls. This
module deliberately has no dependency on Teleopit's sim2real / DDS bridge.
Heavy dependencies are loaded only inside the isolated worker's run call.
"""

from __future__ import annotations

import dataclasses
import importlib
import io
import math
from pathlib import Path
import sys
import time
from typing import Any, Callable


UPSTREAM_VERSION = "0.5.0"
UPSTREAM_COMMIT = "f9263865c581802ad531854b8e547e2403a945f3"
JOINT_COUNT = 29
POLICY_HZ = 50.0
STALE_AFTER_S = 0.5


class _Stopped(Exception):
    """Local cancellation; unwinds through Teleopit's resource cleanup."""


def _number(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _vector(value: Any, size: int, name: str) -> list[float]:
    """Reject wrong-size/nonfinite data, never pad/trim an upstream output."""
    if hasattr(value, "shape") and tuple(value.shape) != (size,):
        raise ValueError(f"{name} must have shape ({size},), got {value.shape}")
    try:
        result = [_number(item, name) for item in value]
    except (TypeError, OverflowError) as exc:
        raise ValueError(f"{name} must contain {size} finite numbers") from exc
    if len(result) != size:
        raise ValueError(f"{name} has {len(result)} entries, expected {size}")
    return result


class _Gate:
    def __init__(self, stop_event: Any, pause_event: Any,
                 emit: Callable[[dict], None], *, clock: Callable[[], float] = time.monotonic) -> None:
        self.stop_event = stop_event
        self.pause_event = pause_event
        self.emit = emit
        self.clock = clock
        self.cycle_start = clock()

    def check(self) -> bool:
        """Wait without advancing physics; return whether a pause occurred."""
        if self.stop_event.is_set():
            raise _Stopped()
        paused = self.pause_event.is_set()
        if paused:
            self.emit({"event": "paused"})
            while self.pause_event.is_set():
                if self.stop_event.wait(0.05):
                    raise _Stopped()
            if self.stop_event.is_set():
                raise _Stopped()
            self.emit({"event": "resumed"})
            self.cycle_start = self.clock()
        return paused

    def next_step(self) -> None:
        paused = self.check()
        # Pace relative to the last actual cycle, never to accumulated simulated
        # time: a slow IK step or card pause must not cause a catch-up burst.
        if not paused:
            remaining = self.cycle_start + 1.0 / POLICY_HZ - self.clock()
            if remaining > 0 and self.stop_event.wait(remaining):
                raise _Stopped()
        self.check()
        self.cycle_start = self.clock()


class _PicoInput:
    """Observe upstream packets, strip headset controls, expose input freshness."""

    def __init__(self, provider: Any, gate: _Gate, timeout_s: float) -> None:
        self.provider = provider
        self.gate = gate
        self.timeout_s = timeout_s
        self.timestamp: float | None = None
        self.seq: int | None = None
        self._get_packet = provider.get_realtime_input_packet

    def read(self) -> Any:
        deadline = self.gate.clock() + self.timeout_s
        while not self.provider.has_frame():
            if self.gate.check():
                deadline = self.gate.clock() + self.timeout_s
            if not self.provider.is_available():
                raise RuntimeError("PICO receiver stopped before a body-tracking frame arrived")
            if self.gate.clock() >= deadline:
                raise TimeoutError("No PICO full-body tracking data received; enable body tracking and connect pico-bridge")
            if self.gate.stop_event.wait(0.02):
                raise _Stopped()
        self.gate.check()
        packet = self._get_packet()
        self.timestamp = _number(packet.timestamp_s, "PICO packet timestamp")
        self.seq = int(packet.seq)
        age = max(0.0, self.gate.clock() - self.timestamp)
        if age > self.timeout_s:
            raise TimeoutError(f"PICO body-tracking input stopped updating for {age:.1f}s")
        # v0.5.0's factory does not forward arms_button=None. Strip ALL controller
        # events at the packet seam so card state remains the single owner of
        # pause/resume and headset buttons cannot silently change simulation mode.
        return dataclasses.replace(packet, control_events=())

    def freshness(self) -> dict[str, Any]:
        age = None if self.timestamp is None else max(0.0, self.gate.clock() - self.timestamp)
        return {
            "input_sequence": self.seq,
            "input_age_ms": None if age is None else round(age * 1000.0, 3),
            "input_stale": age is None or age > STALE_AFTER_S,
        }


def _make_pipeline(options: dict[str, Any]) -> Any:
    """Load the installed, pinned upstream source and its own Hydra configs."""
    root = Path(options["upstream_root"]).resolve()
    package = root / "teleopit"
    if not (package / "pipeline.py").is_file():
        raise FileNotFoundError(f"Teleopit source is missing under {root}; run bootstrap first")
    sys.path.insert(0, str(root))
    upstream = importlib.import_module("teleopit")
    actual = Path(upstream.__file__).resolve().parent
    if actual != package:
        raise RuntimeError(f"Wrong Teleopit import: {actual}; expected pinned source at {package}")

    from hydra import compose, initialize_config_dir
    from omegaconf import open_dict
    from teleopit.pipeline import TeleopPipeline
    from teleopit.runtime.console import PlainConsole

    source = options["source"]
    if source not in ("bvh", "pico"):
        raise ValueError("source must be bvh or pico")
    with initialize_config_dir(config_dir=str(package / "configs"), version_base=None):
        cfg = compose(config_name="pico4_sim" if source == "pico" else "default")
    with open_dict(cfg):
        cfg.viewers = "none"
        cfg.realtime = False
        cfg.keyboard = {"enabled": False}
        cfg.playback = {"pause_on_end": False, "keyboard": {"enabled": False}}
        cfg.debug_trace_path = None
        cfg.policy_hz = POLICY_HZ
        cfg.pd_hz = 200.0
        cfg.controller.policy_path = str(Path(options["policy_path"]).resolve())
        cfg.controller.device = "cpu"
        cfg.input.human_height = float(options.get("human_height", 1.75))
        if source == "bvh":
            cfg.input.bvh_file = str(Path(options["bvh_path"]).resolve())
        else:
            cfg.input.pico4_timeout = float(options.get("input_timeout_s", 10.0))
            cfg.input.bridge_start_timeout = float(options.get("input_timeout_s", 10.0))
            cfg.input.bridge_host = options.get("pico_host", "0.0.0.0")
            cfg.input.bridge_port = int(options.get("pico_port", 63901))
            cfg.input.bridge_advertise_ip = options.get("pico_advertise_host") or None
            cfg.input.pause_button = None
            cfg.input.arms_button = None
            cfg.input.video.enabled = False
    return TeleopPipeline(cfg, console=PlainConsole(title="Teleopit Driver", enabled=False))


def _joint_names(robot: Any) -> list[str]:
    model = robot.model
    by_address = {}
    for i in range(model.njnt):
        address = int(model.jnt_qposadr[i])
        if 7 <= address < 7 + JOINT_COUNT:
            by_address[address] = str(model.joint(i).name)
    names = [by_address.get(address, "") for address in range(7, 7 + JOINT_COUNT)]
    if len(set(names)) != JOINT_COUNT or any(not name for name in names):
        raise ValueError("Teleopit robot model must expose 29 named scalar joints in qpos order")
    return names


class _Preview:
    """Optional 5 Hz view; missing EGL/OpenGL is reported, never hidden."""

    def __init__(self, robot: Any, enabled: bool, emit: Callable[[dict], None]) -> None:
        self.robot = robot
        self.emit = emit
        self.renderer = None
        self.enabled = enabled
        self.next_at = 0.0
        self.camera = None

    def image(self) -> bytes | None:
        now = time.monotonic()
        if not self.enabled or now < self.next_at:
            return None
        self.next_at = now + 0.2
        try:
            import mujoco
            from PIL import Image

            if self.renderer is None:
                self.renderer = mujoco.Renderer(self.robot.model, height=480, width=640)
                self.camera = mujoco.MjvCamera()
                self.camera.distance = 3.0
                self.camera.azimuth = 135
                self.camera.elevation = -20
            self.camera.lookat[:] = self.robot.data.qpos[:3]
            self.renderer.update_scene(self.robot.data, camera=self.camera)
            data = io.BytesIO()
            Image.fromarray(self.renderer.render()).save(data, format="JPEG", quality=75)
            return data.getvalue()
        except Exception as exc:
            self.enabled = False
            self.close()
            self.emit({"event": "diagnostic", "code": "preview_unavailable", "message": str(exc)})
            return None

    def close(self) -> None:
        renderer, self.renderer = self.renderer, None
        if renderer is not None:
            try:
                renderer.close()
            except Exception as exc:
                self.emit({"event": "diagnostic", "code": "preview_close_failed", "message": str(exc)})


def run_backend(options: dict, emit: Callable[[dict], None], stop_event: Any,
                pause_event: Any) -> None:
    """Run real Teleopit until completed/stopped; errors propagate to supervisor.

    ``event=ready`` means the simulation pipeline exists, not that PICO body data
    has arrived. Only ``event=frame`` proves a real GMR/ONNX/MuJoCo step. The
    optional JPEG is a bytes sibling of the finite-JSON snapshot. Computation
    measurements exclude pacing/input waiting and are NOT end-to-end latency.
    """
    gate = _Gate(stop_event, pause_event, emit)
    pipeline = None
    preview = None
    steps = 0
    compute_sum_ms = 0.0
    compute_max_ms = 0.0
    last_snapshot: dict = {}
    reason = "completed"
    try:
        gate.check()
        pipeline = _make_pipeline(options)
        robot = pipeline.robot
        if int(robot.num_actions) != JOINT_COUNT or int(robot.model.nq) != 7 + JOINT_COUNT:
            raise ValueError("Only the upstream G1 29DoF MuJoCo model is supported")
        joint_names = _joint_names(robot)
        target = _vector(robot.default_dof_pos, JOINT_COUNT, "default joint positions")
        stage_ms = {"retarget_ms": 0.0, "observation_ms": 0.0, "policy_ms": 0.0, "physics_ms": 0.0}

        def timed(owner: Any, name: str, key: str, size: int | None = None) -> None:
            original = getattr(owner, name)

            def call(*args: Any, **kwargs: Any) -> Any:
                start = time.perf_counter()
                result = original(*args, **kwargs)
                stage_ms[key] += (time.perf_counter() - start) * 1000.0
                if size is not None:
                    _vector(result, size, name)
                return result

            setattr(owner, name, call)

        runner = pipeline.loop._step_runner
        timed(pipeline.retargeter, "retarget", "retarget_ms", 7 + JOINT_COUNT)
        observation_size = int(getattr(pipeline.controller, "_expected_obs_dim", 167) or 167)
        timed(runner, "build_observation", "observation_ms", observation_size)
        timed(pipeline.controller, "compute_action", "policy_ms", JOINT_COUNT)
        timed(runner, "apply_control", "physics_ms")
        original_target = runner.compute_target_dof_pos

        def capture_target(action: Any) -> Any:
            nonlocal target
            result = original_target(action)
            target = _vector(result, JOINT_COUNT, "target joint positions")
            return result

        runner.compute_target_dof_pos = capture_target
        pico = None
        if options["source"] == "pico":
            pico = _PicoInput(pipeline.input_provider, gate, float(options.get("input_timeout_s", 10.0)))
            pipeline.input_provider.get_realtime_input_packet = pico.read
        preview = _Preview(robot, bool(options.get("render", False)), emit)

        def observe(state: Any) -> None:
            nonlocal steps, last_snapshot, compute_sum_ms, compute_max_ms
            steps += 1
            compute_ms = sum(stage_ms.values())
            compute_sum_ms += compute_ms
            compute_max_ms = max(compute_max_ms, compute_ms)
            last_snapshot = {
                "step": steps,
                "source": options["source"],
                "mode": "simulation",
                "robot_profile": "unitree_g1_29dof",
                "sim_time_s": _number(state.timestamp, "simulation time"),
                "policy_hz": POLICY_HZ,
                "step_compute_ms": round(compute_ms, 3),
                "target_positions": list(target),
                "joint_positions": _vector(state.qpos, JOINT_COUNT, "joint positions"),
                "joint_names": joint_names,
                "root_position": _vector(state.base_pos, 3, "root position"),
                "root_orientation_wxyz": _vector(state.quat, 4, "root orientation"),
                "input_age_ms": None,
                "input_stale": False,
                "hardware_output": False,
                **{key: round(value, 3) for key, value in stage_ms.items()},
            }
            if pico is not None:
                last_snapshot.update(pico.freshness())
            event = {"event": "frame", "snapshot": last_snapshot}
            jpeg = preview.image()
            if jpeg is not None:
                event["preview_jpeg"] = jpeg
            emit(event)
            for key in stage_ms:
                stage_ms[key] = 0.0
            gate.next_step()

        from teleopit.bus.topics import TOPIC_ROBOT_STATE

        pipeline.bus.subscribe(TOPIC_ROBOT_STATE, observe)
        # Heavy model loading may have outlived a stop request. Do not publish a
        # late "ready" transition for a session that has already been cancelled.
        gate.check()
        emit({
            "event": "ready", "upstream_version": UPSTREAM_VERSION,
            "upstream_commit": UPSTREAM_COMMIT, "robot_profile": "unitree_g1_29dof",
            "source": options["source"], "joint_names": joint_names,
            "policy_hz": POLICY_HZ, "hardware_output": False,
            "waiting_for_input": options["source"] == "pico",
        })
        gate.check()
        gate.cycle_start = gate.clock()
        default_steps = 0 if options["source"] == "pico" else 500
        pipeline.run(num_steps=int(options.get("max_steps", default_steps)))
    except _Stopped:
        reason = "stopped"
    finally:
        if preview is not None:
            preview.close()
        if pipeline is not None:
            close = getattr(pipeline.input_provider, "close", None)
            if callable(close):
                close()
    emit({
        "event": "complete",
        "summary": {
            "reason": reason, "steps": steps,
            "sim_time_s": last_snapshot.get("sim_time_s", 0.0),
            "mean_step_compute_ms": round(compute_sum_ms / max(steps, 1), 3),
            "max_step_compute_ms": round(compute_max_ms, 3),
            "hardware_output": False,
        },
    })
