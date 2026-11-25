"""Watch7 telemetry v2: bounded, typed, and distinct from v1/Agent events.

v2 carries small sensor summaries and device status only. Raw PPG/ECG/BIA
waveform arrays are intentionally not accepted by this endpoint.
"""
from __future__ import annotations

import math
import time
from typing import Any

V2_VERSION = 2
METRICS_TOPIC = "/wearable/watch7/v2/metrics"
DEVICE_TOPIC = "/wearable/watch7/v2/device"
RECORDINGS_TOPIC = "/wearable/watch7/v2/recordings"

KINDS = {"summary", "device_state", "signal_manifest"}
SOURCES = {"watch7_real", "simulated"}
WEAR_STATES = {"on_body", "off_body", "unknown"}
QUALITIES = {"valid", "unavailable", "off_body", "low_confidence", "unknown"}
PROVIDERS = {"health_services", "android_sensor", "samsung_sdk", "system"}
METRIC_UNITS = {
    "heart_rate_bpm": "bpm",
    "motion_score": "m/s2",
    "accel_m_s2": "m/s2",
    "gyro_rad_s": "rad/s",
    "gyro_uncalibrated_rad_s": "rad/s",
    "magnetic_field_uT": "uT",
    "rotation_vector": "unitless",
    "game_rotation_vector": "unitless",
    "linear_accel_m_s2": "m/s2",
    "gravity_m_s2": "m/s2",
    "geomagnetic_rotation_vector": "unitless",
    "ibi_ms": "ms",
    "skin_temperature_c": "degC",
    "spo2_pct": "%",
    "pressure_hpa": "hPa",
    "light_lux": "lx",
    "step_count": "count",
}
VECTORS = {
    "accel_m_s2", "gyro_rad_s", "gyro_uncalibrated_rad_s",
    "magnetic_field_uT", "rotation_vector", "game_rotation_vector",
    "linear_accel_m_s2", "gravity_m_s2", "geomagnetic_rotation_vector",
}
VECTOR_LENGTHS = {
    "accel_m_s2": {3}, "gyro_rad_s": {3},
    "gyro_uncalibrated_rad_s": {3, 6}, "magnetic_field_uT": {3},
    "rotation_vector": {3, 4, 5}, "game_rotation_vector": {3, 4, 5},
    "linear_accel_m_s2": {3}, "gravity_m_s2": {3},
    "geomagnetic_rotation_vector": {3, 4, 5},
}
RAW_SIGNALS = {"ppg_green", "ppg_red", "ppg_ir", "ecg"}
MAX_AGE_MS = 30_000
MAX_FUTURE_MS = 60_000


def _bounded(value: Any, field: str, low: float, high: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be a finite number")
    number = float(value)
    if not math.isfinite(number) or not low <= number <= high:
        raise ValueError(f"{field} out of bounds")
    return number


def _identifier(value: Any, field: str, maximum: int = 96) -> str:
    if not isinstance(value, str) or not 1 <= len(value) <= maximum or not value.isascii():
        raise ValueError(f"invalid {field}")
    if not all(char.isalnum() or char in "._:-" for char in value):
        raise ValueError(f"invalid {field}")
    return value


def validate_v2(raw: Any, *, now_ms: int | None = None) -> tuple[str, dict]:
    """Return (ROS topic, cleaned frame). No raw waveform samples leave this API."""
    if not isinstance(raw, dict):
        raise ValueError("frame must be an object")
    if any(key in raw for key in ("samples", "data", "waveform", "raw_payload")):
        raise ValueError("raw waveform payloads are prohibited")
    if raw.get("protocol_version") != V2_VERSION or raw.get("device_id") != "watch7":
        raise ValueError("unsupported version/device")
    source = raw.get("source")
    kind = raw.get("kind")
    wear = raw.get("wear_state")
    if source not in SOURCES or kind not in KINDS or wear not in WEAR_STATES:
        raise ValueError("invalid source/kind/wear_state")
    session = _identifier(raw.get("session_id"), "session_id", 64)
    seq = raw.get("seq")
    ts = raw.get("captured_at_ms")
    if type(seq) is not int or not 0 <= seq < 2**63:
        raise ValueError("invalid seq")
    if type(ts) is not int or ts < 0:
        raise ValueError("invalid timestamp")
    now = int(time.time() * 1000) if now_ms is None else now_ms
    if not now - MAX_AGE_MS <= ts <= now + MAX_FUTURE_MS:
        raise ValueError("stale_or_future_sample")

    cleaned = {
        "protocol_version": V2_VERSION, "device_id": "watch7",
        "session_id": session, "seq": seq, "source": source,
        "kind": kind, "captured_at_ms": ts, "wear_state": wear,
    }
    if kind == "summary":
        metrics = raw.get("metrics")
        if not isinstance(metrics, dict) or not 1 <= len(metrics) <= 16:
            raise ValueError("metrics must have 1..16 entries")
        clean_metrics = {}
        for name, record in metrics.items():
            if name not in METRIC_UNITS or not isinstance(record, dict):
                raise ValueError("unknown or malformed metric")
            if record.get("unit") != METRIC_UNITS[name]:
                raise ValueError("incorrect unit")
            quality = record.get("quality")
            provider = record.get("provider")
            if quality not in QUALITIES or provider not in PROVIDERS:
                raise ValueError("invalid quality/provider")
            age = record.get("age_ms")
            if type(age) is not int or not 0 <= age <= 3_600_000:
                raise ValueError("invalid metric age")
            value = record.get("value")
            if value is None:
                if quality == "valid":
                    raise ValueError("valid metric requires a value")
                clean_value = None
            elif name in VECTORS:
                if (not isinstance(value, list) or
                        len(value) not in VECTOR_LENGTHS[name]):
                    raise ValueError("vector has unsupported length")
                clean_value = [_bounded(v, name, -500, 500) for v in value]
            else:
                low, high = {
                    "heart_rate_bpm": (25, 240), "motion_score": (0, 60),
                    "ibi_ms": (200, 5000), "skin_temperature_c": (-20, 60),
                    "spo2_pct": (0, 100), "pressure_hpa": (300, 1100),
                    "light_lux": (0, 200000), "step_count": (0, 1e10),
                }[name]
                clean_value = _bounded(value, name, low, high)
            if name in {"heart_rate_bpm", "ibi_ms", "spo2_pct", "skin_temperature_c"}:
                if wear == "off_body" and quality == "valid":
                    raise ValueError("off-body physiology cannot be valid")
            clean_metrics[name] = {
                "value": clean_value, "unit": METRIC_UNITS[name],
                "quality": quality, "provider": provider, "age_ms": age,
            }
        cleaned["metrics"] = clean_metrics
        return METRICS_TOPIC, cleaned

    if kind == "device_state":
        online = raw.get("online")
        collecting = raw.get("collecting")
        if type(online) is not bool or type(collecting) is not bool:
            raise ValueError("invalid device state")
        cleaned.update(online=online, collecting=collecting)
        battery = raw.get("battery_pct")
        if battery is not None:
            cleaned["battery_pct"] = _bounded(battery, "battery_pct", 0, 100)
        if "agent_prompts_enabled" in raw:
            if type(raw["agent_prompts_enabled"]) is not bool:
                raise ValueError("invalid task prompt consent")
            cleaned["agent_prompts_enabled"] = raw["agent_prompts_enabled"]
        if "remote_control_enabled" in raw:
            if type(raw["remote_control_enabled"]) is not bool:
                raise ValueError("invalid remote control consent")
            cleaned["remote_control_enabled"] = raw["remote_control_enabled"]
        if "charging" in raw:
            if type(raw["charging"]) is not bool:
                raise ValueError("invalid charging state")
            cleaned["charging"] = raw["charging"]
        if "heart_permission" in raw:
            if raw["heart_permission"] not in ("granted", "denied"):
                raise ValueError("invalid heart permission")
            cleaned["heart_permission"] = raw["heart_permission"]
        if "activity_recognition_permission" in raw:
            if raw["activity_recognition_permission"] not in ("granted", "denied"):
                raise ValueError("invalid activity recognition permission")
            cleaned["activity_recognition_permission"] = raw[
                "activity_recognition_permission"
            ]
        if "last_valid_heart_age_ms" in raw:
            age = raw["last_valid_heart_age_ms"]
            if type(age) is not int or not 0 <= age <= 15_000:
                raise ValueError("invalid last valid heart age")
            if wear == "off_body":
                raise ValueError("off-body cannot report a valid heart age")
            cleaned["last_valid_heart_age_ms"] = age
        if "api_level" in raw:
            api = raw["api_level"]
            if type(api) is not int or not 30 <= api <= 100:
                raise ValueError("invalid API level")
            cleaned["api_level"] = api
        if "app_version" in raw:
            version = raw["app_version"]
            if (not isinstance(version, str) or not 1 <= len(version) <= 64
                    or not version.isascii() or not all(
                        char.isalnum() or char in "._+-" for char in version)):
                raise ValueError("invalid app version")
            cleaned["app_version"] = version
        if "tts_status" in raw:
            status = raw["tts_status"]
            if (not isinstance(status, str) or not 1 <= len(status) <= 32
                    or not status.isascii() or not all(
                        char.isalnum() or char in "._-" for char in status)):
                raise ValueError("invalid tts status")
            cleaned["tts_status"] = status
        if "tts_offline_supported" in raw:
            if type(raw["tts_offline_supported"]) is not bool:
                raise ValueError("invalid tts offline capability")
            cleaned["tts_offline_supported"] = raw["tts_offline_supported"]
        return DEVICE_TOPIC, cleaned

    recording_id = _identifier(raw.get("recording_id"), "recording_id")
    signal = raw.get("signal")
    if signal not in RAW_SIGNALS:
        raise ValueError("unsupported signal")
    count = raw.get("sample_count")
    if type(count) is not int or not 0 <= count <= 2_000_000:
        raise ValueError("invalid sample_count")
    rate = _bounded(raw.get("sample_rate_hz"), "sample_rate_hz", 0.1, 2000)
    if raw.get("stored_on_watch") is not True:
        raise ValueError("manifest requires local storage")
    if "samples" in raw or "data" in raw or "waveform" in raw:
        raise ValueError("raw waveform payloads are prohibited")
    cleaned.update(
        recording_id=recording_id, signal=signal,
        sample_count=count, sample_rate_hz=rate, stored_on_watch=True,
    )
    return RECORDINGS_TOPIC, cleaned
