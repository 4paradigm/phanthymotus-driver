"""Sparse packet regression: every zero remains an observation, never missing."""
import copy
import time

import pytest

from test_payload_mass import raw_frame
from payload_mass_estimator import summarize, digest
from payload_mass_quick import profile_from_home, capture_bend_profile, fit_reference
from payload_mass_live import evaluate_window
from payload_mass_sampling import MassError


def packets(mass, *, phase=0, zero=False):
    frames = []
    for n in range(1201):
        f = raw_frame(0, n+1)
        f["host_monotonic_s"] = n/400
        f["q"][13] = -.8
        f["temperature"] = [40.] * 21
        f["temperature"][0] = 59.
        f["tau"] = [0.] * 21
        if not zero and (n+phase) % 8 == 0:
            for j, base, k in ((10, -.7, -1.3), (11, .05, .4), (12, -.02, .2), (13, -.33, -.8)):
                f["tau"][j] = base+k*mass
        frames.append(f)
    return frames


def setup_sparse(*, zero=False, broad_blocks=False):
    empty = packets(0, zero=zero)
    p = capture_bend_profile(profile_from_home(empty[0], "right", 300), empty)
    if broad_blocks:
        p["torque_block_span_limit"] = [1.] * 4
        p.pop("fingerprint")
        p["fingerprint"] = digest(p)
    baseline = {"id": "sparse", "created_s": time.time(), "profile_fingerprint": p["fingerprint"],
                "summary": summarize(empty, p)}
    return p, baseline, empty


def test_sparse_summary_preserves_zeros_and_nonzero_response():
    p, baseline, empty = setup_sparse()
    s = baseline["summary"]
    assert s["tau"][0] == pytest.approx(sum(f["tau"][10] for f in empty)/len(empty))
    assert s["tau"][0] != 0 and s["torque_nonzero_count"][0] == 151
    assert s["torque_block_count"] >= 4 and s["noise"][0] > .001
    assert s["effective_sample_count"] is None
    legacy = summarize(empty, {**p, "setup_mode": "profile"})
    assert legacy["tau"] == [0.] * 4
    assert legacy["torque_feature"] == "median_raw_v1"


@pytest.mark.parametrize("mass", [0, .2, .3, .5])
def test_sparse_reference_unknown_and_unload(mass):
    p, baseline, _ = setup_sparse()
    model = fit_reference(packets(.4), baseline, p, .4)
    result = evaluate_window(packets(mass, phase=3), baseline, model, p, 3, time.time())
    assert result["valid"] and result["stable"]
    assert abs(result["mass_kg"]-mass) <= model["resolution_kg"]
    assert model["validation_status"] == "not_validated"


def test_sparse_torque_change_does_not_become_stable_from_a_large_raw_span_limit():
    p, baseline, _ = setup_sparse(broad_blocks=True)
    model = fit_reference(packets(.4), baseline, p, .4)
    frames = packets(.2)
    loaded = packets(.4)
    frames[len(frames)//2:] = loaded[len(frames)//2:]
    result = evaluate_window(frames, baseline, model, p, 3, time.time())
    assert result["mass_kg"] is not None and not result["valid"] and not result["sampling_stable"]
    assert result["block_mass_span_kg"] > model["resolution_kg"]


def test_all_zero_reference_is_rejected_without_inventing_a_weight():
    p, baseline, _ = setup_sparse(zero=True)
    with pytest.raises(MassError) as err:
        fit_reference(packets(.4, zero=True), baseline, p, .4)
    assert err.value.code == "UNIDENTIFIABLE"
    assert "delta_tau=" in str(err.value) and "nonzero_count=" in str(err.value)


def test_a_single_torque_impulse_is_not_a_reference_response():
    p, baseline, _ = setup_sparse(zero=True)
    reference = packets(.4, zero=True)
    reference[100]["tau"][10] = 1000
    with pytest.raises(MassError) as err:
        fit_reference(reference, baseline, p, .4)
    assert err.value.code == "UNIDENTIFIABLE"


@pytest.mark.parametrize("motor, current", [(7, 0.), (11, 51.)])
def test_temperature_drift_keeps_limits_and_reports_failed_motor(motor, current):
    p, baseline, _ = setup_sparse()
    reference = packets(.4)
    for f in reference:
        f["temperature"][motor] = current
    with pytest.raises(MassError) as err:
        fit_reference(reference, baseline, p, .4)
    assert err.value.code == "TEMPERATURE_DRIFT"
    assert f"motor_id={motor}" in str(err.value) and "limit=10.00" in str(err.value)


def test_new_torque_feature_refuses_a_median_baseline():
    p, baseline, empty = setup_sparse()
    old = copy.deepcopy(baseline)
    old["summary"] = summarize(empty, {**p, "setup_mode": "profile"})
    with pytest.raises(MassError) as err:
        fit_reference(packets(.4), old, p, .4)
    assert err.value.code == "CONFIG_CHANGED"


def test_many_observations_without_temporal_blocks_cannot_calibrate():
    p, baseline, _ = setup_sparse()
    reference = packets(.4)
    for f in reference:
        f["host_monotonic_s"] = 0.
    with pytest.raises(MassError) as err:
        fit_reference(reference, baseline, p, .4)
    assert err.value.code == "INSUFFICIENT_SAMPLES"
