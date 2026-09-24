from __future__ import annotations

import math

import pytest

from lynsense_webots_sim.scripts.run_stability import STAGES, summarize


def test_passed_summary_controls_orchestrator_exit(tmp_path):
    from lynsense_webots_sim.scripts import run_stability

    summary = tmp_path / "summary.json"
    summary.write_text('{"passed": true}\n', encoding="utf-8")
    assert run_stability.orchestrator_exit_code(summary) == 0
    summary.write_text('{"passed": false}\n', encoding="utf-8")
    assert run_stability.orchestrator_exit_code(summary) == 1


def stable_samples():
    samples = []
    for phase, duration, _, _ in STAGES:
        count = round(duration / 0.032)
        for index in range(count):
            fraction = index / (count - 1)
            yaw = fraction * 2.4 if phase == "turn_left" else 0.0
            if phase == "turn_right":
                yaw = -fraction * 2.4
            samples.append({
                "phase": phase, "x": fraction * 1.2 if phase == "straight" else 0.0,
                "y": 0.0, "z": 0.08, "yaw": yaw,
                "roll_deg": 0.01, "pitch_deg": 0.02,
                "time_s": len(samples) * 0.032,
            })
    return samples


def test_stability_requires_physical_motion_and_small_attitude_changes():
    summary = summarize(stable_samples())
    assert summary["passed"]
    assert summary["phases"]["straight"]["distance_m"] == pytest.approx(1.2)


@pytest.mark.parametrize("field,value", [("roll_deg", 2.0), ("pitch_deg", -2.0), ("z", 0.09)])
def test_stability_rejects_rocking(field, value):
    samples = stable_samples()
    next(row for row in samples if row["phase"] == "straight")[field] = value
    assert not summarize(samples)["passed"]


def test_stability_does_not_accept_a_motionless_robot():
    samples = stable_samples()
    for row in samples:
        row["x"] = row["yaw"] = 0.0
    result = summarize(samples)
    assert result["stable"]
    assert not result["moving"]
    assert not result["passed"]


def test_stability_requires_all_phases():
    with pytest.raises(ValueError, match="incomplete"):
        summarize(stable_samples()[:-3])


def test_stability_rejects_sparse_and_misordered_recordings():
    samples = stable_samples()
    with pytest.raises(ValueError, match="incomplete"):
        summarize(samples[::20])
    with pytest.raises(ValueError, match="out-of-order"):
        summarize(list(reversed(samples)))


def test_stability_rejects_sampling_gaps():
    samples = stable_samples()
    samples[200]["time_s"] += 0.032
    with pytest.raises(ValueError, match="sampling interval"):
        summarize(samples)


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_stability_rejects_nonfinite_poses(value):
    samples = stable_samples()
    samples[200]["roll_deg"] = value
    with pytest.raises(ValueError, match="non-finite"):
        summarize(samples)


def test_yaw_change_handles_crossing_pi():
    samples = stable_samples()
    for row in samples:
        if row["phase"] == "turn_left":
            row["yaw"] = math.atan2(math.sin(row["yaw"] + 3), math.cos(row["yaw"] + 3))
    assert summarize(samples)["phases"]["turn_left"]["yaw_change_rad"] == pytest.approx(2.4)
