from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import pytest

from lynsense_webots_sim.scripts.run_grasp_physics import STAGES as PHYSICAL_STAGES
from lynsense_webots_sim.scripts.run_grasp_physics import summarize as summarize_physical
from lynsense_webots_sim.scripts.run_mechanical import STAGES as MECHANICAL_STAGES
from lynsense_webots_sim.scripts.run_mechanical import (
    ARM_BR_TARGET,
    ARM_UP_TARGET,
    summarize as summarize_mechanical,
)
from lynsense_webots_sim.supervisor_probe import probe_exit_code


ROOT = Path("robots") / "lynsense" / "simulation" / "ros" / "lynsense_webots_sim"
MECHANICAL_PATH = ROOT / "lynsense_webots_sim" / "scripts" / "run_mechanical.py"
PHYSICAL_PATH = ROOT / "lynsense_webots_sim" / "scripts" / "run_grasp_physics.py"
PROBE_LAUNCHER_PATH = ROOT / "lynsense_webots_sim" / "supervisor_probe.py"


def mechanical_samples(
    *, stalled: bool = False, terminal_zero: bool = True, reached: bool = True
) -> list[dict[str, Any]]:
    samples: list[dict[str, Any]] = []
    waist = -0.478
    arms = (0.0,) * 12
    grippers = (0.0, 0.0)
    for phase, duration, target_kind, _target in MECHANICAL_STAGES:
        count = round(duration / 0.032)
        for index in range(count):
            fraction = 0.0 if phase == "settle" else index / (count - 1)
            if stalled and phase == "arms_br":
                fraction = 0.0
            if phase == "waist":
                waist = -0.478 + fraction * 0.2
            elif phase == "arms_br":
                arms = tuple(a + fraction * (b - a) for a, b in zip(arms, ARM_BR_TARGET))
            elif phase == "gripper_open":
                grippers = (fraction * 0.6, fraction * 0.6)
            elif phase == "arms_up":
                arms = tuple(a + fraction * (b - a) for a, b in zip(arms, ARM_UP_TARGET))
            elif phase == "gripper_close":
                grippers = ((1.0 - fraction) * 0.6, (1.0 - fraction) * 0.6)
            moving = phase not in {"settle", "terminal"}
            samples.append(
                {
                    "phase": phase,
                    "time_s": len(samples) * 0.032,
                    "waist_position_m": waist,
                    "waist_velocity_mps": (
                        (0.0 if terminal_zero else 0.1)
                        if phase == "terminal"
                        else 0.01
                    ),
                    "arm_positions_rad": arms,
                    "arm_velocities_radps": (
                        ((0.0,) * 12 if terminal_zero else (0.1,) * 12)
                        if phase == "terminal"
                        else (0.1,) * 12
                        if moving
                        else (0.0,) * 12
                    ),
                    "gripper_positions": grippers,
                    "gripper_velocities": (
                        ((0.0, 0.0) if terminal_zero else (0.2, 0.2))
                        if phase == "terminal"
                        else (0.2, 0.2)
                        if moving
                        else (0.0, 0.0)
                    ),
                    "max_joint_error_rad": 0.01 if reached else 0.2,
                }
            )
            if phase == "waist":
                waist = samples[-1]["waist_position_m"]
            elif phase in {"arms_br", "arms_up"}:
                arms = samples[-1]["arm_positions_rad"]
            elif phase in {"gripper_open", "gripper_close"}:
                grippers = samples[-1]["gripper_positions"]
    return samples


def physical_samples(
    *, contact_loss: bool = False, lift: bool = True, falls: bool = True
) -> list[dict[str, Any]]:
    samples: list[dict[str, Any]] = []
    height = 0.75
    for phase, duration, _command, _value in PHYSICAL_STAGES:
        count = round(duration / 0.032)
        for index in range(count):
            fraction = index / (count - 1)
            if phase == "lift" and lift:
                height = 0.75 + fraction * 0.1
            elif phase == "release" and falls:
                height = 0.85 - fraction * (0.85 - 0.1085)
            contacts = phase in {"close", "lift", "hold", "release"}
            samples.append(
                {
                    "phase": phase,
                    "time_s": len(samples) * 0.032,
                    "gripper_positions": (0.0, 0.0),
                    "box_height_m": height,
                    "left_contact": contacts and not (contact_loss and phase == "hold" and index > 1),
                    "right_contact": contacts and not (contact_loss and phase == "hold" and index > 1),
                    "box_dynamic": phase in {"release"},
                    "motor_rates": {
                        "waist": 0.1 if phase != "settle" else 0.0,
                        "arms": (0.1,) * 12 if phase != "settle" else (0.0,) * 12,
                        "grippers": (0.1, 0.1) if phase != "settle" else (0.0, 0.0),
                    },
                }
            )
    return samples


def test_mechanical_summarizer_requires_complete_finite_reached_zero_terminal() -> None:
    summary = summarize_mechanical(mechanical_samples())
    assert summary["passed"]
    assert summary["targets_reached"]
    assert summary["terminal_rates_zero"]
    assert summary["moved_without_stall"]

    sparse = mechanical_samples()[::20]
    with pytest.raises(ValueError, match="incomplete"):
        summarize_mechanical(sparse)
    gap = mechanical_samples()
    gap[100]["time_s"] += 0.032
    with pytest.raises(ValueError, match="sampling interval"):
        summarize_mechanical(gap)
    nonfinite = mechanical_samples()
    nonfinite[100]["arm_positions_rad"] = (math.nan, *nonfinite[100]["arm_positions_rad"][1:])
    with pytest.raises(ValueError, match="non-finite"):
        summarize_mechanical(nonfinite)
    assert not summarize_mechanical(mechanical_samples(reached=False))["passed"]
    assert not summarize_mechanical(mechanical_samples(terminal_zero=False))["passed"]
    assert not summarize_mechanical(mechanical_samples(stalled=True))["passed"]


def test_physical_summarizer_requires_contacts_lift_and_release_fall() -> None:
    summary = summarize_physical(physical_samples())
    assert summary["passed"]
    assert summary["contacts_held"]
    assert summary["box_lifted"]
    assert summary["box_fell_after_release"]

    assert not summarize_physical(physical_samples(contact_loss=True))["passed"]
    assert not summarize_physical(physical_samples(lift=False))["passed"]
    assert not summarize_physical(physical_samples(falls=False))["passed"]
    sparse = physical_samples()[::20]
    with pytest.raises(ValueError, match="incomplete"):
        summarize_physical(sparse)
    gap = physical_samples()
    gap[100]["time_s"] += 0.032
    with pytest.raises(ValueError, match="sampling interval"):
        summarize_physical(gap)


def test_isolated_regression_scripts_have_no_ros_and_use_fast_webots() -> None:
    readme = Path("robots/lynsense/simulation/README.md").read_text(encoding="utf-8")
    for source in (
        MECHANICAL_PATH.read_text(encoding="utf-8"),
        PHYSICAL_PATH.read_text(encoding="utf-8"),
        PROBE_LAUNCHER_PATH.read_text(encoding="utf-8"),
    ):
        assert "rclpy" not in source
        assert "lynsense_utils.action" not in source
        assert "ros2 run" not in source
        assert "webots-controller" in source
    launcher_source = PROBE_LAUNCHER_PATH.read_text(encoding="utf-8")
    assert '"--mode=fast"' in launcher_source
    assert "run_mechanical.py" in readme
    assert "run_grasp_physics.py" in readme


def test_probe_exit_uses_written_summary(tmp_path):
    summary = tmp_path / "summary.json"
    summary.write_text('{"passed": true}\n', encoding="utf-8")
    assert probe_exit_code(1, summary) == 0
    summary.write_text('{"passed": false}\n', encoding="utf-8")
    assert probe_exit_code(0, summary) == 1
    assert probe_exit_code(3, tmp_path / "missing.json") == 3
