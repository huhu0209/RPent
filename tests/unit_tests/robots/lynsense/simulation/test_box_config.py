from __future__ import annotations

from pathlib import Path

import pytest

from lynsense_webots_sim.box_config import ConfigError, load_box_config


ROOT = Path("robots") / "lynsense" / "simulation" / "ros"
ACTION_ROOT = ROOT / "lynsense_utils_compat" / "action"
BOX_CONFIG = ROOT / "lynsense_webots_sim" / "config" / "box.yaml"


@pytest.mark.parametrize(
    "name,fields",
    [
        (
            "MoveWaist.action",
            [
                "float64 height_mm",
                "bool success",
                "string message",
                "string phase",
                "float64 position_mm",
                "float64 remaining_mm",
            ],
        ),
        (
            "MoveNamedConfig.action",
            [
                "string target",
                "bool initok",
                "bool success",
                "string message",
                "string phase",
                "float64 progress",
                "float64 max_joint_error_rad",
            ],
        ),
        (
            "SetGripper.action",
            [
                "float64 position",
                "bool success",
                "string message",
                "string phase",
                "float64 position",
                "float64 remaining",
            ],
        ),
        (
            "BoxPhase.action",
            [
                "string action",
                "string flow",
                "string config",
                "bool initok",
                "bool success",
                "string message",
                "string phase",
                "float64 progress",
                "float64 box_error_m",
            ],
        ),
    ],
)
def test_action_contracts_are_explicit(name: str, fields: list[str]) -> None:
    sections = (ACTION_ROOT / name).read_text(encoding="utf-8").split("---")
    assert len(sections) == 3
    goal_count = {"MoveNamedConfig.action": 2, "BoxPhase.action": 4}.get(name, 1)
    assert _lines(sections[0]) == ["# Goal", *fields[:goal_count]]
    assert _lines(sections[1]) == ["# Result", *fields[goal_count : goal_count + 2]]
    assert _lines(sections[2]) == ["# Feedback", *fields[goal_count + 2 :]]


def test_box_config_loads_complete_model() -> None:
    config = load_box_config(BOX_CONFIG)

    assert config.box.dimensions_m == (0.397, 0.295, 0.217)
    assert config.box.mass_kg == pytest.approx(1.0)
    assert config.waist.motor_name == "waist_motor"
    assert config.left_arm.joint_names == tuple(
        f"left_joint{index}" for index in range(1, 7)
    )
    assert config.right_arm.joint_names == tuple(
        f"right_joint{index}" for index in range(1, 7)
    )
    assert set(config.named_targets) == {
        "dualjo:joints_s",
        "dualjo:joints_br",
        "dualposi_armbase_abso:pt_1f1_ready",
        "dualposi_armbase_abso:pt_up",
    }
    assert config.thresholds.grasp_alignment_m == pytest.approx(0.03)
    assert config.thresholds.carry_drift_m == pytest.approx(0.02)


def test_box_config_rejects_duplicate_and_out_of_range_joints(tmp_path: Path) -> None:
    source = BOX_CONFIG.read_text(encoding="utf-8")
    path = tmp_path / "box.yaml"
    path.write_text(
        source.replace("left_joint2, left_joint3", "left_joint1, left_joint3"),
        encoding="utf-8",
    )

    with pytest.raises(ConfigError, match="duplicate joint"):
        load_box_config(path)


def _lines(section: str) -> list[str]:
    return [line.strip() for line in section.splitlines() if line.strip()]
