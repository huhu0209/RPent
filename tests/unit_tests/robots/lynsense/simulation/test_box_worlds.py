from __future__ import annotations

from pathlib import Path


WORLD_ROOT = (
    Path("robots")
    / "lynsense"
    / "simulation"
    / "ros"
    / "lynsense_webots_sim"
    / "worlds"
)


def _world(name: str) -> str:
    return (WORLD_ROOT / name).read_text(encoding="utf-8")


def test_box_world_declares_all_actuators_and_box() -> None:
    source = _world("lynsense_box.wbt")

    for name in ("waist_motor", "left_gripper_motor", "right_gripper_motor"):
        assert f'name "{name}"' in source
    for side in ("left", "right"):
        for index in range(1, 7):
            assert f'name "{side}_joint{index}_motor"' in source
            assert f'name "{side}_joint{index}_sensor"' in source
    assert source.count("LinearMotor {") == 1
    assert source.count("RotationalMotor {") == 16
    assert "DEF BOX_1 Solid" in source
    assert "translation 2.024931 -2.493846 0.08" in source
    assert "rotation 0 0 1 3.124139" in source
    assert "size 0.397 0.295 0.217" in source
    assert source.count("physics Physics {") >= 3
    assert "EXTERNPROTO" not in source
    assert "http://" not in source
    assert "https://" not in source


def test_box_world_grippers_open_in_the_gripper_plane() -> None:
    source = _world("lynsense_box.wbt")

    for side in ("left", "right"):
        assert f"DEF {side.upper()}_GRIPPER_PALM Shape" in source
        assert f'name "{side}_gripper_finger"' in source
        assert "geometry Box { size 0.14 0.025 0.13 }" in source

    assert "axis 0 0 1 anchor 0 0.035 0" in source
    assert "axis 0 0 -1 anchor 0 -0.035 0" in source
    assert source.count("translation 0.07 0 0") == 2
    assert source.count("translation 0.07 -0.035 0") == 1
    assert source.count("translation 0.07 0.035 0") == 1
    assert "axis 0 1 0 anchor 0.08 0.035 0" not in source
    assert "axis 0 1 0 anchor 0.08 -0.035 0" not in source


def test_grasp_world_fixes_base_and_exposes_contact_sensors() -> None:
    source = _world("lynsense_grasp_physics.wbt")

    assert 'name "EA200_BOX"' in source
    assert 'name "LEFT_GRIPPER_CONTACT"' in source
    assert 'name "RIGHT_GRIPPER_CONTACT"' in source
    assert "TouchSensor {" in source
    assert "DEF BOX_1 Solid" in source
    assert 'translation 1.3435 -2.482 0.75' in source
    assert 'translation 1.3435 -2.482 0.32075' in source
    assert "size 0.50 0.20 0.6415" in source
    assert "size 0.08 0.10 0.40" in source
    assert source.count("SliderJoint {") == 3
    assert source.count("LinearMotor {") == 3
    assert source.count("RotationalMotor {") == 12
