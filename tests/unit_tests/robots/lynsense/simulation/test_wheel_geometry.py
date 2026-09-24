from __future__ import annotations

import math
import re
from pathlib import Path

import pytest


WORLD_ROOT = (
    Path("robots")
    / "lynsense"
    / "simulation"
    / "ros"
    / "lynsense_webots_sim"
    / "worlds"
)
WORLD_NAMES = ("lynsense_smoke.wbt", "lynsense_blocked.wbt", "lynsense_match.wbt")
WHEEL_NAMES = ("LEFT", "RIGHT")


def _pose_block(world: str, wheel: str) -> str:
    match = re.search(
        rf"DEF {wheel}_WHEEL_POSE Pose \{{(?P<body>.*?)\n          \}}",
        world,
        re.DOTALL,
    )
    assert match is not None
    return match.group("body")


@pytest.mark.parametrize("world_name", WORLD_NAMES)
@pytest.mark.parametrize("wheel", WHEEL_NAMES)
def test_wheel_visual_and_collision_share_a_y_axis_pose(world_name: str, wheel: str):
    world = (WORLD_ROOT / world_name).read_text(encoding="utf-8")
    pose = _pose_block(world, wheel)

    rotation = tuple(
        float(value)
        for value in re.search(r"rotation ([^\n]+)", pose).group(1).split()
    )
    axis = rotation[:3]
    angle = rotation[3]
    rotation_matrix = (
        (1.0, 0.0, 0.0),
        (0.0, math.cos(angle), -math.sin(angle)),
        (0.0, math.sin(angle), math.cos(angle)),
    )

    def rotate(vector: tuple[float, float, float]) -> tuple[float, float, float]:
        return tuple(
            sum(row[index] * vector[index] for index in range(3))
            for row in rotation_matrix
        )

    rotated_cylinder_axis = rotate((0.0, 0.0, 1.0))
    rotated_radial_axes = (rotate((1.0, 0.0, 0.0)), rotate((0.0, 1.0, 0.0)))
    radius = float(re.search(r"radius ([^\n]+)", pose).group(1))

    assert axis == pytest.approx((1.0, 0.0, 0.0))
    assert angle == pytest.approx(math.pi / 2)
    assert rotated_cylinder_axis == pytest.approx((0.0, -1.0, 0.0))
    assert all(
        math.hypot(radius * vector[0], radius * vector[2])
        == pytest.approx(radius)
        for vector in rotated_radial_axes
    )
    assert f"geometry DEF {wheel}_WHEEL_GEOMETRY Cylinder" in pose
    assert f"boundingObject USE {wheel}_WHEEL_POSE" in world
    assert f"boundingObject USE {wheel}_WHEEL_GEOMETRY" not in world


@pytest.mark.parametrize("world_name", WORLD_NAMES)
def test_wheel_pose_preserves_support_radius_and_endpoint_inertia(world_name: str):
    world = (WORLD_ROOT / world_name).read_text(encoding="utf-8")

    assert world.count("Cylinder {") == 2
    assert world.count("radius 0.08") == 2
    assert world.count("height 0.078") == 2
    assert world.count("axis 0 1 0") == 2
    assert world.count("translation 0 0.2305 0") == 1
    assert world.count("translation 0 -0.2305 0") == 1
    assert world.count("mass 5") == 4
    assert world.count("inertiaMatrix [ 0.010535 0.016 0.010535 ]") == 2


@pytest.mark.parametrize("world_name", WORLD_NAMES)
def test_floor_contacts_explicitly_disable_bounce_and_reduce_compliance(world_name: str):
    world = (WORLD_ROOT / world_name).read_text(encoding="utf-8")
    contacts = re.findall(r"ContactProperties \{([^}]+)\}", world)
    assert len(contacts) == 2
    for contact in contacts:
        assert 'material2 "floor"' in contact
        assert "bounce 0" in contact
        assert "softCFM 0.00001" in contact
