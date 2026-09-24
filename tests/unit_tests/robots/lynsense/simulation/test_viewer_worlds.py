from __future__ import annotations

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

WORLD_CASES = (
    ("lynsense_smoke.wbt", (-2.5, -1.5, -0.5, 0.5, 1.5, 2.5), 5),
    ("lynsense_blocked.wbt", (-2.5, -1.5, -0.5, 0.5, 1.5, 2.5), 5),
    ("lynsense_match.wbt", (-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0), 6),
    ("lynsense_box.wbt", (-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0), 6),
)


def _node_block(source: str, node_start: int) -> str:
    opening = source.index("{", node_start)
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[node_start : index + 1]
    raise AssertionError("unclosed Webots node")


def _named_node_block(source: str, node_name: str, node_type: str) -> str:
    name_index = source.index(node_name)
    node_start = source.rfind(f"{node_type} {{", 0, name_index)
    assert node_start >= 0
    return _node_block(source, node_start)


@pytest.mark.parametrize("filename,y_positions,grid_length", WORLD_CASES)
def test_reference_grid_is_visual_only_and_matches_arena(
    filename: str, y_positions: tuple[float, ...], grid_length: int
):
    world = (WORLD_ROOT / filename).read_text(encoding="utf-8")
    grid = _node_block(world, world.index("DEF VIEWER_REFERENCE_GRID Pose {"))

    assert "translation 0 0 -0.01" in grid
    assert "geometry Plane" in grid
    assert "size 30 30" in grid
    assert grid.count("geometry Box") == 9 + len(y_positions)
    assert "boundingObject" not in grid
    assert "physics" not in grid

    for x in range(-4, 5):
        assert f"translation {x} 0 0.012" in grid
        assert f"size 0.018 {grid_length} 0.004" in grid
    for y in y_positions:
        assert f"translation 0 {y:g} 0.012" in grid
        assert f"size 8 0.018 0.004" in grid


@pytest.mark.parametrize("filename,_,__", WORLD_CASES)
def test_front_camera_defaults_are_shared_and_tilted_for_ground_view(
    filename: str, _: tuple[float, ...], __: int
):
    world = (WORLD_ROOT / filename).read_text(encoding="utf-8")
    camera = _named_node_block(world, 'name "viewer_front_camera"', "Camera")

    assert 'name "viewer_front_camera"' in camera
    assert "translation 0.16 0 1.05" in camera
    assert "rotation 0 1 0 0.45" in camera
    assert "width 960" in camera
    assert "height 540" in camera
    assert "fieldOfView 1.22" in camera
    assert "near 0.05" in camera
    assert "far 30" in camera
    assert "enable" not in camera


@pytest.mark.parametrize("filename,_,__", WORLD_CASES)
def test_worlds_keep_local_visual_assets_and_unchanged_viewpoint(
    filename: str, _: tuple[float, ...], __: int
):
    world = (WORLD_ROOT / filename).read_text(encoding="utf-8")

    assert "EXTERNPROTO" not in world
    assert "textureURL" not in world
    assert "http://" not in world
    assert "https://" not in world
    assert "orientation -0.350323 0.145108 0.925320 2.409572" in world
    assert "position 8 -8 10" in world
