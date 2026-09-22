# Copyright 2026 The RPent Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import re
from pathlib import Path

import pytest

from robots.lynsense.isaac.archive import extract_archive
from robots.lynsense.isaac.contracts import IsaacProbeError
from robots.lynsense.isaac.urdf_contract import (
    REQUIRED_JOINT_NAMES,
    inspect_urdf,
    resolve_mesh_reference,
)


def _joint(
    name: str,
    joint_type: str = "revolute",
    child_link: str | None = None,
) -> str:
    child = child_link or f"{name}_link"
    if joint_type == "fixed":
        return f'<joint name="{name}" type="fixed"><parent link="base_link"/><child link="{child}"/></joint>'
    return f'''<joint name="{name}" type="{joint_type}">
      <parent link="base_link"/><child link="{child}"/><axis xyz="0 0 1"/>
      <limit lower="-1.0" upper="1.0" effort="200" velocity="3.14"/>
      <dynamics damping="16.6" friction="9.6"/>
    </joint>'''


def _urdf(mesh_reference: str = "package://pkg/meshes/part.stl") -> str:
    gripper_names = (
        "left_R1",
        "left_R2",
        "left_R3",
        "left_L1",
        "left_L2",
        "left_L3",
        "right_R1",
        "right_R2",
        "right_R3",
        "right_L1",
        "right_L2",
        "right_L3",
    )
    joints = (
        _joint("connector_joint", "prismatic")
        + "\n"
        + "\n".join(_joint(name) for name in REQUIRED_JOINT_NAMES[1:])
    )
    grippers = "\n".join(
        _joint(name, child_link=f"{name}_Link") for name in gripper_names
    )
    moving_link_names = tuple(
        f"{name}_Link" if name in gripper_names else f"{name}_link"
        for name in (*REQUIRED_JOINT_NAMES, *gripper_names)
    )
    moving_links = "\n".join(f'<link name="{name}"/>' for name in moving_link_names)
    sensor_links = (
        "head_camera_link",
        "left_camera_link",
        "right_camera_link",
        "left_force_sensor_link",
        "right_force_sensor_link",
    )
    sensor_joints = "\n".join(
        f'<joint name="{link.removesuffix("_link")}_fixture_joint" type="fixed">'
        f'<parent link="base_link"/><child link="{link}"/></joint>'
        for link in sensor_links
    )
    return f'''<robot name="fixture">
      <link name="base_link">
        <visual><geometry><mesh filename="{mesh_reference}"/></geometry></visual>
        <collision><geometry><mesh filename="{mesh_reference}"/></geometry></collision>
      </link>
      {moving_links}
      <link name="head_camera_link"/>
      <link name="left_camera_link"/>
      <link name="right_camera_link"/>
      <link name="left_force_sensor_link"/>
      <link name="right_force_sensor_link"/>
      {sensor_joints}
      {joints}
      {grippers}
    </robot>'''


def _write_urdf(tmp_path: Path, urdf: str) -> Path:
    path = tmp_path / "URDF_robot_1" / "robot.urdf"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(urdf, encoding="utf-8")
    return path


def _remove_joint(urdf: str, name: str) -> str:
    pattern = rf'<joint name="{re.escape(name)}".*?</joint>'
    updated, count = re.subn(pattern, "", urdf, count=1, flags=re.DOTALL)
    assert count == 1
    return updated


def test_mesh_references_resolve_inside_extraction_root(tmp_path: Path) -> None:
    urdf = tmp_path / "URDF_robot_1" / "robot.urdf"
    urdf.parent.mkdir(parents=True)
    urdf.write_text(_urdf(), encoding="utf-8")
    mesh = tmp_path / "pkg" / "meshes" / "part.stl"
    mesh.parent.mkdir(parents=True)
    mesh.write_bytes(b"mesh")

    inventory = inspect_urdf(urdf)
    assert inventory.root_link == "base_link"
    assert inventory.required_joint_names == REQUIRED_JOINT_NAMES
    assert len(inventory.meshes) == 2
    assert inventory.meshes[0].resolved_path == mesh
    assert inventory.meshes[0].sha256.startswith("sha256:")
    assert inventory.to_dict()["root_link"] == "base_link"


def test_file_reference_resolves_relative_to_extraction_root(tmp_path: Path) -> None:
    urdf = tmp_path / "URDF_robot_1" / "robot.urdf"
    urdf.parent.mkdir(parents=True)
    urdf.write_text(
        _urdf("file://realsense2_description/meshes/d405.stl"), encoding="utf-8"
    )
    mesh = tmp_path / "realsense2_description" / "meshes" / "d405.stl"
    mesh.parent.mkdir(parents=True)
    mesh.write_bytes(b"camera")
    assert (
        resolve_mesh_reference(urdf, "file://realsense2_description/meshes/d405.stl")
        == mesh
    )


def test_mesh_references_resolve_inside_urdf_bundle_root(tmp_path: Path) -> None:
    urdf = tmp_path / "URDF_robot_1" / "robot.urdf"
    urdf.parent.mkdir(parents=True)
    urdf.write_text(
        _urdf("package://cr100_description/meshes/part.stl"), encoding="utf-8"
    )
    mesh = tmp_path / "URDF_robot_1" / "cr100_description" / "meshes" / "part.stl"
    mesh.parent.mkdir(parents=True)
    mesh.write_bytes(b"mesh")
    assert (
        resolve_mesh_reference(urdf, "package://cr100_description/meshes/part.stl")
        == mesh
    )


def test_mesh_package_and_file_traversal_are_rejected(tmp_path: Path) -> None:
    urdf = tmp_path / "URDF_robot_1" / "robot.urdf"
    urdf.parent.mkdir(parents=True)
    urdf.write_text("<robot name='fixture'/>", encoding="utf-8")
    outside = tmp_path / "outside.stl"
    outside.write_bytes(b"outside")

    references = (
        f"package://pkg/../../{outside.name}",
        f"file://pkg/../../{outside.name}",
    )
    for reference in references:
        with pytest.raises(IsaacProbeError, match="mesh reference"):
            resolve_mesh_reference(urdf, reference)


def test_mesh_absolute_file_reference_is_rejected(tmp_path: Path) -> None:
    urdf = tmp_path / "URDF_robot_1" / "robot.urdf"
    urdf.parent.mkdir(parents=True)
    urdf.write_text("<robot name='fixture'/>", encoding="utf-8")
    absolute_named_mesh = tmp_path / "tmp" / "absolute.stl"
    absolute_named_mesh.parent.mkdir(parents=True)
    absolute_named_mesh.write_bytes(b"inside")

    with pytest.raises(IsaacProbeError, match="absolute"):
        resolve_mesh_reference(urdf, "file:///tmp/absolute.stl")


def test_mesh_unsupported_scheme_is_rejected(tmp_path: Path) -> None:
    urdf = tmp_path / "URDF_robot_1" / "robot.urdf"
    urdf.parent.mkdir(parents=True)
    urdf.write_text("<robot name='fixture'/>", encoding="utf-8")

    with pytest.raises(IsaacProbeError, match="unsupported mesh reference scheme"):
        resolve_mesh_reference(urdf, "https://example.com/mesh.stl")


def test_mesh_encoded_traversal_is_rejected(tmp_path: Path) -> None:
    urdf = tmp_path / "URDF_robot_1" / "robot.urdf"
    urdf.parent.mkdir(parents=True)
    urdf.write_text("<robot name='fixture'/>", encoding="utf-8")
    outside = tmp_path / "outside.stl"
    outside.write_bytes(b"outside")
    literal_path = tmp_path / "pkg" / "%2e%2e" / "%2e%2e"
    literal_path.mkdir(parents=True)
    (literal_path / "outside.stl").write_bytes(b"literal traversal")

    with pytest.raises(IsaacProbeError, match="mesh reference"):
        resolve_mesh_reference(
            urdf,
            "package://pkg/%2e%2e/%2e%2e/outside.stl",
        )


def test_mesh_symlinked_path_components_are_rejected(tmp_path: Path) -> None:
    urdf = tmp_path / "URDF_robot_1" / "robot.urdf"
    urdf.parent.mkdir(parents=True)
    urdf.write_text("<robot name='fixture'/>", encoding="utf-8")
    real_mesh = tmp_path / "pkg" / "meshes" / "part.stl"
    real_mesh.parent.mkdir(parents=True)
    real_mesh.write_bytes(b"mesh")
    internal_alias = tmp_path / "alias"
    internal_alias.symlink_to(tmp_path / "pkg", target_is_directory=True)

    with pytest.raises(IsaacProbeError, match="symbolic link"):
        resolve_mesh_reference(urdf, "package://alias/meshes/part.stl")


def test_missing_mesh_is_rejected(tmp_path: Path) -> None:
    urdf = tmp_path / "URDF_robot_1" / "robot.urdf"
    urdf.parent.mkdir(parents=True)
    urdf.write_text(_urdf(), encoding="utf-8")
    with pytest.raises(Exception, match="mesh"):
        inspect_urdf(urdf)


@pytest.mark.parametrize(
    ("mutation_name", "mutation"),
    [
        (
            "wrong-root",
            lambda urdf: urdf.replace('name="base_link"', 'name="chassis_link"'),
        ),
        (
            "duplicate-joint",
            lambda urdf: urdf.replace(
                "</robot>",
                re.search(
                    r'<joint name="connector_joint".*?</joint>', urdf, flags=re.DOTALL
                ).group(0)
                + "</robot>",
            ),
        ),
        ("missing-connector", lambda urdf: _remove_joint(urdf, "connector_joint")),
        ("missing-left-arm", lambda urdf: _remove_joint(urdf, "left_joint1")),
        ("missing-right-arm", lambda urdf: _remove_joint(urdf, "right_joint6")),
        (
            "wrong-connector-type",
            lambda urdf: urdf.replace(
                '<joint name="connector_joint" type="prismatic">',
                '<joint name="connector_joint" type="revolute">',
            ),
        ),
        (
            "wrong-arm-type",
            lambda urdf: urdf.replace(
                '<joint name="left_joint1" type="revolute">',
                '<joint name="left_joint1" type="prismatic">',
            ),
        ),
        ("missing-effort", lambda urdf: urdf.replace(' effort="200"', "")),
        (
            "non-finite-effort",
            lambda urdf: urdf.replace('effort="200"', 'effort="INF"'),
        ),
        ("zero-effort", lambda urdf: urdf.replace('effort="200"', 'effort="0"')),
        (
            "inverted-limits",
            lambda urdf: urdf.replace(
                'lower="-1.0" upper="1.0"', 'lower="1.0" upper="-1.0"'
            ),
        ),
    ],
)
def test_required_joint_contract_failures(
    tmp_path: Path, mutation_name: str, mutation
) -> None:
    path = _write_urdf(tmp_path, mutation(_urdf()))
    (tmp_path / "pkg" / "meshes").mkdir(parents=True)
    (tmp_path / "pkg" / "meshes" / "part.stl").write_bytes(b"mesh")
    with pytest.raises(IsaacProbeError):
        inspect_urdf(path)


@pytest.mark.parametrize(
    "gripper_name",
    (
        "left_R1",
        "left_R2",
        "left_R3",
        "left_L1",
        "left_L2",
        "left_L3",
        "right_R1",
        "right_R2",
        "right_R3",
        "right_L1",
        "right_L2",
        "right_L3",
    ),
)
@pytest.mark.parametrize("missing_part", ["joint", "link"])
def test_gripper_subtree_requirements(
    tmp_path: Path, gripper_name: str, missing_part: str
) -> None:
    urdf = _urdf()
    if missing_part == "joint":
        urdf = _remove_joint(urdf, gripper_name)
    else:
        urdf = urdf.replace(f'<link name="{gripper_name}_Link"/>', "")
    path = _write_urdf(tmp_path, urdf)
    (tmp_path / "pkg" / "meshes").mkdir(parents=True)
    (tmp_path / "pkg" / "meshes" / "part.stl").write_bytes(b"mesh")
    with pytest.raises(IsaacProbeError, match=gripper_name):
        inspect_urdf(path)


@pytest.mark.parametrize(
    "link_name",
    (
        "head_camera_link",
        "left_camera_link",
        "right_camera_link",
        "left_force_sensor_link",
        "right_force_sensor_link",
    ),
)
def test_required_sensor_links(tmp_path: Path, link_name: str) -> None:
    urdf = _urdf().replace(f'<link name="{link_name}"/>', "")
    path = _write_urdf(tmp_path, urdf)
    (tmp_path / "pkg" / "meshes").mkdir(parents=True)
    (tmp_path / "pkg" / "meshes" / "part.stl").write_bytes(b"mesh")
    with pytest.raises(IsaacProbeError, match=link_name):
        inspect_urdf(path)


@pytest.mark.parametrize(
    ("failure_name", "failure_reason", "mutation"),
    [
        (
            "alternate-parent",
            "exactly one parent joint",
            lambda urdf: urdf.replace(
                "</robot>",
                '<joint name="head_camera_alternate_parent" type="fixed">'
                '<parent link="connector_joint_link"/>'
                '<child link="head_camera_link"/></joint></robot>',
            ),
        ),
        (
            "self-cycle",
            "cycle",
            lambda urdf: urdf.replace(
                '<parent link="base_link"/><child link="head_camera_link"/>',
                '<parent link="head_camera_link"/><child link="head_camera_link"/>',
            ),
        ),
        (
            "required-link-cycle",
            "cycle",
            lambda urdf: urdf.replace(
                '<parent link="base_link"/><child link="head_camera_link"/>',
                '<parent link="left_camera_link"/><child link="head_camera_link"/>',
            ).replace(
                '<parent link="base_link"/><child link="left_camera_link"/>',
                '<parent link="head_camera_link"/><child link="left_camera_link"/>',
            ),
        ),
        (
            "unreachable-link",
            "unreachable",
            lambda urdf: urdf.replace(
                "</robot>",
                '<link name="synthetic_island"/></robot>',
            ),
        ),
    ],
)
def test_urdf_graph_must_be_a_reachable_tree(
    tmp_path: Path, failure_name: str, failure_reason: str, mutation
) -> None:
    path = _write_urdf(tmp_path, mutation(_urdf()))
    (tmp_path / "pkg" / "meshes").mkdir(parents=True)
    (tmp_path / "pkg" / "meshes" / "part.stl").write_bytes(b"mesh")
    with pytest.raises(IsaacProbeError, match=failure_reason):
        inspect_urdf(path)


def test_valid_probe_archive_satisfies_both_contracts(
    valid_probe_archive: Path, tmp_path: Path
) -> None:
    archive_inventory = extract_archive(valid_probe_archive, tmp_path / "staged")
    urdf_inventory = inspect_urdf(archive_inventory.robot_urdf)

    assert archive_inventory.file_count == 2
    assert urdf_inventory.robot_name == "fixture"
    assert urdf_inventory.root_link == "base_link"
    assert urdf_inventory.required_joint_names == REQUIRED_JOINT_NAMES
    assert len(urdf_inventory.meshes) == 2
