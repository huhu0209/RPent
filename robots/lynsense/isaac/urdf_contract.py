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

import hashlib
import math
import stat
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from urllib.parse import unquote, urlparse

from robots.lynsense.isaac.contracts import IsaacProbeError

REQUIRED_JOINT_NAMES = (
    "connector_joint",
    *(f"left_joint{i}" for i in range(1, 7)),
    *(f"right_joint{i}" for i in range(1, 7)),
)
REQUIRED_LINK_NAMES = (
    "head_camera_link",
    "left_camera_link",
    "right_camera_link",
    "left_force_sensor_link",
    "right_force_sensor_link",
)
REQUIRED_GRIPPER_JOINT_NAMES = (
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
REQUIRED_GRIPPER_LINK_NAMES = tuple(
    f"{name}_Link" for name in REQUIRED_GRIPPER_JOINT_NAMES
)


@dataclass(frozen=True)
class UrdfJoint:
    name: str
    joint_type: str
    parent_link: str
    child_link: str
    lower: float | None
    upper: float | None
    effort: float | None
    velocity: float | None
    damping: float | None
    friction: float | None


@dataclass(frozen=True)
class UrdfMesh:
    reference: str
    resolved_path: Path
    geometry_type: str
    sha256: str


@dataclass(frozen=True)
class UrdfInventory:
    robot_name: str
    root_link: str
    joints: tuple[UrdfJoint, ...]
    links: tuple[str, ...]
    meshes: tuple[UrdfMesh, ...]
    required_joint_names: tuple[str, ...]

    def to_dict(self) -> dict[str, object]:
        return {
            "robot_name": self.robot_name,
            "root_link": self.root_link,
            "joints": [
                {
                    "name": joint.name,
                    "joint_type": joint.joint_type,
                    "parent_link": joint.parent_link,
                    "child_link": joint.child_link,
                    "lower": joint.lower,
                    "upper": joint.upper,
                    "effort": joint.effort,
                    "velocity": joint.velocity,
                    "damping": joint.damping,
                    "friction": joint.friction,
                }
                for joint in self.joints
            ],
            "links": list(self.links),
            "meshes": [
                {
                    "reference": mesh.reference,
                    "resolved_path": str(mesh.resolved_path),
                    "geometry_type": mesh.geometry_type,
                    "sha256": mesh.sha256,
                }
                for mesh in self.meshes
            ],
            "required_joint_names": list(self.required_joint_names),
        }


def _finite_float(value: str, description: str) -> float:
    try:
        number = float(value)
    except ValueError as exc:
        raise IsaacProbeError(f"invalid {description} {value!r}") from exc
    if not math.isfinite(number):
        raise IsaacProbeError(f"non-finite {description} {value!r}")
    return number


def _required_attribute(element: ET.Element, name: str, description: str) -> str:
    value = element.get(name)
    if not value:
        raise IsaacProbeError(f"missing {name} for {description}")
    return value


def _parse_joint(joint: ET.Element) -> UrdfJoint:
    name = _required_attribute(joint, "name", "joint")
    description = f"joint {name!r}"
    parent = joint.find("parent")
    child = joint.find("child")
    if parent is None or child is None:
        raise IsaacProbeError(f"{description} must declare parent and child links")

    lower = upper = effort = velocity = damping = friction = None
    limit = joint.find("limit")
    if limit is not None:
        limits: dict[str, float | None] = {}
        for field in ("lower", "upper", "effort", "velocity"):
            value = limit.get(field)
            limits[field] = (
                _finite_float(value, f"{field} limit for {description}")
                if value is not None
                else None
            )
        lower = limits["lower"]
        upper = limits["upper"]
        effort = limits["effort"]
        velocity = limits["velocity"]
    dynamics = joint.find("dynamics")
    if dynamics is not None:
        damping_value = dynamics.get("damping")
        friction_value = dynamics.get("friction")
        damping = (
            _finite_float(damping_value, f"damping for {description}")
            if damping_value is not None
            else None
        )
        friction = (
            _finite_float(friction_value, f"friction for {description}")
            if friction_value is not None
            else None
        )

    return UrdfJoint(
        name=name,
        joint_type=_required_attribute(joint, "type", description),
        parent_link=_required_attribute(parent, "link", f"{description} parent"),
        child_link=_required_attribute(child, "link", f"{description} child"),
        lower=lower,
        upper=upper,
        effort=effort,
        velocity=velocity,
        damping=damping,
        friction=friction,
    )


def _require_limit(joint: UrdfJoint, field: str, getter) -> float:
    value = getter(joint)
    if value is None:
        raise IsaacProbeError(f"missing {field} limit for joint {joint.name!r}")
    return value


def _validate_required_joints(joints: dict[str, UrdfJoint]) -> None:
    for name in REQUIRED_JOINT_NAMES:
        joint = joints.get(name)
        if joint is None:
            raise IsaacProbeError(f"missing required joint {name!r}")
        expected_type = "prismatic" if name == "connector_joint" else "revolute"
        if joint.joint_type != expected_type:
            raise IsaacProbeError(
                f"joint {name!r} must be {expected_type}, got {joint.joint_type!r}"
            )

    for name in REQUIRED_JOINT_NAMES:
        joint = joints[name]
        values = (
            _require_limit(joint, "lower", lambda item: item.lower),
            _require_limit(joint, "upper", lambda item: item.upper),
            _require_limit(joint, "effort", lambda item: item.effort),
            _require_limit(joint, "velocity", lambda item: item.velocity),
        )
        lower, upper, effort, velocity = values
        if lower >= upper:
            raise IsaacProbeError(
                f"joint {joint.name!r} must have lower limit below upper limit"
            )
        if effort <= 0:
            raise IsaacProbeError(f"joint {joint.name!r} must have positive effort")
        if velocity <= 0:
            raise IsaacProbeError(f"joint {joint.name!r} must have positive velocity")


def _validate_required_structure(links: set[str], joints: dict[str, UrdfJoint]) -> None:
    missing_gripper_joints = set(REQUIRED_GRIPPER_JOINT_NAMES) - joints.keys()
    if missing_gripper_joints:
        name = sorted(missing_gripper_joints)[0]
        raise IsaacProbeError(f"missing required gripper joint {name!r}")
    for joint_name in REQUIRED_GRIPPER_JOINT_NAMES:
        link_name = f"{joint_name}_Link"
        if link_name not in links:
            raise IsaacProbeError(f"missing required gripper link {link_name!r}")
        if joints[joint_name].child_link != link_name:
            raise IsaacProbeError(
                f"gripper joint {joint_name!r} must use child link {link_name!r}"
            )

    missing_sensors = set(REQUIRED_LINK_NAMES) - links
    if missing_sensors:
        name = sorted(missing_sensors)[0]
        raise IsaacProbeError(f"missing required sensor link {name!r}")


def _extraction_root(urdf: Path) -> Path:
    if urdf.parent.name == "URDF_robot_1":
        return urdf.parent.parent
    return urdf.parent


def _validate_root(root_link: str) -> None:
    if root_link != "base_link":
        raise IsaacProbeError(f"root link must be base_link, got {root_link!r}")


def _validate_kinematic_graph(link_names: list[str], joints: list[UrdfJoint]) -> str:
    parent_joints: dict[str, list[UrdfJoint]] = {
        link_name: [] for link_name in link_names
    }
    child_links: dict[str, list[str]] = {link_name: [] for link_name in link_names}
    for joint in joints:
        parent_joints[joint.child_link].append(joint)
        child_links[joint.parent_link].append(joint.child_link)

    for link_name, joints_from_parent in parent_joints.items():
        if len(joints_from_parent) > 1:
            raise IsaacProbeError(
                f"link {link_name!r} must have exactly one parent joint"
            )

    roots = [
        link_name
        for link_name, joints_from_parent in parent_joints.items()
        if not joints_from_parent
    ]
    if len(roots) != 1:
        raise IsaacProbeError(
            "URDF must have exactly one root link and no unreachable links; "
            f"found {len(roots)} roots"
        )
    root_link = roots[0]

    state: dict[str, int] = dict.fromkeys(link_names, 0)
    for start in link_names:
        if state[start] != 0:
            continue
        state[start] = 1
        pending = [(start, iter(child_links[start]))]
        while pending:
            link_name, children = pending[-1]
            child_link = next(children, None)
            if child_link is None:
                state[link_name] = 2
                pending.pop()
                continue
            if state[child_link] == 1:
                raise IsaacProbeError(
                    f"URDF kinematic graph contains a cycle through {child_link!r}"
                )
            if state[child_link] == 0:
                state[child_link] = 1
                pending.append((child_link, iter(child_links[child_link])))

    reachable = {root_link}
    frontier = [root_link]
    while frontier:
        next_frontier: list[str] = []
        for link_name in frontier:
            for child_link in child_links[link_name]:
                if child_link not in reachable:
                    reachable.add(child_link)
                    next_frontier.append(child_link)
        frontier = next_frontier
    unreachable = [link_name for link_name in link_names if link_name not in reachable]
    if unreachable:
        raise IsaacProbeError(
            "URDF contains unreachable links: "
            + ", ".join(repr(name) for name in unreachable)
        )
    return root_link


def resolve_mesh_reference(urdf: Path, reference: str) -> Path:
    decoded_reference = unquote(reference)
    if not reference or "\\" in decoded_reference:
        raise IsaacProbeError(f"unsupported mesh reference {reference!r}")
    extraction_root = _extraction_root(urdf).resolve()
    parsed = urlparse(reference)
    if parsed.query or parsed.fragment:
        raise IsaacProbeError(f"unsupported mesh reference {reference!r}")

    if parsed.scheme == "package":
        if not parsed.netloc:
            raise IsaacProbeError(f"unsupported package mesh reference {reference!r}")
        decoded_path = unquote(parsed.netloc + parsed.path)
        segments = [segment for segment in decoded_path.split("/") if segment]
        if len(segments) < 2:
            raise IsaacProbeError(f"invalid package mesh reference {reference!r}")
    elif parsed.scheme == "file":
        if not parsed.netloc and parsed.path.startswith("/"):
            raise IsaacProbeError(
                f"absolute file mesh reference is not supported: {reference!r}"
            )
        decoded_path = unquote(parsed.netloc + parsed.path)
        segments = [segment for segment in decoded_path.split("/") if segment]
        if not segments:
            raise IsaacProbeError(f"invalid file mesh reference {reference!r}")
    else:
        raise IsaacProbeError(f"unsupported mesh reference scheme {parsed.scheme!r}")

    if any(segment in {".", ".."} for segment in segments):
        raise IsaacProbeError(f"unsafe mesh reference {reference!r}")

    roots = [extraction_root]
    bundle_root = urdf.parent.resolve()
    if bundle_root != extraction_root:
        roots.append(bundle_root)

    last_candidate = roots[0].joinpath(*segments)
    for root in roots:
        candidate = root.joinpath(*segments)
        last_candidate = candidate
        component = root
        try:
            for segment in segments:
                component = component / segment
                mode = component.lstat().st_mode
                if stat.S_ISLNK(mode):
                    raise IsaacProbeError(
                        f"mesh path contains a symbolic link: {component}"
                    )
            candidate = candidate.resolve()
            if not candidate.is_relative_to(root):
                raise IsaacProbeError(
                    f"mesh reference escapes extraction root: {reference!r}"
                )
            mode = candidate.lstat().st_mode
        except FileNotFoundError:
            continue
        except OSError as exc:
            raise IsaacProbeError(f"mesh file does not exist: {candidate}") from exc
        if not stat.S_ISREG(mode):
            raise IsaacProbeError(f"mesh is not a regular file: {candidate}")
        return candidate

    raise IsaacProbeError(f"mesh file does not exist: {last_candidate}")


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file_object:
        for chunk in iter(lambda: file_object.read(1024 * 1024), b""):
            digest.update(chunk)
    return f"sha256:{digest.hexdigest()}"


def _collect_meshes(robot: ET.Element, urdf_path: Path) -> tuple[UrdfMesh, ...]:
    meshes: list[UrdfMesh] = []
    for link in robot.findall("link"):
        for geometry_type in ("visual", "collision"):
            geometry = link.find(f"{geometry_type}/geometry")
            if geometry is None:
                continue
            for mesh in geometry.findall("mesh"):
                reference = mesh.get("filename")
                if not reference:
                    raise IsaacProbeError("mesh element is missing filename")
                resolved_path = resolve_mesh_reference(urdf_path, reference)
                meshes.append(
                    UrdfMesh(
                        reference=reference,
                        resolved_path=resolved_path,
                        geometry_type=geometry_type,
                        sha256=_sha256_file(resolved_path),
                    )
                )
    return tuple(meshes)


def inspect_urdf(path: Path) -> UrdfInventory:
    if not path.is_file():
        raise IsaacProbeError(f"URDF is not a regular file: {path}")
    try:
        robot = ET.parse(path).getroot()
    except ET.ParseError as exc:
        raise IsaacProbeError(f"invalid URDF XML in {path}") from exc
    if robot.tag != "robot":
        raise IsaacProbeError("URDF root element must be robot")

    link_names: list[str] = []
    for link in robot.findall("link"):
        name = _required_attribute(link, "name", "link")
        if name in link_names:
            raise IsaacProbeError(f"duplicate link {name!r}")
        link_names.append(name)

    joints_list: list[UrdfJoint] = []
    joints: dict[str, UrdfJoint] = {}
    for joint_element in robot.findall("joint"):
        joint = _parse_joint(joint_element)
        if joint.name in joints:
            raise IsaacProbeError(f"duplicate joint {joint.name!r}")
        joints[joint.name] = joint
        joints_list.append(joint)

    link_set = set(link_names)
    for joint in joints_list:
        for relation in (joint.parent_link, joint.child_link):
            if relation not in link_set:
                raise IsaacProbeError(
                    f"joint {joint.name!r} references unknown link {relation!r}"
                )

    _validate_required_structure(link_set, joints)
    _validate_required_joints(joints)
    root_link = _validate_kinematic_graph(link_names, joints_list)
    _validate_root(root_link)
    return UrdfInventory(
        robot_name=_required_attribute(robot, "name", "robot"),
        root_link=root_link,
        joints=tuple(joints_list),
        links=tuple(link_names),
        meshes=_collect_meshes(robot, path),
        required_joint_names=REQUIRED_JOINT_NAMES,
    )
