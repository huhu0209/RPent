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

import io
import tarfile
from pathlib import Path


def synthetic_probe_urdf() -> str:
    required_joint_names = (
        "connector_joint",
        *(f"left_joint{i}" for i in range(1, 7)),
        *(f"right_joint{i}" for i in range(1, 7)),
    )
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

    def joint(
        name: str, joint_type: str = "revolute", child_link: str | None = None
    ) -> str:
        child = child_link or f"{name}_link"
        if joint_type == "fixed":
            return (
                f'<joint name="{name}" type="fixed">'
                f'<parent link="base_link"/><child link="{child}"/></joint>'
            )
        return f'''<joint name="{name}" type="{joint_type}">
          <parent link="base_link"/><child link="{child}"/>
          <axis xyz="0 0 1"/>
          <limit lower="-1.0" upper="1.0" effort="200" velocity="3.14"/>
          <dynamics damping="16.6" friction="9.6"/>
        </joint>'''

    required_joints = (
        joint("connector_joint", "prismatic")
        + "\n"
        + "\n".join(joint(name) for name in required_joint_names[1:])
    )
    gripper_joints = "\n".join(
        joint(name, child_link=f"{name}_Link") for name in gripper_names
    )
    moving_links = "\n".join(f'<link name="{name}_Link"/>' for name in gripper_names)
    moving_links += "\n" + "\n".join(
        f'<link name="{name}_link"/>' for name in required_joint_names
    )
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
    return f"""<robot name="fixture">
      <link name="base_link">
        <visual><geometry><mesh filename="package://pkg/meshes/part.stl"/></geometry></visual>
        <collision><geometry><mesh filename="package://pkg/meshes/part.stl"/></geometry></collision>
      </link>
      {moving_links}
      <link name="head_camera_link"/>
      <link name="left_camera_link"/>
      <link name="right_camera_link"/>
      <link name="left_force_sensor_link"/>
      <link name="right_force_sensor_link"/>
      {sensor_joints}
      {required_joints}
      {gripper_joints}
    </robot>"""


def write_valid_probe_archive(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    urdf = synthetic_probe_urdf().encode("utf-8")
    with tarfile.open(path, "w:gz") as archive:
        for name, payload in (
            ("URDF_robot_1/", b""),
            ("pkg/", b""),
            ("pkg/meshes/", b""),
            ("URDF_robot_1/robot.urdf", urdf),
            ("pkg/meshes/part.stl", b"synthetic mesh"),
        ):
            info = tarfile.TarInfo(name)
            if name.endswith("/"):
                info.type = tarfile.DIRTYPE
                info.mode = 0o700
                archive.addfile(info)
                continue
            info.size = len(payload)
            archive.addfile(info, io.BytesIO(payload))
