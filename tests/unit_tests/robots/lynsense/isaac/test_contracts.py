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

import json
from pathlib import Path

import pytest

from robots.lynsense.isaac.contracts import (
    IsaacEnvironment,
    IsaacProbeError,
    environment_from_dict,
    validate_environment,
    validate_environment_version,
    validate_visual_review,
)


def _environment(version: str = "2023.1.1") -> IsaacEnvironment:
    return IsaacEnvironment(
        executable=Path("/opt/isaac-sim/python.sh"),
        semantic_version=version,
        kit_version="106.0.0",
        python_version="3.10.14",
        renderer="RTX",
        run_mode="desktop",
        capabilities=frozenset(
            {
                "urdf_importer",
                "articulation",
                "viewport_screenshot",
                "physx_joint_limits",
            }
        ),
    )


@pytest.mark.parametrize(
    "version",
    [
        "2023.1.0",
        "2023.1.1",
        "2023.1.9",
        "2023.1.1-rc.8+2023.1.688.573e0291.tc",
    ],
)
def test_environment_accepts_only_the_company_2023_1_family(version: str) -> None:
    assert validate_environment_version(version) is None


@pytest.mark.parametrize(
    "version",
    ["2022.2.1", "2023.2.0", "4.2.0", "2023.1", "2023.1.rc", "latest"],
)
def test_environment_rejects_unapproved_versions(version: str) -> None:
    with pytest.raises(IsaacProbeError, match="2023.1"):
        validate_environment_version(version)


def test_environment_serializes_without_secret_values() -> None:
    document = _environment().to_dict()
    assert document == {
        "executable": "/opt/isaac-sim/python.sh",
        "semantic_version": "2023.1.1",
        "kit_version": "106.0.0",
        "python_version": "3.10.14",
        "renderer": "RTX",
        "run_mode": "desktop",
        "capabilities": [
            "articulation",
            "physx_joint_limits",
            "urdf_importer",
            "viewport_screenshot",
        ],
    }
    json.dumps(document, allow_nan=False)


def test_environment_json_round_trip_rejects_extra_fields() -> None:
    document = _environment().to_dict()
    assert environment_from_dict(document) == _environment()
    document["license_token"] = "secret"
    with pytest.raises(IsaacProbeError, match="fields"):
        environment_from_dict(document)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("executable", 1),
        ("semantic_version", 2023.1),
        ("kit_version", ["106.0.0"]),
        ("python_version", {"version": "3.10.14"}),
        ("renderer", None),
        ("run_mode", True),
        ("capabilities", "articulation"),
        ("capabilities", ["articulation", 1]),
    ],
)
def test_environment_rejects_malformed_field_types(field: str, value: object) -> None:
    document = _environment().to_dict()
    document[field] = value
    with pytest.raises(IsaacProbeError, match=field):
        environment_from_dict(document)


def test_environment_metadata_and_capabilities_must_be_complete() -> None:
    complete = _environment()
    assert validate_environment(complete) is None
    for field in (
        "executable",
        "semantic_version",
        "kit_version",
        "python_version",
        "renderer",
        "run_mode",
    ):
        changes = {field: Path("") if field == "executable" else ""}
        with pytest.raises(IsaacProbeError, match=field):
            validate_environment(IsaacEnvironment(**{**complete.__dict__, **changes}))
    with pytest.raises(IsaacProbeError, match="capabilities"):
        validate_environment(
            IsaacEnvironment(**{**complete.__dict__, "capabilities": frozenset()})
        )


def _visual_review(**changes: object) -> dict[str, object]:
    document: dict[str, object] = {
        "schema_version": 1,
        "status": "accepted",
        "screenshots": [
            "scene-initial.png",
            "scene-action.png",
            "scene-final.png",
        ],
        "visible_subsystems": [
            "chassis",
            "dual_arms",
            "left_gripper",
            "right_gripper",
            "head_camera",
            "wrist_cameras",
        ],
        "reviewer": "company-reviewer",
        "reviewed_at": "2026-09-22T10:30:00Z",
        "notes": "Company model matches the supplied URDF.",
    }
    document.update(changes)
    return document


def test_visual_review_accepts_the_complete_contract() -> None:
    assert validate_visual_review(_visual_review()) is None


@pytest.mark.parametrize(
    "changes",
    [
        {"schema_version": True},
        {"status": ["accepted"]},
        {"screenshots": "scene-initial.png"},
        {"screenshots": [1, 2, 3]},
        {"visible_subsystems": "chassis"},
        {"visible_subsystems": [{"name": "chassis"}]},
        {"reviewer": 10},
        {"reviewed_at": 20260922103000},
        {"notes": None},
    ],
)
def test_visual_review_rejects_malformed_field_types(
    changes: dict[str, object],
) -> None:
    with pytest.raises(IsaacProbeError):
        validate_visual_review(_visual_review(**changes))


def test_visual_review_requires_all_visual_and_metadata_fields() -> None:
    invalid = [
        _visual_review(status="passed"),
        _visual_review(screenshots=["scene-initial.png"]),
        _visual_review(visible_subsystems=["chassis"]),
        _visual_review(reviewer=""),
        _visual_review(reviewed_at="not-a-timestamp"),
        _visual_review(status="rejected", notes=""),
    ]
    for document in invalid:
        with pytest.raises(IsaacProbeError):
            validate_visual_review(document)


def test_rejected_visual_review_requires_reason() -> None:
    document = _visual_review(
        status="rejected",
        notes="The left gripper geometry does not match.",
    )
    assert validate_visual_review(document) is None
