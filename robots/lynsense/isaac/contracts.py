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
from dataclasses import dataclass
from pathlib import Path
from typing import Any

SUPPORTED_ISAAC_VERSION = re.compile(r"^2023\.1\.\d+(?:[-+][0-9A-Za-z_.]+)*$")
SCREENSHOT_NAMES = (
    "scene-initial.png",
    "scene-action.png",
    "scene-final.png",
)
VISIBLE_SUBSYSTEMS = (
    "chassis",
    "dual_arms",
    "left_gripper",
    "right_gripper",
    "head_camera",
    "wrist_cameras",
)
REQUIRED_CAPABILITIES = frozenset(
    {
        "urdf_importer",
        "articulation",
        "viewport_screenshot",
        "physx_joint_limits",
    }
)
VISUAL_REVIEW_KEYS = {
    "schema_version",
    "status",
    "screenshots",
    "visible_subsystems",
    "reviewer",
    "reviewed_at",
    "notes",
}
REVIEWED_AT_PATTERN = re.compile(r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z$")


class IsaacProbeError(RuntimeError):
    """A Lynsense Isaac probe contract was violated."""


@dataclass(frozen=True)
class IsaacEnvironment:
    executable: Path
    semantic_version: str
    kit_version: str
    python_version: str
    renderer: str
    run_mode: str = "desktop"
    capabilities: frozenset[str] = frozenset()

    def to_dict(self) -> dict[str, str]:
        return {
            "executable": str(self.executable),
            "semantic_version": self.semantic_version,
            "kit_version": self.kit_version,
            "python_version": self.python_version,
            "renderer": self.renderer,
            "run_mode": self.run_mode,
            "capabilities": sorted(self.capabilities),
        }


def validate_environment_version(version: str) -> None:
    if not SUPPORTED_ISAAC_VERSION.fullmatch(version):
        raise IsaacProbeError(
            f"unsupported Isaac Sim version {version!r}; expected 2023.1.*"
        )


def validate_environment(environment: IsaacEnvironment) -> None:
    """Require non-empty runtime metadata and all probe capabilities."""
    if not environment.executable.name:
        raise IsaacProbeError("executable must be supplied")
    if not environment.semantic_version:
        raise IsaacProbeError("semantic_version must be supplied")
    validate_environment_version(environment.semantic_version)
    for field in ("kit_version", "python_version", "renderer", "run_mode"):
        if not getattr(environment, field):
            raise IsaacProbeError(f"{field} must not be empty")
    if not REQUIRED_CAPABILITIES.issubset(environment.capabilities):
        raise IsaacProbeError("capabilities must include all probe capabilities")


def environment_from_dict(document: Any) -> IsaacEnvironment:
    expected_keys = {
        "executable",
        "semantic_version",
        "kit_version",
        "python_version",
        "renderer",
        "run_mode",
        "capabilities",
    }
    if not isinstance(document, dict) or set(document) != expected_keys:
        raise IsaacProbeError("environment fields do not match the contract")
    for field in (
        "executable",
        "semantic_version",
        "kit_version",
        "python_version",
        "renderer",
        "run_mode",
    ):
        if not isinstance(document[field], str):
            raise IsaacProbeError(f"{field} must be a string")
    capabilities = document["capabilities"]
    if not isinstance(capabilities, list) or not all(
        isinstance(capability, str) for capability in capabilities
    ):
        raise IsaacProbeError("capabilities must be a list of strings")
    try:
        environment = IsaacEnvironment(
            executable=Path(document["executable"]),
            semantic_version=document["semantic_version"],
            kit_version=document["kit_version"],
            python_version=document["python_version"],
            renderer=document["renderer"],
            run_mode=document["run_mode"],
            capabilities=frozenset(document["capabilities"]),
        )
    except (TypeError, ValueError) as exc:
        raise IsaacProbeError("environment fields do not match the contract") from exc
    validate_environment(environment)
    return environment


@dataclass(frozen=True)
class VisualReview:
    status: str
    screenshots: tuple[str, ...]
    visible_subsystems: tuple[str, ...]
    reviewer: str
    reviewed_at: str
    notes: str
    schema_version: int = 1


def validate_visual_review(document: Any) -> None:
    if not isinstance(document, dict) or set(document) != VISUAL_REVIEW_KEYS:
        raise IsaacProbeError("visual review fields do not match the contract")
    if type(document["schema_version"]) is not int or document["schema_version"] != 1:
        raise IsaacProbeError("schema_version must be 1")
    status = document["status"]
    if not isinstance(status, str) or status not in {"accepted", "rejected"}:
        raise IsaacProbeError("status must be accepted or rejected")
    screenshots = document["screenshots"]
    if not isinstance(screenshots, list) or screenshots != list(SCREENSHOT_NAMES):
        raise IsaacProbeError("screenshots do not match the required contract")
    visible_subsystems = document["visible_subsystems"]
    if not isinstance(visible_subsystems, list) or visible_subsystems != list(
        VISIBLE_SUBSYSTEMS
    ):
        raise IsaacProbeError("visible subsystems do not match the required contract")
    for field in ("reviewer", "notes"):
        if not isinstance(document[field], str) or not document[field]:
            raise IsaacProbeError(f"{field} must be a non-empty string")
    reviewed_at = document["reviewed_at"]
    if not isinstance(reviewed_at, str) or not REVIEWED_AT_PATTERN.fullmatch(
        reviewed_at
    ):
        raise IsaacProbeError("reviewed_at must be an ISO-8601 UTC timestamp")
    if document["status"] == "rejected" and not document["notes"].strip():
        raise IsaacProbeError("a rejected review must explain the reason in notes")
