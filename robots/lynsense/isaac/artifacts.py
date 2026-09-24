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
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from robots.lynsense.isaac.contracts import SCREENSHOT_NAMES


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"non-standard JSON numeric constant {value}")


@dataclass(frozen=True)
class ArtifactPaths:
    run_directory: Path
    extracted_root: Path
    robot_urdf: Path
    generated_usd: Path
    environment_json: Path
    technical_summary_json: Path
    visual_review_json: Path
    isaac_log: Path
    screenshots: tuple[Path, ...]


def create_artifact_paths(root: Path, run_id: str) -> ArtifactPaths:
    run_directory = root / run_id
    extracted_root = run_directory / "extracted"
    return ArtifactPaths(
        run_directory=run_directory,
        extracted_root=extracted_root,
        robot_urdf=extracted_root / "URDF_robot_1" / "robot.urdf",
        generated_usd=run_directory / "robot_1.usd",
        environment_json=run_directory / "environment.json",
        technical_summary_json=run_directory / "technical-summary.json",
        visual_review_json=run_directory / "visual-review.json",
        isaac_log=run_directory / "isaac.log",
        screenshots=tuple(run_directory / name for name in SCREENSHOT_NAMES),
    )


def atomic_write_json(path: Path, value: Any) -> None:
    try:
        payload = json.dumps(value, indent=2, sort_keys=True, allow_nan=False)
    except ValueError as exc:
        raise ValueError("value contains non-finite JSON numbers") from exc
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.tmp-",
            delete=False,
        ) as temporary_file:
            temporary_path = Path(temporary_file.name)
            temporary_file.write(payload)
            temporary_file.flush()
            os.fsync(temporary_file.fileno())
        os.replace(temporary_path, path)
        temporary_path = None
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)


def load_json_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(
            path.read_text(encoding="utf-8"),
            parse_constant=_reject_json_constant,
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"could not read JSON object from {path}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"JSON root in {path} is not an object")
    return value
