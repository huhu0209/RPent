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

from robots.lynsense.isaac.artifacts import (
    atomic_write_json,
    create_artifact_paths,
    load_json_object,
)


def test_artifact_paths_are_complete_and_run_scoped(tmp_path: Path) -> None:
    paths = create_artifact_paths(tmp_path, "probe-run")
    assert paths.run_directory == tmp_path / "probe-run"
    assert paths.extracted_root == tmp_path / "probe-run" / "extracted"
    assert paths.robot_urdf == (
        tmp_path / "probe-run" / "extracted" / "URDF_robot_1" / "robot.urdf"
    )
    assert paths.generated_usd == tmp_path / "probe-run" / "robot_1.usd"
    assert paths.environment_json.name == "environment.json"
    assert paths.technical_summary_json.name == "technical-summary.json"
    assert paths.visual_review_json.name == "visual-review.json"
    assert [path.name for path in paths.screenshots] == [
        "scene-initial.png",
        "scene-action.png",
        "scene-final.png",
    ]
    assert paths.isaac_log.name == "isaac.log"


def test_atomic_json_replaces_only_after_success(tmp_path: Path) -> None:
    target = tmp_path / "summary.json"
    atomic_write_json(target, {"ok": True})
    assert json.loads(target.read_text(encoding="utf-8")) == {"ok": True}
    assert list(tmp_path.glob(".summary.json.tmp-*")) == []

    with pytest.raises(ValueError, match="non-finite"):
        atomic_write_json(target, {"value": float("nan")})
    assert json.loads(target.read_text(encoding="utf-8")) == {"ok": True}


def test_json_object_loader_rejects_non_objects_and_invalid_json(
    tmp_path: Path,
) -> None:
    valid = tmp_path / "valid.json"
    valid.write_text(json.dumps({"value": 1}), encoding="utf-8")
    assert load_json_object(valid) == {"value": 1}

    array = tmp_path / "array.json"
    array.write_text("[]", encoding="utf-8")
    malformed = tmp_path / "malformed.json"
    malformed.write_text("{", encoding="utf-8")
    for path in (array, malformed):
        with pytest.raises(ValueError):
            load_json_object(path)
