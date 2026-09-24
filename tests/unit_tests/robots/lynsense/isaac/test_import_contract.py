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

import pytest

from robots.lynsense.isaac.import_contract import (
    ImporterEvent,
    classify_importer_events,
    require_no_blocking_events,
)


def event(category: str, severity: str = "warning") -> ImporterEvent:
    return ImporterEvent(
        category=category,
        severity=severity,
        source="URDF importer",
        message="fixture message",
        decision="recorded",
    )


@pytest.mark.parametrize(
    "category",
    [
        "material_change",
        "color_change",
        "texture_change",
        "skipped_link",
        "mimic_joint_decision",
        "fixed_link_reduction",
        "naming_change",
    ],
)
def test_non_destructive_changes_are_recorded(category: str) -> None:
    result = classify_importer_events([event(category)])
    assert result[0].decision == "recorded"


@pytest.mark.parametrize(
    "category",
    [
        "missing_required_joint",
        "invalid_joint_limit",
        "unreadable_mesh",
        "mesh_substitution",
        "dropped_gripper_link",
        "dropped_gripper_coupling",
        "dropped_camera",
        "dropped_sensor",
    ],
)
def test_destructive_changes_block(category: str) -> None:
    classified = classify_importer_events([event(category)])
    assert classified[0].decision == "blocking"
    with pytest.raises(Exception, match=category):
        require_no_blocking_events(classified)


def test_error_severity_and_unknown_category_block() -> None:
    classified = classify_importer_events([event("material_change", severity="error")])
    assert classified[0].decision == "blocking"
    with pytest.raises(Exception, match="error"):
        require_no_blocking_events(classified)

    with pytest.raises(Exception, match="uncategorized"):
        require_no_blocking_events(
            classify_importer_events([event("unknown_category")])
        )


def test_require_does_not_trust_caller_supplied_decision() -> None:
    unclassified = ImporterEvent(
        category="material_change",
        severity="error",
        source="URDF importer",
        message="fixture message",
        decision="recorded",
    )

    with pytest.raises(Exception, match="error"):
        require_no_blocking_events([unclassified])
