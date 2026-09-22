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

from dataclasses import dataclass
from typing import Literal

from robots.lynsense.isaac.contracts import IsaacProbeError

EventSeverity = Literal["info", "warning", "error"]
EventDecision = Literal["recorded", "blocking"]


@dataclass(frozen=True)
class ImporterEvent:
    category: str
    severity: EventSeverity
    source: str
    message: str
    decision: EventDecision = "recorded"

    def to_dict(self) -> dict[str, object]:
        return {
            "category": self.category,
            "severity": self.severity,
            "source": self.source,
            "message": self.message,
            "decision": self.decision,
        }


@dataclass(frozen=True)
class ImporterContractError(IsaacProbeError):
    events: tuple[ImporterEvent, ...]

    def __post_init__(self) -> None:
        super().__init__(
            "; ".join(
                f"{event.category} [{event.severity}]: {event.message}"
                for event in self.events
            )
        )

    def to_dict(self) -> dict[str, object]:
        return {
            "reason": str(self),
            "events": [event.to_dict() for event in self.events],
        }


RECORDED_CATEGORIES = frozenset(
    {
        "material_change",
        "color_change",
        "texture_change",
        "skipped_link",
        "mimic_joint_decision",
        "fixed_link_reduction",
        "naming_change",
    }
)

BLOCKING_CATEGORIES = frozenset(
    {
        "uncategorized",
        "missing_required_joint",
        "invalid_joint_limit",
        "unreadable_mesh",
        "mesh_substitution",
        "dropped_gripper_link",
        "dropped_gripper_coupling",
        "dropped_camera",
        "dropped_sensor",
    }
)

VALID_SEVERITIES = frozenset({"info", "warning", "error"})


def classify_importer_events(
    events: tuple[ImporterEvent, ...],
) -> tuple[ImporterEvent, ...]:
    classified: list[ImporterEvent] = []
    for event in events:
        if not event.category:
            raise IsaacProbeError("category must not be empty")
        if not event.source:
            raise IsaacProbeError("source must not be empty")
        if not event.message:
            raise IsaacProbeError("message must not be empty")
        if event.severity not in VALID_SEVERITIES:
            raise IsaacProbeError(
                f"severity must be info, warning, or error: {event.severity!r}"
            )

        category = event.category
        message = event.message
        if category not in RECORDED_CATEGORIES | BLOCKING_CATEGORIES:
            message = f"{message} (category={event.category!r})"
            category = "uncategorized"
        decision: EventDecision
        if event.severity == "error" or category in BLOCKING_CATEGORIES:
            decision = "blocking"
        else:
            decision = "recorded"
        classified.append(
            ImporterEvent(
                category=category,
                severity=event.severity,
                source=event.source,
                message=message,
                decision=decision,
            )
        )
    return tuple(classified)


def require_no_blocking_events(events: tuple[ImporterEvent, ...]) -> None:
    classified = classify_importer_events(events)
    if any(event.decision == "blocking" for event in classified):
        raise ImporterContractError(events=classified)
