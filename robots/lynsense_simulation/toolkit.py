"""Three-tool model boundary that writes only the validated simulation plan."""

from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path
from typing import Any

from robots.lynsense.simulation.rpent_plan import (
    PlanError,
    task_description,
    validate_plan,
)
from rpent.memory import MemoryManager
from rpent.tools.toolkit import Toolkit, readonly


class LynsenseSimulationToolkit(Toolkit):
    include_image_reader = False

    def __init__(
        self,
        *,
        dashboard_events: Any,
        memory: MemoryManager,
        plan_path: Path,
    ) -> None:
        super().__init__(
            dashboard_events=dashboard_events,
            memory=memory,
        )
        self._plan_path = plan_path.expanduser().resolve()
        self._accepted_plan: dict[str, Any] | None = None
        self.add_tool(
            "read_simulation_task",
            {
                "name": "read_simulation_task",
                "description": (
                    "Read the isolated Webots single-box task and strict plan "
                    "schema."
                ),
                "input_schema": {
                    "type": "object",
                    "properties": {},
                    "additionalProperties": False,
                },
            },
            self.read_simulation_task,
        )
        self.add_tool(
            "submit_simulation_plan",
            {
                "name": "submit_simulation_plan",
                "description": (
                    "Validate one complete plan and atomically write it to the "
                    "configured plan path."
                ),
                "input_schema": {
                    "type": "object",
                    "properties": {
                        "plan": {
                            "type": "object",
                            "required": ["task_id", "version", "actions"],
                        }
                    },
                    "required": ["plan"],
                    "additionalProperties": False,
                },
            },
            self.submit_simulation_plan,
        )
        self.add_tool(
            "finish",
            {
                "name": "finish",
                "description": (
                    "Finish after a plan decision. Save the accepted plan "
                    "before finishing."
                ),
                "input_schema": {
                    "type": "object",
                    "properties": {
                        "status": {
                            "type": "string",
                            "enum": ["success", "failure", "stuck"],
                        },
                        "summary": {"type": "string", "minLength": 1},
                    },
                    "required": ["status", "summary"],
                    "additionalProperties": False,
                },
            },
            self.finish,
        )

    def _register_common_tools(self) -> None:
        """No file, image, memory, or common control tools are model-visible."""

    @readonly
    def read_simulation_task(self) -> dict[str, Any]:
        return {"task": task_description()}

    @readonly
    def submit_simulation_plan(self, plan: dict[str, Any]) -> dict[str, Any]:
        if self._accepted_plan is not None:
            return {
                "accepted": False,
                "error": "simulation plan has already been accepted for this session",
                "error_code": "plan_already_submitted",
            }

        try:
            actions = validate_plan(plan)
        except PlanError as exc:
            return {"error": str(exc)}

        document = {
            "task_id": plan["task_id"],
            "version": plan["version"],
            "actions": list(actions),
        }
        self._atomic_write_json(document)
        self._accepted_plan = document
        return {
            "accepted": True,
            "action_count": len(actions),
            "path": str(self._plan_path),
        }

    @readonly
    def finish(self, status: str, summary: str) -> dict[str, Any]:
        if status == "success" and self._accepted_plan is None:
            return {
                "error": "finish(status='success') requires an accepted simulation plan",
                "error_code": "success_requires_accepted_plan",
            }
        return {"_finish": True, "status": status, "summary": summary}

    def get_env_state(
        self,
        *,
        command: dict[str, Any],
        result: dict[str, Any],
        elapsed_s: float,
    ) -> dict[str, Any]:
        return {
            "environment": "webots-isolated",
            "plan_path": str(self._plan_path),
            "plan_written": self._plan_path.exists(),
            "solved": False,
        }

    def solved(self) -> bool:
        return False

    def _atomic_write_json(self, document: dict[str, Any]) -> None:
        self._plan_path.parent.mkdir(parents=True, exist_ok=True)
        temporary_name: str | None = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w",
                encoding="utf-8",
                dir=self._plan_path.parent,
                prefix=f".{self._plan_path.name}.",
                suffix=".tmp",
                delete=False,
            ) as stream:
                temporary_name = stream.name
                json.dump(document, stream, ensure_ascii=False, allow_nan=False)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary_name, self._plan_path)
            temporary_name = None
        finally:
            if temporary_name is not None:
                Path(temporary_name).unlink(missing_ok=True)
