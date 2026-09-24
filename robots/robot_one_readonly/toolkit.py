"""Exact one-tool read-only boundary for Robot One."""

from __future__ import annotations

from typing import Any

from rpent.dashboard.events import DashboardEventSink
from rpent.memory import MemoryManager
from rpent.tools.toolkit import Toolkit, readonly


class RobotOneReadOnlyToolkit(Toolkit):
    include_image_reader = False

    def __init__(
        self,
        *,
        reader: Any,
        dashboard_events: DashboardEventSink,
        memory: MemoryManager,
    ) -> None:
        super().__init__(dashboard_events=dashboard_events, memory=memory)
        self._reader = reader
        self.add_tool(
            "read_robot_state",
            {
                "name": "read_robot_state",
                "description": (
                    "Read both arms, both grippers, waist lift, and both force "
                    "sensors from owned LynrotControl. This is observation only."
                ),
                "input_schema": {
                    "type": "object",
                    "properties": {},
                    "additionalProperties": False,
                },
            },
            self.read_robot_state,
        )

    def _register_common_tools(self) -> None:
        """No file, image, finish, or motion tools are exposed."""

    @readonly
    def read_robot_state(self) -> dict[str, Any]:
        return self._reader.read()

    def get_env_state(
        self,
        *,
        command: dict[str, Any],
        result: dict[str, Any],
        elapsed_s: float,
    ) -> dict[str, Any]:
        return {"status": "read_only"}

    def solved(self) -> bool:
        return False

    def close(self) -> None:
        self._reader.close()
