"""Owned LynrotControl state reader for the Robot One read-only surface."""

from __future__ import annotations

import hashlib
import os
import platform
from pathlib import Path
from typing import Any


INSTANCE = "ea200"
SITE_ID = "robot-one-site"
ROBOT_ID = "robot-one"
EXECUTION_HOST = "rpp-PC"
ROS_DOMAIN_ID = 3
CONFIG_PATH = Path("/home/zxh/zxh/lynrotcontrol/config/ea200.yaml")
CONFIG_SHA256 = "77a4bbd207bf0c441310a8b4c2250d703f262d75441e5b71a29bf014300ea5dd"
OPERATOR_VERIFICATION = {
    "operator": "huhu",
    "verification": "state_read_only",
}
STATE_GROUPS = (
    "left_arm",
    "right_arm",
    "left_gripper",
    "right_gripper",
    "waist_lift",
    "left_force",
    "right_force",
)


class RobotOneReadOnlyError(RuntimeError):
    """The read-only Robot One runtime cannot be safely used."""


class OwnedRobotOneStateReader:
    """Own one LynrotControl claim for no-argument fixed state reads."""

    def __init__(self) -> None:
        self._runtime: Any | None = None
        self._closed = False

    def connect(self) -> dict[str, Any]:
        if self._closed:
            raise RobotOneReadOnlyError("read-only runtime is closed")
        if self._runtime is not None:
            raise RobotOneReadOnlyError("read-only runtime is already connected")
        if platform.node() != EXECUTION_HOST:
            raise RobotOneReadOnlyError("execution host does not match Robot One")
        if os.environ.get("ROS_DOMAIN_ID") != str(ROS_DOMAIN_ID):
            raise RobotOneReadOnlyError("ROS domain does not match Robot One")
        try:
            observed_config_sha256 = hashlib.sha256(
                CONFIG_PATH.read_bytes()
            ).hexdigest()
        except OSError as error:
            raise RobotOneReadOnlyError(
                "cannot read the pinned LynrotControl config"
            ) from error
        if observed_config_sha256 != CONFIG_SHA256:
            raise RobotOneReadOnlyError(
                "LynrotControl config digest does not match the read-only runtime"
            )

        from lynrotcontrol.implementation.config import ROOT
        from lynrotcontrol.interfaces.ownership import initialize_under_ownership
        from lynrotcontrol.lynarmcontrol.implementation import service_ownership
        from lynrotcontrol.lynarmcontrol.implementation.endpoint_inventory import (
            instance_endpoints,
        )

        if ROOT / "config" != CONFIG_PATH.parent:
            raise RobotOneReadOnlyError("LynrotControl root does not match config")
        endpoints = instance_endpoints(INSTANCE)
        scope = service_ownership.endpoint_scope_digest(
            SITE_ID,
            ROBOT_ID,
            endpoints,
        )
        self._runtime = initialize_under_ownership(
            INSTANCE,
            site_id=SITE_ID,
            robot_id=ROBOT_ID,
            endpoint_scope_sha256=scope,
            ros_domain_id=ROS_DOMAIN_ID,
            approved_effects=service_ownership.APPROVED_EFFECTS,
            expected_endpoints=endpoints,
            operator_verification=dict(OPERATOR_VERIFICATION),
        )
        return {"status": "ok"}

    def read(self) -> dict[str, Any]:
        runtime = self._require_runtime()
        identity = runtime.identity()
        return {
            "status": "ok",
            "identity": identity,
            "service": runtime.service_identity(),
            "gripper_cancel_supported": runtime.gripper_cancel_supported(),
            "state": {
                group: runtime.read_state(group) for group in STATE_GROUPS
            },
            "capabilities": {
                "motion": False,
                "gripper": False,
                "chassis": False,
                "perception": False,
            },
        }

    def close(self) -> None:
        if self._closed:
            return
        runtime = self._runtime
        self._closed = True
        self._runtime = None
        if runtime is not None:
            runtime.release()

    def _require_runtime(self) -> Any:
        if self._closed or self._runtime is None:
            raise RobotOneReadOnlyError("read-only runtime is not connected")
        return self._runtime
