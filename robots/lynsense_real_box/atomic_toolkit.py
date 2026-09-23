"""Model-facing Toolkit for guarded Robot One atomic capabilities."""

from __future__ import annotations

import copy
from collections.abc import Callable, Mapping
from typing import Any, Protocol

from rpent.dashboard.events import DashboardEventSink
from rpent.memory import MemoryManager
from rpent.tools.toolkit import ToolResult, Toolkit, readonly

from robots.lynsense_real_box.atomic_adapter import (
    LynrotControlAtomicAdapter,
    RuntimeIdentityReader,
)
from robots.lynsense_real_box.atomic_contract import (
    atomic_call_rejection,
    atomic_tool_specs,
)
from robots.lynsense_real_box.atomic_profile import (
    AtomicCapabilityProfile,
    atomic_profile_hash,
)
from robots.lynsense_real_box.evidence import EvidenceRecorder


class BoxPerceptionCapability(Protocol):
    """The narrow perception boundary owned by this Toolkit."""

    def detect_box(self) -> dict[str, Any]:
        """Run one bounded perception request."""

    def close(self) -> None:
        """Release perception-owned resources."""


class LynsenseAtomicToolkit(Toolkit):
    """Expose only reviewed atomic capabilities to the API Planner."""

    include_image_reader = False

    class _NoStepStore:
        def latest_record(self) -> None:
            return None

    def _register_common_tools(self) -> None:
        """The atomic boundary intentionally exposes no common tools."""

    def get_tools_spec(self) -> list[dict[str, Any]]:
        """Return stable local schemas without output-dir templating."""

        return [copy.deepcopy(spec) for spec, _ in self._tools.values()]

    def __init__(
        self,
        *,
        adapter: LynrotControlAtomicAdapter,
        perception: BoxPerceptionCapability,
        perception_identity: RuntimeIdentityReader | None = None,
        evidence: EvidenceRecorder,
        dashboard_events: DashboardEventSink,
        memory: MemoryManager,
        profile: AtomicCapabilityProfile,
    ) -> None:
        expected_profile_sha256 = atomic_profile_hash(profile)
        self._profile = profile
        self._profile_sha256 = expected_profile_sha256
        if profile.mode != "dry_run":
            raise ValueError(
                "the offline atomic Toolkit rejects mode='live'; live dispatch "
                "requires a separate reviewed integration"
            )
        if (
            adapter.profile_sha256 != expected_profile_sha256
            or evidence.profile_sha256 != expected_profile_sha256
        ):
            raise ValueError(
                "adapter, evidence, and Toolkit profiles must have the same "
                "SHA-256"
            )
        perception_identity_reason = self._perception_identity_reason(
            perception_identity,
            expected_profile_sha256,
        )
        if perception_identity_reason is not None:
            raise ValueError(
                "perception identity rejected: "
                + perception_identity_reason
            )
        super().__init__(
            dashboard_events=dashboard_events,
            memory=memory,
            state=self._NoStepStore(),
        )
        self._adapter = adapter
        self._perception = perception
        self._perception_identity = perception_identity
        self._evidence = evidence
        self._closed = False
        self._finish_success = False

        for spec in atomic_tool_specs(profile):
            name = spec["name"]
            self.add_tool(name, spec, getattr(self, name))

    def _perception_identity_reason(
        self,
        identity: RuntimeIdentityReader | None,
        profile_sha256: str,
    ) -> str | None:
        if self._profile.version < 2:
            return None
        if identity is None:
            return "perception_identity_missing"
        try:
            receipt = identity.read()
            if not isinstance(receipt, Mapping):
                return "perception_identity_invalid"
            binding = self._profile.perception_binding
            assert binding is not None
            expected = {
                "profile_sha256": profile_sha256,
                "pose_topic": binding.pose_topic,
                "status_topic": binding.status_topic,
                "trigger_service": binding.trigger_service,
                "expected_frame": binding.expected_frame,
            }
            if set(receipt) != set(expected) or any(
                receipt.get(key) != value for key, value in expected.items()
            ):
                return "perception_identity_mismatch"
            return None
        except Exception:
            return "perception_identity_invalid"

    @readonly
    def read_capability_state(self) -> dict[str, Any]:
        """Read normalized state without exposing raw subsystem objects."""

        return self._record_name(
            "read_capability_state",
            {},
            self._adapter.read_capability_state(),
        )

    @readonly
    def detect_box(self) -> dict[str, Any]:
        """Run one perception request through the owned capability."""

        rejection = self._guard("detect_box", {})
        if rejection is not None:
            return rejection
        intent_evidence_id = self._record_intent(
            "detect_box",
            {},
            self._adapter.capability_mode,
        )
        if intent_evidence_id is None:
            return {
                "status": "failed",
                "reason": "evidence_exception",
                "error": "tool intent evidence unavailable",
                "capability_mode": self._adapter.capability_mode,
            }
        perception_reason = self._perception_identity_reason(
            self._perception_identity,
            self._profile_sha256,
        )
        if perception_reason is not None:
            result = {
                "status": "rejected",
                "reason": perception_reason,
                "capability_mode": self._adapter.capability_mode,
            }
            return self._record_name(
                "detect_box",
                {},
                result,
                intent_evidence_id=intent_evidence_id,
            )
        transition = self._adapter.begin_perception()
        if transition.get("status") != "ok":
            result = {
                "status": "rejected",
                "reason": transition.get("reason", "capability_busy"),
                "capability_mode": self._adapter.capability_mode,
            }
        try:
            if transition.get("status") == "ok":
                result = self._perception.detect_box()
        except Exception:
            result = {
                "status": "failed",
                "reason": "perception_exception",
            }
        finally:
            if transition.get("status") == "ok":
                self._adapter.end_perception()
        try:
            return self._record_name(
                "detect_box",
                {},
                result,
                intent_evidence_id=intent_evidence_id,
            )
        except Exception:
            return self._completion_failure(
                "detect_box",
                intent_evidence_id,
                self._adapter.capability_mode,
            )

    def move_chassis(self, *, profile_name: str) -> dict[str, Any]:
        """Execute one reviewed chassis capability."""

        arguments = {"profile_name": profile_name}
        rejection = self._guard("move_chassis", arguments)
        if rejection is not None:
            return rejection
        return self._adapter_call(
            "move_chassis",
            arguments,
            lambda: self._adapter.move_chassis(profile_name),
        )

    def move_waist(self, *, profile_name: str) -> dict[str, Any]:
        """Execute one reviewed waist capability."""

        arguments = {"profile_name": profile_name}
        rejection = self._guard("move_waist", arguments)
        if rejection is not None:
            return rejection
        return self._adapter_call(
            "move_waist",
            arguments,
            lambda: self._adapter.move_waist(profile_name),
        )

    def move_dual_arms(self, *, profile_name: str) -> dict[str, Any]:
        """Execute one paired dual-arm capability."""

        arguments = {"profile_name": profile_name}
        rejection = self._guard("move_dual_arms", arguments)
        if rejection is not None:
            return rejection
        return self._adapter_call(
            "move_dual_arms",
            arguments,
            lambda: self._adapter.move_dual_arms(profile_name),
        )

    def set_dual_grippers(self, *, command: str) -> dict[str, Any]:
        """Execute one paired gripper capability."""

        arguments = {"command": command}
        rejection = self._guard("set_dual_grippers", arguments)
        if rejection is not None:
            return rejection
        return self._adapter_call(
            "set_dual_grippers",
            arguments,
            lambda: self._adapter.set_dual_grippers(command),
        )

    def stop_all(self) -> dict[str, Any]:
        """Request bounded software stops; this is not an E-stop."""

        return self._adapter_call("stop_all", {}, self._adapter.stop_all)

    def acknowledge_operator_review(self, operator: str) -> dict[str, Any]:
        """Record a non-model operator acknowledgement and open failure exit."""

        if self._closed:
            return {
                "status": "closed",
                "capability_mode": self._adapter.capability_mode,
            }
        if not isinstance(operator, str) or not operator.strip():
            return {
                "status": "rejected",
                "reason": "operator_invalid",
                "capability_mode": self._adapter.capability_mode,
            }
        try:
            self._evidence.record_operator_acknowledgement(operator)
        except Exception:
            return {
                "status": "failed",
                "reason": "evidence_exception",
                "capability_mode": self._adapter.capability_mode,
            }
        return self._adapter.acknowledge_operator_review()

    def resume_from_acknowledged_review(self) -> dict[str, Any]:
        """Resume idle operation after external acknowledgement."""

        if self._closed:
            return {
                "status": "closed",
                "capability_mode": self._adapter.capability_mode,
            }
        return self._adapter.resume_from_acknowledged_review()

    def finish(self, *, status: str) -> dict[str, Any]:
        """Finish only from an explicitly safe capability mode."""

        arguments = {"status": status}
        rejection = self._guard(
            "finish",
            arguments,
            result_error="finish refused",
        )
        if rejection is not None:
            return rejection
        result = self._record_name(
            "finish",
            arguments,
            {
                "status": "ok",
                "reason": None,
                "_finish": True,
            },
        )
        if (
            status == "success"
            and result.get("status") == "ok"
            and result.get("_finish") is True
        ):
            self._finish_success = True
        return result

    def close(self) -> None:
        """Close owned perception, adapter, and evidence resources once."""

        if self._closed:
            return
        if self._adapter.capability_mode in {
            "chassis_moving",
            "waist_moving",
            "dual_arms_moving",
            "grippers_moving",
            "stopping",
        }:
            raise RuntimeError(
                "cannot close while an atomic capability is active; request "
                "and confirm stop first"
            )
        errors: list[BaseException] = []
        for close in (
            self._perception.close,
            self._adapter.close,
            self._evidence.close,
        ):
            try:
                close()
            except BaseException as error:
                errors.append(error)
        if errors:
            raise RuntimeError(
                "atomic Toolkit close failed: "
                + "; ".join(str(error) for error in errors)
            )
        self._closed = True

    def solved(self) -> bool:
        """Return whether an explicitly safe success finish occurred."""

        return self._finish_success

    def get_env_state(
        self,
        *,
        command: dict[str, Any],
        result: dict[str, Any],
        elapsed_s: float,
    ) -> dict[str, Any]:
        """Keep base-class state capture inside the atomic result."""

        del command, elapsed_s
        return result

    def execute_tool(
        self,
        name: str,
        input_dict: dict[str, Any],
    ) -> ToolResult:
        """Validate the exact schema before entering the base dispatcher."""

        if name not in self._tools:
            return self._tool_result(
                name,
                self._dispatch_rejection(name, {}, "unknown_tool"),
            )
        if not isinstance(input_dict, Mapping):
            return self._tool_result(
                name,
                self._dispatch_rejection(name, {}, "schema_invalid"),
            )
        schema = self._tools[name][0]["input_schema"]
        expected = set(schema.get("properties", {}))
        if set(input_dict) != expected:
            return self._tool_result(
                name,
                self._dispatch_rejection(
                    name,
                    dict(input_dict),
                    "schema_invalid",
                ),
            )

        result = super().execute_tool(name, dict(input_dict)).result
        if not isinstance(result, dict) or "evidence_id" not in result:
            reason = "handler_error" if "error" in result else "missing_evidence"
            result = self._dispatch_rejection(
                name,
                dict(input_dict),
                reason,
            )
        return self._tool_result(name, result)

    def _guard(
        self,
        tool: str,
        arguments: Mapping[str, Any],
        result_error: str | None = None,
    ) -> dict[str, Any] | None:
        state_result = self._adapter.read_capability_state()
        robot_state = (
            state_result.get("robot_state")
            if isinstance(state_result, dict)
            else None
        )
        rejection = atomic_call_rejection(
            tool=tool,
            arguments=arguments,
            profile=self._profile,
            capability_mode=self._adapter.capability_mode,
            robot_state=robot_state if isinstance(robot_state, Mapping) else None,
        )
        if rejection is None:
            if tool in {
                "move_chassis",
                "move_waist",
                "move_dual_arms",
                "set_dual_grippers",
            }:
                runtime_reason = self._adapter.runtime_identity_reason()
                if runtime_reason is not None:
                    return self._record_name(
                        tool,
                        dict(arguments),
                        {
                            "status": "rejected",
                            "reason": runtime_reason,
                        },
                    )
            return None

        result = {
            "status": "rejected",
            "reason": rejection,
        }
        if result_error is not None:
            result["error"] = result_error
        return self._record_name(
            tool,
            dict(arguments),
            result,
        )

    def cancel_active_and_wait(self) -> None:
        """Request a confirmed software stop before waiting on cancellation."""

        stop_result: dict[str, Any] | None = None
        stop_error: BaseException | None = None
        with self._operation_lock:
            tool_active = self._active_operation is not None
        if tool_active or self._adapter.capability_mode not in {"idle", "closed"}:
            try:
                stop_result = self._adapter.stop_all()
            except BaseException as error:
                stop_error = error
        try:
            super().cancel_active_and_wait()
        finally:
            if stop_error is not None:
                raise RuntimeError("atomic cancellation stop failed") from stop_error
            if stop_result is not None and stop_result.get("status") != "ok":
                raise RuntimeError(
                    "atomic cancellation stop was not confirmed: "
                    + str(stop_result.get("reason", "unknown"))
                )

    def _adapter_call(
        self,
        tool: str,
        arguments: Mapping[str, Any],
        request: Callable[[], dict[str, Any]],
    ) -> dict[str, Any]:
        state_before = self._adapter.capability_mode
        intent_evidence_id = self._record_intent(tool, dict(arguments), state_before)
        if intent_evidence_id is None:
            return {
                "status": "failed",
                "reason": "evidence_exception",
                "error": "tool intent evidence unavailable",
                "capability_mode": self._adapter.capability_mode,
            }
        try:
            result = request()
        except Exception:
            result = {
                "status": "failed",
                "reason": "adapter_exception",
            }
        try:
            return self._record_name(
                tool,
                dict(arguments),
                result,
                state_before=state_before,
                intent_evidence_id=intent_evidence_id,
            )
        except Exception:
            return self._completion_failure(
                tool,
                intent_evidence_id,
                state_before,
            )

    def _completion_failure(
        self,
        tool: str,
        intent_evidence_id: str,
        state_before: str,
    ) -> dict[str, Any]:
        try:
            stop_result = self._adapter.stop_all()
            stop_status = stop_result.get("status", "unknown")
            stop_reason = stop_result.get("reason")
        except Exception as stop_error:
            stop_status = "exception"
            stop_reason = str(stop_error)
        failure_id = self._evidence.record_tool_completion_failure(
            tool,
            intent_evidence_id,
            state_before,
            self._adapter.capability_mode,
            "completion_evidence_failed",
            stop_status,
            stop_reason,
        )
        return {
            "status": "failed",
            "reason": "evidence_exception",
            "error": "tool completion evidence unavailable",
            "capability_mode": self._adapter.capability_mode,
            "profile_id": self._profile.profile_id,
            "profile_sha256": self._profile_sha256,
            "stop_status": stop_status,
            "stop_reason": stop_reason,
            "intent_evidence_id": intent_evidence_id,
            "evidence_id": failure_id,
        }

    def _dispatch_rejection(
        self,
        name: str,
        arguments: Mapping[str, Any],
        reason: str,
    ) -> dict[str, Any]:
        return self._record_name(
            name,
            dict(arguments),
            {
                "status": "rejected",
                "reason": reason,
            },
        )

    def _record_name(
        self,
        tool: str,
        arguments: dict[str, Any],
        result: dict[str, Any],
        state_before: str | None = None,
        intent_evidence_id: str | None = None,
    ) -> dict[str, Any]:
        sealed = dict(result)
        sealed.setdefault("capability_mode", self._adapter.capability_mode)
        sealed["profile_id"] = self._profile.profile_id
        sealed["profile_sha256"] = self._profile_sha256
        evidence_id = self._evidence.record_tool_call(
            tool,
            arguments,
            sealed,
            state_before or self._adapter.capability_mode,
            self._adapter.capability_mode,
            intent_evidence_id=intent_evidence_id,
        )
        sealed["evidence_id"] = evidence_id
        return sealed

    def _record_intent(
        self,
        tool: str,
        arguments: dict[str, Any],
        state_before: str,
    ) -> str | None:
        try:
            return self._evidence.record_tool_intent(
                tool,
                arguments,
                state_before,
            )
        except Exception:
            return None

    @staticmethod
    def _tool_result(name: str, result: dict[str, Any]) -> ToolResult:
        return ToolResult(name=name, result=result)
