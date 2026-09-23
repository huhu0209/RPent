"""Fixed child-process boundary for one guarded commissioning initialization."""

from __future__ import annotations

import json
import math
import os
import re
import sys
import tempfile
import time
from dataclasses import asdict, replace
from datetime import datetime
from pathlib import Path
from typing import Any

from .commissioning_contract import (
    EndpointIdentity,
    IdentityEvidence,
    InitializationWorkerRequest,
    InitializationWorkerResult,
    ResourceDisposition,
    ServiceEvidence,
    ServiceOwnershipUnavailable,
    SourceInventoryMismatch,
    StateEvidence,
)
from .ownership_protocol import (
    PROTOCOL_ID, PROTOCOL_VERSION, READ_ALLOWLIST_SHA256, RUNTIME_PROTOCOL,
)


REQUIRED_GROUPS = (
    "left_arm", "right_arm", "left_gripper", "right_gripper",
    "waist_lift", "left_force", "right_force",
)
_WORKER_STATUSES = frozenset({
    "settled_success", "known_failure", "ownership_unavailable", "late_settled",
    "worker_crashed", "precontact_rejected",
})
_PROVENANCES = frozenset({
    "device_timestamp", "device_sequence", "request_response_acquisition",
})
_DISPOSITION_STATUSES = frozenset({"settled", "retained_for_operator", "unknown"})
_APPROVED_EFFECTS = (
    "initialize_configuration", "fixed_read_ros_resource_creation",
    "fixed_read_ros_parameter_query",
)


def _optional_positive_pid(value: Any) -> bool:
    return value is None or (
        isinstance(value, int) and not isinstance(value, bool) and value > 0
    )


def _optional_text(value: Any) -> bool:
    return value is None or (isinstance(value, str) and bool(value.strip()))


def _text_items(value: Any) -> bool:
    return isinstance(value, list) and all(
        isinstance(item, str) and bool(item.strip()) for item in value
    )


class WorkerDeadlineExceeded(TimeoutError):
    def __init__(self, pid: int, start_id: str | None):
        super().__init__("worker deadline expired; worker was not cancelled")
        self.pid, self.start_id = pid, start_id


class _UnavailableOwnershipGate:
    def initialize_under_guard(self, **_: Any) -> Any:
        from .commissioning_contract import ServiceOwnershipUnavailable
        raise ServiceOwnershipUnavailable("actual service spawn/reuse ownership enforcement is unavailable")


def _worker_start_id(pid: int) -> str | None:
    try:
        fields = Path(f"/proc/{pid}/stat").read_text(encoding="ascii").split()
        return fields[21]
    except (OSError, IndexError):
        return None


def _identity_data(value: IdentityEvidence | None) -> Any:
    return None if value is None else asdict(value)


def worker_result_to_dict(result: InitializationWorkerResult) -> dict[str, Any]:
    return {
        "status": result.status, "attempt_id": result.attempt_id,
        "manifest_sha256": result.manifest_sha256,
        "endpoint_scope_sha256": result.endpoint_scope_sha256,
        "identity": _identity_data(result.identity),
        "observations": [
            {**asdict(item), "source_timestamp": item.source_timestamp.isoformat() if item.source_timestamp else None}
            for item in result.observations
        ],
        "service": asdict(result.service), "disposition": asdict(result.disposition),
        "error": result.error, "worker_pid": result.worker_pid,
        "worker_start_id": result.worker_start_id,
    }


def worker_result_from_dict(data: dict[str, Any]) -> InitializationWorkerResult:
    result_fields = {
        "status", "attempt_id", "manifest_sha256", "endpoint_scope_sha256",
        "identity", "observations",
        "service", "disposition", "error", "worker_pid", "worker_start_id",
    }
    if not isinstance(data, dict) or set(data) != result_fields:
        raise ValueError("worker result fields do not match the fixed protocol")
    if (
        data["status"] not in _WORKER_STATUSES
        or not isinstance(data["attempt_id"], str)
        or not re.fullmatch(r"[A-Za-z0-9_-]{1,128}", data["attempt_id"])
        or not isinstance(data["manifest_sha256"], str)
        or not re.fullmatch(r"[0-9a-f]{64}", data["manifest_sha256"])
        or not isinstance(data["endpoint_scope_sha256"], str)
        or not re.fullmatch(r"[0-9a-f]{64}", data["endpoint_scope_sha256"])
        or not isinstance(data.get("observations"), list)
        or data.get("error") is not None and not isinstance(data["error"], str)
        or data.get("worker_pid") is not None and (
            not isinstance(data["worker_pid"], int)
            or isinstance(data["worker_pid"], bool)
            or data["worker_pid"] <= 0
        )
        or data.get("worker_start_id") is not None and not isinstance(data["worker_start_id"], str)
    ):
        raise ValueError("worker result values do not match the fixed protocol")
    identity_data = data.get("identity")
    if identity_data is not None and (
        not isinstance(identity_data, dict)
        or set(identity_data) != {"configured", "device_reported", "operator_verified"}
    ):
        raise ValueError("worker identity evidence does not match the fixed protocol")
    identity = IdentityEvidence(**identity_data) if identity_data is not None else None
    observations = []
    for item in data.get("observations", []):
        if not isinstance(item, dict) or set(item) != {
            "group", "payload", "source_timestamp", "sample_provenance",
            "receipt_monotonic", "source_sequence", "adapter_sequence",
        }:
            raise ValueError("worker observation does not match the fixed protocol")
        observation = dict(item)
        stamp = observation.get("source_timestamp")
        observation["source_timestamp"] = datetime.fromisoformat(stamp) if stamp else None
        if (
            observation["group"] not in REQUIRED_GROUPS
            or not isinstance(observation["payload"], dict)
            or (observation["source_timestamp"] is not None) == (
                observation["source_sequence"] is not None
            )
            or observation["sample_provenance"] not in _PROVENANCES
            or not isinstance(observation["receipt_monotonic"], (int, float))
            or isinstance(observation["receipt_monotonic"], bool)
            or not math.isfinite(float(observation["receipt_monotonic"]))
            or observation["source_sequence"] is not None and (
                not isinstance(observation["source_sequence"], int)
                or isinstance(observation["source_sequence"], bool)
                or observation["source_sequence"] < 0
            )
            or observation["adapter_sequence"] is not None and (
                not isinstance(observation["adapter_sequence"], int)
                or isinstance(observation["adapter_sequence"], bool)
                or observation["adapter_sequence"] < 0
            )
        ):
            raise ValueError("worker observation provenance is invalid")
        if observation["group"] in ("left_force", "right_force"):
            observation["payload"]["units"] = tuple(observation["payload"]["units"])
        observations.append(StateEvidence(**observation))
    if not isinstance(data["service"], dict) or set(data["service"]) != {"pid", "start_id", "epoch"}:
        raise ValueError("worker service identity does not match the fixed protocol")
    service = data["service"]
    if (
        not _optional_positive_pid(service["pid"])
        or not _optional_text(service["start_id"])
        or not _optional_text(service["epoch"])
    ):
        raise ValueError("worker service identity values are invalid")
    disposition_data = data["disposition"]
    if not isinstance(disposition_data, dict) or set(disposition_data) != {
        "status", "service_pid", "service_start_id", "service_epoch",
        "initialized_endpoints", "owned_ros_resources", "operator", "reason",
    } or not isinstance(disposition_data["initialized_endpoints"], list) or not isinstance(
        disposition_data["owned_ros_resources"], list
    ):
        raise ValueError("worker disposition does not match the fixed protocol")
    if (
        disposition_data["status"] not in _DISPOSITION_STATUSES
        or not _optional_positive_pid(disposition_data["service_pid"])
        or not _optional_text(disposition_data["service_start_id"])
        or not _optional_text(disposition_data["service_epoch"])
        or not _text_items(disposition_data["initialized_endpoints"])
        or not _text_items(disposition_data["owned_ros_resources"])
        or not _optional_text(disposition_data["operator"])
        or not _optional_text(disposition_data["reason"])
    ):
        raise ValueError("worker disposition values are invalid")
    return InitializationWorkerResult(
        status=data["status"], attempt_id=data["attempt_id"],
        manifest_sha256=data["manifest_sha256"],
        endpoint_scope_sha256=data["endpoint_scope_sha256"], identity=identity,
        observations=tuple(observations), service=ServiceEvidence(**data["service"]),
        disposition=ResourceDisposition(**{
            **data["disposition"],
            "initialized_endpoints": tuple(data["disposition"]["initialized_endpoints"]),
            "owned_ros_resources": tuple(data["disposition"]["owned_ros_resources"]),
        }), error=data.get("error"), worker_pid=data.get("worker_pid"),
        worker_start_id=data.get("worker_start_id"),
    )


def request_to_dict(request: InitializationWorkerRequest) -> dict[str, Any]:
    return {
        "attempt_id": request.attempt_id, "manifest_sha256": request.manifest_sha256,
        "endpoint_scope_sha256": request.endpoint_scope_sha256,
        "instance": request.instance, "ros_domain_id": request.ros_domain_id,
        "endpoints": [asdict(item) for item in request.endpoints],
        "approved_effects": list(request.approved_effects),
        "required_groups": list(request.required_groups),
        "required_samples": request.required_samples,
        "initialization_deadline_monotonic": request.initialization_deadline_monotonic,
        "deadline_monotonic": request.deadline_monotonic,
        "result_path": str(request.result_path),
        "source_artifacts": [str(path) for path in request.source_artifacts],
        "source_artifact_sha256": request.source_artifact_sha256,
        "site_id": request.site_id, "robot_id": request.robot_id,
        "ownership_protocol_artifact": str(request.ownership_protocol_artifact),
        "ownership_protocol_sha256": request.ownership_protocol_sha256,
        "ownership_protocol_id": request.ownership_protocol_id,
        "ownership_protocol_version": request.ownership_protocol_version,
        "ownership_read_allowlist_sha256": request.ownership_read_allowlist_sha256,
        "endpoint_authority_root": str(request.endpoint_authority_root),
        "ownership_runtime_protocol": request.ownership_runtime_protocol,
        "operator_verification": request.operator_verification,
        "config_sha256": request.config_sha256,
    }


def request_from_dict(data: dict[str, Any]) -> InitializationWorkerRequest:
    allowed = {
        "attempt_id", "manifest_sha256", "instance", "endpoints", "approved_effects",
        "ros_domain_id", "endpoint_scope_sha256",
        "required_groups", "required_samples", "deadline_monotonic", "result_path",
        "initialization_deadline_monotonic", "source_artifacts", "source_artifact_sha256",
        "site_id", "robot_id", "ownership_protocol_artifact",
        "ownership_protocol_sha256", "ownership_protocol_id",
        "ownership_protocol_version", "ownership_read_allowlist_sha256",
        "endpoint_authority_root", "ownership_runtime_protocol",
        "operator_verification",
        "config_sha256",
    }
    if set(data) != allowed:
        raise ValueError("worker request fields do not match the fixed protocol")
    request = InitializationWorkerRequest(
        data["attempt_id"], data["manifest_sha256"], data["endpoint_scope_sha256"],
        data["instance"],
        data["ros_domain_id"],
        tuple(EndpointIdentity(**endpoint) for endpoint in data["endpoints"]),
        tuple(data["approved_effects"]), tuple(data["required_groups"]),
        data["required_samples"], data["initialization_deadline_monotonic"],
        data["deadline_monotonic"], Path(data["result_path"]),
        tuple(Path(path) for path in data["source_artifacts"]),
        dict(data["source_artifact_sha256"]),
        data["site_id"], data["robot_id"],
        Path(data["ownership_protocol_artifact"]),
        data["ownership_protocol_sha256"], data["ownership_protocol_id"],
        data["ownership_protocol_version"], data["ownership_read_allowlist_sha256"],
        Path(data["endpoint_authority_root"]), data["ownership_runtime_protocol"],
        dict(data["operator_verification"]),
        dict(data["config_sha256"]),
    )
    if (
        not isinstance(request.attempt_id, str)
        or not re.fullmatch(r"[A-Za-z0-9_-]{1,128}", request.attempt_id)
        or not isinstance(request.manifest_sha256, str)
        or not re.fullmatch(r"[0-9a-f]{64}", request.manifest_sha256)
        or not isinstance(request.endpoint_scope_sha256, str)
        or not re.fullmatch(r"[0-9a-f]{64}", request.endpoint_scope_sha256)
        or not isinstance(request.instance, str) or not request.instance
        or type(request.ros_domain_id) is not int or request.ros_domain_id != 3
        or not request.endpoints
        or tuple(request.approved_effects) != _APPROVED_EFFECTS
        or any(not isinstance(value, str) or not value for value in request.approved_effects)
        or request.required_groups != REQUIRED_GROUPS
        or isinstance(request.required_samples, bool)
        or not isinstance(request.required_samples, int) or request.required_samples < 2
        or isinstance(request.deadline_monotonic, bool)
        or not isinstance(request.deadline_monotonic, (int, float))
        or not math.isfinite(float(request.deadline_monotonic))
        or isinstance(request.initialization_deadline_monotonic, bool)
        or not isinstance(request.initialization_deadline_monotonic, (int, float))
        or not math.isfinite(float(request.initialization_deadline_monotonic))
        or request.initialization_deadline_monotonic > request.deadline_monotonic
        or not request.result_path.is_absolute()
        or request.result_path.name != f"{request.attempt_id}.result.json"
        or not isinstance(request.site_id, str) or not request.site_id
        or not isinstance(request.robot_id, str) or not request.robot_id
        or not request.ownership_protocol_artifact.is_absolute()
        or ".." in request.ownership_protocol_artifact.parts
        or not isinstance(request.ownership_protocol_sha256, str)
        or not re.fullmatch(r"[0-9a-f]{64}", request.ownership_protocol_sha256)
        or request.ownership_protocol_id != PROTOCOL_ID
        or type(request.ownership_protocol_version) is not int
        or request.ownership_protocol_version != PROTOCOL_VERSION
        or request.ownership_read_allowlist_sha256 != READ_ALLOWLIST_SHA256
        or not request.endpoint_authority_root.is_absolute()
        or ".." in request.endpoint_authority_root.parts
        or type(request.ownership_runtime_protocol) is not int
        or request.ownership_runtime_protocol != RUNTIME_PROTOCOL
        or not isinstance(request.operator_verification, dict)
        or any(not isinstance(key, str) or not key or not isinstance(value, str)
               for key, value in request.operator_verification.items())
    ):
        raise ValueError("worker request values do not match the fixed protocol")
    return request


def run_worker(request: InitializationWorkerRequest, gate: Any = None) -> InitializationWorkerResult:
    """Initialize once through the supplied guarded boundary and collect fixed reads."""
    from .commissioning_binding import _normalize_state
    pid, start_id = os.getpid(), _worker_start_id(os.getpid())
    service = ServiceEvidence(None, None, None)
    disposition = ResourceDisposition("unknown", None, None, None, (), (), None, "initialization_not_settled")
    identity = None
    observations: list[StateEvidence] = []
    if set(request.required_groups) - set(REQUIRED_GROUPS) or request.required_samples < 2:
        error = "invalid fixed observation request"
        status = "worker_crashed"
    else:
        try:
            from .commissioning_binding import verify_artifact_inventory
            verify_artifact_inventory(
                request.source_artifacts,
                {**request.source_artifact_sha256, **request.config_sha256},
            )
        except SourceInventoryMismatch as exc:
            return InitializationWorkerResult(
                "precontact_rejected", request.attempt_id, request.manifest_sha256,
                request.endpoint_scope_sha256,
                None, (), service,
                ResourceDisposition(
                    "settled", None, None, None, (), (), None,
                    "source inventory mismatch before dependency import",
                ),
                str(exc), pid, start_id,
            )
        gate = gate if gate is not None else _UnavailableOwnershipGate()
        try:
            runtime = gate.initialize_under_guard(
                instance=request.instance, site_id=request.site_id,
                robot_id=request.robot_id, endpoints=request.endpoints,
                endpoint_scope_sha256=request.endpoint_scope_sha256,
                approved_effects=request.approved_effects,
                ros_domain_id=request.ros_domain_id,
            )
        except Exception as exc:
            from .commissioning_contract import ServiceOwnershipUnavailable
            unavailable = isinstance(exc, ServiceOwnershipUnavailable)
            status = "ownership_unavailable" if unavailable else "known_failure"
            error = f"{type(exc).__name__}: {exc}"
        else:
            try:
                service_before = runtime.service_identity()
                service = service_before
                if time.monotonic() >= request.initialization_deadline_monotonic:
                    status, error = "late_settled", "initialization completed after deadline"
                    owned_resources = runtime.owned_ros_resources()
                    service_after_resources = runtime.service_identity()
                    if service_after_resources != service:
                        raise RuntimeError("service identity or epoch changed during resource inventory")
                    service = service_after_resources
                    disposition = ResourceDisposition(
                        "unknown", service.pid, service.start_id, service.epoch,
                        tuple(endpoint.endpoint_id for endpoint in request.endpoints),
                        owned_resources, None,
                        "initialization_settled_after_deadline",
                    )
                else:
                    identity = runtime.identity()
                    operator_verified = runtime.operator_verification()
                    identity = replace(identity, operator_verified=operator_verified)
                    for group in request.required_groups:
                        for _ in range(request.required_samples):
                            if time.monotonic() >= request.deadline_monotonic:
                                raise TimeoutError("observation deadline expired")
                            observations.append(
                                _normalize_state(group, runtime.read_state(group))
                            )
                    service = runtime.service_identity()
                    if service != service_before:
                        raise RuntimeError("service identity or epoch changed during observation")
                    owned_resources = runtime.owned_ros_resources()
                    service_after_resources = runtime.service_identity()
                    if service_after_resources != service:
                        raise RuntimeError("service identity or epoch changed during resource inventory")
                    service = service_after_resources
                    try:
                        runtime.release()
                    except Exception as release_exc:
                        status = "known_failure"
                        error = f"claim release failed: {type(release_exc).__name__}: {release_exc}"
                        disposition = ResourceDisposition(
                            "unknown", service.pid, service.start_id, service.epoch,
                            tuple(endpoint.endpoint_id for endpoint in request.endpoints),
                            owned_resources, None, "claim_release_unknown",
                        )
                    else:
                        status, error = "settled_success", None
                        disposition = ResourceDisposition(
                            "settled", service.pid, service.start_id, service.epoch,
                            tuple(endpoint.endpoint_id for endpoint in request.endpoints),
                            owned_resources, None, None,
                        )
            except Exception as exc:
                status, error = "known_failure", f"{type(exc).__name__}: {exc}"
                try:
                    service = runtime.service_identity()
                except Exception:
                    pass
                disposition = ResourceDisposition(
                    "unknown", service.pid, service.start_id, service.epoch,
                    tuple(endpoint.endpoint_id for endpoint in request.endpoints), (), None,
                    "initialization_or_observation_failed",
                )
    return InitializationWorkerResult(
        status, request.attempt_id, request.manifest_sha256,
        request.endpoint_scope_sha256, identity,
        tuple(observations), service, disposition, error, pid, start_id,
    )


def persist_worker_result(path: Path, result: InitializationWorkerResult) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = (json.dumps(worker_result_to_dict(result), sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode()
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        directory = os.open(path.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        Path(temporary).unlink(missing_ok=True)


def _child_main(request_path: Path) -> int:
    try:
        request = request_from_dict(json.loads(request_path.read_text(encoding="utf-8")))
        try:
            from .commissioning_binding import build_effectful_gate
            gate = build_effectful_gate(request)
        except ServiceOwnershipUnavailable:
            gate = _UnavailableOwnershipGate()
        persist_worker_result(request.result_path, run_worker(request, gate))
        return 0
    finally:
        request_path.unlink(missing_ok=True)


class SubprocessInitializationSupervisor:
    """Launch a fixed module worker; timeout never terminates the child."""

    def __init__(self, *, python: str = sys.executable, poll_interval_s: float = 0.02):
        self.python, self.poll_interval_s = python, poll_interval_s

    def run_worker(self, request: InitializationWorkerRequest) -> InitializationWorkerResult:
        import subprocess

        request.result_path.parent.mkdir(parents=True, exist_ok=True)
        request_path = request.result_path.with_name(f".{request.attempt_id}.request.json")
        payload = json.dumps(request_to_dict(request), sort_keys=True, separators=(",", ":")).encode("utf-8")
        with request_path.open("xb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            process = subprocess.Popen(
                [self.python, "-m", "robots.lynsense_real_box.commissioning_worker", str(request_path)],
                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                close_fds=True, start_new_session=True,
            )
        except OSError:
            request_path.unlink(missing_ok=True)
            raise
        start_id = _worker_start_id(process.pid)
        while time.monotonic() < request.deadline_monotonic:
            if process.poll() is not None:
                request_path.unlink(missing_ok=True)
                if request.result_path.exists():
                    return worker_result_from_dict(json.loads(request.result_path.read_text(encoding="utf-8")))
                return InitializationWorkerResult(
                    "worker_crashed", request.attempt_id, request.manifest_sha256,
                    request.endpoint_scope_sha256, None, (),
                    ServiceEvidence(None, None, None),
                    ResourceDisposition("unknown", None, None, None, (), (), None, "worker_result_missing"),
                    "worker exited without a durable result", process.pid, start_id,
                )
            time.sleep(min(self.poll_interval_s, max(0.0, request.deadline_monotonic - time.monotonic())))
        raise WorkerDeadlineExceeded(process.pid, start_id)


if __name__ == "__main__" and len(sys.argv) == 2:
    raise SystemExit(_child_main(Path(sys.argv[1])))
