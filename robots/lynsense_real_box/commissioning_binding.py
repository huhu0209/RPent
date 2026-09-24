"""Fixed, read-only LynrotControl commissioning boundary."""

from __future__ import annotations

import hashlib
import importlib
import math
import json
import time
from datetime import datetime, timezone
from typing import Any

from robots.lynsense_real_box.commissioning_contract import (
    CommissioningManifest,
    endpoint_scope_digest,
    IdentityEvidence,
    ResultNormalizationError,
    ServiceEvidence,
    ServiceOwnershipGate,
    ServiceOwnershipUnavailable,
    SourceInventoryMismatch,
    StateEvidence,
)
from robots.lynsense_real_box.ownership_protocol import (
    OwnershipProtocolContract,
    load_ownership_protocol,
)

_GROUPS = (
    "left_arm", "right_arm", "left_gripper", "right_gripper",
    "waist_lift", "left_force", "right_force",
)
_SIDES = {"left_arm": "left", "right_arm": "right", "left_gripper": "left",
          "right_gripper": "right", "left_force": "left", "right_force": "right"}
_UNITS = {
    "left_arm": "rad", "right_arm": "rad", "left_gripper": "rad",
    "right_gripper": "rad", "waist_lift": "mm",
    "left_force": ("N", "N", "N", "N.m", "N.m", "N.m"),
    "right_force": ("N", "N", "N", "N.m", "N.m", "N.m"),
}
_ALLOWED_EFFECTS = (
    "initialize_configuration", "fixed_read_ros_resource_creation",
    "fixed_read_ros_parameter_query",
)


def _artifact_digests(manifest: CommissioningManifest) -> dict[str, str]:
    return verify_artifact_inventory(
        manifest.source_artifacts,
        {**manifest.source_artifact_sha256, **manifest.config_sha256},
    )


def verify_artifact_inventory(
    paths: tuple[Any, ...], expected: dict[str, str]
) -> dict[str, str]:
    if expected and not paths:
        raise SourceInventoryMismatch("declared source/config digest has no inventoried artifact path")
    observed: dict[str, str] = {}
    for path in paths:
        try:
            resolved = path.resolve(strict=True)
            if not resolved.is_file() or path.is_symlink():
                raise OSError("not a regular non-symlink file")
            digest = hashlib.sha256(resolved.read_bytes()).hexdigest()
        except OSError as exc:
            raise SourceInventoryMismatch(f"cannot verify source artifact {path}") from exc
        observed[str(path)] = digest
        observed[path.name] = digest
    missing = set(expected) - set(observed)
    mismatched = [key for key, value in expected.items() if key in observed and observed[key] != value]
    if missing or mismatched:
        raise SourceInventoryMismatch(
            f"source/config inventory mismatch (unresolved={sorted(missing)}, mismatched={sorted(mismatched)})"
        )
    return observed


def build_effectful_gate(request: Any) -> ServiceOwnershipGate:
    """Build the fixed child gate after verifying the pre-import protocol."""
    protocol = _verify_request_protocol(request)
    return _DependencyOwnershipGate(request, protocol)


def _unwrap(result: Any) -> Any:
    if not hasattr(result, "ok") or result.ok is not True or not hasattr(result, "data"):
        raise ResultNormalizationError("expected successful dependency Result")
    return result.data


def _number(value: Any) -> bool:
    if isinstance(value, bool):
        return False
    if isinstance(value, (int, float)):
        return math.isfinite(float(value))
    if isinstance(value, (list, tuple)):
        return bool(value) and all(_number(item) for item in value)
    return False


def _verify_request_protocol(request: Any) -> OwnershipProtocolContract:
    required = (
        "ownership_protocol_artifact", "ownership_protocol_sha256",
        "ownership_protocol_id", "ownership_protocol_version",
        "ownership_read_allowlist_sha256", "endpoint_authority_root",
        "ownership_runtime_protocol", "site_id", "robot_id",
        "approved_effects",
    )
    if not all(hasattr(request, field) for field in required):
        raise ServiceOwnershipUnavailable("worker request lacks ownership protocol fields")
    try:
        protocol = load_ownership_protocol(request.ownership_protocol_artifact)
    except ValueError as exc:
        raise ServiceOwnershipUnavailable(str(exc)) from exc
    observed = (
        protocol.artifact_sha256, protocol.protocol_id, protocol.protocol_version,
        protocol.read_allowlist_sha256, protocol.endpoint_authority_root,
        protocol.runtime_protocol,
    )
    supplied = (
        request.ownership_protocol_sha256, request.ownership_protocol_id,
        request.ownership_protocol_version, request.ownership_read_allowlist_sha256,
        request.endpoint_authority_root, request.ownership_runtime_protocol,
    )
    if observed != supplied:
        raise ServiceOwnershipUnavailable("ownership protocol request does not match artifact")
    if tuple(request.approved_effects) != _ALLOWED_EFFECTS:
        raise ServiceOwnershipUnavailable("approved effect tuple does not match protocol v1")
    endpoint_values = [
        {"endpoint_id": item.endpoint_id, "locator": item.locator,
         "subsystem": item.subsystem, "transport": item.transport}
        for item in request.endpoints
    ]
    endpoint_values.sort(key=lambda item: (
        item["subsystem"], item["endpoint_id"], item["transport"], item["locator"]
    ))
    endpoint_payload = json.dumps(
        {"endpoints": endpoint_values, "robot_id": request.robot_id,
         "site_id": request.site_id},
        sort_keys=True, separators=(",", ":"), ensure_ascii=True,
    ).encode("utf-8")
    expected_scope = hashlib.sha256(endpoint_payload).hexdigest()
    supplied_scope = getattr(request, "endpoint_scope_sha256", expected_scope)
    if supplied_scope != expected_scope:
        raise ServiceOwnershipUnavailable("endpoint scope digest does not match descriptors")
    return protocol


class _DependencyOwnershipGate:
    def __init__(self, request: Any, protocol: OwnershipProtocolContract) -> None:
        self._request, self._protocol = request, protocol

    def initialize_under_guard(self, **kwargs: Any) -> Any:
        request = self._request
        endpoint_scope = getattr(
            request, "endpoint_scope_sha256", endpoint_scope_digest(request)
        )
        expected = {
            "instance": request.instance, "site_id": request.site_id,
            "robot_id": request.robot_id, "endpoints": tuple(request.endpoints),
            "endpoint_scope_sha256": endpoint_scope,
            "approved_effects": tuple(request.approved_effects),
            "ros_domain_id": request.ros_domain_id,
        }
        if kwargs != expected:
            raise ServiceOwnershipUnavailable("guarded initialization arguments are not approved")
        try:
            ownership = importlib.import_module("lynrotcontrol.interfaces.ownership")
            endpoint_type = ownership.CanonicalEndpoint
            raw = ownership.initialize_under_ownership(
                request.instance,
                site_id=request.site_id,
                robot_id=request.robot_id,
                endpoint_scope_sha256=endpoint_scope,
                ros_domain_id=request.ros_domain_id,
                approved_effects=tuple(request.approved_effects),
                operator_verification=dict(getattr(request, "operator_verification", {})),
                expected_endpoints=tuple(
                    endpoint_type(
                        item.subsystem, item.endpoint_id, item.transport, item.locator
                    )
                    for item in request.endpoints
                ),
            )
        except (AttributeError, ImportError, NotImplementedError, TypeError) as exc:
            raise ServiceOwnershipUnavailable(
                f"dependency ownership initialization is unavailable: {type(exc).__name__}"
            ) from exc
        if raw is None or not all(callable(getattr(raw, name, None)) for name in (
            "arm_identity", "read_state", "service_identity", "owned_ros_resources", "release",
        )):
            raise ServiceOwnershipUnavailable("dependency returned no narrow owned runtime")
        return raw


def inspect_dependency_claim(manifest: CommissioningManifest) -> str:
    """Inspect the dependency claim after the same pre-import artifact gate."""
    try:
        protocol = load_ownership_protocol(manifest.ownership_protocol_artifact)
        if (
            protocol.artifact_sha256 != manifest.ownership_protocol_sha256
            or protocol.protocol_id != manifest.ownership_protocol_id
            or protocol.protocol_version != manifest.ownership_protocol_version
            or protocol.read_allowlist_sha256 != manifest.ownership_read_allowlist_sha256
            or protocol.endpoint_authority_root != manifest.endpoint_authority_root
            or protocol.runtime_protocol != manifest.ownership_runtime_protocol
        ):
            raise ValueError("manifest ownership protocol fields do not match artifact")
        ownership = importlib.import_module(
            "lynrotcontrol.lynarmcontrol.implementation.service_ownership"
        )
        inspection = ownership.inspect_claim(
            manifest.site_id,
            manifest.robot_id,
            tuple(
                ownership.CanonicalEndpoint(
                    item.subsystem, item.endpoint_id, item.transport, item.locator
                )
                for item in manifest.endpoints
            ),
        )
    except (AttributeError, ImportError, NotImplementedError, TypeError, ValueError) as exc:
        return "authority_record_invalid"
    return getattr(inspection, "state", "authority_record_invalid")


def _normalize_state(group: str, data: Any) -> StateEvidence:
    if isinstance(data, StateEvidence):
        if data.group != group:
            raise ResultNormalizationError("result group does not match requested group")
        payload = data.payload
        source_timestamp, source_sequence = data.source_timestamp, data.source_sequence
        provenance = data.sample_provenance
        receipt = data.receipt_monotonic
        adapter_sequence = data.adapter_sequence
    elif isinstance(data, dict):
        payload = dict(data)
        raw_timestamp = payload.get("timestamp")
        try:
            source_timestamp = raw_timestamp if isinstance(raw_timestamp, datetime) else datetime.fromisoformat(raw_timestamp) if isinstance(raw_timestamp, str) else None
        except ValueError as exc:
            raise ResultNormalizationError("invalid source timestamp") from exc
        source_sequence = payload.get("sequence")
        provenance = payload.get("provenance")
        receipt = time.monotonic()
        adapter_sequence = None
    else:
        raise ResultNormalizationError("state result data must be a mapping or StateEvidence")
    if not isinstance(payload, dict) or not _number(payload.get("values")):
        raise ResultNormalizationError("state values must be finite numeric values")
    if group in ("left_force", "right_force") and len(payload.get("values", ())) != 6:
        raise ResultNormalizationError("force state values must contain exactly six elements")
    if group in ("left_force", "right_force") and isinstance(payload.get("units"), list):
        payload["units"] = tuple(payload["units"])
    expected_units = _UNITS[group]
    observed_units = payload.get("units")
    units_ok = (
        observed_units == expected_units
        if isinstance(expected_units, str)
        else isinstance(observed_units, (list, tuple))
        and tuple(observed_units) == expected_units
    )
    if not units_ok or not isinstance(payload.get("frame"), str) or not payload["frame"].strip():
        raise ResultNormalizationError("state units/frame are missing or invalid")
    side = _SIDES.get(group)
    if side is not None and payload.get("side") != side:
        raise ResultNormalizationError("state side does not match fixed peripheral mapping")
    if group == "waist_lift" and payload.get("side") not in (None, "waist"):
        raise ResultNormalizationError("waist side metadata is invalid")
    axes = payload.get("axes")
    if not isinstance(axes, (tuple, list)) or not axes or not all(isinstance(axis, str) and axis for axis in axes):
        raise ResultNormalizationError("state axes are missing or invalid")
    if source_timestamp is not None:
        if source_sequence is not None:
            raise ResultNormalizationError("exactly one device timestamp or sequence is required")
        if source_timestamp.tzinfo is None or source_timestamp.utcoffset() is None:
            raise ResultNormalizationError("source timestamp must be timezone-aware")
        provenance = "device_timestamp"
    elif isinstance(source_sequence, int) and not isinstance(source_sequence, bool) and source_sequence >= 0:
        provenance = "device_sequence"
    elif provenance == "request_response_acquisition":
        raise ResultNormalizationError(
            "request-response provenance must come from the trusted adapter result"
        )
    else:
        raise ResultNormalizationError("sample provenance is not device-backed")
    if not isinstance(receipt, (float, int)) or not math.isfinite(float(receipt)):
        raise ResultNormalizationError("receipt monotonic time must be finite")
    return StateEvidence(group, payload, source_timestamp, provenance, float(receipt), source_sequence, adapter_sequence)


class RealLynrotControlBinding:
    """Bind reads after inventory verification and the actual ownership gate."""

    def __init__(self, *, gate: ServiceOwnershipGate, manifest: CommissioningManifest) -> None:
        self._gate = gate
        self._manifest = manifest

    def initialize(self, instance: str):
        if instance != self._manifest.instance:
            raise SourceInventoryMismatch("instance differs from approved manifest")
        _artifact_digests(self._manifest)
        for key, side in _SIDES.items():
            configured_side = self._manifest.peripheral_mapping.get(key)
            if configured_side is not None and configured_side != side:
                raise ResultNormalizationError(f"peripheral mapping mismatch for {key}")
        guard = getattr(self._gate, "initialize_under_guard", None)
        if not callable(guard):
            raise ServiceOwnershipUnavailable("gate cannot enforce actual service spawn/reuse ownership")
        try:
            raw = guard(
                instance=instance,
                site_id=self._manifest.site_id,
                robot_id=self._manifest.robot_id,
                endpoints=tuple(self._manifest.endpoints),
                endpoint_scope_sha256=endpoint_scope_digest(self._manifest),
                approved_effects=tuple(self._manifest.approved_effects),
                ros_domain_id=self._manifest.ros_domain_id,
            )
        except ServiceOwnershipUnavailable:
            raise
        except (NotImplementedError, AttributeError) as exc:
            raise ServiceOwnershipUnavailable("actual service ownership is unavailable") from exc
        if raw is None:
            raise ServiceOwnershipUnavailable("ownership gate returned no initialized runtime")
        if isinstance(raw, (_OwnedRuntimeAdapter, _FakeRuntime)):
            return raw
        return _OwnedRuntimeAdapter(raw, self._manifest, {})


class _OwnedRuntimeAdapter:
    def __init__(self, raw: Any, manifest: Any, operator_verification: dict[str, str]) -> None:
        self._raw, self._manifest = raw, manifest
        self._operator_verification = dict(operator_verification)

    def identity(self) -> IdentityEvidence:
        arms = {
            "left": self._raw.arm_identity("left"),
            "right": self._raw.arm_identity("right"),
        }
        if any(arms[side]["side"] != side for side in arms):
            raise ResultNormalizationError("device-reported arm side mismatch")
        configured = {
            "site_id": self._manifest.site_id,
            "robot_id": self._manifest.robot_id,
            "instance": self._manifest.instance,
            "peripheral_mapping": dict(getattr(self._manifest, "peripheral_mapping", {})),
        }
        return IdentityEvidence(configured, {"arms": arms}, {})

    def read_group(self, group: str) -> StateEvidence:
        if group not in _GROUPS:
            raise ResultNormalizationError(f"unknown group: {group}")
        try:
            data = self._raw.read_state(group)
        except ResultNormalizationError:
            raise
        except (AttributeError, KeyError, TypeError) as exc:
            raise ResultNormalizationError(f"fixed read unavailable for {group}") from exc
        return _normalize_state(group, data)

    def read_state(self, group: str) -> StateEvidence:
        return self.read_group(group)

    def service_identity(self) -> ServiceEvidence:
        value = getattr(self._raw, "service_identity", None)
        if not callable(value):
            return ServiceEvidence(None, None, None)
        try:
            data = value()
        except Exception:
            return ServiceEvidence(None, None, None)
        if not isinstance(data, dict):
            return ServiceEvidence(None, None, None)
        return ServiceEvidence(
            data.get("service_pid") if isinstance(data.get("service_pid"), int) and data["service_pid"] > 0 else None,
            data.get("service_start_id") if isinstance(data.get("service_start_id"), str) else None,
            data.get("service_epoch") if isinstance(data.get("service_epoch"), str) else None,
        )

    def operator_verification(self) -> dict[str, object]:
        return dict(self._operator_verification)

    def owned_ros_resources(self) -> tuple[str, ...]:
        resources = self._raw.owned_ros_resources()
        if not isinstance(resources, tuple) or not all(isinstance(item, str) and item for item in resources):
            raise ResultNormalizationError("owned ROS resource inventory is invalid")
        return resources

    def release(self) -> None:
        self._raw.release()


class FakeCommissioningBinding:
    """Offline-only binding with an explicit fixed-operation trace."""

    def __init__(self, *, samples: dict[str, StateEvidence] | None = None) -> None:
        self.operation_trace: list[tuple[str, str]] = []
        self._samples = samples or {}

    def initialize(self, instance: str):
        if not isinstance(instance, str) or not instance:
            raise ResultNormalizationError("instance is required")
        self.operation_trace.append(("initialize", instance))
        return _FakeRuntime(self)


class _FakeRuntime:
    def __init__(self, binding: FakeCommissioningBinding) -> None:
        self._binding = binding

    def identity(self) -> IdentityEvidence:
        self._binding.operation_trace.append(("identity", "fixed"))
        return IdentityEvidence({}, {}, {})

    def read_group(self, group: str) -> StateEvidence:
        if group not in _GROUPS:
            raise ResultNormalizationError(f"unknown group: {group}")
        self._binding.operation_trace.append(("read_group", group))
        sample = self._binding._samples.get(group)
        if sample is None:
            now = datetime.now(timezone.utc)
            side = _SIDES.get(group)
            values = [0.0] * 6 if group in ("left_force", "right_force") else [0.0]
            sample = StateEvidence(group, {"values": values, "units": _UNITS[group], "frame": "offline", "side": side,
                                          "axes": ["value"]}, now, "device_timestamp", time.monotonic(), None, None)
        return _normalize_state(group, sample)

    def service_identity(self) -> ServiceEvidence:
        self._binding.operation_trace.append(("service_identity", "fixed"))
        return ServiceEvidence(None, None, None)

    def operator_verification(self) -> dict[str, object]:
        self._binding.operation_trace.append(("operator_verification", "fixed"))
        return {"left_arm": "operator-verified", "right_arm": "operator-verified"}

    def owned_ros_resources(self) -> tuple[str, ...]:
        self._binding.operation_trace.append(("owned_ros_resources", "fixed"))
        return ("left-arm-service", "right-arm-service")

    def release(self) -> None:
        if getattr(self, "_released", False):
            raise ResultNormalizationError("runtime was already released")
        self._released = True
        self._binding.operation_trace.append(("release", "fixed"))

    def __getattr__(self, name: str) -> Any:
        if name.startswith("_"):
            raise AttributeError(name)
        self._binding.operation_trace.append(("rejected", name))
        raise ResultNormalizationError(f"unsupported operation: {name}")
