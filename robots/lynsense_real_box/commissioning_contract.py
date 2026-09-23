"""Pure data contracts and validation for offline commissioning preflight."""

from __future__ import annotations

import hashlib
import json
import math
import re
from collections import Counter
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path, PurePath
from typing import Literal, Protocol

from robots.lynsense_real_box.ownership_protocol import load_ownership_protocol


class CommissioningContractError(ValueError):
    """A commissioning input violates the reviewed static contract."""


class ServiceOwnershipUnavailable(CommissioningContractError):
    """The dependency cannot enforce ownership at its service boundary."""


class SourceInventoryMismatch(CommissioningContractError):
    """Installed source or configuration differs from reviewed inventory."""


class ResultNormalizationError(CommissioningContractError):
    """A dependency result cannot be represented by the fixed evidence schema."""


CommissioningStatus = Literal[
    "accepted", "rejected_before_initialization", "failed",
    "initialization_outcome_unknown", "service_ownership_enforcement_unavailable",
]
WorkerStatus = Literal[
    "settled_success", "known_failure", "ownership_unavailable", "late_settled",
    "worker_crashed", "precontact_rejected",
]
DispositionStatus = Literal["settled", "retained_for_operator", "unknown"]
SampleProvenance = Literal[
    "device_timestamp", "device_sequence", "request_response_acquisition",
]


@dataclass(frozen=True)
class EndpointIdentity:
    subsystem: str
    endpoint_id: str
    transport: str
    locator: str


@dataclass(frozen=True)
class CommissioningManifest:
    site_id: str
    robot_id: str
    instance: str
    ros_domain_id: int
    execution_host_machine_id: str
    execution_hostname: str
    execution_account: str
    execution_interpreter: str
    trusted_lock_root: Path
    lock_path: Path
    evidence_dir: Path
    host_enrollment_sha256: str
    authorization_id: str
    authorization_expires_at: datetime
    manifest_sha256: str
    source_artifacts: tuple[Path, ...]
    source_artifact_sha256: dict[str, str]
    ownership_protocol_artifact: Path
    ownership_protocol_sha256: str
    ownership_protocol_id: str
    ownership_protocol_version: int
    ownership_read_allowlist_sha256: str
    endpoint_authority_root: Path
    ownership_runtime_protocol: int
    config_sha256: dict[str, str]
    required_samples: int
    budgets: dict[str, float]
    freshness_s: dict[str, float]
    approved_effects: tuple[str, ...]
    endpoints: tuple[EndpointIdentity, ...]
    peripheral_mapping: dict[str, str]
    expected_identity: dict[str, dict[str, object]]
    expected_operator_verification: dict[str, str]
    expected_owned_ros_resources: tuple[str, ...]
    retention_policy: str


@dataclass(frozen=True)
class HostIdentity:
    machine_id: str
    hostname: str
    account: str
    interpreter: str


@dataclass(frozen=True)
class HostEnrollmentRecord:
    site_id: str
    machine_id: str
    hostname: str
    account: str
    enabled: bool


@dataclass(frozen=True)
class HostEnrollmentAuthority:
    authority_id: str
    authority_sha256: str
    records: tuple[HostEnrollmentRecord, ...]


@dataclass(frozen=True)
class TrustedOperatorPolicy:
    path: Path
    sha256: str
    trusted_lock_root: Path
    allowed_operators: tuple[str, ...]
    approved_authorizations: dict[str, str]


@dataclass(frozen=True)
class AuthorizationRecord:
    authorization_id: str
    purpose: str
    attempt_id: str
    authorization_sha256: str
    manifest_sha256: str
    site_id: str
    robot_id: str
    machine_id: str
    host_enrollment_sha256: str
    policy_sha256: str
    operator: str
    ros_domain_id: int
    expires_at: datetime


@dataclass(frozen=True)
class ResourceDisposition:
    status: DispositionStatus
    service_pid: int | None
    service_start_id: str | None
    service_epoch: str | None
    initialized_endpoints: tuple[str, ...]
    owned_ros_resources: tuple[str, ...]
    operator: str | None
    reason: str | None


@dataclass(frozen=True)
class CommissioningOutcome:
    status: CommissioningStatus
    identity_verified: bool
    required_state_groups_observed: bool
    resource_disposition_verified: bool
    attempt_id: str
    manifest_sha256: str
    lock_status: str
    report_path: Path | None
    reason: str | None


@dataclass(frozen=True)
class LockRecord:
    site_id: str
    robot_id: str
    endpoint_scope_sha256: str
    manifest_sha256: str
    attempt_id: str
    authorization_id: str
    host_machine_id: str
    created_at: datetime
    state: str
    service_pid: int | None
    service_start_id: str | None
    service_epoch: str | None


@dataclass(frozen=True)
class IdentityEvidence:
    configured: dict[str, object]
    device_reported: dict[str, object]
    operator_verified: dict[str, object]


@dataclass(frozen=True)
class StateEvidence:
    group: str
    payload: dict[str, object]
    source_timestamp: datetime | None
    sample_provenance: SampleProvenance
    receipt_monotonic: float
    source_sequence: int | None
    adapter_sequence: int | None


@dataclass(frozen=True)
class ServiceEvidence:
    pid: int | None
    start_id: str | None
    epoch: str | None


class InitializedRuntime(Protocol):
    def identity(self) -> IdentityEvidence: ...
    def read_group(self, group: str) -> StateEvidence: ...
    def service_identity(self) -> ServiceEvidence: ...
    def operator_verification(self) -> dict[str, object]: ...
    def owned_ros_resources(self) -> tuple[str, ...]: ...
    def release(self) -> None: ...


class CommissioningBinding(Protocol):
    def initialize(self, instance: str) -> InitializedRuntime: ...


class CommissioningLockHandle(Protocol):
    def release_after_settled(self, disposition: ResourceDisposition) -> None: ...
    def retain_unknown(self, reason: str) -> None: ...
    def record_service_identity(self, service: ServiceEvidence) -> None: ...


class ServiceOwnershipGate(Protocol):
    def initialize_under_guard(
        self, *, instance: str, site_id: str, robot_id: str,
        endpoints: tuple[EndpointIdentity, ...], endpoint_scope_sha256: str,
        approved_effects: tuple[str, ...], ros_domain_id: int,
    ) -> InitializedRuntime: ...


class InitializationSupervisor(Protocol):
    def run_worker(self, request: InitializationWorkerRequest) -> InitializationWorkerResult: ...


class Clock(Protocol):
    def now(self) -> datetime: ...
    def monotonic(self) -> float: ...


@dataclass(frozen=True)
class InitializationWorkerRequest:
    attempt_id: str
    manifest_sha256: str
    endpoint_scope_sha256: str
    instance: str
    ros_domain_id: int
    endpoints: tuple[EndpointIdentity, ...]
    approved_effects: tuple[str, ...]
    required_groups: tuple[str, ...]
    required_samples: int
    initialization_deadline_monotonic: float
    deadline_monotonic: float
    result_path: Path
    source_artifacts: tuple[Path, ...]
    source_artifact_sha256: dict[str, str]
    site_id: str
    robot_id: str
    ownership_protocol_artifact: Path
    ownership_protocol_sha256: str
    ownership_protocol_id: str
    ownership_protocol_version: int
    ownership_read_allowlist_sha256: str
    endpoint_authority_root: Path
    ownership_runtime_protocol: int
    operator_verification: dict[str, str]
    config_sha256: dict[str, str]


@dataclass(frozen=True)
class InitializationWorkerResult:
    status: WorkerStatus
    attempt_id: str
    manifest_sha256: str
    endpoint_scope_sha256: str
    identity: IdentityEvidence | None
    observations: tuple[StateEvidence, ...]
    service: ServiceEvidence
    disposition: ResourceDisposition
    error: str | None
    worker_pid: int | None
    worker_start_id: str | None


_MANIFEST_KEYS = frozenset({
    "site_id", "robot_id", "instance", "ros_domain_id",
    "execution_host_machine_id", "execution_hostname", "execution_account",
    "execution_interpreter",
    "trusted_lock_root", "lock_path", "evidence_dir", "host_enrollment_sha256",
    "authorization_id", "authorization_expires_at", "manifest_sha256",
    "source_artifacts", "source_artifact_sha256", "config_sha256", "required_samples",
    "budgets", "freshness_s", "approved_effects", "endpoints", "peripheral_mapping",
    "expected_identity", "expected_operator_verification", "expected_owned_ros_resources",
    "ownership_protocol_artifact", "ownership_protocol_sha256",
    "ownership_protocol_id", "ownership_protocol_version",
    "ownership_read_allowlist_sha256", "endpoint_authority_root",
    "ownership_runtime_protocol",
    "retention_policy",
})
_ENDPOINT_KEYS = frozenset({"subsystem", "endpoint_id", "transport", "locator"})
_MACHINE_ID = re.compile(r"^[0-9a-f]{32}$")
_SHA256 = re.compile(r"^[0-9a-f]{64}$")


def _canonical_json(value: object) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")


def _sha256(value: object) -> str:
    return hashlib.sha256(_canonical_json(value)).hexdigest()


def manifest_digest(data: dict[str, object]) -> str:
    """Return the canonical manifest digest with its own digest field omitted."""
    payload = {key: value for key, value in data.items() if key != "manifest_sha256"}
    for key in ("trusted_lock_root", "lock_path", "evidence_dir"):
        if key in payload:
            payload[key] = _path(payload[key], key).as_posix()
    if "source_artifacts" in payload:
        values = payload["source_artifacts"]
        if not isinstance(values, list):
            raise CommissioningContractError("source_artifacts must be a list")
        payload["source_artifacts"] = [
            _path(value, "source_artifacts item").as_posix() for value in values
        ]
    if "ownership_protocol_artifact" in payload:
        payload["ownership_protocol_artifact"] = _path(
            payload["ownership_protocol_artifact"], "ownership_protocol_artifact"
        ).as_posix()
    if "endpoint_authority_root" in payload:
        payload["endpoint_authority_root"] = _path(
            payload["endpoint_authority_root"], "endpoint_authority_root"
        ).as_posix()
    return _sha256(payload)


def authorization_digest(data: dict[str, object]) -> str:
    """Digest the authority payload without self-digest or policy binding."""
    unsigned = {
        key: value
        for key, value in data.items()
        if key not in {"authorization_sha256", "policy_sha256"}
    }
    return _sha256(unsigned)


def host_enrollment_digest(authority_id: str, records: tuple[HostEnrollmentRecord, ...]) -> str:
    rows = [
        {"site_id": r.site_id, "machine_id": r.machine_id, "hostname": r.hostname,
         "account": r.account, "enabled": r.enabled}
        for r in records
    ]
    return _sha256({"authority_id": authority_id, "records": rows})


def endpoint_scope_digest(manifest: CommissioningManifest) -> str:
    endpoints = [
        {"endpoint_id": e.endpoint_id, "locator": e.locator,
         "subsystem": e.subsystem, "transport": e.transport}
        for e in manifest.endpoints
    ]
    endpoints.sort(key=lambda e: (e["subsystem"], e["endpoint_id"], e["transport"], e["locator"]))
    payload = {"robot_id": manifest.robot_id, "site_id": manifest.site_id, "endpoints": endpoints}
    return hashlib.sha256(_canonical_json(payload)).hexdigest()


def load_trusted_operator_policy(path: Path) -> TrustedOperatorPolicy:
    try:
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise CommissioningContractError(f"cannot read trusted operator policy: {exc}") from exc
    if not isinstance(raw, dict):
        raise CommissioningContractError("trusted operator policy must be an object")
    if set(raw) != {"trusted_lock_root", "allowed_operators", "approved_authorizations", "sha256"}:
        raise CommissioningContractError("trusted operator policy keys are invalid")
    supplied = raw["sha256"]
    unsigned = {key: value for key, value in raw.items() if key != "sha256"}
    expected = hashlib.sha256(_canonical_json(unsigned)).hexdigest()
    if supplied != expected:
        raise CommissioningContractError("trusted operator policy digest mismatch")
    operators = raw["allowed_operators"]
    if not isinstance(operators, list) or not operators or any(
        not isinstance(operator, str) or not operator.strip() for operator in operators
    ):
        raise CommissioningContractError("trusted operator policy operators are invalid")
    approved = raw["approved_authorizations"]
    if not isinstance(approved, dict) or any(
        not isinstance(key, str) or not key.strip() or not isinstance(value, str)
        or not _SHA256.fullmatch(value)
        for key, value in approved.items()
    ):
        raise CommissioningContractError("trusted operator policy authorizations are invalid")
    return TrustedOperatorPolicy(
        path=Path(path).resolve(strict=True),
        sha256=expected,
        trusted_lock_root=_path(raw["trusted_lock_root"], "trusted_lock_root"),
        allowed_operators=tuple(operators),
        approved_authorizations=dict(approved),
    )


def _required_text(value: object, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        detail = "machine identity" if field == "execution_host_machine_id" else field
        raise CommissioningContractError(f"{detail} must be a nonempty string")
    return value


def _path(value: object, field: str) -> Path:
    text = _required_text(value, field)
    path = Path(text)
    if not path.is_absolute():
        raise CommissioningContractError(f"{field} must be absolute")
    if ".." in PurePath(text).parts:
        raise CommissioningContractError(f"{field} must not contain '..'")
    current = Path(path.anchor)
    for part in path.parts[1:]:
        current = current / part
        if current.is_symlink():
            raise CommissioningContractError(f"{field} contains a symlink: {current}")
    return path.resolve(strict=False)


def _datetime(value: object, field: str) -> datetime:
    if not isinstance(value, str):
        raise CommissioningContractError(f"{field} must be an ISO-8601 string")
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as exc:
        raise CommissioningContractError(f"{field} must be an ISO-8601 string") from exc
    if parsed.tzinfo is None or parsed.utcoffset() is None:
        raise CommissioningContractError(f"{field} must be timezone-aware")
    return parsed


def _string_map(value: object, field: str, *, digest_values: bool = False) -> dict[str, str]:
    if not isinstance(value, dict):
        raise CommissioningContractError(f"{field} must be an object")
    result: dict[str, str] = {}
    for key, item in value.items():
        _required_text(key, f"{field} key")
        _required_text(item, f"{field}.{key}")
        if digest_values and not _SHA256.fullmatch(item):
            raise CommissioningContractError(f"{field}.{key} must be a lowercase SHA-256 digest")
        result[key] = item
    return result


def _expected_arm_identity(value: object) -> dict[str, dict[str, object]]:
    if not isinstance(value, dict) or set(value) != {"left_arm", "right_arm"}:
        raise CommissioningContractError("expected_identity must contain left_arm and right_arm")
    result: dict[str, dict[str, object]] = {}
    for side in ("left", "right"):
        key = f"{side}_arm"
        item = value[key]
        if not isinstance(item, dict) or set(item) != {"side", "model", "ip", "axes"}:
            raise CommissioningContractError(f"expected_identity.{key} fields are invalid")
        model = _required_text(item["model"], f"expected_identity.{key}.model")
        ip = _required_text(item["ip"], f"expected_identity.{key}.ip")
        if item["side"] != side:
            raise CommissioningContractError(f"expected_identity.{key}.side must equal {side}")
        axes = item["axes"]
        if type(axes) is not int or axes <= 0:
            raise CommissioningContractError(
                f"expected_identity.{key}.axes must be a positive integer"
            )
        result[key] = {"side": side, "model": model, "ip": ip, "axes": axes}
    return result


def _string_items(value: object, field: str) -> tuple[str, ...]:
    if not isinstance(value, list) or not value or any(
        not isinstance(item, str) or not item for item in value
    ):
        raise CommissioningContractError(f"{field} must be a nonempty list of nonempty strings")
    if len(set(value)) != len(value):
        raise CommissioningContractError(f"{field} values must be unique")
    return tuple(value)


def load_commissioning_manifest(path: Path) -> CommissioningManifest:
    try:
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise CommissioningContractError(f"cannot read commissioning manifest: {exc}") from exc
    if not isinstance(raw, dict):
        raise CommissioningContractError("manifest must be a JSON object")
    missing, extra = _MANIFEST_KEYS - raw.keys(), raw.keys() - _MANIFEST_KEYS
    if missing:
        label = "machine identity" if "execution_host_machine_id" in missing else ", ".join(sorted(missing))
        raise CommissioningContractError(f"manifest missing required fields: {label}")
    if extra:
        raise CommissioningContractError(f"manifest has unexpected fields: {', '.join(sorted(extra))}")

    texts = {key: _required_text(raw[key], key) for key in (
        "site_id", "robot_id", "instance", "execution_host_machine_id",
        "execution_hostname", "execution_account", "host_enrollment_sha256",
        "authorization_id", "manifest_sha256", "retention_policy",
    )}
    interpreter = _required_text(raw["execution_interpreter"], "execution_interpreter")
    if not Path(interpreter).is_absolute() or ".." in PurePath(interpreter).parts:
        raise CommissioningContractError("execution_interpreter must be an absolute path without '..'")
    if not _MACHINE_ID.fullmatch(texts["execution_host_machine_id"]):
        raise CommissioningContractError("execution_host_machine_id must be a canonical 32-character lowercase machine ID")
    for key in ("host_enrollment_sha256", "manifest_sha256"):
        if not _SHA256.fullmatch(texts[key]):
            raise CommissioningContractError(f"{key} must be a lowercase SHA-256 digest")
    if type(raw["ros_domain_id"]) is not int or raw["ros_domain_id"] != 3:
        raise CommissioningContractError("ros_domain_id must equal ROS_DOMAIN_ID=3")
    if type(raw["required_samples"]) is not int or raw["required_samples"] < 2:
        raise CommissioningContractError("required_samples must be at least 2")

    root = _path(raw["trusted_lock_root"], "trusted_lock_root")
    lock_path = _path(raw["lock_path"], "lock_path")
    evidence_dir = _path(raw["evidence_dir"], "evidence_dir")
    expiry = _datetime(raw["authorization_expires_at"], "authorization_expires_at")

    artifacts_raw = raw["source_artifacts"]
    if not isinstance(artifacts_raw, list):
        raise CommissioningContractError("source_artifacts must be a list")
    artifacts = tuple(_path(item, "source_artifacts item") for item in artifacts_raw)
    source_digests = _string_map(raw["source_artifact_sha256"], "source_artifact_sha256", digest_values=True)
    config_digests = _string_map(raw["config_sha256"], "config_sha256", digest_values=True)
    artifact_keys = {artifact.as_posix() for artifact in artifacts}
    for field, values in (
        ("source_artifact_sha256", source_digests),
        ("config_sha256", config_digests),
    ):
        unresolved = sorted(set(values) - artifact_keys)
        if unresolved:
            raise CommissioningContractError(
                f"{field} keys must be normalized paths from source_artifacts: {unresolved}"
            )
    protocol_path = _path(
        raw["ownership_protocol_artifact"], "ownership_protocol_artifact"
    )
    if protocol_path.as_posix() not in artifact_keys:
        raise CommissioningContractError(
            "ownership_protocol_artifact must be listed in source_artifacts"
        )
    try:
        protocol = load_ownership_protocol(protocol_path)
    except ValueError as exc:
        raise CommissioningContractError(str(exc)) from exc
    if raw["ownership_protocol_sha256"] != protocol.artifact_sha256:
        raise CommissioningContractError("ownership protocol digest mismatch")
    if (
        source_digests.get(str(protocol_path)) != protocol.artifact_sha256
    ):
        raise CommissioningContractError(
            "ownership protocol source artifact digest is missing or conflicting"
        )
    ownership_values = (
        protocol.protocol_id,
        protocol.protocol_version,
        protocol.read_allowlist_sha256,
        protocol.endpoint_authority_root,
        protocol.runtime_protocol,
    )
    manifest_ownership_values = (
        raw["ownership_protocol_id"],
        raw["ownership_protocol_version"],
        raw["ownership_read_allowlist_sha256"],
        _path(raw["endpoint_authority_root"], "endpoint_authority_root"),
        raw["ownership_runtime_protocol"],
    )
    if ownership_values != manifest_ownership_values:
        raise CommissioningContractError(
            "manifest ownership protocol fields do not match artifact"
        )
    covered_artifacts = set(source_digests) | set(config_digests)
    uncovered = sorted(artifact_keys - covered_artifacts)
    if uncovered:
        raise CommissioningContractError(
            f"every source artifact must have a source or config digest: {uncovered}"
        )
    budgets = _number_map(raw["budgets"], "budgets", strictly_positive=True)
    if not {"initialize", "read"}.issubset(budgets):
        raise CommissioningContractError("budgets must include initialize and read")
    freshness = _number_map(raw["freshness_s"], "freshness_s", strictly_positive=True)

    effects = raw["approved_effects"]
    if not isinstance(effects, list) or any(not isinstance(v, str) or not v for v in effects):
        raise CommissioningContractError("approved_effects must be a list of nonempty strings")
    endpoint_values = raw["endpoints"]
    if not isinstance(endpoint_values, list) or not endpoint_values:
        raise CommissioningContractError("endpoints must be a nonempty list")
    endpoints: list[EndpointIdentity] = []
    for entry in endpoint_values:
        if not isinstance(entry, dict) or entry.keys() != _ENDPOINT_KEYS:
            raise CommissioningContractError("endpoint entries must have exactly subsystem, endpoint_id, transport, locator")
        endpoints.append(EndpointIdentity(**{key: _required_text(entry[key], f"endpoint.{key}") for key in _ENDPOINT_KEYS}))
    endpoint_keys = [(e.subsystem, e.endpoint_id, e.transport, e.locator) for e in endpoints]
    if len(set(endpoint_keys)) != len(endpoint_keys):
        raise CommissioningContractError("endpoint identities must be unique")

    mapping = _string_map(raw["peripheral_mapping"], "peripheral_mapping")
    expected_identity = _expected_arm_identity(raw["expected_identity"])
    expected_operator = _string_map(
        raw["expected_operator_verification"], "expected_operator_verification"
    )
    expected_resources = _string_items(
        raw["expected_owned_ros_resources"], "expected_owned_ros_resources"
    )
    manifest = CommissioningManifest(
        **texts,
        ros_domain_id=3,
        execution_interpreter=interpreter,
        trusted_lock_root=root,
        lock_path=lock_path,
        evidence_dir=evidence_dir,
        authorization_expires_at=expiry,
        source_artifacts=artifacts,
        source_artifact_sha256=source_digests,
        ownership_protocol_artifact=protocol_path,
        ownership_protocol_sha256=protocol.artifact_sha256,
        ownership_protocol_id=protocol.protocol_id,
        ownership_protocol_version=protocol.protocol_version,
        ownership_read_allowlist_sha256=protocol.read_allowlist_sha256,
        endpoint_authority_root=protocol.endpoint_authority_root,
        ownership_runtime_protocol=protocol.runtime_protocol,
        config_sha256=config_digests,
        required_samples=raw["required_samples"],
        budgets=budgets,
        freshness_s=freshness,
        approved_effects=tuple(effects),
        endpoints=tuple(endpoints),
        peripheral_mapping=mapping,
        expected_identity=expected_identity,
        expected_operator_verification=expected_operator,
        expected_owned_ros_resources=expected_resources,
    )
    expected_lock = root / endpoint_scope_digest(manifest) / "commissioning.lock"
    if lock_path != expected_lock:
        raise CommissioningContractError("lock_path must equal trusted_lock_root/endpoint_scope_sha256/commissioning.lock")
    if manifest_digest(raw) != manifest.manifest_sha256:
        raise CommissioningContractError("manifest digest mismatch")
    return manifest


def _number_map(value: object, field: str, *, strictly_positive: bool) -> dict[str, float]:
    if not isinstance(value, dict) or not value:
        raise CommissioningContractError(f"{field} must be a nonempty object")
    result: dict[str, float] = {}
    for key, number in value.items():
        if not isinstance(key, str) or not key or isinstance(number, bool) or not isinstance(number, (int, float)):
            raise CommissioningContractError(f"{field} values must be finite numbers")
        numeric = float(number)
        if not math.isfinite(numeric) or (strictly_positive and numeric <= 0):
            raise CommissioningContractError(f"{field} values must be finite and positive")
        result[key] = numeric
    return result


def validate_commissioning_manifest(
    manifest: CommissioningManifest,
    *,
    actual_host: HostIdentity,
    enrollment: HostEnrollmentAuthority,
    authorization: AuthorizationRecord | None = None,
    policy: TrustedOperatorPolicy | None = None,
    trusted_lock_root: Path,
) -> None:
    """Validate host uniqueness, policy-root pinning, and optional authorization."""
    if actual_host.machine_id != manifest.execution_host_machine_id:
        raise CommissioningContractError("actual host machine identity does not match manifest")
    if not _MACHINE_ID.fullmatch(actual_host.machine_id):
        raise CommissioningContractError("actual host machine ID is not canonical")
    if actual_host.hostname != manifest.execution_hostname or actual_host.account != manifest.execution_account:
        raise CommissioningContractError("actual hostname/account do not match manifest")
    if not actual_host.interpreter or actual_host.interpreter != manifest.execution_interpreter:
        raise CommissioningContractError("actual interpreter does not match manifest")
    root = _path(str(trusted_lock_root), "trusted_lock_root")
    if root != manifest.trusted_lock_root:
        raise CommissioningContractError("manifest trusted lock root does not match trusted operator policy")
    expected_authority_digest = host_enrollment_digest(enrollment.authority_id, enrollment.records)
    if enrollment.authority_sha256 != manifest.host_enrollment_sha256 or expected_authority_digest != enrollment.authority_sha256:
        raise CommissioningContractError("host enrollment authority digest mismatch")
    machine_records = [record for record in enrollment.records if record.machine_id == actual_host.machine_id]
    if len(machine_records) != 1:
        raise CommissioningContractError("host machine identity must be enrolled exactly once")
    matching = [record for record in machine_records if record.site_id == manifest.site_id]
    if len(matching) != 1:
        raise CommissioningContractError("host enrollment must list this site/machine identity exactly once")
    enrolled = matching[0]
    if not enrolled.enabled:
        raise CommissioningContractError("host enrollment is disabled")
    if (enrolled.hostname, enrolled.account) != (actual_host.hostname, actual_host.account):
        raise CommissioningContractError("enrolled hostname/account do not match actual host")
    if authorization is None:
        return
    if policy is None:
        raise CommissioningContractError(
            "trusted operator policy is required when authorization is supplied"
        )
    if not policy.sha256 or not _SHA256.fullmatch(policy.sha256):
        raise CommissioningContractError("trusted operator policy digest is invalid")
    if policy.trusted_lock_root != root:
        raise CommissioningContractError("trusted operator policy root does not match")
    if authorization.expires_at.tzinfo is None or authorization.expires_at.utcoffset() is None:
        raise CommissioningContractError("authorization expiry must be timezone-aware")
    now = datetime.now(timezone.utc)
    if authorization.expires_at <= now:
        raise CommissioningContractError("authorization has expired")
    if authorization.expires_at != manifest.authorization_expires_at:
        raise CommissioningContractError("authorization expiry does not match manifest")
    if authorization.purpose != "commissioning_run":
        raise CommissioningContractError("authorization purpose is not commissioning_run")
    if not isinstance(authorization.attempt_id, str) or not re.fullmatch(
        r"[A-Za-z0-9_-]{1,128}", authorization.attempt_id
    ):
        raise CommissioningContractError("authorization attempt_id is invalid")
    if not authorization.operator.strip():
        raise CommissioningContractError("authorization operator must be nonempty")
    if authorization.operator not in policy.allowed_operators:
        raise CommissioningContractError("authorization operator is not allowed by trusted policy")
    if not _SHA256.fullmatch(authorization.authorization_sha256):
        raise CommissioningContractError("authorization digest is invalid")
    if policy.approved_authorizations.get(authorization.authorization_id) != authorization.authorization_sha256:
        raise CommissioningContractError("authorization digest is not approved by trusted policy")
    expected = (
        manifest.authorization_id, manifest.manifest_sha256, manifest.site_id,
        manifest.robot_id, actual_host.machine_id, manifest.host_enrollment_sha256,
        manifest.ros_domain_id,
    )
    supplied = (
        authorization.authorization_id, authorization.manifest_sha256, authorization.site_id,
        authorization.robot_id, authorization.machine_id, authorization.host_enrollment_sha256,
        authorization.ros_domain_id,
    )
    if supplied != expected:
        raise CommissioningContractError("authorization does not match manifest, host enrollment, and ROS domain")
    if authorization.policy_sha256 != policy.sha256:
        raise CommissioningContractError("authorization policy digest does not match trusted policy")
