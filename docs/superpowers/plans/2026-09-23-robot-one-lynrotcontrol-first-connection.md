# Robot One LynrotControl First Connection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an operator-only commissioning entry that performs one approved LynrotControl initialization attempt and accepts Robot One state evidence without opening RPent live motion or Toolkit access.

**Architecture:** A pure manifest/host-enrollment contract validates all authority and identity inputs before any dependency import. A durable physical-endpoint lock and a separate initialization worker make uncertainty explicit across process boundaries. The real binding is behind a service-ownership gate; it fails closed until the actual LynrotControl spawn/reuse decision can be guarded, so a preflight check is not presented as race-free.

**Tech Stack:** Python 3.10+, dataclasses, JSON/JSONL, `pathlib`, `hashlib`, `os.fsync`, atomic lock-directory creation, existing `pytest` fixtures, injected fake binding, and the local LynrotControl package loaded only at effectful execution time.

**Spec:** `docs/superpowers/specs/2026-09-23-robot-one-lynrotcontrol-first-connection-design.md`

## Global Constraints

- Do not change `api_loop.py`, the eight-tool atomic contract, existing live rejection, Dashboard behavior, or the model-visible tool surface.
- The commissioning entry is operator-only and never callable by an LLM, Dashboard primitive, or RPent planner.
- Static validation imports no driver, opens no socket, starts no process, initializes no ROS context, and writes no controller setting.
- The effectful run requires one pinned execution host, a canonical machine ID registered by a reviewed host-enrollment authority, `ROS_DOMAIN_ID=3`, a matching manifest digest, and a durable endpoint lock acquired before initialization.
- Reuse `lynrotcontrol.initialize(instance)` only after the approved effect inventory is validated; do not add a connect-only dependency mode in this plan.
- Initialization is one-shot: no retry, implicit recovery, stop fallback, manual enable, trajectory, gripper command, waist command, force zeroing, perception trigger, chassis command, or cancellation test.
- A timeout or unknown outcome retains the durable lock across runner exit until explicit operator reconciliation; it never expires automatically.
- If the installed LynrotControl dependency cannot enforce ownership at its actual service reuse/spawn decision, the effectful command must return `service_ownership_enforcement_unavailable` before importing or calling `lynrotcontrol.initialize()`.
- A clean accepted attempt releases the lock only after all approved reads settle and resource disposition is recorded.
- Required evidence covers both arms, both grippers, waist lift, and both force sources with at least two distinct observations per group; raw ROS observations cannot substitute for failed LynrotControl reads.
- No real Robot One, SSH, ROS, SDK initialization, dependency installation, deployment, commit, or push is part of implementation or offline validation.

---

## File Map

Create the commissioning implementation under `robots/lynsense_real_box/`:

- `commissioning_contract.py`: pure manifest, host identity, enrollment, state, and result dataclasses plus validation; no driver imports or side effects.
- `commissioning_lock.py`: durable atomic lock and explicit reconciliation operations.
- `commissioning_binding.py`: fixed protocol and the real/fake LynrotControl adapters; dependency import occurs only in the real effectful binding.
- `commissioning_worker.py`: child-process entry that imports LynrotControl only after parent validation and executes one guarded initialization request.
- `commissioning_evidence.py`: credential-filtered canonical JSONL events and final report writer for commissioning, separate from the atomic profile evidence schema.
- `commissioning_runner.py`: one-attempt state machine, deadlines, read ordering, partial-failure handling, and lock disposition.
- `commissioning_cli.py`: operator-only `python -m` entry with static-validation and explicitly authorized effectful modes.

Create focused tests under `tests/unit_tests/robots/lynsense_real_box/`:

- `test_commissioning_contract.py`
- `test_commissioning_lock.py`
- `test_commissioning_binding.py`
- `test_commissioning_worker.py`
- `test_commissioning_evidence.py`
- `test_commissioning_runner.py`
- `test_commissioning_cli.py`
- `conftest.py`: shared offline-only manifest, authority, JSON, child-process, and fake-runtime fixtures used by the focused tests.

Update documentation only after the implementation is complete:

- `robots/lynsense_real_box/README.md`: commissioning entry, explicit non-motion boundary, and evidence/report locations.
- `docs/superpowers/plans/2026-09-23-robot-one-lynrotcontrol-first-connection-runbook.md`: exact command contract and required site manifest fields; the runbook must reject missing values rather than contain estimated Robot One numbers.

## Cross-Task Interfaces

Declare these names in Task 1 and use them unchanged later. The manifest digest
is computed from canonical UTF-8 JSON with the `manifest_sha256` key omitted,
sorted keys, compact separators, and normalized POSIX paths. It is never a hash
of a document containing itself. `lock_path` must equal
`trusted_lock_root / endpoint_scope_sha256 / "commissioning.lock"`; it is not an
arbitrary manifest-selected path.

```python
@dataclass(frozen=True)
class EndpointIdentity:
    subsystem: str                 # arm, gripper, waist, force, rpc
    endpoint_id: str               # reviewed stable identity
    transport: str                 # tcp, unix, ros2
    locator: str                   # redacted URI or ROS graph name

CommissioningStatus = Literal[
    "accepted", "rejected_before_initialization", "failed", "initialization_outcome_unknown",
    "service_ownership_enforcement_unavailable",
]
WorkerStatus = Literal[
    "settled_success", "known_failure", "ownership_unavailable", "late_settled", "worker_crashed",
]
DispositionStatus = Literal["settled", "retained_for_operator", "unknown"]

@dataclass(frozen=True)
class CommissioningManifest:
    site_id: str
    robot_id: str
    instance: str
    ros_domain_id: int
    execution_host_machine_id: str
    execution_hostname: str
    execution_account: str
    trusted_lock_root: Path
    lock_path: Path
    evidence_dir: Path
    host_enrollment_sha256: str
    authorization_id: str
    authorization_expires_at: datetime
    manifest_sha256: str
    source_artifacts: tuple[Path, ...]
    source_artifact_sha256: dict[str, str]
    config_sha256: dict[str, str]
    required_samples: int
    budgets: dict[str, float]
    freshness_s: dict[str, float]
    approved_effects: tuple[str, ...]
    endpoints: tuple[EndpointIdentity, ...]
    peripheral_mapping: dict[str, str]
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
    sample_provenance: Literal[
        "device_timestamp", "device_sequence", "request_response_acquisition"
    ]
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

class CommissioningBinding(Protocol):
    def initialize(self, instance: str) -> InitializedRuntime: ...

class CommissioningLockHandle(Protocol):
    def release_after_settled(self, disposition: ResourceDisposition) -> None: ...
    def retain_unknown(self, reason: str) -> None: ...
    def record_service_identity(self, service: ServiceEvidence) -> None: ...

class ServiceOwnershipGate(Protocol):
    def initialize_under_guard(self, *, instance: str, endpoints: tuple[EndpointIdentity, ...], approved_effects: tuple[str, ...]) -> "InitializedRuntime": ...

class InitializationSupervisor(Protocol):
    def run_worker(self, request: "InitializationWorkerRequest") -> "InitializationWorkerResult": ...

class Clock(Protocol):
    def now(self) -> datetime: ...
    def monotonic(self) -> float: ...

def load_trusted_operator_policy(path: Path) -> TrustedOperatorPolicy: ...

class CommissioningContractError(Exception): ...
class ServiceOwnershipUnavailable(Exception): ...
class SourceInventoryMismatch(Exception): ...
class ResultNormalizationError(Exception): ...

@dataclass(frozen=True)
class InitializationWorkerRequest:
    attempt_id: str
    manifest_sha256: str
    instance: str
    endpoints: tuple[EndpointIdentity, ...]
    approved_effects: tuple[str, ...]
    required_groups: tuple[str, ...]
    required_samples: int
    deadline_monotonic: float
    result_path: Path
    source_artifacts: tuple[Path, ...]
    source_artifact_sha256: dict[str, str]
    config_sha256: dict[str, str]

@dataclass(frozen=True)
class InitializationWorkerResult:
    status: WorkerStatus
    attempt_id: str
    manifest_sha256: str
    identity: IdentityEvidence | None
    observations: tuple[StateEvidence, ...]
    service: ServiceEvidence
    disposition: ResourceDisposition
    error: str | None
    worker_pid: int | None
    worker_start_id: str | None
```

`endpoint_scope_digest(manifest)` builds exactly this canonical object:
`{"robot_id": <robot_id>, "site_id": <site_id>, "endpoints": [<sorted endpoint objects>]}`.
Each endpoint object has exactly `endpoint_id`, `locator`, `subsystem`, and
`transport`; the list is sorted by `(subsystem, endpoint_id, transport,
locator)`. Serialize the object with `json.dumps(..., sort_keys=True,
separators=(",", ":"), ensure_ascii=True)` and hash those UTF-8 bytes. No
string concatenation or delimiter inference is permitted.
Paths are absolute, slash-normalized, resolved without following an existing
symlink, and rejected when any existing component is a symlink or contains
`..`. The CLI reads a fixed, root-owned `TrustedOperatorPolicy` file selected
by site installation configuration, not a caller-provided root. The manifest
root must match that policy exactly; its policy digest and the authorization
record bind the root. The same rule applies to `host_enrollment_sha256`: the
authority file is accepted only when its canonical digest matches the manifest
and authorization record.

The shared test fixture module defines these concrete helpers with the stated
signatures: `valid_manifest_data(tmp_path: Path) -> dict[str, object]`,
`write_json(path: Path, data: Mapping[str, object]) -> Path`,
`valid_manifest(tmp_path: Path) -> CommissioningManifest`,
`valid_authority(manifest: CommissioningManifest) -> HostEnrollmentAuthority`,
`acquire_in_child(manifest: CommissioningManifest, attempt_id: str) ->
dict[str, object]`, and `release_child(result: dict[str, object]) -> None`.
Task 1 implements their bodies before any production test consumes them:
manifest helpers create the trusted policy and authority digests, child helpers
launch the lock API in a separate Python process, and release helpers wait for
the child to report a matching attempt before releasing it.

Task-local test doubles have explicit names and signatures: `fake_modules`
installs a fake dependency module with an `initialize_calls` counter;
`request_for(instance: str) -> InitializationWorkerRequest` builds a fully
approved worker request; `run_worker(request, gate) ->
InitializationWorkerResult` invokes the worker entry with injected imports;
`UnavailableGate` raises `ServiceOwnershipUnavailable`; and
`PreflightOnlyGate` reports `service_ownership_enforcement_unavailable`
without entering dependency startup. These are test-only helpers, never
production interfaces.

## Task 1: Pure Manifest and Host-Enrollment Contract

**Files:**
- Create: `robots/lynsense_real_box/commissioning_contract.py`
- Create: `tests/unit_tests/robots/lynsense_real_box/test_commissioning_contract.py`
- Create: `tests/unit_tests/robots/lynsense_real_box/conftest.py`

**Interfaces:**
- Produces `load_commissioning_manifest(path: Path) -> CommissioningManifest`.
- Produces `validate_commissioning_manifest(manifest: CommissioningManifest, *, actual_host: HostIdentity, enrollment: HostEnrollmentAuthority, authorization: AuthorizationRecord | None = None, policy: TrustedOperatorPolicy | None = None, trusted_lock_root: Path) -> None`.
- Produces `CommissioningManifest`, `HostIdentity`, `HostEnrollmentAuthority`, `TrustedOperatorPolicy`, `AuthorizationRecord`, and `CommissioningOutcome` frozen dataclasses plus the status literals above.
- `CommissioningManifest` additionally includes `trusted_lock_root`, `host_enrollment_sha256`, `source_artifact_sha256`, `config_sha256`, `budgets`, `freshness_s`, `endpoints`, `peripheral_mapping`, and `retention_policy`.
- `source_artifact_sha256` and `config_sha256` keys are normalized absolute POSIX paths present in `source_artifacts`; unresolved logical names are rejected before binding or dependency import.
- `load_trusted_operator_policy()` verifies the policy file's self-excluding canonical digest and returns its allowed operator set; authorization validation rejects a nonempty operator absent from that set.
- The trusted policy also contains `approved_authorizations: dict[authorization_id, authorization_sha256]`; the CLI verifies the authorization file's self-excluding digest before contract validation. `AuthorizationRecord.attempt_id` binds the approval to the exact one-shot attempt.

- [x] **Step 1: Write failing contract tests.** Cover exact top-level keys, required nonempty identifiers, `ros_domain_id == 3`, timezone-aware nonexpired authorization, positive finite budgets, `required_samples >= 2`, absolute lock/evidence paths, canonical machine ID format, and rejection of missing/extra fields.

```python
def test_manifest_rejects_missing_machine_identity(tmp_path: Path):
    data = valid_manifest_data(tmp_path)
    data.pop("execution_host_machine_id")
    with pytest.raises(CommissioningContractError, match="machine identity"):
        load_commissioning_manifest(write_json(tmp_path / "manifest.json", data))
```

- [x] **Step 2: Add the shared fixture module and host-enrollment tests.** Implement `valid_manifest_data(tmp_path)`, `write_json(path, data)`, `valid_manifest(tmp_path)`, `valid_authority(manifest)`, `acquire_in_child(manifest, attempt_id)`, and `release_child(result)` in `conftest.py`. The fixture creates a non-symlink temporary trusted root, computes the self-excluding manifest digest, and uses a fake clock/authority. Tests verify the local host ID equals the manifest ID, the authority lists the site/ID exactly once, duplicate IDs are rejected, hostname is only a cross-check, and a different host fails before any lock path is touched.

- [x] **Step 3: Run RED for the contract behavior.**

Run: `/home/huhu/work/RPent_lynsense/RPent/.venv/bin/pytest -q tests/unit_tests/robots/lynsense_real_box/test_commissioning_contract.py`

Expected: fixtures collect, then the test fails with missing contract symbols; no runtime dependency module is imported.

- [x] **Step 4: Implement pure parsing and validation.** Use strict key checks, finite-number validation, canonical JSON serialization with the digest field excluded, the path and endpoint serialization rules above, and no imports from `lynrotcontrol`, `rclpy`, sockets, subprocess, or ROS packages. Reject symlinks, `..`, non-absolute evidence paths, a manifest root different from the operator policy root, an authority digest mismatch, and a manifest digest mismatch.

- [x] **Step 5: Run GREEN and the import audit.**

Run: `/home/huhu/work/RPent_lynsense/RPent/.venv/bin/pytest -q tests/unit_tests/robots/lynsense_real_box/test_commissioning_contract.py`

Expected: all contract and host-enrollment tests pass; an import audit confirms no runtime dependency module is loaded.

## Task 2: Durable Pre-Initialization Lock

**Files:**
- Create: `robots/lynsense_real_box/commissioning_lock.py`
- Create: `tests/unit_tests/robots/lynsense_real_box/test_commissioning_lock.py`

**Interfaces:**
- `LockRecord` binds `site_id`, `robot_id`, `endpoint_scope_sha256`, `manifest_sha256`, `attempt_id`, host identity, creation time, state, and service identity.
- `CommissioningLock.acquire(manifest: CommissioningManifest, record: LockRecord) -> CommissioningLockHandle` claims the manifest-derived lock directory with exclusive `mkdir`, writes canonical JSON, flushes and `fsync`s it, and rejects an existing record without deleting it.
- `CommissioningLockHandle.release_after_settled(disposition: ResourceDisposition) -> None` removes only a lock whose record still matches the attempt and manifest digest.
- `CommissioningLockHandle.retain_unknown(reason: str) -> None` atomically updates the record to unresolved state and leaves it in place.
- `CommissioningLockHandle.record_service_identity(service: ServiceEvidence) -> None` atomically records verified PID/start ID/epoch on a held or unknown lock before reconciliation.
- `reconcile_lock(path: Path, *, operator: str, authorization_id: str, expected_attempt_id: str, expected_manifest_sha256: str, expected_endpoint_scope_sha256: str, disposition: ResourceDisposition) -> None` releases only a matching unresolved record. It requires a nonempty disposition operator and verified service PID/start ID/epoch matching the retained lock; missing or mismatched service identity never releases the lock.
- `read_lock_record(path: Path) -> LockRecord` never treats age or expiration as permission to remove a lock.

- [x] **Step 1: Write failing lock tests.** Cover first acquisition, same-process duplicate, cross-process duplicate, alternate runtime-directory attempt using the same canonical lock path, malformed existing record, stale record retention, matching release, mismatched release rejection, and explicit reconciliation.

```python
def test_second_process_is_rejected_while_first_holds_lock(tmp_path: Path):
    manifest = valid_manifest(tmp_path)
    first = acquire_in_child(manifest, attempt_id="first")
    second = acquire_in_child(manifest, attempt_id="second")
    assert second == {"status": "rejected", "reason": "lock_exists"}
    release_child(first)
```

- [x] **Step 2: Run RED.**

Run: `/home/huhu/work/RPent_lynsense/RPent/.venv/bin/pytest -q tests/unit_tests/robots/lynsense_real_box/test_commissioning_lock.py`

Expected: collection fails because `CommissioningLock` and its record/handle types are absent.

- [x] **Step 3: Implement atomic lock records.** Use a lock directory, not an overwriteable lock file, so an update cannot replace the ownership claim. Bind the record to `site_id`, `robot_id`, endpoint identities, pinned machine ID, manifest digest, attempt ID, creation time, and state. Derive the path from the validated endpoint scope, never from a runtime socket or caller-selected path. After `mkdir(exist_ok=False)`, fsync the parent directory before writing `record.json`; fsync the record, then fsync the lock directory before initialization may begin.

- [x] **Step 4: Implement unresolved retention and reconciliation.** Make timeout/unknown records durable across process exit; require matching attempt, service epoch when available, operator and authorization metadata for release; never auto-expire. For every record update, write the temporary file, flush and `fsync` the temporary file, atomically rename it over `record.json`, then `fsync` the lock directory. After release, remove the record/directory and `fsync` the parent directory.

- [x] **Step 5: Run GREEN including subprocess coverage.**

Run: `/home/huhu/work/RPent_lynsense/RPent/.venv/bin/pytest -q tests/unit_tests/robots/lynsense_real_box/test_commissioning_lock.py`

Expected: all lock lifecycle and cross-process race tests pass, with no lock file left behind except deliberately retained unknown fixtures.

## Task 3: Fixed LynrotControl Binding and Read Normalization

**Files:**
- Create: `robots/lynsense_real_box/commissioning_binding.py`
- Create: `tests/unit_tests/robots/lynsense_real_box/test_commissioning_binding.py`

**Interfaces:**
- `ServiceOwnershipGate.initialize_under_guard(instance: str, endpoints: tuple[EndpointIdentity, ...], approved_effects: tuple[str, ...]) -> InitializedRuntime` is the only operation allowed to enter the dependency spawn/reuse decision.
- `CommissioningBinding.initialize(instance: str) -> InitializedRuntime` delegates to that gate and raises `ServiceOwnershipUnavailable` when actual ownership cannot be enforced. `SourceInventoryMismatch` and `ResultNormalizationError` are typed failures, not generic `ValueError` results.
- `InitializedRuntime.identity() -> IdentityEvidence` returns configured and device-reported identity separately.
- `InitializedRuntime.read_group(group: Literal["left_arm", "right_arm", "left_gripper", "right_gripper", "waist_lift", "left_force", "right_force"]) -> StateEvidence` calls only fixed public methods.
- `InitializedRuntime.service_identity() -> ServiceEvidence` returns PID/start identity/epoch when observable; missing fields remain unavailable.
- `RealLynrotControlBinding` imports the dependency only inside `initialize`; `FakeCommissioningBinding` records operation traces and delayed completions for offline tests.

- [x] **Step 1: Write failing fake-binding tests.** Verify no dependency import occurs during contract import or fake construction, methods are fixed rather than input-selected, a left/right mapping reversal fails, unknown result objects fail closed, fake traces reject motion/recovery/cancel/publish operations, and a preflight-only gate returns `service_ownership_enforcement_unavailable` without calling initialization.

```python
def test_source_digest_mismatch_blocks_before_initialize(fake_modules, manifest):
    changed = dataclasses.replace(
        manifest, source_artifact_sha256={"lynrotcontrol": "deadbeef"}
    )
    binding = RealLynrotControlBinding(gate=RecordingGate(), manifest=changed)
    with pytest.raises(SourceInventoryMismatch):
        binding.initialize(changed.instance)
    assert fake_modules.initialize_calls == 0
```

- [x] **Step 2: Run RED.**

Run: `/home/huhu/work/RPent_lynsense/RPent/.venv/bin/pytest -q tests/unit_tests/robots/lynsense_real_box/test_commissioning_binding.py`

Expected: collection fails because the binding, gate, and normalization symbols are absent.

- [x] **Step 3: Define normalization helpers.** Unwrap the dependency `Result` only when `ok` is true; validate finite values, units, source/frame, side, axes, and timestamps; keep `configured`, `device_reported`, and `operator_verified` fields distinct. Set `sample_provenance` only to `device_timestamp`, `device_sequence`, or an adapter-reviewed `request_response_acquisition` carried by a trusted `StateEvidence`, never from a self-declared payload field. Never use a local callback/request counter as the sole freshness proof. A failed arm mapping, unknown force transform, missing waist parameter, or reversed side is a typed rejection, not a degraded success.

- [x] **Step 4: Implement the real fixed binding.** The binding may call `lynrotcontrol.initialize(instance)` only from the implementation of `ServiceOwnershipGate.initialize_under_guard`; a normal wrapper call is forbidden because it cannot prove the actual reuse/spawn decision was guarded. After initialization expose only these fixed reads: arm `reading.get_model/get_ip/get_side/get_axes/get_version/get_state/get_joints`, gripper `read`, waist `lift.read`, force `read`, robot `get_info/get_bindings/get_modules`. Do not expose `move`, `cancel`, `recover`, `set_state`, or arbitrary method dispatch.

- [x] **Step 5: Add source/configuration identity checks.** Resolve the approved dependency root and bundled SDK/config file list from the manifest's normalized `source_artifacts`, hash every referenced file before initialization, and reject an unresolved key or path/hash mismatch without importing or contacting the device. Treat `get_version()` as library/configuration evidence unless the selected adapter explicitly identifies it as device-reported.

- [x] **Step 6: Run GREEN binding tests.**

Run: `/home/huhu/work/RPent_lynsense/RPent/.venv/bin/pytest -q tests/unit_tests/robots/lynsense_real_box/test_commissioning_binding.py`

Expected: fake binding tests pass; the real binding is tested only through monkeypatched dependency modules and never opens a real socket or ROS context.

## Task 4: Evidence Writer and One-Attempt Runner

**Files:**
- Create: `robots/lynsense_real_box/commissioning_worker.py`
- Create: `robots/lynsense_real_box/commissioning_evidence.py`
- Create: `robots/lynsense_real_box/commissioning_runner.py`
- Create: `tests/unit_tests/robots/lynsense_real_box/test_commissioning_worker.py`
- Create: `tests/unit_tests/robots/lynsense_real_box/test_commissioning_evidence.py`
- Create: `tests/unit_tests/robots/lynsense_real_box/test_commissioning_runner.py`

**Interfaces:**
- `CommissioningEvidenceWriter(run_id: str, manifest: CommissioningManifest, path: Path)` writes canonical credential-filtered JSONL and a final JSON report; it rejects forbidden keys and non-finite values using the existing `EvidenceRecorder` validation conventions.
- `InitializationWorkerRequest` carries only attempt ID, manifest digest, instance, approved endpoints/effects, required groups/sample count, an absolute monotonic deadline, a result path, and normalized source/config artifact paths plus digests; it never carries arbitrary Python or shell input.
- `InitializationWorkerResult` is JSON-serializable and carries status, identity evidence, all worker-collected observations, service evidence, resource disposition, error, and worker identity. No `InitializedRuntime` crosses the process boundary.
- `InitializationSupervisor.run_worker(request: InitializationWorkerRequest) -> InitializationWorkerResult` starts the worker and reads its result file/pipe under the deadline.
- `CommissioningRunner(lock_manager: CommissioningLock, evidence: CommissioningEvidenceWriter, supervisor: InitializationSupervisor, clock: Clock)`.
- `CommissioningRunner.validate_only(manifest: CommissioningManifest, host: HostIdentity, enrollment: HostEnrollmentAuthority, policy: TrustedOperatorPolicy) -> CommissioningOutcome` performs no dependency import or side effect.
- `CommissioningRunner.run_once(manifest: CommissioningManifest, host: HostIdentity, enrollment: HostEnrollmentAuthority, authorization: AuthorizationRecord, policy: TrustedOperatorPolicy) -> CommissioningOutcome` performs preflight, lock acquisition, one worker initialization, worker-collected identity reads, two-or-more state observations, and explicit lock disposition.

- [x] **Step 1: Write failing worker and evidence tests.** Cover worker import ordering, exactly one initialization invocation, worker-collected identity/state evidence, canonical JSONL, run/manifest linkage, forbidden credential-like keys, finite numeric enforcement, final report fields, truncated existing evidence, and write failure after device contact.

```python
def test_worker_does_not_initialize_without_actual_ownership_gate(fake_modules):
    result = run_worker(request_for("robot_one"), gate=UnavailableGate())
    assert result.status == "service_ownership_enforcement_unavailable"
    assert fake_modules.initialize_calls == 0
```

- [x] **Step 2: Write failing runner state-machine tests.** Cover `prepared -> preflight_passed -> initializing -> observing -> accepted`, rejection before contact, service conflict, host mismatch, left-success/right-failure without continued reads, force/waist/gripper read failure, delayed worker timeout with retained lock, late worker completion, second-process lock rejection while the worker is late, identity/epoch change, Ctrl+C, and evidence failure.

- [x] **Step 3: Run RED.**

Run: `/home/huhu/work/RPent_lynsense/RPent/.venv/bin/pytest -q tests/unit_tests/robots/lynsense_real_box/test_commissioning_worker.py tests/unit_tests/robots/lynsense_real_box/test_commissioning_evidence.py tests/unit_tests/robots/lynsense_real_box/test_commissioning_runner.py`

Expected: collection fails because the worker result protocol, evidence writer, and runner are absent.

- [x] **Step 4: Implement evidence and worker supervision.** Persist a validated intent event and create the evidence destination before contact. The parent starts a dedicated worker only after validation and lock acquisition. The worker rechecks source/config digests, imports the selected installation, and enters initialization only through `ServiceOwnershipGate.initialize_under_guard`. The worker performs identity and required-group reads itself only when initialization settles before the shared absolute deadline. If initialization settles after the deadline, it writes `late_settled` with no peripheral reads. If the parent deadline expires first, the parent returns `initialization_outcome_unknown`, retains the lock, records worker/service identities, and leaves the worker result file for reconciliation; it does not cancel or kill the worker/service.
- [x] **Step 4: Implement evidence and worker supervision.** Persist a validated intent event and create the evidence destination before contact; fsync the new JSONL file's parent directory after creation. The parent starts a dedicated worker only after validation and lock acquisition. The worker rechecks source/config digests from the request before entering the fixed gate, imports the selected installation only inside that gate, and performs identity and required-group reads itself only when initialization settles before the shared absolute deadline. If initialization settles after the deadline, it writes `late_settled` with no peripheral reads. If the parent deadline expires first, the parent returns `initialization_outcome_unknown`, retains the lock, records worker/service identities, and leaves the worker result file for reconciliation; it does not cancel or kill the worker/service.
- [x] **Step 4a: Keep the child gate fixed and fail closed until approved.** The child entry calls one project-owned `build_effectful_gate(request)` factory; the current implementation raises `ServiceOwnershipUnavailable`. A future dependency-side lifecycle hook replaces this implementation under a separately reviewed scope; requests cannot name arbitrary import paths or callables.
- [x] **Step 5: Define late-result persistence and reconciliation semantics.** The worker writes a temporary result, fsyncs it, atomically renames it to `<attempt_id>.result.json`, and fsyncs the result directory. The parent never changes an unknown lock to settled from a late result automatically. `reconcile` reads the durable result, verifies attempt/digest/service identity, records the operator's disposition, and only then releases the lock. A worker that remains alive is recorded by PID/start identity; no process kill is inferred from a missing result.

- [x] **Step 6: Implement preflight, lock ordering, and one-shot observation.** Validate host authority and effective code/config hashes, create the evidence destination, inspect service ownership, acquire the durable endpoint lock, recheck ownership at the actual guarded startup/reuse boundary, start one worker, and accept only a `settled_success` result whose `attempt_id`, `manifest_sha256`, service epoch, and endpoint scope match the current lock. Require identity plus two distinct observations for every required group. For cached subscriptions, a new local callback sequence is insufficient: require an advancing device source sequence, device timestamp, or an adapter-reviewed `request_response_acquisition` provenance. A returned initialization failure stops all peripheral reads; a late result never releases the lock or enables a retry.

- [x] **Step 7: Implement resource disposition.** Record service PID/start identity/epoch, initialized endpoints, owned ROS resources, worker PID/start identity, result path, and retention policy. Release a clean lock only after worker result and report are durable; retain unknown/uncertain records until reconciliation. `retained_for_operator` is valid only when explicitly selected, settled, and assigned to an operator; otherwise the result is failed or unknown.

- [x] **Step 8: Run GREEN including worker lifecycle tests.**

Run: `/home/huhu/work/RPent_lynsense/RPent/.venv/bin/pytest -q tests/unit_tests/robots/lynsense_real_box/test_commissioning_worker.py tests/unit_tests/robots/lynsense_real_box/test_commissioning_evidence.py tests/unit_tests/robots/lynsense_real_box/test_commissioning_runner.py`

Expected: all fake-binding, delayed-operation, partial-failure, lock-retention, and report-integrity tests pass without importing the real dependency.

## Task 5: Operator CLI, Runbook, and Regression Gates

**Files:**
- Create: `robots/lynsense_real_box/commissioning_cli.py`
- Create: `tests/unit_tests/robots/lynsense_real_box/test_commissioning_cli.py`
- Modify: `robots/lynsense_real_box/README.md`
- Create: `docs/superpowers/plans/2026-09-23-robot-one-lynrotcontrol-first-connection-runbook.md`

**Interfaces:**
- `python -m robots.lynsense_real_box.commissioning_cli validate --manifest <path> --host-enrollment <path>` runs pure validation only.
- `python -m robots.lynsense_real_box.commissioning_cli run --manifest <path> --host-enrollment <path> --authorization-file <path>` is the only effectful mode and requires an explicit authorization record matching the manifest digest, host identity, operator, Robot One, ROS domain, and expiration.
- `python -m robots.lynsense_real_box.commissioning_cli reconcile --manifest <path> --lock <derived-path> --authorization-file <path>` performs no initialization and can release only a matching retained record.
- CLI exit codes: `0` only for accepted identity/state/resource disposition; `2` for pre-contact validation rejection; `3` for known initialization/read failure; `4` for unknown/retained lock outcome; `5` for unavailable actual service-ownership enforcement.

- [x] **Step 1: Write failing CLI tests.** Verify help and `validate` never import LynrotControl; `run` requires authorization; wrong host/domain/digest rejects before worker start; unavailable service ownership rejects before initialization; no Dashboard/planner path is created; `reconcile` requires a matching retained lock; and exit codes map exactly to `CommissioningOutcome`.

- [x] **Step 2: Run RED.**

Run: `/home/huhu/work/RPent_lynsense/RPent/.venv/bin/pytest -q tests/unit_tests/robots/lynsense_real_box/test_commissioning_cli.py`

Expected: collection fails because the CLI module and exit-code mapping are absent.

- [x] **Step 3: Implement CLI argument parsing and authorization loading.** Keep the default command pure; load the fixed `TrustedOperatorPolicy`, verify the authorization file's self-digest and policy-approved authorization digest, require its exact `attempt_id`, reject extra positional values, unknown effect names, arbitrary Python/shell input, and paths outside trusted roots. `reconcile` never initializes or contacts the robot.

- [x] **Step 4: Run GREEN CLI tests.**

Run: `/home/huhu/work/RPent_lynsense/RPent/.venv/bin/pytest -q tests/unit_tests/robots/lynsense_real_box/test_commissioning_cli.py`

Expected: all import-order, authorization, gate-block, reconciliation, and exit-code tests pass.

- [x] **Step 5: Write the exact-command runbook.** Document pinned host, machine enrollment, manifest digest, lock path, operator authorization, local evidence output, preflight checks, effect inventory, no-motion restrictions, reconciliation procedure, and the fact that this runbook does not authorize motion or Toolkit live mode.

- [x] **Step 6: Update README.** Explain the commissioning command, the three independent acceptance fields, retained-lock handling, and the separation from atomic Toolkit/live motion.

- [x] **Step 7: Run focused, merged, and static gates.**

```bash
/home/huhu/work/RPent_lynsense/RPent/.venv/bin/pytest -q \
  tests/unit_tests/robots/lynsense_real_box/test_commissioning_contract.py \
  tests/unit_tests/robots/lynsense_real_box/test_commissioning_lock.py \
  tests/unit_tests/robots/lynsense_real_box/test_commissioning_binding.py \
  tests/unit_tests/robots/lynsense_real_box/test_commissioning_worker.py \
  tests/unit_tests/robots/lynsense_real_box/test_commissioning_evidence.py \
  tests/unit_tests/robots/lynsense_real_box/test_commissioning_runner.py \
  tests/unit_tests/robots/lynsense_real_box/test_commissioning_cli.py
/home/huhu/work/RPent_lynsense/RPent/.venv/bin/python -m compileall -q \
  robots/lynsense_real_box tests/unit_tests/robots/lynsense_real_box
timeout 180s /home/huhu/work/RPent_lynsense/RPent/.venv/bin/pytest -q \
  tests/unit_tests --ignore=tests/unit_tests/rpent/robots/components/test_pi05_vla_server_contracts.py
git diff --check
/home/huhu/.local/bin/rtk rg -n "TB[D]|TO[D]O|placehol[d]er|gues[s]ed Robot One|read[-]only connection" \
  docs/superpowers/plans/2026-09-23-robot-one-lynrotcontrol-first-connection.md \
  docs/superpowers/plans/2026-09-23-robot-one-lynrotcontrol-first-connection-runbook.md \
  robots/lynsense_real_box/README.md
```

Expected: focused and merged tests pass; compilation, whitespace, and documentation scans pass; no ROS, SSH, SDK initialization, hardware command, dependency installation, or remote write occurs.

## Explicit Feasibility Gate

The local dependency reference shows that `ensure_service()` can ping/reuse an
existing persistent service or spawn one after an external preflight check.
That is a race window. Before any real `run`, one of these must be separately
reviewed and implemented:

1. A dependency-side lifecycle hook that holds the endpoint lock through the
   actual reuse/spawn decision and returns service identity/epoch; or
2. An equivalent site adapter whose implementation and tests prove the same
   atomic ownership boundary.

Until that gate exists, the code may implement validation, fake execution,
evidence schemas, lock/reconciliation, and negative tests, but the real
effectful CLI must return exit code `5` without importing or calling
`lynrotcontrol.initialize()`.

This is a deliberate implementation boundary, not permission to silently
modify the external dependency. Any dependency patch, deployment, or Robot
One initialization needs separate scope approval and a new review of its exact
diff.

## Self-Review Checklist

- **Spec coverage:** Tasks 1–2 cover all authority, host identity, durable-lock and reconciliation requirements; Task 3 covers fixed dependency boundary, identity provenance and read normalization; Task 4 covers state machine, partial/unknown outcomes, evidence and resource disposition; Task 5 covers operator-only entry, runbook, docs and all regression gates. No task opens live motion or Dashboard.
- **Marker scan:** No unspecified markers or “handle edge cases” steps are used. Site-specific values are intentionally validated as required manifest input and are never estimated in code or the plan.
- **Type consistency:** `CommissioningManifest`, `HostIdentity`, `HostEnrollmentAuthority`, `CommissioningBinding`, `InitializedRuntime`, `CommissioningEvidenceWriter`, `CommissioningRunner`, `CommissioningOutcome`, `CommissioningLockHandle`, and `ResourceDisposition` are introduced before downstream tasks consume them.
- **Feasibility:** A normal wrapper cannot enforce the dependency's actual service reuse/spawn race. The implementation must fail closed with exit code `5` until a separately reviewed dependency hook or equivalent site adapter implements `ServiceOwnershipGate.initialize_under_guard`.
- **Boundary check:** The plan does not modify the existing atomic Toolkit, `robot_spec.py` live rejection, Dashboard, or planner APIs. Any future live motion work remains a separate approved design.

## Review Record

- Spec: `docs/superpowers/specs/2026-09-23-robot-one-lynrotcontrol-first-connection-design.md` (approved by user; independent review approved).
- Plan self-review: complete.
- Independent plan review: APPROVED; final read-only review found no Blocker or Important findings.
- User approval of this plan: approved on 2026-09-23.
- Implementation: complete for the offline fail-closed boundary. The fixed `build_effectful_gate` raises `ServiceOwnershipUnavailable`, so `run` exits `5` without importing or calling LynrotControl.
- Final implementation review: independent review found no remaining Blocker or Important finding. Exact effect inventory, interpreter and authorization-expiry binding, `ROS_DOMAIN_ID=3` worker/gate transport, expected arm/operator/resource acceptance, endpoint-scope binding, consumed-attempt records, advancing device samples, worker-result schema validation, and reconcile audit/purpose separation were tightened after review.
- Validation: commissioning focused suite `90 passed`; Lynsense real-box plus registry suite `468 passed`; full unit suite `1147 passed, 3 skipped`; `compileall`, whitespace, documentation-marker scan, and offline import audit passed.
- Residual minor risks: consumed-marker matching can later compare additional lock fields; a process crash after lock release but before the final reconcile report leaves the durable pending report; identity nested payload schema can be deepened for future local-tamper resistance. These do not enable hardware contact or bypass the unavailable ownership gate.
- Hardware execution: not run. No ROS connection, SSH, dependency deployment, LynrotControl initialization, motion, gripper, waist, chassis, perception trigger, stop, or cancellation was performed.
