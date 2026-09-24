# LynrotControl Service Ownership Gate Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the approved dependency-owned LynrotControl service ownership gate so Robot One commissioning can move beyond the current fail-closed exit `5` boundary without exposing motion or model access.

**Architecture:** LynrotControl owns endpoint authority, bootstrap claiming, durable claim records, legacy-path enforcement, and exact read dispatch. RPent only verifies a fixed pre-import protocol artifact, calls `initialize_under_ownership`, validates receipts, and maps reconciliation failures. All implementation and tests remain offline and fake-only.

**Tech Stack:** Python 3.10+, stdlib `fcntl`/`socket`/`subprocess`/`json`/`hashlib`, existing LynrotControl Unix JSON-RPC service, existing RPent pytest suites, `unittest` for the external dependency.

**Spec:** `docs/superpowers/specs/2026-09-24-lynrotcontrol-service-ownership-gate-design.md`

## Global Constraints

- Do not run ROS, connect to Robot One, spawn the real LynrotControl service, publish a topic, call a ROS service, send an action goal, initialize hardware, stop hardware, or trigger perception.
- Do not modify `/home/huhu/work/RPent_lynsense/lynrotcontrol` until the user separately authorizes external dependency implementation after approving this plan.
- Preserve the existing RPent worktree changes and untracked commissioning files; never reset or clean the worktree.
- No motion, trajectory, recovery, calibration, force zeroing, gripper/waist/chassis command, Toolkit, planner, Dashboard, or pytree surface is part of this plan.
- The only approved effect tuple is exactly `("initialize_configuration", "fixed_read_ros_resource_creation", "fixed_read_ros_parameter_query")`.
- The ownership protocol artifact is exactly:

```json
{
  "endpoint_authority_root": "/tmp/lynrotcontrol-authority-1008",
  "protocol_id": "lynrotcontrol.service-ownership",
  "protocol_version": 1,
  "read_allowlist_sha256": "38015e139b4b7ae2c0ca7f2261be2a90cb4d40ee12d35420875c6d150962ceee",
  "runtime_protocol": 15
}
```

- The SHA-256 of those exact artifact bytes, including the final newline, is `53a6ea5b7787ae2926f2ee951cd2de630c1af7b03e2bf4bca8cc622729225062`.
- Bump the runtime socket protocol from `14` to `15`; never fall back to protocol `14`.
- The endpoint authority root selected above is tied to the verified local UID `1000`. Deployment to a Robot One account with another UID must fail closed and require a new artifact, digest, manifest, review, and approval.
- Canonical JSON means sorted keys, `(',', ':')` separators, UTF-8 bytes, no trailing newline, and ASCII-only endpoint/scope identifiers.
- Do not commit unless the user explicitly authorizes commits.

---

### Task 1: Protocol Constants And Artifact

**Files:**
- Create: `/home/huhu/work/RPent_lynsense/lynrotcontrol/OWNERSHIP_PROTOCOL.json`
- Create: `/home/huhu/work/RPent_lynsense/lynrotcontrol/lynarmcontrol/implementation/service_ownership.py`
- Modify: `/home/huhu/work/RPent_lynsense/lynrotcontrol/pyproject.toml`
- Test: `/home/huhu/work/RPent_lynsense/lynrotcontrol/development/tests/test_service_ownership_protocol.py`

**Interfaces:**
- Consumes: existing dependency package layout and Python stdlib.
- Produces:
  - `PROTOCOL_ID = "lynrotcontrol.service-ownership"`
  - `PROTOCOL_VERSION = 1`
  - `RUNTIME_PROTOCOL = 15`
  - `READ_ALLOWLIST_SHA256 = "38015e139b4b7ae2c0ca7f2261be2a90cb4d40ee12d35420875c6d150962ceee"`
  - `CanonicalEndpoint(subsystem, endpoint_id, transport, locator)`
  - `canonical_endpoint_json(endpoint: CanonicalEndpoint) -> bytes`
  - `endpoint_scope_digest(site_id: str, robot_id: str, endpoints: tuple[CanonicalEndpoint, ...]) -> str`
  - `load_protocol_artifact(path: Path) -> dict[str, object]`

- [ ] **Step 1: Write the failing protocol tests**

```python
"""Offline ownership protocol constants; no service, ROS, or device contact."""
import hashlib
import tempfile
import unittest
from pathlib import Path

from lynrotcontrol.lynarmcontrol.implementation import service_ownership as ownership


class ServiceOwnershipProtocolTests(unittest.TestCase):
    def test_canonical_endpoint_and_scope_digests(self):
        endpoint = ownership.CanonicalEndpoint(
            subsystem="arm",
            endpoint_id="left_arm",
            transport="tcp",
            locator="192.168.11.60",
        )
        self.assertEqual(
            hashlib.sha256(ownership.canonical_endpoint_json(endpoint)).hexdigest(),
            hashlib.sha256(
                b'{"endpoint_id":"left_arm","locator":"192.168.11.60",'
                b'"subsystem":"arm","transport":"tcp"}'
            ).hexdigest(),
        )
        self.assertEqual(
            ownership.endpoint_scope_digest(
                "site-one", "robot-one", (endpoint,)
            ),
            "7ca4d08855f0b3448a23f031ae8b0d39a7cb79962e8ddef3e86368c80e554bc5",
        )

    def test_artifact_has_exact_schema_values_and_digest(self):
        artifact = Path(ownership.__file__).parents[2] / "OWNERSHIP_PROTOCOL.json"
        metadata = ownership.load_protocol_artifact(artifact)
        self.assertEqual(metadata, {
            "endpoint_authority_root": "/tmp/lynrotcontrol-authority-1008",
            "protocol_id": "lynrotcontrol.service-ownership",
            "protocol_version": 1,
            "read_allowlist_sha256": ownership.READ_ALLOWLIST_SHA256,
            "runtime_protocol": 15,
        })
        self.assertEqual(
            hashlib.sha256(artifact.read_bytes()).hexdigest(),
            "53a6ea5b7787ae2926f2ee951cd2de630c1af7b03e2bf4bca8cc622729225062",
        )

    def test_bad_artifact_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "OWNERSHIP_PROTOCOL.json"
            path.write_text('{"protocol_id":"wrong"}\n', encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "ownership protocol artifact"):
                ownership.load_protocol_artifact(path)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the new dependency test and verify the expected failure**

Run from `/home/huhu/work/RPent_lynsense/lynrotcontrol`:

```bash
/home/huhu/work/RPent_lynsense/RPent/.venv/bin/python -m unittest \
  development.tests.test_service_ownership_protocol -v
```

Expected: import/error failure because `service_ownership.py` and the artifact do not exist.

- [ ] **Step 3: Add the exact artifact and minimal protocol module**

```python
"""Canonical service-ownership protocol values; no SDK, ROS, or device imports."""
from __future__ import annotations

import hashlib
import json
import re
from dataclasses import dataclass
from pathlib import Path

PROTOCOL_ID = "lynrotcontrol.service-ownership"
PROTOCOL_VERSION = 1
RUNTIME_PROTOCOL = 15
READ_ALLOWLIST_SHA256 = "38015e139b4b7ae2c0ca7f2261be2a90cb4d40ee12d35420875c6d150962ceee"
_IDENTIFIER = re.compile(r"[A-Za-z0-9_.:/-]+")
_ARTIFACT_KEYS = {
    "endpoint_authority_root",
    "protocol_id",
    "protocol_version",
    "read_allowlist_sha256",
    "runtime_protocol",
}


@dataclass(frozen=True, order=True)
class CanonicalEndpoint:
    subsystem: str
    endpoint_id: str
    transport: str
    locator: str

    def __post_init__(self):
        for name in ("subsystem", "endpoint_id", "transport", "locator"):
            value = getattr(self, name)
            if not isinstance(value, str) or not _IDENTIFIER.fullmatch(value):
                raise ValueError(f"canonical endpoint {name} must be ASCII identifier-like")


def _canonical(value: object) -> bytes:
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")


def canonical_endpoint_json(endpoint: CanonicalEndpoint) -> bytes:
    return _canonical({
        "endpoint_id": endpoint.endpoint_id,
        "locator": endpoint.locator,
        "subsystem": endpoint.subsystem,
        "transport": endpoint.transport,
    })


def endpoint_scope_digest(
    site_id: str, robot_id: str, endpoints: tuple[CanonicalEndpoint, ...]
) -> str:
    for name, value in (("site_id", site_id), ("robot_id", robot_id)):
        if not isinstance(value, str) or not _IDENTIFIER.fullmatch(value):
            raise ValueError(f"{name} must be an ASCII identifier")
    values = [{
        "endpoint_id": endpoint.endpoint_id,
        "locator": endpoint.locator,
        "subsystem": endpoint.subsystem,
        "transport": endpoint.transport,
    } for endpoint in sorted(endpoints)]
    return hashlib.sha256(_canonical({
        "endpoints": values,
        "robot_id": robot_id,
        "site_id": site_id,
    })).hexdigest()


def load_protocol_artifact(path: Path) -> dict[str, object]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read ownership protocol artifact: {exc}") from exc
    expected = {
        "endpoint_authority_root": "/tmp/lynrotcontrol-authority-1008",
        "protocol_id": PROTOCOL_ID,
        "protocol_version": PROTOCOL_VERSION,
        "read_allowlist_sha256": READ_ALLOWLIST_SHA256,
        "runtime_protocol": RUNTIME_PROTOCOL,
    }
    if not isinstance(value, dict) or set(value) != _ARTIFACT_KEYS or value != expected:
        raise ValueError("ownership protocol artifact does not match protocol v1")
    return value
```

- [ ] **Step 4: Re-run the focused dependency test**

```bash
/home/huhu/work/RPent_lynsense/RPent/.venv/bin/python -m unittest \
  development.tests.test_service_ownership_protocol -v
```

Expected: `3 tests` pass.

- [ ] **Step 5: Review the external dependency diff**

```bash
git -C /home/huhu/work/RPent_lynsense/lynrotcontrol diff -- \
  OWNERSHIP_PROTOCOL.json \
  lynarmcontrol/implementation/service_ownership.py \
  development/tests/test_service_ownership_protocol.py
```

Do not commit without separate authorization.

Also add `"OWNERSHIP_PROTOCOL.json"` to `[tool.setuptools.package-data]` so
installed deployments cannot silently omit the pre-import protocol artifact.

---

### Task 2: Source-Derived Endpoint Inventory

**Files:**
- Create: `/home/huhu/work/RPent_lynsense/lynrotcontrol/lynarmcontrol/implementation/endpoint_inventory.py`
- Modify: `/home/huhu/work/RPent_lynsense/lynrotcontrol/adapters/gripper/ctag2f120/implementation.py`
- Modify: `/home/huhu/work/RPent_lynsense/lynrotcontrol/adapters/gripper/ctpm2f50f/implementation.py`
- Modify: `/home/huhu/work/RPent_lynsense/lynrotcontrol/adapters/waist/waist01/implementation.py`
- Modify: `/home/huhu/work/RPent_lynsense/lynrotcontrol/adapters/waist/lyn2_lift/implementation.py`
- Modify: `/home/huhu/work/RPent_lynsense/lynrotcontrol/adapters/waist/lyn2_pitch/implementation.py`
- Modify: `/home/huhu/work/RPent_lynsense/lynrotcontrol/adapters/force/force01/implementation.py`
- Test: `/home/huhu/work/RPent_lynsense/lynrotcontrol/development/tests/test_service_ownership_endpoints.py`

**Interfaces:**
- Consumes: `CanonicalEndpoint` from Task 1 and existing instance/arm configuration loaders.
- Produces:
  - `arm_endpoint(spec: dict[str, object], side: str) -> CanonicalEndpoint`
  - `instance_endpoints(instance: str) -> tuple[CanonicalEndpoint, ...]`
  - module-level `ownership_endpoints(side: str | None) -> tuple[CanonicalEndpoint, ...]` in each peripheral adapter implementation.

- [ ] **Step 1: Add endpoint inventory tests**

```python
"""Endpoint inventory is source-derived and includes read and command resources."""
import unittest

from lynrotcontrol.lynarmcontrol.implementation.endpoint_inventory import instance_endpoints


class EndpointInventoryTests(unittest.TestCase):
    def test_ea200_inventory_is_complete_and_deduplicated(self):
        endpoints = instance_endpoints("ea200")
        keys = [(e.subsystem, e.endpoint_id, e.transport, e.locator) for e in endpoints]
        self.assertEqual(len(keys), len(set(keys)))
        self.assertEqual(keys, [
            ("arm", "left_arm", "tcp", "192.168.11.60"),
            ("arm", "right_arm", "tcp", "192.168.11.61"),
            ("force", "left_force_feedback", "ros2_topic", "/left/wrench"),
            ("force", "right_force_feedback", "ros2_topic", "/right/wrench"),
            ("gripper", "left_gripper_command", "ros2_service", "/left/set_gripper_pos_srv"),
            ("gripper", "left_gripper_feedback", "ros2_topic", "/left/joint_states"),
            ("gripper", "right_gripper_command", "ros2_service", "/right/set_gripper_pos_srv"),
            ("gripper", "right_gripper_feedback", "ros2_topic", "/right/joint_states"),
            ("waist", "waist_lift_command", "ros2_topic", "/motor_lift/set_position"),
            ("waist", "waist_lift_enable", "ros2_service", "/motor_lift/set_enable"),
            ("waist", "waist_lift_feedback", "ros2_topic", "/motor_lift/position"),
            ("waist", "waist_lift_parameters", "ros2_service", "/motor_lift_ros_node/get_parameters"),
        ])

    def test_unsupported_axis_has_no_physical_endpoint(self):
        endpoints = instance_endpoints("ea200")
        self.assertNotIn("waist_pitch", {e.endpoint_id for e in endpoints})


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run it and verify the collection module is missing**

```bash
/home/huhu/work/RPent_lynsense/RPent/.venv/bin/python -m unittest \
  development.tests.test_service_ownership_endpoints -v
```

Expected: import failure.

- [ ] **Step 3: Add source-local descriptor functions**

In each adapter implementation, export only constants already used by that adapter. For example, `ctag2f120` gets:

```python
from ....lynarmcontrol.implementation.service_ownership import CanonicalEndpoint


def ownership_endpoints(side):
    service, topic, _joint = ENDPOINTS[side]
    return (
        CanonicalEndpoint("gripper", f"{side}_gripper_feedback", "ros2_topic", topic),
        CanonicalEndpoint("gripper", f"{side}_gripper_command", "ros2_service", service),
    )
```

`ctpm2f50f` additionally returns the exact parameter service:

```python
CanonicalEndpoint(
    "gripper", f"{side}_gripper_parameters", "ros2_service",
    f"/{side}/crt_gripper_driver/get_parameters",
)
```

Waist adapters return only endpoints defined by their own source constants.
`waist01` returns feedback, command, enable, and parameter endpoints; it has no
emergency-stop endpoint and must not invent one. `lyn2_lift` and `lyn2_pitch`
return feedback, command, enable, stop, and parameter endpoints because those
constants exist in their implementations. `force01` returns `/{side}/wrench`.
Unsupported packages return `()`.

- [ ] **Step 4: Implement `endpoint_inventory.py`**

```python
"""Resolve the complete physical endpoint set without importing ROS or SDK clients."""
from __future__ import annotations

import importlib

from ...implementation.config import load as load_instance
from ..implementation.config import read, resolve, ROOT as ARM_ROOT
from .service_ownership import CanonicalEndpoint


def arm_endpoint(spec: dict, side: str) -> CanonicalEndpoint:
    return CanonicalEndpoint("arm", f"{side}_arm", "tcp", spec["ip"])


def _package_endpoints(kind: str, package: str, side: str | None):
    if package == "unsupported":
        return ()
    module = importlib.import_module(
        f"lynrotcontrol.adapters.{kind}.{package}.implementation"
    )
    function = getattr(module, "ownership_endpoints", None)
    return tuple(function(side)) if callable(function) else ()


def instance_endpoints(instance: str) -> tuple[CanonicalEndpoint, ...]:
    cfg = load_instance(instance)
    arm_cfg = read(ARM_ROOT / "config" / f"{cfg['arms']['config']}.yaml")
    entries = arm_cfg.get("model_arms", {}).get(
        cfg["arms"]["model"],
        arm_cfg.get("arms", {}) if cfg["arms"]["model"] == arm_cfg.get("default_model") else {},
    )
    values = [
        arm_endpoint(resolve(cfg["arms"]["model"], entries[side]["ip"], side), side)
        for side in ("left", "right")
    ]
    for side, package in cfg["grippers"].items():
        values.extend(_package_endpoints("gripper", package, side))
    for axis, package in cfg["waist"].items():
        values.extend(_package_endpoints("waist", package, axis))
    for side, binding in cfg["force"].items():
        if binding.get("provider") == "package":
            values.extend(_package_endpoints("force", binding["package"], side))
    return tuple(sorted(set(values)))
```

- [ ] **Step 5: Run focused inventory and existing peripheral tests**

```bash
/home/huhu/work/RPent_lynsense/RPent/.venv/bin/python -m unittest \
  development.tests.test_service_ownership_protocol \
  development.tests.test_service_ownership_endpoints \
  development.tests.test_lyn2_peripherals -v
```

Expected: all selected tests pass without ROS or hardware contact.

---

### Task 3: Endpoint Authority And Durable Claims

**Files:**
- Modify: `/home/huhu/work/RPent_lynsense/lynrotcontrol/lynarmcontrol/implementation/service_ownership.py`
- Test: `/home/huhu/work/RPent_lynsense/lynrotcontrol/development/tests/test_service_ownership_authority.py`

**Interfaces:**
- Consumes: `CanonicalEndpoint`, canonical JSON, and protocol artifact.
- Produces:
  - `authority_root() -> Path`
  - `acquire_authority(endpoints) -> AuthorityHandle`
  - `OwnershipError`
  - `EndpointAuthorityConflict`
  - `ProcessRecord(pid, start_id, exe, cmdline)`
  - `AuthorityHandle.close() -> None`
  - `read_claim_records(root, endpoints, scope) -> tuple[dict, ...]`
  - `write_active_claim(handle, record) -> str`
  - `mark_claim(handle, scope, state) -> None`
  - `inspect_claim(site_id, robot_id, expected_endpoints) -> ClaimInspection`
  - `service_processes() -> tuple[ProcessRecord, ...]`
  - `matching_service_processes() -> tuple[ProcessRecord, ...]`

- [ ] **Step 1: Write authority and record tests**

Add these concrete `unittest` cases:

```python
def test_two_runtime_directories_conflict_on_one_endpoint(self):
    with tempfile.TemporaryDirectory() as directory, \
         patch.object(ownership, "authority_root", return_value=Path(directory)):
        one = ownership.acquire_authority((LEFT_ARM,))
        try:
            with self.assertRaises(ownership.EndpointAuthorityConflict):
                ownership.acquire_authority((LEFT_ARM,))
        finally:
            one.close()

def test_partial_overlap_releases_only_acquired_locks(self):
    with patch.object(ownership, "authority_root", return_value=self.root):
        right = ownership.acquire_authority((RIGHT_ARM,))
        try:
            with self.assertRaises(ownership.EndpointAuthorityConflict):
                ownership.acquire_authority((LEFT_ARM, RIGHT_ARM))
            left = ownership.acquire_authority((LEFT_ARM,))
            left.close()
        finally:
            right.close()

def test_orphan_record_blocks_new_attempt_without_cleanup(self):
    with patch.object(ownership, "authority_root", return_value=self.root):
        handle = ownership.acquire_authority((LEFT_ARM,))
        try:
            ownership.write_active_claim(handle, self.active_record)
        finally:
            handle.close()
        inspection = ownership.inspect_claim(
            self.site_id, self.robot_id, (LEFT_ARM,)
        )
        self.assertEqual(inspection.state, "claim_orphaned")
        self.assertTrue(inspection.blocking)
        with self.assertRaises(ownership.EndpointAuthorityConflict):
            ownership.acquire_authority((LEFT_ARM,))
```

Add separate named tests for `claim_active`, `claim_released`,
`claim_release_failed`, missing records, partial multi-endpoint records,
malformed JSON, conflicting duplicate records, and authority-without-record
mapping to `authority_record_invalid`; that last test is the externally visible
`claim_pending` case. Each test must assert both the exact inspection state and
whether a new authority acquisition is rejected.

- [ ] **Step 2: Run the test and verify failures**

```bash
/home/huhu/work/RPent_lynsense/RPent/.venv/bin/python -m unittest \
  development.tests.test_service_ownership_authority -v
```

Expected: failures for the missing authority API.

- [ ] **Step 3: Implement authority handles and claim records**

Implementation requirements:

- Create `<root>/endpoints/<sha256(canonical_endpoint_json)>/` with mode `0700`.
- Production callers always use `authority_root()`; only the dependency test harness patches that function or uses a private test constructor.
- Validate every path component with `lstat`: no symlink, current UID, no group/other permission bits.
- Open `authority.lock` with `O_RDWR|O_CREAT|O_CLOEXEC|O_NOFOLLOW`, mode `0o600`.
- Use nonblocking exclusive `flock`; close all acquired descriptors on any conflict.
- Sort endpoints by `CanonicalEndpoint.order=True`.
- Write token-free records to each endpoint's `claims/<scope>.json` with a temporary file, `fsync`, atomic replace, and parent-directory `fsync`.
- Record fields are exactly:

```json
{
  "approved_effects": ["initialize_configuration", "fixed_read_ros_resource_creation", "fixed_read_ros_parameter_query"],
  "claim_state": "claim_active",
  "created_at": "2026-09-24T00:00:00+00:00",
  "endpoint_scope_sha256": "<64 lowercase hex>",
  "endpoints": [canonical endpoint objects in sorted four-field order],
  "protocol_id": "lynrotcontrol.service-ownership",
  "protocol_version": 1,
  "record_sha256": "<64 lowercase hex>",
  "robot_id": approved robot identifier,
  "ros_domain_id": 3,
  "runtime_protocol": 15,
  "service_epoch": "<32 hex>",
  "service_pid": 123,
  "service_start_id": proc start identity
}
```

- `record_sha256` is computed with that field omitted.
- No token or token digest may appear in any record, exception message, receipt, or log.
- `inspect_claim` never mutates records and returns only `state`, `scope`, `record_sha256`, service identity fields, and blocking boolean.
- Do not implement tokenless reconcile or record deletion.

- [ ] **Step 4: Run authority tests and static credential scan**

```bash
/home/huhu/work/RPent_lynsense/RPent/.venv/bin/python -m unittest \
  development.tests.test_service_ownership_authority -v
rg -n "token.*record|record.*token|claim_token" \
  /home/huhu/work/RPent_lynsense/lynrotcontrol/lynarmcontrol/implementation/service_ownership.py \
  /home/huhu/work/RPent_lynsense/lynrotcontrol/development/tests/test_service_ownership_authority.py
```

Expected: tests pass; scan shows no persistent token fields.
Also serialize an actual record, receipt, typed exception message, and log
record, then assert none contains the raw token, token hex, or token digest;
do not rely only on the source-text search.

---

### Task 4: Universal Legacy Spawn Enforcement And Bootstrap

**Files:**
- Modify: `/home/huhu/work/RPent_lynsense/lynrotcontrol/lynarmcontrol/implementation/service_ownership.py`
- Modify: `/home/huhu/work/RPent_lynsense/lynrotcontrol/lynarmcontrol/implementation/client.py`
- Modify: `/home/huhu/work/RPent_lynsense/lynrotcontrol/lynarmcontrol/implementation/service.py`
- Modify: `/home/huhu/work/RPent_lynsense/lynrotcontrol/lynarmcontrol/interfaces/initialization.py`
- Modify: `/home/huhu/work/RPent_lynsense/lynrotcontrol/implementation/initialization.py`
- Test: `/home/huhu/work/RPent_lynsense/lynrotcontrol/development/tests/test_service_ownership_bootstrap.py`

**Interfaces:**
- Consumes: authority API from Task 3 and endpoint inventory from Task 2.
- Produces:
  - `ServiceReceipt`
  - `ensure_service(endpoints=None, *, authority=None, owned=False) -> ServiceReceipt`
  - `probe_service_lock() -> bool`
  - `spawn_service(endpoints, authority, *, owned) -> tuple[Popen-like, int, int]`
  - `run_bootstrap_service(bootstrap_fd: int) -> int`
  - top-level `initialize(instance, *, authority=None)`
  - arm-level `initialize(model=None, ip=None, side=None, *, arm=None, _force_binding=None, authority=None)`

`ServiceReceipt` contains exactly:

```text
runtime_protocol
protocol_id
protocol_version
read_allowlist_sha256
service_pid
service_start_id
service_epoch
endpoint_scope_sha256
endpoints
ros_domain_id
approved_effects
runtime_paths
authority_record_sha256
initialized_endpoints
owned_resources
claim_state
```

`runtime_paths` contains exactly the JSON-string forms of `runtime_dir`,
`control_socket`, `startup_lock`, `service_lock`, `service_log`, and
`authority_root`. The receipt never contains a claim token or token digest.
Task 4 initially sets `initialized_endpoints` and `owned_resources` to empty
tuples and `claim_state` to `unclaimed`; Task 5 refreshes these fields after
claiming and initialization.

- [ ] **Step 1: Write bootstrap and legacy-path tests**

Use mocked `socket.socketpair`, mocked `subprocess.Popen`, fake child identity, and fake `/proc` readers. Do not start the real service. Required assertions:

```python
def test_control_socket_is_absent_until_claim_is_active(self):
    harness = FakeBootstrapHarness(mode="owned")
    harness.spawn()
    self.assertEqual(harness.public_bind_calls, [])
    harness.parent_send_claim()
    self.assertEqual(harness.claim_record_calls, ["write_active_claim"])
    self.assertEqual(harness.public_bind_calls, ["control.sock"])
    self.assertEqual(harness.private_ack.state, "claim_active")

def test_direct_initialize_settings_task_and_motion_are_rejected_during_bootstrap(self):
    harness = FakeBootstrapHarness(mode="owned")
    harness.spawn()
    for payload in (INITIALIZE, SETTINGS, TASK, MOTION):
        with self.assertRaises(ConnectionError):
            client.exchange(payload)
    harness.parent_send_claim()
    for payload in (INITIALIZE, SETTINGS, TASK, MOTION):
        reply = registry.dispatch(payload)
        self.assertFalse(reply.ok)
        self.assertEqual(reply.code, ErrorCode.NOT_INITIALIZED)

def test_legacy_initialize_requires_authority_before_spawn(self):
    events = []
    def acquire(*args, **kwargs):
        events.append("authority")
        return self.authority
    def popen(*args, **kwargs):
        events.append("spawn")
        return FakeChildProcess()
    with patch.object(ownership, "acquire_authority", side_effect=acquire), \
         patch.object(subprocess, "Popen", side_effect=popen):
        arm_initialize(MODEL, IP, SIDE)
    self.assertEqual(events, ["authority", "spawn"])

def test_compatibility_ensure_service_without_endpoints_never_spawns(self):
    with patch.object(client, "exchange", side_effect=ConnectionError("no socket")), \
         patch.object(subprocess, "Popen") as popen:
        with self.assertRaises(client.OwnershipUnavailable):
            client.ensure_service()
    popen.assert_not_called()

def test_crash_after_claim_blocks_legacy_restart(self):
    self.make_orphan_claim()
    with patch.object(subprocess, "Popen") as popen:
        for call in (lambda: robot_initialize(INSTANCE),
                     lambda: arm_initialize(MODEL, IP, SIDE)):
            with self.assertRaises(ownership.EndpointAuthorityConflict):
                call()
    popen.assert_not_called()

def test_old_protocol_service_process_is_conflict(self):
    old = FakeProcess(
        pid=999,
        start_id="999",
        exe="/usr/bin/python",
        cmdline=["python", "-m", "lynrotcontrol.lynarmcontrol.implementation.service"],
    )
    with patch.object(ownership, "service_processes", return_value=[old]), \
         patch.object(subprocess, "Popen") as popen:
        with self.assertRaises(client.ExistingServiceConflict):
            client.ensure_service((LEFT_ARM,), authority=self.authority)
    popen.assert_not_called()

def test_process_match_requires_exact_module_token(self):
    unrelated = FakeProcess(
        pid=1000,
        start_id="1000",
        exe="/usr/bin/python",
        cmdline=["python", "notes.py", "lynrotcontrol.lynarmcontrol.implementation.service"],
    )
    with patch.object(ownership, "service_processes", return_value=[unrelated]):
        self.assertEqual(ownership.matching_service_processes(), ())

def test_held_service_lock_conflict_matrix(self):
    for exchange_result in (PingReply(RECEIPT), ConnectionError("no reply")):
        with self.subTest(exchange_result=exchange_result), \
             patch.object(client, "exchange", side_effect=exchange_result), \
             patch.object(client, "probe_service_lock", return_value=True), \
             patch.object(subprocess, "Popen") as popen:
            with self.assertRaises(client.ExistingServiceConflict):
                client.ensure_service((LEFT_ARM,), authority=self.authority)
            popen.assert_not_called()

def test_no_ping_and_free_service_lock_uses_bootstrap_spawner(self):
    with patch.object(client, "exchange", side_effect=ConnectionError("no reply")), \
         patch.object(client, "probe_service_lock", return_value=False), \
         patch.object(client, "spawn_service", return_value=FakeBootstrapService()) as spawn:
        receipt = client.ensure_service((LEFT_ARM,), authority=self.authority)
    self.assertEqual(receipt.runtime_protocol, 15)
    spawn.assert_called_once_with((LEFT_ARM,), self.authority, owned=False)
```

`FakeBootstrapHarness`, `FakeBootstrapService`, fake request payloads,
`FakeChildProcess`, and `FakeProcess` belong to the test module. They use
in-memory queues and never spawn the real service module.

- [ ] **Step 2: Run tests and verify expected failure**

```bash
/home/huhu/work/RPent_lynsense/RPent/.venv/bin/python -m unittest \
  development.tests.test_service_ownership_bootstrap -v
```

- [ ] **Step 3: Rework the client startup contract**

Exact behavior:

- Set `PROTOCOL = service_ownership.RUNTIME_PROTOCOL`.
- The owned initialization path ignores `LYNROTCONTROL_RUNTIME_DIR` and uses the fixed dependency-derived production runtime directory; tests inject a runtime directory through a private harness rather than the public API.
- Keep `exchange(payload, timeout=None)` for existing reads, but reject protocol mismatch.
- Replace the old no-endpoint spawn behavior:
  - `ensure_service()` with no endpoints may ping a matching service and return its receipt;
  - if no responsive service exists, raise `OwnershipUnavailable`;
  - it must never call `Popen`.
  - typed errors are `OwnershipUnavailable`, `ExistingServiceConflict`, `EndpointAuthorityConflict`, and `UnknownStartupOutcome`.
- `ensure_service(endpoints, authority=authority)` acquires runtime `startup.lock` while the caller already holds endpoint authority.
- Before declaring no service, `service_processes()` reads `/proc/<pid>/exe` and `/proc/<pid>/cmdline`; `/proc/<pid>/exe` supplies executable identity, while classification requires the exact adjacent argv tokens `-m` and `lynrotcontrol.lynarmcontrol.implementation.service`. A substring in unrelated arguments or a native executable path alone is not a match. Any matching process other than the receipt PID/start identity is `ExistingServiceConflict`.
- Acquire `service.lock` nonblocking in the child.
- Stale socket removal is allowed only under startup lock, held service lock, verified current-account Unix socket, and failed ping.
- Pass endpoint authority FDs and a private socketpair FD with `pass_fds`; send descriptors' endpoint mapping and mode over JSON frames on the private socket. Never put a claim token in argv or environment.
- Owned mode child does not bind public `control.sock` until claim activation.
- Legacy mode child may bind after authority handshake because it carries no commissioning claim, but its registry still verifies inherited endpoint authority.

- [ ] **Step 4: Rework service entrypoint and registry ownership state**

`service.py` requirements:

- Direct execution without `--bootstrap-fd <fd>` exits nonzero before binding.
- Validate inherited endpoint FDs and bootstrap JSON before acquiring `service.lock`.
- Acquire `service.lock` with `LOCK_EX | LOCK_NB`; report failure over private channel and exit nonzero.
- Keep authority and service lock descriptors open for the process lifetime.
- Before every initialization dispatch, verify the requested endpoint belongs to the service's inherited authority and re-read blocking records.
- On exit, close process-local transports only; do not call adapter `shutdown()` when it could issue cancel/recovery.

- [ ] **Step 5: Update top-level and arm-level legacy initialization**

Exact flow:

```text
top-level initialize(instance):
  cfg = load(instance)
  endpoints = instance_endpoints(instance)
  with acquire_authority(endpoints) as authority:
      ensure_service(endpoints, authority=authority)
      initialize peripherals and arms using the same authority handle
```

```text
standalone arm initialize(model, ip, side, *, arm=None, _force_binding=None, authority=None):
  spec = resolve(model, ip, side, arm)
  endpoints = (arm_endpoint(spec, spec["side"]),)
  with acquire_authority(endpoints) as authority:
      ensure_service(endpoints, authority=authority)
      send tokenless initialize with that endpoint and receipt
```

No path may acquire the same endpoint lock twice.

- [ ] **Step 6: Run bootstrap tests plus protocol/authority tests**

```bash
/home/huhu/work/RPent_lynsense/RPent/.venv/bin/python -m unittest \
  development.tests.test_service_ownership_protocol \
  development.tests.test_service_ownership_endpoints \
  development.tests.test_service_ownership_authority \
  development.tests.test_service_ownership_bootstrap -v
```

Expected: all pass with fake subprocess/socket/proc providers only.

---

### Task 5: Claim Dispatch And Owned Initialization

**Files:**
- Modify: `/home/huhu/work/RPent_lynsense/lynrotcontrol/lynarmcontrol/implementation/service.py`
- Create: `/home/huhu/work/RPent_lynsense/lynrotcontrol/lynarmcontrol/implementation/owned_runtime.py`
- Create: `/home/huhu/work/RPent_lynsense/lynrotcontrol/interfaces/ownership.py`
- Test: `/home/huhu/work/RPent_lynsense/lynrotcontrol/development/tests/test_service_ownership_dispatch.py`

**Interfaces:**
- Consumes: `ServiceReceipt`, bootstrap service, authority records, exact endpoint inventory, and existing top-level initialization.
- Produces:
  - `OwnedRuntime`
  - `initialize_under_ownership(instance, *, site_id, robot_id, endpoint_scope_sha256, ros_domain_id, approved_effects, expected_endpoints) -> OwnedRuntime`
  - `OwnedRuntime.arm_identity(side: Literal["left", "right"]) -> dict`
  - `OwnedRuntime.read_state(group: FixedStateGroup) -> dict`
  - `OwnedRuntime.robot_info() -> dict`
  - `OwnedRuntime.robot_bindings() -> dict`
  - `OwnedRuntime.robot_modules() -> list[dict]`
  - `OwnedRuntime.service_identity() -> dict`
  - `OwnedRuntime.owned_ros_resources() -> tuple[str, ...]`
  - `OwnedRuntime.release() -> None`

`FixedStateGroup` is exactly `left_arm`, `right_arm`, `left_gripper`,
`right_gripper`, `waist_lift`, `left_force`, or `right_force`.

Exact JSON-compatible return schemas are:

```text
arm_identity:
  model: string
  ip: IPv4 string
  side: "left" | "right"
  axes: positive integer equal to the reviewed model's configured axis count

read_state common:
  group: FixedStateGroup
  values: nonempty list[finite number]
  frame: nonempty string
  axes: nonempty list[string]
  timestamp: RFC3339 string | null
  sequence: nonnegative integer | null
  provenance: "device_timestamp" | "device_sequence"

read_state group invariants:
  left_arm, right_arm: units="rad", side matches group, values length equals arm axes
  left_gripper, right_gripper: units="rad", side matches group
  waist_lift: units="mm", side is null
  left_force, right_force: units=["N","N","N","N.m","N.m","N.m"], side matches group

Exactly one of timestamp and sequence is present. The service sets provenance
from that source and fails the read when neither is available; it never allows
self-declared request-response provenance.

Normalization from source-like adapter payloads is exact and fail-closed:

- arm `get_joints()` converts each degree value with `math.radians(value)`,
  uses normalized ordinal axes `joint_1` through `joint_<axis_count>`, and
  uses frame `joint_space`;
- gripper scalar `value` remains radians, becomes the one-element `values`
  list, uses axis `gripper_joint` and frame `gripper_joint`; no millimeter
  conversion is inferred;
- waist scalar `value` remains millimeters, becomes the one-element `values`
  list, uses axis `lift` and frame `lift`;
- force preserves the six-element flange wrench and its mixed units, uses axes
  `force.x`, `force.y`, `force.z`, `torque.x`, `torque.y`, and `torque.z`;
- when a nonnegative integer sequence exists, the facade keeps it, sets
  `timestamp=null`, and reports `provenance="device_sequence"`; otherwise it
  converts an epoch-seconds timestamp to UTC RFC3339 and reports
  `provenance="device_timestamp"`;
- malformed length, non-finite values, missing sequence/timestamp, or an
  unknown source unit fails the fixed read rather than being converted by an
  unreviewed scale.

robot_info:
  instance: string
  model: string
  version: string

robot_bindings:
  reviewed_configuration: JSON object from the fixed instance config

robot_modules:
  role: string
  model: string when present
  binding: JSON object when present
  status: string
  operations: list[string]

service_identity:
  service_pid: positive integer
  service_start_id: nonempty string
  service_epoch: nonempty string
```

`robot_modules` is a list of the objects described by those fields. Missing
optional model/binding keys are omitted rather than set to null.

`ServiceReceipt` remains the exact Task 4 schema; Task 5 refreshes its
`initialized_endpoints`, `owned_resources`, and `claim_state` after claiming
and initialization without adding fields.

- [ ] **Step 1: Write exact dispatch tests**

Test every positive row from the spec's JSON allowlist and at least these negatives:

```python
NEGATIVE_REQUESTS = [
    {"op": "initialize", "spec": ARBITRARY_SPEC},
    {"op": "call", "resource": "left_arm", "section": "settings", "name": "set_joint_speed", "args": [10]},
    {"op": "call", "resource": "left_arm", "section": "task", "name": "poll", "args": []},
    {"op": "call", "resource": "left_arm", "section": "motion", "name": "movejoint", "args": [0]},
    {"op": "call", "resource": "left_arm", "section": "reading", "name": "get_error", "args": []},
    {"op": "call", "resource": "left_arm", "section": "reading", "name": "get_joints", "args": [], "kwargs": {"extra": 1}},
    {"op": "call", "resource": "unknown", "section": "reading", "name": "get_joints", "args": []},
    {"op": "call", "resource": "left_arm", "section": "reading", "name": "get_joints", "args": [], "token_hex": "wrong"},
]
```

Add this responsive-service test:

```python
def test_responsive_compatible_service_conflicts_before_spawn(self):
    receipt = make_service_receipt(service_pid=4321, service_start_id="4321")
    with patch.object(client, "exchange", return_value=PingReply(receipt)), \
         patch.object(client, "spawn_service") as spawn:
        with self.assertRaises(client.ExistingServiceConflict):
            initialize_under_ownership(INSTANCE, **APPROVED_ARGUMENTS)
    spawn.assert_not_called()
```

`make_service_receipt` is a test-module helper with keyword-only defaults. It
constructs the full Task 4 `ServiceReceipt`, using `runtime_protocol=15`,
`protocol_id="lynrotcontrol.service-ownership"`, `protocol_version=1`, the
Task 1 allowlist digest, empty endpoint/resource tuples before override,
`claim_state="unclaimed"`, and fixed canonical paths. Every test may override
only the fields it cares about; no partial receipt is allowed.

Add source-shaped normalization tests. The input fixtures must resemble the
current adapters, not the already-normalized facade:

```python
def test_facade_normalizes_source_shaped_reads(self):
    runtime = owned_runtime_with_raw_reads({
        "left_arm": {"values": [0.0, 90.0], "unit": "deg", "sequence": 7},
        "left_gripper": {"value": 0.25, "unit": "rad", "sequence": 8,
                         "timestamp": 1790000000.0},
        "waist_lift": {"value": 12.0, "unit": "mm", "sequence": 9},
        "left_force": {"values": [1.0, 2.0, 3.0, 0.1, 0.2, 0.3],
                       "units": ["N"] * 3 + ["N.m"] * 3, "sequence": 10},
    })
    arm = runtime.read_state("left_arm")
    self.assertEqual(arm["values"], [0.0, math.pi / 2])
    self.assertEqual(arm["axes"], ["joint_1", "joint_2"])
    self.assertEqual(arm["provenance"], "device_sequence")
    self.assertEqual(runtime.read_state("left_gripper")["units"], "rad")
    self.assertEqual(runtime.read_state("waist_lift")["units"], "mm")
    self.assertEqual(runtime.read_state("left_force")["units"],
                     ["N", "N", "N", "N.m", "N.m", "N.m"])
```

Also test epoch-only timestamp conversion, malformed length, non-finite data,
missing sequence/timestamp, and unknown source units; each must fail closed.
`owned_runtime_with_raw_reads` is another test-module helper; it wraps a fake
raw Robot whose fixed methods return exactly the source-shaped dictionaries
supplied above and performs no normalization itself.

Also test:

- `claim.inspect` is tokenless and read-only;
- token-bearing release succeeds only after final identity check;
- release failure changes the record to `claim_release_failed`;
- retained service rejects initialization/settings/task/motion/fixed reads;
- service epoch/PID/start mismatch invalidates the runtime;
- arbitrary dependency exceptions map to typed ownership failures.
- `OwnedRuntime` public attributes are limited to the fixed methods in this task and never include `arms`, `grippers`, `waist`, `forces`, or `robot`;
- each positive fixed method emits its exact allowlisted RPC row and current `token_hex`, with no raw-object traversal in RPent.
- serialized receipts, evidence events, exception text, and service logs never contain the raw token, token hex, or token digest.

- [ ] **Step 2: Run and verify failure**

```bash
/home/huhu/work/RPent_lynsense/RPent/.venv/bin/python -m unittest \
  development.tests.test_service_ownership_dispatch -v
```

- [ ] **Step 3: Implement registry claim enforcement**

Use a constant-time token digest comparison (`hmac.compare_digest`) and a frozen dispatch table generated from the canonical JSON object. Enforce before import, endpoint construction, ROS context creation, defaulting, or `getattr`.

The parent generates `secrets.token_bytes(32)`. The service stores only its
SHA-256 digest; JSON-RPC carries the token as a one-line lowercase hex string
in the request field named `token_hex`. RPC serialization, receipts, evidence,
logs, argv, and durable records never contain the raw token or its digest.

Logical resources map to service arm keys only through the receipt from internal initialization:

```python
RESOURCE_ARM = {
    "left_arm": internal_left_endpoint_key,
    "right_arm": internal_right_endpoint_key,
}
```

Token-bearing internal initialization accepts only the reviewed left/right specs in that order. It accepts no caller model, IP, side, force binding, or endpoint key.

- [ ] **Step 4: Implement public owned initialization**

`interfaces/ownership.py` must:

1. validate environment `ROS_DOMAIN_ID == ros_domain_id`;
2. validate effect tuple, site/robot IDs, endpoint set, and recomputed scope digest;
3. acquire endpoint authority;
4. reject any responsive service for this first connection;
5. spawn owned bootstrap service;
6. send one random 256-bit token;
7. wait for public ping and receipt;
8. run one internal top-level initialization;
9. return `OwnedRuntime`;
10. retain claim on every failure and expose no retry.

`OwnedRuntime` keeps the raw Robot private and exposes only:

```python
arm_identity(side)
read_state(group)
robot_info()
robot_bindings()
robot_modules()
service_identity()
owned_ros_resources()
release()
```

Every fixed method internally attaches the current `token_hex`, validates the
exact resource/method/argument row before dispatch, unwraps the dependency
`Result`, and returns only plain JSON-compatible data. `__getattr__` rejects
every other public attribute rather than forwarding to the raw Robot.

- [ ] **Step 5: Run all external ownership tests**

```bash
/home/huhu/work/RPent_lynsense/RPent/.venv/bin/python -m unittest \
  development.tests.test_service_ownership_protocol \
  development.tests.test_service_ownership_endpoints \
  development.tests.test_service_ownership_authority \
  development.tests.test_service_ownership_bootstrap \
  development.tests.test_service_ownership_dispatch -v
```

Expected: all pass offline.

---

### Task 6: Read-Only ROS Transport Split

**Files:**
- Modify: `/home/huhu/work/RPent_lynsense/lynrotcontrol/adapters/gripper/ctag2f120/implementation.py`
- Modify: `/home/huhu/work/RPent_lynsense/lynrotcontrol/adapters/gripper/ctpm2f50f/implementation.py`
- Modify: `/home/huhu/work/RPent_lynsense/lynrotcontrol/adapters/waist/waist01/implementation.py`
- Modify: `/home/huhu/work/RPent_lynsense/lynrotcontrol/adapters/waist/lyn2_lift/implementation.py`
- Modify: `/home/huhu/work/RPent_lynsense/lynrotcontrol/adapters/waist/lyn2_pitch/implementation.py`
- Test: `/home/huhu/work/RPent_lynsense/lynrotcontrol/development/tests/test_readonly_ros_split.py`

**Interfaces:**
- Consumes: existing adapter implementations and `common.ros.RosNode`.
- Produces per adapter:
  - `_connect_readonly(side) -> None`
  - `_connect_command(side) -> None`
  - `readonly_resources() -> tuple[str, ...]`

- [ ] **Step 1: Write failing read-only construction tests**

For each gripper and waist package, fake the ROS node and assert:

```python
def test_read_creates_only_read_resources(self):
    impl.read()
    created = self.created_resources
    self.assertTrue(all(resource.endswith(("/position", "/joint_states", "/wrench"))
                        or resource.endswith("/get_parameters") for resource in created))
    self.assertFalse(any("set_position" in r or "set_enable" in r or
                         "set_emergency_stop" in r or "set_gripper_pos_srv" in r
                         for r in created))
```

Also assert:

- exact waist `GetParameters` names from the existing adapter;
- no `publish()` call during read;
- command paths still create their command resources only on `move()`, `validate()` with command requirement, `cancel()`, or `recover()`;
- `readonly_resources()` reports only created node/topic/service names.

- [ ] **Step 2: Run and verify failure**

```bash
/home/huhu/work/RPent_lynsense/RPent/.venv/bin/python -m unittest \
  development.tests.test_readonly_ros_split development.tests.test_lyn2_peripherals -v
```

- [ ] **Step 3: Split each adapter**

Pattern:

```python
def _connect_readonly(self):
    with self._lock:
        if self._read_ros is not None:
            return
        # Create node, subscription, and read-only parameter client only.

def _connect_command(self):
    self._connect_readonly()
    with self._lock:
        if self._command_ros is not None:
            return
        # Create publisher/control client only when a command path needs it.
```

`read()` calls `_connect_readonly()` and never `_connect_command()`. Keep existing command semantics and tests unchanged.

- [ ] **Step 4: Re-run adapter tests**

```bash
/home/huhu/work/RPent_lynsense/RPent/.venv/bin/python -m unittest \
  development.tests.test_readonly_ros_split development.tests.test_lyn2_peripherals -v
```

Expected: pass with fake ROS only.

---

### Task 7: RPent Pre-Import Gate, Manifest, And Worker Binding

**Files:**
- Create: `robots/lynsense_real_box/ownership_protocol.py`
- Modify: `robots/lynsense_real_box/commissioning_contract.py`
- Modify: `robots/lynsense_real_box/commissioning_binding.py`
- Modify: `robots/lynsense_real_box/commissioning_worker.py`
- Modify: `robots/lynsense_real_box/commissioning_runner.py`
- Modify: `robots/lynsense_real_box/commissioning_cli.py`
- Test: `tests/unit_tests/robots/lynsense_real_box/test_commissioning_contract.py`
- Test: `tests/unit_tests/robots/lynsense_real_box/test_commissioning_binding.py`
- Test: `tests/unit_tests/robots/lynsense_real_box/test_commissioning_worker.py`
- Test: `tests/unit_tests/robots/lynsense_real_box/test_commissioning_runner.py`
- Test: `tests/unit_tests/robots/lynsense_real_box/test_commissioning_cli.py`

**Interfaces:**
- Consumes: exact dependency artifact and API from Tasks 1-6.
- Produces:
  - `load_ownership_protocol(path: Path) -> OwnershipProtocolContract`
  - extended `CommissioningManifest` fields:
    - `ownership_protocol_artifact: Path`
    - `ownership_protocol_sha256: str`
    - `ownership_protocol_id: str`
    - `ownership_protocol_version: int`
    - `ownership_read_allowlist_sha256: str`
    - `endpoint_authority_root: Path`
    - `ownership_runtime_protocol: int`
  - `build_effectful_gate(request) -> ServiceOwnershipGate`
  - gate method extended to:

```python
initialize_under_guard(
    *,
    instance: str,
    site_id: str,
    robot_id: str,
    endpoints: tuple[EndpointIdentity, ...],
    endpoint_scope_sha256: str,
    approved_effects: tuple[str, ...],
    ros_domain_id: int,
) -> InitializedRuntime
```
  - `InitializedRuntime.release() -> None`
  - `InitializationWorkerRequest.operator_verification: dict[str, str]`

- [ ] **Step 1: Extend contract tests**

Add strict-schema tests that reject:

- missing or extra ownership manifest fields;
- artifact digest mismatch;
- authority-root mismatch;
- protocol ID/version mismatch;
- read-allowlist digest mismatch;
- runtime protocol other than `15`;
- artifact path not present in `source_artifacts`;
- duplicate `ownership_protocol_sha256` disagreeing with `source_artifact_sha256`.

Keep `DispositionStatus` exactly `settled`, `retained_for_operator`, and
`unknown`; represent release uncertainty through `status="unknown"` and
`reason="claim_release_unknown"`.

Also change expected arm identity `axes` from a list of labels to a positive
integer matching the dependency model registry. Add contract tests rejecting
zero, negative, non-integer, and boolean values.
Add contract tests proving the fixed-state unit policy is exactly radians for
arms and grippers, millimeters for waist lift, and the six-element
`N,N,N,N.m,N.m,N.m` force vector.

- [ ] **Step 2: Extend pre-import binding tests**

```python
def test_bad_ownership_artifact_blocks_before_dependency_import(self, monkeypatch, tmp_path):
    monkeypatch.delitem(sys.modules, "lynrotcontrol", raising=False)
    with pytest.raises(ServiceOwnershipUnavailable):
        binding.initialize(manifest.instance)
    assert gate.calls == []
    assert "lynrotcontrol" not in sys.modules
```

Also test:

- exact artifact success calls only `initialize_under_ownership`;
- old dependency exception maps to `ServiceOwnershipUnavailable`, not generic exit `5`;
- scope digest and endpoint descriptor mismatch fail before import;
- receipt PID/start/epoch/protocol/scope/resource validation;
- RPent identity/state methods consume only the narrow `OwnedRuntime` facade;
- operator verification is copied from the authorization-bound manifest into the worker request;
- release occurs only after final identity and resource inventory;
- release failure keeps disposition status `unknown`, sets reason `claim_release_unknown`, and retains both locks;
- worker crash retains claim;
- evidence serialization filters the raw token, token hex, and token digest;
- planner/Dashboard/Toolkit public surfaces are unchanged.

- [ ] **Step 3: Implement `ownership_protocol.py`**

Use only `json`, `hashlib`, `pathlib`, and dataclasses. Do not import `lynrotcontrol`.

```python
@dataclass(frozen=True)
class OwnershipProtocolContract:
    artifact_path: Path
    artifact_sha256: str
    protocol_id: str
    protocol_version: int
    read_allowlist_sha256: str
    endpoint_authority_root: Path
    runtime_protocol: int
```

Reject non-file/symlink paths, non-absolute authority roots, extra JSON fields, and any value mismatch.

- [ ] **Step 4: Extend manifest and worker request**

Add the seven ownership fields to the exact manifest/request key sets and serialization. Pass them into `build_effectful_gate`. Extend `InitializationWorkerRequest` with:

```python
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
```

Update fake fixtures and all exact-key tests. `_ALLOWED_EFFECTS` in
`commissioning_cli.py` becomes the three-value tuple from the spec. The runner
sets `operator_verification` to a copy of
`manifest.expected_operator_verification` only after authorization validation
has bound `authorization.operator` to the manifest, attempt, host, and policy.
Update every real/fake expected arm identity from label lists such as
`["a", "b"]` to the reviewed integer count (for example, `6` for `uf850`);
state-feedback `axes` remain string labels and are not changed.
Update `_UNITS` in `commissioning_binding.py` to `"rad"` for both grippers
and the exact six-element force tuple above. Update `_normalize_state` so all
non-force groups compare a string unit and force compares the exact tuple/list;
do not accept a scalar `N` vector for wrench data.

Update `RealLynrotControlBinding.initialize` to pass the fixed values through:

```python
raw = guard(
    instance=instance,
    site_id=self._manifest.site_id,
    robot_id=self._manifest.robot_id,
    endpoints=tuple(self._manifest.endpoints),
    endpoint_scope_sha256=endpoint_scope_digest(self._manifest),
    approved_effects=tuple(self._manifest.approved_effects),
    ros_domain_id=self._manifest.ros_domain_id,
)
```

Replace raw-object traversal with `_OwnedRuntimeAdapter`:

```python
def identity(self):
    arms = {
        "left": self._raw.arm_identity("left"),
        "right": self._raw.arm_identity("right"),
    }
    return IdentityEvidence(configured=self._configured_identity(),
                            device_reported={"arms": arms},
                            operator_verified=dict(self._operator_verification))

def _configured_identity(self):
    return {
        "site_id": self._manifest.site_id,
        "robot_id": self._manifest.robot_id,
        "instance": self._manifest.instance,
        "peripheral_mapping": dict(self._manifest.peripheral_mapping),
    }

def read_group(self, group):
    if group not in _GROUPS:
        raise ResultNormalizationError(f"unknown group: {group}")
    return _normalize_state(group, self._raw.read_state(group))

def operator_verification(self):
    return dict(self._operator_verification)

def service_identity(self):
    data = self._raw.service_identity()
    return ServiceEvidence(
        data.get("service_pid") if isinstance(data.get("service_pid"), int) else None,
        data.get("service_start_id") if isinstance(data.get("service_start_id"), str) else None,
        data.get("service_epoch") if isinstance(data.get("service_epoch"), str) else None,
    )

def owned_ros_resources(self):
    return self._raw.owned_ros_resources()

def release(self):
    self._raw.release()
```

The adapter constructor is `_OwnedRuntimeAdapter(raw, manifest,
operator_verification)`. It must not access `raw.arms`, `raw.grippers`,
`raw.waist`, `raw.forces`, or `raw.robot`. Fake runtimes expose the same fixed
facade and record rejected attribute access.

- [ ] **Step 5: Implement the fixed real gate**

`build_effectful_gate` returns a small fixed adapter that:

1. verifies all source/config and ownership artifact digests;
2. imports `lynrotcontrol.interfaces.ownership`;
3. converts `EndpointIdentity` to dependency `CanonicalEndpoint`;
4. calls `initialize_under_ownership`;
5. returns `_OwnedRuntimeAdapter(raw, manifest, request.operator_verification)`;
6. retains the token only in worker process memory;
7. filters tokens from every exception and evidence path.

- [ ] **Step 6: Update reconciliation behavior**

The existing `reconcile` command:

- may consume only a matching RPent attempt lock;
- calls dependency `inspect_claim` through the same pre-import artifact gate;
- on `no_claim` or `claim_released`, records dependency disposition and may settle RPent reconciliation;
- on `claim_active`, `claim_orphaned`, `claim_release_failed`, or `authority_record_invalid` (including pending authority), records `dependency_reconciliation_unavailable` and does not delete or mutate dependency records;
- never calls stop, recovery, shutdown, service spawn, or initialization.

- [ ] **Step 7: Release the dependency claim only on settled success**

Extend the fake and real runtime protocol with `release()`. In
`commissioning_worker.run_worker`, call it only after:

1. all required fixed reads complete;
2. service identity remains unchanged;
3. final resource inventory completes;
4. the second service identity check matches.

If `release()` raises, catch it separately from observation failures, keep the
existing disposition status `unknown`, set its reason to
`claim_release_unknown`, and leave both RPent and dependency claim state
retained. Do not call `release()` for `known_failure`, `late_settled`,
`worker_crashed`, or `precontact_rejected`.

Update fake bindings to record `("release",)` and reject a second release.
Add `_InitializedRuntime.release()`, which delegates to the dependency runtime's
bound `release()` method without exposing a token to RPent code or evidence.

- [ ] **Step 8: Run focused RPent tests**

```bash
/home/huhu/work/RPent_lynsense/RPent/.venv/bin/pytest -q \
  tests/unit_tests/robots/lynsense_real_box/test_commissioning_contract.py \
  tests/unit_tests/robots/lynsense_real_box/test_commissioning_binding.py \
  tests/unit_tests/robots/lynsense_real_box/test_commissioning_worker.py \
  tests/unit_tests/robots/lynsense_real_box/test_commissioning_runner.py \
  tests/unit_tests/robots/lynsense_real_box/test_commissioning_cli.py
```

Expected: all pass with fakes only.

---

### Task 8: Documentation, Full Offline Gates, And Handoff Evidence

**Files:**
- Modify: `robots/lynsense_real_box/README.md`
- Modify: `docs/superpowers/plans/2026-09-23-robot-one-lynrotcontrol-first-connection-runbook.md`
- Test: `tests/unit_tests/robots/lynsense_real_box/test_commissioning_cli.py`
- Test: `tests/unit_tests/robots/lynsense_real_box/test_inventory_runbook.py`

**Interfaces:**
- Consumes: all previous tasks.
- Produces: updated operator-facing fail-closed, inspection, deployment, and authorization instructions.

- [ ] **Step 1: Update tests that pin runbook language**

Require the runbook to state:

- dependency artifact path and digest;
- authority root and runtime protocol;
- no live operation is authorized by passing tests;
- old dependency remains exit `5`;
- orphan/active/release-failed claims require dependency reconciliation-unavailable handling;
- no tokenless dependency cleanup exists in protocol v1;
- deployment and Robot One operation need separate approvals.

- [ ] **Step 2: Update README and runbook**

Document only the offline implementation. Do not add simulated or unverified Robot One values. Replace any claim that `build_effectful_gate` is permanently unavailable with the new pre-import protocol contract and its remaining deployment/hardware gates.

- [ ] **Step 3: Run the full external offline discovery suite**

```bash
cd /home/huhu/work/RPent_lynsense/lynrotcontrol
/home/huhu/work/RPent_lynsense/RPent/.venv/bin/python -m unittest discover \
  --start-directory development/tests --verbose
```

- [ ] **Step 4: Run focused RPent commissioning suite**

```bash
cd /home/huhu/work/RPent_lynsense/.worktrees/RPent-real-box-phase1
/home/huhu/work/RPent_lynsense/RPent/.venv/bin/pytest -q \
  tests/unit_tests/robots/lynsense_real_box/test_commissioning_contract.py \
  tests/unit_tests/robots/lynsense_real_box/test_commissioning_lock.py \
  tests/unit_tests/robots/lynsense_real_box/test_commissioning_binding.py \
  tests/unit_tests/robots/lynsense_real_box/test_commissioning_worker.py \
  tests/unit_tests/robots/lynsense_real_box/test_commissioning_evidence.py \
  tests/unit_tests/robots/lynsense_real_box/test_commissioning_runner.py \
  tests/unit_tests/robots/lynsense_real_box/test_commissioning_cli.py \
  tests/unit_tests/robots/lynsense_real_box/test_inventory_runbook.py
```

- [ ] **Step 5: Run full unit and offline safety gates**

```bash
/home/huhu/work/RPent_lynsense/RPent/.venv/bin/pytest -q tests/unit_tests
/home/huhu/work/RPent_lynsense/RPent/.venv/bin/python -m compileall -q \
  robots/lynsense_real_box tests/unit_tests/robots/lynsense_real_box
rg -n "TB[D]|TO[D]O|placeholde[r]|gue[s]sed|may[b]e|late[r]|handle edge case[s]" \
  docs/superpowers/plans/2026-09-24-lynrotcontrol-service-ownership-gate.md \
  docs/superpowers/specs/2026-09-24-lynrotcontrol-service-ownership-gate-design.md \
  robots/lynsense_real_box tests/unit_tests/robots/lynsense_real_box
git diff --check
git -C /home/huhu/work/RPent_lynsense/lynrotcontrol diff --check
```

Expected: full unit suite matches or improves the prior `1147 passed, 3 skipped` baseline; scans return no results; diff checks pass.

- [ ] **Step 6: Record handoff evidence without deployment**

Record:

- both repository HEADs and statuses;
- all test commands and results;
- artifact and allowlist digests;
- external dependency changed-file list;
- explicit statement that ROS and Robot One were not contacted.

Compute the final runbook SHA:

```bash
sha256sum docs/superpowers/plans/2026-09-23-robot-one-lynrotcontrol-first-connection-runbook.md
```

Do not deploy, install, copy, SSH, or run on Robot One.
