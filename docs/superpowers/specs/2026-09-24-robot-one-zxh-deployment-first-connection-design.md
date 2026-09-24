# Robot One ZXH Deployment And First Connection Design

**Date:** 2026-09-24
**Status:** Draft for re-review
**Operator:** huhu
**Target host:** zxh@192.168.11.11
**Deployment root:** `~/zxh/`
**LynrotControl:** `~/zxh/lynrotcontrol` (`dev_hu`, `46514683f2beb532e007f84a5b13e1794f3ee995`)
**RPent:** `~/zxh/RPent` (`main`, `0d5077a` full SHA pinned at implementation-plan time)

## Goal

Deploy both `lynrotcontrol@dev_hu` and `RPent@main` to Robot One PC at
`zxh@192.168.11.11` under `~/zxh/`, verify offline tests pass on target
hardware, then execute one authorized first-connection state acceptance run
through the reviewed RPent commissioning CLI.

## Explicit Non-Goals

- No Gitee PR merge for lynrotcontrol; deploy `dev_hu` directly.
- No robot motion, trajectory, recovery, calibration, or force zeroing.
- No chassis, gripper, waist, arm, or perception commands.
- No RPent model, Toolkit, Dashboard, or planner control.
- No pytree task-flow integration.
- No software emergency-stop implementation; the operator has a physical
  E-stop and remains responsible for physical safety.

## Execution Topology

Both repositories run on the target machine:

```text
~/zxh/RPent
  robots/lynsense_real_box/commissioning_cli.py     (operator entry)
  robots/lynsense_real_box/commissioning_runner.py   (supervisor)
  robots/lynsense_real_box/commissioning_worker.py   (subprocess worker)
    -> imports ~/zxh/lynrotcontrol
    -> initialize_under_ownership(...)
    -> OwnedRuntime fixed reads
    -> release
```

The commissioning CLI enforces host enrollment, trusted policy, exact
one-shot authorization, source/config inventory, RPent durable lock, and
structured evidence. Direct Python API invocation without the CLI is
prohibited for Phase 3.

## Phase 1: Target Machine Precheck (Read-Only)

**Authorization:** No hardware effect; SSH read-only commands only.

All commands are run via `ssh zxh@192.168.11.11`.

1. Identity and environment:
   ```bash
   date && hostname && whoami && id -u && python3 --version && df -h ~/
   ```
   Pass criteria: `id -u` returns `1000`; Python >= 3.10; >= 100 MB free.
   If UID is not 1000, fail closed: the authority root is bound to UID 1000.

2. Deployment path state:
   ```bash
   ls -ld ~/zxh 2>/dev/null || echo dir-absent
   ls -ld ~/zxh/lynrotcontrol 2>/dev/null || echo repo-absent
   ls -ld ~/zxh/RPent 2>/dev/null || echo repo-absent
   ```
   Pass criteria: each path either does not exist, or is a real directory
   (not a symlink) owned by the current user with mode 0700 or 0750.

3. LynrotControl process check (exact module match):
   ```bash
   pgrep -af 'python.*[ ]-m lynrotcontrol[.]lynarmcontrol[.]implementation[.]service' || echo no-service
   ```
   Pass criteria: output is exactly `no-service`. Any PID match is a
   hard stop.

4. Authority directory state:
   ```bash
   find /tmp/lynrotcontrol-authority-1000/endpoints -name '*.json' -type f 2>/dev/null | head -5 || echo authority-clean
   ```
   Interpretation:
   - `authority-clean`: pass
   - Directory exists with only `authority.lock` files and no
     `claim.json` or other record files: pass (locks from released tests)
   - Any `*.json` record file beneath `claims/`: hard stop, report
     `authority_record_present`; do not delete or modify. Empty `claims/`
     directories and lock-only directories are acceptable.

5. ROS driver check (read-only, optional):
   ```bash
   source /opt/ros/humble/setup.bash 2>/dev/null; \
   source /home/rpp/rpp_ws/install/setup.bash 2>/dev/null; \
   ROS_DOMAIN_ID=3 timeout 10s ros2 node list
   ```
   Pass criteria: command exits 0 and lists at least the xarm driver
   nodes. If it fails, Phase 3 is blocked but Phase 2 may proceed.

## Phase 2: Deploy To ~/zxh/

**Authorization:** File system write on target machine only; no hardware
contact, no ROS initialization.

### 2A: Deploy lynrotcontrol dev_hu

1. If `~/zxh/lynrotcontrol` does not exist:
   ```bash
   git clone -b dev_hu https://gitee.com/lynsense/lynrotcontrol.git ~/zxh/lynrotcontrol
   ```
2. If it exists, verify it is a clean git repository on `dev_hu` with no
   local changes, no symlinks, and no divergence; then `git pull`. If any
   check fails, stop and report; do not reset or force.
3. Verify commit:
   ```bash
   cd ~/zxh/lynrotcontrol && git rev-parse HEAD
   ```
   Expected: `46514683f2beb532e007f84a5b13e1794f3ee995`
4. Verify artifact:
   ```bash
   sha256sum ~/zxh/lynrotcontrol/OWNERSHIP_PROTOCOL.json
   ```
   Expected: `78c45f6796cd90fdf63f51ac407ac261c04e42ab454bb6d0897749d81904c84a`

### 2B: Deploy RPent main

1. If `~/zxh/RPent` does not exist:
   ```bash
   git clone -b main https://github.com/huhu0209/RPent.git ~/zxh/RPent
   ```
2. If it exists, same clean/dirty/divergence checks as 2A step 2.
3. Record the deployed commit SHA.
4. Verify `robots/lynsense_real_box/` and the commissioning CLI exist.

### 2C: Offline Test Gate On Target

```bash
cd ~/zxh/lynrotcontrol && python3 -m unittest discover \
  --start-directory development/tests --verbose
```
Expected: exactly 127 tests, 0 failures, 0 errors, 0 skipped.

```bash
cd ~/zxh/RPent && python3 -m pytest -q \
  tests/unit_tests/robots/lynsense_real_box/
```
Expected: all pass (no torch dependency in this subset).

If Python dependencies are missing on target, the deployment report lists
them and stops. No automatic dependency installation without operator
review.

### Rollback

Only applies to a tree created by this deployment. If the deployment
created `~/zxh/lynrotcontrol` or `~/zxh/RPent` and a later step fails
before Phase 3, the operator may explicitly approve deleting only that
newly-created tree. Never delete an existing tree that was present before
this deployment.

## Phase 3: First-Connection State Acceptance

**Authorization:** Requires a machine-checkable authorization file that
passes the existing `commissioning_cli.py` schema, including at minimum:
operator identity, manifest digest, attempt ID, host enrollment digest,
policy digest, expiry, Robot One identity, and the exact three-value
approved-effects tuple. Verbal approval alone is insufficient.

### Preconditions

- Phase 2 offline gates passed on target.
- Phase 1 ROS driver check passed.
- Physical E-stop accessible to the on-site operator.
- No active LynrotControl process or authority claim.
- Authorization file generated with a bounded expiry and reviewed by the
  operator before invocation.

### Invocation

Run on the target machine from the RPent tree:

```bash
cd ~/zxh/RPent
python3 -m robots.lynsense_real_box.commissioning_cli run \
  --manifest <manifest.json> \
  --host-enrollment <host-enrollment.json> \
  --authorization-file <authorization.json>
```

Prerequisites:
- `/etc/rpent/commissioning/operator-policy.json` must exist, be
  root-owned, mode 0600, and contain the reviewed trusted operator policy.
- The manifest, host enrollment, and authorization files must pass the
  exact schema in `commissioning_contract.py`.
- The manifest must reference `~/zxh/lynrotcontrol` and the reviewed
  instance configuration.

### Acceptance Conditions

All conditions from `commissioning_runner.py` are enforced:

1. Seven fixed state groups return valid data:
   - left_arm, right_arm: radians, count matches model axes
   - left_gripper, right_gripper: radians
   - waist_lift: millimetres
   - left_force, right_force: exact six-element wrench units
2. Each read has device provenance (non-null sequence or RFC3339
   timestamp; exactly one present).
3. Provenance advances or is distinct across required samples (minimum
   sample count from manifest).
4. Freshness: each read within the manifest's per-subsystem budget.
5. Arm identity: model, IP, side, and positive-integer axis count match
   the reviewed instance configuration.
6. Device-reported arm identity equals the reviewed expected identity
   (model, IP, side, positive-integer axes). Operator verification equals
   the manifest exactly.
7. Operator verification fields are present and copied from the manifest.
8. Expected owned ROS resources match the manifest.
9. Service identity (PID, start_id, epoch) is stable across the run and
   checked at least twice.
10. Claim transitions: unclaimed -> claim_active -> claim_released.
11. Release completes within its timeout; any release uncertainty keeps
    disposition `unknown` with reason `claim_release_unknown`.

### Failure Handling

| Outcome | Action |
|---|---|
| Endpoint authority conflict | Stop before spawn; record disposition; report |
| Bootstrap handshake failure | Stop; claim may be orphaned; record; report |
| Device initialization failure | Stop; retain claim; record; report |
| State read failure (invalid/missing) | Stop; record; dependency claim state may be active or unknown; report |
| Read deadline expiry | Stop; retain RPent lock; dependency claim state unknown pending inspection; record `worker_timeout`; report |
| Worker crash | Retain RPent lock; dependency claim state unknown; record `worker_crashed`; report |
| Supervisor interruption | Retain locks and claim; record `supervisor_interrupted`; report |
| Evidence write failure after contact | Retain claim; record `evidence_write_failed`; report |
| Late worker result after timeout | Record `late_settled`; do not treat as success; do not auto-release |
| Release rejection or uncertainty | Record `claim_release_unknown`; retain both locks; report |

None of these outcomes trigger motion, recovery, stop, shutdown, service
spawn, initialization retry, or automatic claim cleanup. Reconciliation of
retained claims requires a separate operator decision using the
`reconcile` command and reviewing the durable claim record.

### Success Criteria

- All 11 acceptance conditions pass.
- Evidence JSON is written on the target machine under the RPent run
  directory.
- No motion, recovery, perception trigger, or ROS command was issued.
- Operator confirms the evidence artifact.

## Evidence Model

Evidence is first written on the target machine under the RPent
commissioning output directory. After Phase 3, evidence is copied to the
local development machine via `scp` and its SHA-256 is recorded in the
local repository. The copy is verified by digest; any mismatch rejects
the evidence transfer.

No credentials, tokens, token digests, or authorization secrets appear
in evidence files.

## Safety Boundary

| Operation | Phase 1 | Phase 2 | Phase 3 |
|---|---|---|---|
| SSH read commands | yes | yes | yes |
| File write on target | no | yes | no |
| git clone/pull | no | yes | no |
| Offline unit tests | no | yes | no |
| ROS node/topic list | read-only | no | no |
| Service spawn | no | no | yes |
| Device initialization | no | no | yes |
| State reads | no | no | yes |
| Motion commands | no | no | no |
| Perception trigger | no | no | no |
| Recovery/stop commands | no | no | no |
| Automatic claim cleanup | no | no | no |
