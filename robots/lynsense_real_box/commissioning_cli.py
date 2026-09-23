"""Operator-only entry point for bounded LynrotControl commissioning."""

from __future__ import annotations

import argparse
import getpass
import hashlib
import json
import os
import sys
import uuid
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

from .commissioning_contract import (
    AuthorizationRecord,
    HostEnrollmentAuthority,
    HostEnrollmentRecord,
    HostIdentity,
    endpoint_scope_digest,
    authorization_digest,
    load_commissioning_manifest,
    load_trusted_operator_policy,
    validate_commissioning_manifest,
)


DEFAULT_POLICY_PATH = Path("/etc/rpent/commissioning/operator-policy.json")
_EXIT_CODES = {
    "accepted": 0,
    "rejected_before_initialization": 2,
    "failed": 3,
    "initialization_outcome_unknown": 4,
    "service_ownership_enforcement_unavailable": 5,
}
_ALLOWED_EFFECTS = (
    "initialize_configuration", "fixed_read_ros_resource_creation",
    "fixed_read_ros_parameter_query",
)


class _Parser(argparse.ArgumentParser):
    def error(self, message: str) -> None:
        raise ValueError(message)


def exit_code_for_status(status: str) -> int:
    try:
        return _EXIT_CODES[status]
    except KeyError as exc:
        raise ValueError(f"unknown commissioning outcome: {status}") from exc


def _actual_host() -> HostIdentity:
    machine_id = Path("/etc/machine-id").read_text(encoding="ascii").strip()
    return HostIdentity(machine_id, os.uname().nodename, getpass.getuser(), sys.executable)


def _load_policy():
    policy_path = DEFAULT_POLICY_PATH
    stat = policy_path.stat()
    if stat.st_uid != 0 or stat.st_mode & 0o022:
        raise ValueError("trusted operator policy must be root-owned and not group/world writable")
    return load_trusted_operator_policy(policy_path)


def _read_object(path: Path, required: set[str], label: str) -> dict:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read {label}: {exc}") from exc
    if not isinstance(data, dict) or set(data) != required:
        raise ValueError(f"{label} fields do not match the fixed schema")
    return data


def _load_enrollment(path: Path) -> HostEnrollmentAuthority:
    fields = {"authority_id", "authority_sha256", "records"}
    data = _read_object(path, fields, "host enrollment")
    if not isinstance(data["records"], list):
        raise ValueError("host enrollment records must be a list")
    rows = []
    row_fields = {"site_id", "machine_id", "hostname", "account", "enabled"}
    for row in data["records"]:
        if not isinstance(row, dict) or set(row) != row_fields:
            raise ValueError("host enrollment record fields are invalid")
        rows.append(HostEnrollmentRecord(**row))
    return HostEnrollmentAuthority(data["authority_id"], data["authority_sha256"], tuple(rows))


def _load_authorization(path: Path) -> AuthorizationRecord:
    fields = {
        "authorization_id", "purpose", "attempt_id", "authorization_sha256", "manifest_sha256",
        "site_id", "robot_id", "machine_id",
        "host_enrollment_sha256", "policy_sha256", "operator", "ros_domain_id", "expires_at",
    }
    data = _read_object(path, fields, "authorization")
    supplied_digest = data["authorization_sha256"]
    if supplied_digest != authorization_digest(data):
        raise ValueError("authorization digest mismatch")
    try:
        data["expires_at"] = datetime.fromisoformat(data["expires_at"].replace("Z", "+00:00"))
        return AuthorizationRecord(**data)
    except (AttributeError, TypeError, ValueError) as exc:
        raise ValueError("authorization record is invalid") from exc


def _validate(manifest_path: Path, enrollment_path: Path, authorization_path: Path | None):
    manifest = load_commissioning_manifest(manifest_path)
    enrollment = _load_enrollment(enrollment_path)
    policy = _load_policy()
    host = _actual_host()
    authorization = _load_authorization(authorization_path) if authorization_path else None
    validate_commissioning_manifest(
        manifest, actual_host=host, enrollment=enrollment,
        authorization=authorization, policy=policy,
        trusted_lock_root=policy.trusted_lock_root,
    )
    if tuple(manifest.approved_effects) != _ALLOWED_EFFECTS:
        raise ValueError("approved effects do not match the fixed ownership protocol")
    declared_digests = {**manifest.source_artifact_sha256, **manifest.config_sha256}
    for artifact in manifest.source_artifacts:
        if artifact.is_symlink() or not artifact.is_file():
            raise ValueError(f"source inventory artifact is missing or unsafe: {artifact}")
        observed = hashlib.sha256(artifact.read_bytes()).hexdigest()
        expected = declared_digests.get(str(artifact))
        if expected is None or observed != expected:
            raise ValueError(f"source inventory digest mismatch: {artifact}")
    if authorization is not None and authorization.operator not in policy.allowed_operators:
        raise ValueError("operator is not allowed by trusted policy")
    return manifest, host, enrollment, authorization, policy


def _parser() -> argparse.ArgumentParser:
    parser = _Parser(prog="python -m robots.lynsense_real_box.commissioning_cli")
    sub = parser.add_subparsers(dest="command", required=True, parser_class=_Parser)
    validate = sub.add_parser("validate", help="pure local validation")
    validate.add_argument("--manifest", required=True, type=Path)
    validate.add_argument("--host-enrollment", required=True, type=Path)
    run = sub.add_parser("run", help="one explicitly authorized commissioning attempt")
    run.add_argument("--manifest", required=True, type=Path)
    run.add_argument("--host-enrollment", required=True, type=Path)
    run.add_argument("--authorization-file", required=True, type=Path)
    reconcile = sub.add_parser("reconcile", help="reconcile one verified retained result")
    reconcile.add_argument("--manifest", required=True, type=Path)
    reconcile.add_argument("--lock", required=True, type=Path)
    reconcile.add_argument("--authorization-file", required=True, type=Path)
    return parser


def _run(args: argparse.Namespace) -> int:
    manifest, host, enrollment, authorization, policy = _validate(
        args.manifest, args.host_enrollment, args.authorization_file
    )
    from .commissioning_evidence import CommissioningEvidenceWriter
    from .commissioning_lock import CommissioningLock
    from .commissioning_runner import CommissioningRunner
    from .commissioning_worker import SubprocessInitializationSupervisor

    attempt = authorization.attempt_id
    evidence = CommissioningEvidenceWriter(
        uuid.uuid4().hex, manifest,
        manifest.evidence_dir / f"{attempt}.commissioning.jsonl",
    )
    try:
        outcome = CommissioningRunner(
            CommissioningLock, evidence, SubprocessInitializationSupervisor()
        ).run_once(manifest, host, enrollment, authorization, policy)
    finally:
        evidence.close()
    print(json.dumps(asdict(outcome), default=str, sort_keys=True))
    return exit_code_for_status(outcome.status)


def _reconcile(args: argparse.Namespace) -> int:
    manifest = load_commissioning_manifest(args.manifest)
    policy = _load_policy()
    authorization = _load_authorization(args.authorization_file)
    actual_host = _actual_host()
    if (actual_host.machine_id, actual_host.hostname, actual_host.account) != (
        manifest.execution_host_machine_id, manifest.execution_hostname, manifest.execution_account
    ):
        raise ValueError("reconciliation host identity does not match manifest")
    if manifest.trusted_lock_root != policy.trusted_lock_root:
        raise ValueError("manifest lock root does not match trusted operator policy")
    if args.lock != manifest.lock_path or authorization.operator not in policy.allowed_operators:
        raise ValueError("reconciliation lock path or operator is not authorized")
    if policy.approved_authorizations.get(authorization.authorization_id) != authorization.authorization_sha256:
        raise ValueError("reconciliation authorization digest is not approved by trusted policy")
    if (
        authorization.manifest_sha256 != manifest.manifest_sha256
        or authorization.site_id != manifest.site_id
        or authorization.robot_id != manifest.robot_id
        or authorization.machine_id != manifest.execution_host_machine_id
        or authorization.host_enrollment_sha256 != manifest.host_enrollment_sha256
        or authorization.policy_sha256 != policy.sha256
        or authorization.ros_domain_id != 3
        or authorization.expires_at != manifest.authorization_expires_at
        or authorization.purpose != "commissioning_reconcile"
    ):
        raise ValueError("reconciliation authorization does not match manifest and policy")

    from .commissioning_evidence import CommissioningEvidenceWriter
    from .commissioning_lock import (
        CommissioningLock, read_lock_record, reconcile_lock, verify_consumed_attempt,
    )
    from .commissioning_worker import worker_result_from_dict
    from .commissioning_binding import inspect_dependency_claim

    scope = endpoint_scope_digest(manifest)
    dependency_claim_state = inspect_dependency_claim(manifest)
    record = read_lock_record(args.lock)
    if (
        record.state != "unknown" or record.site_id != manifest.site_id
        or record.robot_id != manifest.robot_id or record.endpoint_scope_sha256 != scope
        or record.manifest_sha256 != manifest.manifest_sha256
        or record.authorization_id != manifest.authorization_id
        or record.attempt_id != authorization.attempt_id
        or record.host_machine_id != manifest.execution_host_machine_id
        or record.created_at >= authorization.expires_at
    ):
        raise ValueError("retained lock identity does not match manifest and authorization")
    if dependency_claim_state not in {"no_claim", "claim_released"}:
        dependency_evidence = CommissioningEvidenceWriter(
            f"reconcile-blocked-{record.attempt_id}", manifest,
            manifest.evidence_dir / f"{record.attempt_id}.reconcile.jsonl",
        )
        try:
            dependency_evidence.record(
                "dependency_reconciliation_unavailable",
                attempt_id=record.attempt_id, dependency_claim_state=dependency_claim_state,
            )
            dependency_evidence.write_report({
                "attempt_id": record.attempt_id,
                "status": "dependency_reconciliation_unavailable",
                "dependency_claim_state": dependency_claim_state,
            })
        finally:
            dependency_evidence.close()
        print(
            "dependency reconciliation unavailable; "
            f"claim state is {dependency_claim_state}"
        )
        return 4
    verify_consumed_attempt(manifest, record)
    result_path = manifest.evidence_dir / f"{record.attempt_id}.result.json"
    if result_path.is_symlink() or result_path.resolve(strict=True).parent != manifest.evidence_dir:
        raise ValueError("retained worker result path is unsafe")
    result = worker_result_from_dict(json.loads(result_path.read_text(encoding="utf-8")))
    disposition = result.disposition
    service = result.service
    endpoints = {endpoint.endpoint_id for endpoint in manifest.endpoints}
    if (
        result.status != "late_settled" or result.attempt_id != record.attempt_id
        or result.manifest_sha256 != manifest.manifest_sha256
        or result.endpoint_scope_sha256 != scope
        or disposition.status not in {"settled", "unknown"}
        or disposition.service_pid not in {None, service.pid}
        or disposition.service_start_id not in {None, service.start_id}
        or disposition.service_epoch not in {None, service.epoch}
        or set(disposition.initialized_endpoints) != endpoints
        or tuple(disposition.owned_ros_resources) != manifest.expected_owned_ros_resources
        or not all((service.pid, service.start_id, service.epoch))
    ):
        raise ValueError("late result lacks matching settled service identity and disposition")
    disposition = type(disposition)(
        "settled", service.pid, service.start_id, service.epoch,
        tuple(sorted(endpoints)), disposition.owned_ros_resources,
        authorization.operator, "operator explicitly reconciled retained late result",
    )
    evidence = CommissioningEvidenceWriter(
        f"reconcile-{record.attempt_id}", manifest,
        manifest.evidence_dir / f"{record.attempt_id}.reconcile.jsonl",
    )
    try:
        evidence.record(
            "reconcile_intent", attempt_id=record.attempt_id,
            run_authorization_id=record.authorization_id,
            reconcile_authorization_id=authorization.authorization_id,
            lock_path=args.lock, service=service,
            initialized_endpoints=tuple(sorted(endpoints)),
            owned_ros_resources=disposition.owned_ros_resources,
        )
        evidence.write_report({
            "attempt_id": record.attempt_id,
            "status": "reconcile_pending_lock_release",
            "run_authorization_id": record.authorization_id,
            "reconcile_authorization_id": authorization.authorization_id,
            "operator": authorization.operator,
            "service": service,
            "resource_disposition": disposition,
        })
        if record.service_pid is None:
            CommissioningLock.record_service_identity(
                args.lock, expected_attempt_id=record.attempt_id,
                expected_authorization_id=record.authorization_id,
                expected_manifest_sha256=manifest.manifest_sha256,
                expected_endpoint_scope_sha256=scope, service=service,
            )
        elif (record.service_pid, record.service_start_id, record.service_epoch) != (
            service.pid, service.start_id, service.epoch
        ):
            raise ValueError("late service identity differs from retained lock")
        reconcile_lock(
            args.lock, operator=authorization.operator,
            authorization_id=record.authorization_id,
            expected_attempt_id=record.attempt_id,
            expected_manifest_sha256=manifest.manifest_sha256,
            expected_endpoint_scope_sha256=scope, disposition=disposition,
        )
        evidence.record(
            "reconcile_released", attempt_id=record.attempt_id,
            run_authorization_id=record.authorization_id,
            reconcile_authorization_id=authorization.authorization_id,
        )
        evidence.write_report({
            "attempt_id": record.attempt_id,
            "status": "reconciled",
            "run_authorization_id": record.authorization_id,
            "reconcile_authorization_id": authorization.authorization_id,
            "operator": authorization.operator,
            "service": service,
            "resource_disposition": disposition,
        })
    finally:
        evidence.close()
    print(f"reconciled retained attempt {record.attempt_id}; no robot contact performed")
    return 0


def main(argv: list[str] | None = None) -> int:
    try:
        args = _parser().parse_args(argv)
        if args.command == "validate":
            manifest, *_ = _validate(args.manifest, args.host_enrollment, None)
            print(f"validated manifest {manifest.manifest_sha256}; no robot contact performed")
            return 0
        if args.command == "run":
            return _run(args)
        return _reconcile(args)
    except SystemExit as exc:
        return int(exc.code or 0)
    except (ValueError, OSError, KeyError, TypeError) as exc:
        print(f"commissioning rejected: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
