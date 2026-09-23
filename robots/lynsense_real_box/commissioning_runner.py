"""One-attempt commissioning state machine and durable lock disposition."""

from __future__ import annotations

from datetime import timezone
from pathlib import Path
from typing import Any

from .commissioning_contract import (
    AuthorizationRecord,
    CommissioningManifest,
    CommissioningOutcome,
    HostEnrollmentAuthority,
    HostIdentity,
    InitializationWorkerRequest,
    LockRecord,
    ResourceDisposition,
    ServiceEvidence,
    StateEvidence,
    TrustedOperatorPolicy,
    endpoint_scope_digest,
    validate_commissioning_manifest,
)
from .commissioning_lock import CommissioningLock, consume_attempt
from .commissioning_worker import REQUIRED_GROUPS, WorkerDeadlineExceeded


class _SystemClock:
    @staticmethod
    def now():
        from datetime import datetime
        return datetime.now(timezone.utc)

    @staticmethod
    def monotonic() -> float:
        import time
        return time.monotonic()


class CommissioningRunner:
    def __init__(self, lock_manager: Any, evidence: Any, supervisor: Any, clock: Any = None):
        self.lock_manager = lock_manager
        self.evidence = evidence
        self.supervisor = supervisor
        self.clock = clock or _SystemClock()

    def validate_only(
        self,
        manifest: CommissioningManifest,
        host: HostIdentity,
        enrollment: HostEnrollmentAuthority,
        policy: TrustedOperatorPolicy,
    ) -> CommissioningOutcome:
        try:
            self._validate_inputs(manifest, host, enrollment, None, policy)
        except Exception as exc:
            return self._outcome(manifest, "rejected_before_initialization", str(exc), "not_acquired")
        return self._outcome(manifest, "accepted", None, "not_acquired")

    def run_once(
        self,
        manifest: CommissioningManifest,
        host: HostIdentity,
        enrollment: HostEnrollmentAuthority,
        authorization: AuthorizationRecord,
        policy: TrustedOperatorPolicy,
    ) -> CommissioningOutcome:
        try:
            self._validate_inputs(manifest, host, enrollment, authorization, policy)
        except Exception as exc:
            return self._finish(manifest, "rejected_before_initialization", str(exc), "not_acquired")

        attempt = authorization.attempt_id
        scope = endpoint_scope_digest(manifest)
        result_path = manifest.evidence_dir / f"{attempt}.result.json"
        if result_path.exists() or result_path.is_symlink():
            return self._finish(
                manifest, "rejected_before_initialization",
                "attempt result already exists; exact attempt IDs are not reusable",
                "not_acquired", attempt,
            )
        try:
            self.evidence.record("intent", attempt_id=attempt, endpoint_scope_sha256=scope)
        except Exception as exc:
            return self._finish(manifest, "rejected_before_initialization", f"evidence intent failed: {exc}", "not_acquired")

        record = LockRecord(
            site_id=manifest.site_id, robot_id=manifest.robot_id,
            endpoint_scope_sha256=scope, manifest_sha256=manifest.manifest_sha256,
            attempt_id=attempt, authorization_id=authorization.authorization_id,
            host_machine_id=host.machine_id, created_at=self.clock.now(), state="held",
            service_pid=None, service_start_id=None, service_epoch=None,
        )
        try:
            consume_attempt(manifest, record)
            handle = self.lock_manager.acquire(manifest, record)
        except Exception as exc:
            return self._finish(manifest, "rejected_before_initialization", str(exc), "not_acquired", attempt)

        initialization_deadline = self.clock.monotonic() + manifest.budgets["initialize"]
        deadline = initialization_deadline + manifest.budgets["read"]
        request = InitializationWorkerRequest(
            attempt, manifest.manifest_sha256, scope, manifest.instance, manifest.ros_domain_id,
            tuple(manifest.endpoints),
            tuple(manifest.approved_effects), REQUIRED_GROUPS, manifest.required_samples,
            initialization_deadline, deadline, result_path, tuple(manifest.source_artifacts),
            dict(manifest.source_artifact_sha256),
            manifest.site_id, manifest.robot_id,
            manifest.ownership_protocol_artifact,
            manifest.ownership_protocol_sha256,
            manifest.ownership_protocol_id,
            manifest.ownership_protocol_version,
            manifest.ownership_read_allowlist_sha256,
            manifest.endpoint_authority_root,
            manifest.ownership_runtime_protocol,
            dict(manifest.expected_operator_verification),
            dict(manifest.config_sha256),
        )
        try:
            result = self.supervisor.run_worker(request)
        except WorkerDeadlineExceeded as exc:
            reason = "initialization_outcome_unknown: worker deadline expired"
            self._retain(handle, reason)
            try:
                self.evidence.record("worker_timeout", attempt_id=attempt, worker_pid=exc.pid,
                                     worker_start_id=exc.start_id, result_path=result_path)
            except Exception:
                pass
            return self._finish(manifest, "initialization_outcome_unknown", reason, "unknown", attempt)
        except KeyboardInterrupt:
            reason = "initialization_outcome_unknown: interrupted while worker may still be active"
            self._retain(handle, reason)
            return self._finish(manifest, "initialization_outcome_unknown", reason, "unknown", attempt)
        except Exception as exc:
            reason = f"initialization_outcome_unknown: supervisor failed: {type(exc).__name__}: {exc}"
            self._retain(handle, reason)
            return self._finish(manifest, "initialization_outcome_unknown", reason, "unknown", attempt)

        try:
            self.evidence.record("worker_result", result=result)
        except Exception as exc:
            self._record_result_identity(handle, result)
            self._retain(handle, f"evidence failure after worker contact: {exc}")
            return self._finish(manifest, "initialization_outcome_unknown", "evidence failure after worker contact", "unknown", attempt)

        if (
            result.attempt_id != attempt
            or result.manifest_sha256 != manifest.manifest_sha256
            or result.endpoint_scope_sha256 != scope
        ):
            self._record_result_identity(handle, result)
            self._retain(handle, "worker result attempt or manifest digest mismatch")
            return self._finish(manifest, "initialization_outcome_unknown", "worker result identity mismatch", "unknown", attempt)

        if result.status == "ownership_unavailable":
            safe = ResourceDisposition("settled", None, None, None, (), (), authorization.operator, "no initialization was entered")
            try:
                self.evidence.write_report(self._report(manifest, attempt, "service_ownership_enforcement_unavailable", result.error, safe, False, False))
                handle.release_after_settled(safe)
            except Exception as exc:
                self._retain(handle, f"ownership gate unavailable; disposition persistence failed: {exc}")
                return self._finish(manifest, "initialization_outcome_unknown", str(exc), "unknown", attempt)
            return self._outcome(manifest, "service_ownership_enforcement_unavailable", result.error, "released", attempt, self.evidence.report_path)

        if result.status == "precontact_rejected":
            safe = ResourceDisposition(
                "settled", None, None, None, (), (), authorization.operator,
                result.error or "worker precontact validation failed",
            )
            try:
                self.evidence.write_report(
                    self._report(manifest, attempt, "rejected_before_initialization",
                                 result.error, safe, False, False)
                )
                handle.release_after_settled(safe)
            except Exception as exc:
                self._retain(handle, f"precontact rejection disposition failed: {exc}")
                return self._finish(manifest, "initialization_outcome_unknown", str(exc), "unknown", attempt)
            return self._outcome(
                manifest, "rejected_before_initialization", result.error,
                "released", attempt, self.evidence.report_path,
            )

        if result.status != "settled_success":
            reason = result.error or f"worker status was {result.status}"
            self._record_result_identity(handle, result)
            self._retain(handle, reason)
            return self._finish(manifest, "failed", reason, "unknown", attempt, result=result)

        service = result.service
        if service.pid and service.start_id and service.epoch:
            try:
                handle.record_service_identity(service)
            except Exception as exc:
                self._retain(handle, f"cannot persist service identity: {exc}")
                return self._finish(manifest, "initialization_outcome_unknown", str(exc), "unknown", attempt, result=result)

        identity_ok = self._identity_verified(result.identity, manifest)
        groups_ok = self._groups_verified(result.observations, manifest, self.clock.monotonic())
        service_ok = all((service.pid, service.start_id, service.epoch))
        if not (identity_ok and groups_ok and service_ok):
            reason = "required identity, distinct fresh state evidence, or service identity is missing"
            self._retain(handle, reason)
            return self._finish(manifest, "failed", reason, "unknown", attempt, result=result,
                                identity=identity_ok, groups=groups_ok)

        disposition = result.disposition
        expected_endpoints = {endpoint.endpoint_id for endpoint in manifest.endpoints}
        if (
            disposition.status != "settled"
            or disposition.service_epoch != service.epoch
            or set(disposition.initialized_endpoints) != expected_endpoints
            or disposition.owned_ros_resources != manifest.expected_owned_ros_resources
            or (disposition.service_pid, disposition.service_start_id) != (
                service.pid, service.start_id
            )
        ):
            self._retain(handle, "resource disposition or service epoch is not verified")
            return self._finish(manifest, "failed", "resource disposition is not verified", "unknown", attempt,
                                result=result, identity=True, groups=True)
        try:
            self.evidence.write_report(self._report(manifest, attempt, "accepted", None, disposition, True, True))
            handle.release_after_settled(disposition)
        except Exception as exc:
            self._retain(handle, f"acceptance persistence or lock release failed: {exc}")
            return self._finish(manifest, "initialization_outcome_unknown", str(exc), "unknown", attempt,
                                result=result, identity=True, groups=True)
        return self._outcome(manifest, "accepted", None, "released", attempt, self.evidence.report_path, True, True, True)

    def _retain(self, handle: Any, reason: str) -> None:
        try:
            handle.retain_unknown(reason)
        except Exception:
            pass

    def _record_result_identity(self, handle: Any, result: Any) -> None:
        service = getattr(result, "service", None)
        if service is None or not all((service.pid, service.start_id, service.epoch)):
            return
        try:
            handle.record_service_identity(service)
        except Exception:
            pass

    @staticmethod
    def _validate_inputs(manifest: CommissioningManifest, host: HostIdentity,
                         enrollment: HostEnrollmentAuthority,
                         authorization: AuthorizationRecord | None,
                         policy: TrustedOperatorPolicy) -> None:
        validate_commissioning_manifest(
            manifest, actual_host=host, enrollment=enrollment,
            authorization=authorization, policy=policy,
            trusted_lock_root=policy.trusted_lock_root,
        )
        from .commissioning_binding import _artifact_digests
        _artifact_digests(manifest)

    def _finish(
        self, manifest: CommissioningManifest, status: str, reason: str | None,
        lock_status: str, attempt: str = "", result: Any = None,
        identity: bool = False, groups: bool = False,
    ) -> CommissioningOutcome:
        try:
            path = self.evidence.write_report(self._report(
                manifest, attempt, status, reason,
                getattr(result, "disposition", None), identity, groups,
            ))
        except Exception:
            path = None
        return self._outcome(manifest, status, reason, lock_status, attempt, path, identity, groups, False)

    def _report(self, manifest: CommissioningManifest, attempt: str, status: str,
                reason: str | None, disposition: ResourceDisposition | None,
                identity: bool, groups: bool) -> dict[str, Any]:
        return {
            "attempt_id": attempt, "status": status, "reason": reason,
            "identity_verified": identity, "required_state_groups_observed": groups,
            "resource_disposition_verified": bool(disposition and disposition.status == "settled"),
            "resource_disposition": disposition,
            "lock_path": manifest.lock_path,
        }

    def _outcome(self, manifest: CommissioningManifest, status: str, reason: str | None,
                 lock_status: str, attempt: str = "", report_path: Path | None = None,
                 identity: bool = False, groups: bool = False,
                 disposition: bool = False) -> CommissioningOutcome:
        return CommissioningOutcome(status, identity, groups, disposition, attempt,
                                    manifest.manifest_sha256, lock_status, report_path, reason)

    def _identity_verified(self, identity: Any, manifest: CommissioningManifest) -> bool:
        if identity is None:
            return False
        configured = identity.configured
        if (configured.get("site_id"), configured.get("robot_id"), configured.get("instance")) != (
            manifest.site_id, manifest.robot_id, manifest.instance
        ):
            return False
        arms = identity.device_reported.get("arms")
        if not isinstance(arms, dict) or set(arms) != {"left", "right"}:
            return False
        for key, expected in manifest.expected_identity.items():
            side = expected["side"]
            arm = arms.get(side)
            if not isinstance(arm, dict) or any(
                arm.get(field) != value for field, value in expected.items()
            ):
                return False
        return identity.operator_verified == manifest.expected_operator_verification

    def _groups_verified(self, observations: tuple[StateEvidence, ...], manifest: CommissioningManifest,
                         now: float) -> bool:
        for group in REQUIRED_GROUPS:
            samples = [item for item in observations if item.group == group]
            if len(samples) < manifest.required_samples:
                return False
            samples = samples[-manifest.required_samples:]
            freshness = manifest.freshness_s.get(group, manifest.freshness_s.get("state"))
            if freshness is None:
                freshness = min(manifest.freshness_s.values())
            if any(now - item.receipt_monotonic < 0 or now - item.receipt_monotonic > freshness for item in samples):
                return False
            provenance = samples[0].sample_provenance
            if provenance == "request_response_acquisition":
                distinct = len({item.receipt_monotonic for item in samples}) == len(samples)
            elif provenance == "device_sequence":
                values = [item.source_sequence for item in samples]
                distinct = (
                    all(value is not None for value in values)
                    and len(set(values)) == len(values)
                    and all(right > left for left, right in zip(values, values[1:]))
                )
            elif provenance == "device_timestamp":
                values = [item.source_timestamp for item in samples]
                distinct = (
                    all(value is not None for value in values)
                    and len(set(values)) == len(values)
                    and all(right > left for left, right in zip(values, values[1:]))
                )
            else:
                distinct = False
            if not distinct or any(item.sample_provenance != provenance for item in samples):
                return False
        return True
