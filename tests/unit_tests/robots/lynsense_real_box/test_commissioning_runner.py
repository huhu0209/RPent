from __future__ import annotations

import time
from dataclasses import replace
from datetime import datetime, timedelta, timezone

import pytest

from robots.lynsense_real_box.commissioning_contract import (
    IdentityEvidence, InitializationWorkerResult, ResourceDisposition,
    ServiceEvidence, StateEvidence, TrustedOperatorPolicy,
    endpoint_scope_digest,
)
from robots.lynsense_real_box.commissioning_evidence import CommissioningEvidenceWriter
from robots.lynsense_real_box.commissioning_lock import CommissioningLock, read_lock_record
from robots.lynsense_real_box.commissioning_runner import CommissioningRunner
from robots.lynsense_real_box.commissioning_worker import REQUIRED_GROUPS, WorkerDeadlineExceeded


def _identity(manifest):
    return IdentityEvidence(
        {"site_id": manifest.site_id, "robot_id": manifest.robot_id, "instance": manifest.instance},
        {"arms": {
            side: {"side": side, "model": "arm", "ip": f"127.0.0.{index + 1}", "axes": 6}
            for index, side in enumerate(("left", "right"))
        }},
        {"left_arm": "operator-verified", "right_arm": "operator-verified"},
    )


def _scope_digest(manifest):
    return endpoint_scope_digest(manifest)


def _successful_result(manifest, attempt, deadline):
    receipt = time.monotonic() - 0.02
    service = ServiceEvidence(456, "start-2", "epoch-2")
    observations = []
    for index, group in enumerate(REQUIRED_GROUPS):
        observations.extend((
            StateEvidence(group, {"values": [0.0]}, None, "device_sequence", receipt, 1, None),
            StateEvidence(group, {"values": [0.0]}, None, "device_sequence", receipt + 0.005, 2, None),
        ))
    return InitializationWorkerResult(
        "settled_success", attempt, manifest.manifest_sha256,
        _scope_digest(manifest), _identity(manifest),
        tuple(observations), service,
        ResourceDisposition("settled", service.pid, service.start_id, service.epoch,
                            tuple(item.endpoint_id for item in manifest.endpoints),
                            manifest.expected_owned_ros_resources, None, None),
        None, 789, "worker-start",
    )


class Supervisor:
    def run_worker(self, request):
        assert request.operator_verification == {
            "left_arm": "operator-verified", "right_arm": "operator-verified"
        }
        return _successful_result(self.manifest, request.attempt_id, request.deadline_monotonic)


class TimedOutSupervisor:
    def run_worker(self, request):
        raise WorkerDeadlineExceeded(789, "worker-start")


def _runner(tmp_path, manifest, supervisor, authorization):
    evidence = CommissioningEvidenceWriter("run-1", manifest, tmp_path / "commissioning.jsonl")
    policy = TrustedOperatorPolicy(tmp_path / "policy.json", "e" * 64,
                                   manifest.trusted_lock_root, (manifest.execution_account,),
                                   {authorization.authorization_id: authorization.authorization_sha256})
    return CommissioningRunner(CommissioningLock, evidence, supervisor), policy, evidence


def test_runner_accepts_only_bound_fresh_distinct_evidence_and_releases_lock(tmp_path, valid_manifest, valid_authority):
    authority, host, authorization = valid_authority
    supervisor = Supervisor()
    runner, policy, evidence = _runner(tmp_path, valid_manifest, supervisor, authorization)
    supervisor.manifest = valid_manifest

    outcome = runner.run_once(valid_manifest, host, authority, authorization, policy)

    assert outcome.status == "accepted", outcome
    assert outcome.lock_status == "released"
    assert outcome.identity_verified and outcome.required_state_groups_observed
    assert outcome.resource_disposition_verified
    assert not valid_manifest.lock_path.exists()
    consumed = (
        valid_manifest.trusted_lock_root / "consumed" / _scope_digest(valid_manifest)
        / f"{authorization.attempt_id}.json"
    )
    assert consumed.is_file()
    second = runner.run_once(valid_manifest, host, authority, authorization, policy)
    assert second.status == "rejected_before_initialization"
    assert "already consumed" in second.reason
    evidence.close()


def test_runner_timeout_retains_unknown_lock_and_does_not_retry(tmp_path, valid_manifest, valid_authority):
    authority, host, authorization = valid_authority
    runner, policy, evidence = _runner(tmp_path, valid_manifest, TimedOutSupervisor(), authorization)

    outcome = runner.run_once(valid_manifest, host, authority, authorization, policy)

    assert outcome.status == "initialization_outcome_unknown"
    assert outcome.lock_status == "unknown"
    assert read_lock_record(valid_manifest.lock_path).state == "unknown"
    evidence.close()


@pytest.mark.parametrize(
    "mismatch", ["operator_verification", "owned_resources", "extra_reported_arm"]
)
def test_runner_rejects_unverified_identity_or_resource_inventory(
    tmp_path, valid_manifest, valid_authority, mismatch
):
    authority, host, authorization = valid_authority

    class MismatchedSupervisor:
        def run_worker(self, request):
            result = _successful_result(
                valid_manifest, request.attempt_id, request.deadline_monotonic
            )
            if mismatch == "operator_verification":
                identity = replace(result.identity, operator_verified={})
                return replace(result, identity=identity)
            if mismatch == "extra_reported_arm":
                identity = replace(
                    result.identity,
                    device_reported={
                        **result.identity.device_reported,
                        "arms": {
                            **result.identity.device_reported["arms"],
                            "center": {"side": "center"},
                        },
                    },
                )
                return replace(result, identity=identity)
            disposition = replace(
                result.disposition, owned_ros_resources=("unreviewed-resource",)
            )
            return replace(result, disposition=disposition)

    runner, policy, evidence = _runner(
        tmp_path, valid_manifest, MismatchedSupervisor(), authorization
    )

    outcome = runner.run_once(valid_manifest, host, authority, authorization, policy)

    assert outcome.status == "failed"
    assert read_lock_record(valid_manifest.lock_path).state == "unknown"
    evidence.close()


def test_runner_rejects_host_mismatch_before_worker_and_lock(tmp_path, valid_manifest, valid_authority):
    authority, host, authorization = valid_authority
    runner, policy, evidence = _runner(tmp_path, valid_manifest, TimedOutSupervisor(), authorization)
    wrong_host = replace(host, machine_id="b" * 32)

    outcome = runner.run_once(valid_manifest, wrong_host, authority, authorization, policy)

    assert outcome.status == "rejected_before_initialization"
    assert not valid_manifest.lock_path.exists()
    evidence.close()


def test_runner_source_inventory_mismatch_rejects_before_lock(tmp_path, valid_manifest, valid_authority):
    authority, host, authorization = valid_authority
    runner, policy, evidence = _runner(tmp_path, valid_manifest, TimedOutSupervisor(), authorization)
    valid_manifest.source_artifacts[0].write_text("changed offline fixture", encoding="utf-8")

    outcome = runner.run_once(valid_manifest, host, authority, authorization, policy)

    assert outcome.status == "rejected_before_initialization"
    assert "inventory mismatch" in outcome.reason
    assert not valid_manifest.lock_path.exists()
    evidence.close()


def test_runner_rejects_reuse_of_an_existing_attempt_result(tmp_path, valid_manifest, valid_authority):
    authority, host, authorization = valid_authority
    runner, policy, evidence = _runner(tmp_path, valid_manifest, TimedOutSupervisor(), authorization)
    valid_manifest.evidence_dir.mkdir(parents=True, exist_ok=True)
    result_path = valid_manifest.evidence_dir / f"{authorization.attempt_id}.result.json"
    result_path.write_text("{}\n", encoding="utf-8")

    outcome = runner.run_once(valid_manifest, host, authority, authorization, policy)

    assert outcome.status == "rejected_before_initialization"
    assert "attempt IDs are not reusable" in outcome.reason
    assert not valid_manifest.lock_path.exists()
    evidence.close()


def test_runner_rejects_worker_result_from_a_different_endpoint_scope(
    tmp_path, valid_manifest, valid_authority
):
    authority, host, authorization = valid_authority

    class WrongScopeSupervisor:
        def run_worker(self, request):
            result = _successful_result(
                valid_manifest, request.attempt_id, request.deadline_monotonic
            )
            return replace(result, endpoint_scope_sha256="0" * 64)

    runner, policy, evidence = _runner(
        tmp_path, valid_manifest, WrongScopeSupervisor(), authorization
    )

    outcome = runner.run_once(valid_manifest, host, authority, authorization, policy)

    assert outcome.status == "initialization_outcome_unknown"
    assert "worker result identity mismatch" in outcome.reason
    assert read_lock_record(valid_manifest.lock_path).state == "unknown"
    evidence.close()


def test_runner_rejects_non_advancing_device_sequences(tmp_path, valid_manifest, valid_authority):
    authority, host, authorization = valid_authority

    class NonAdvancingSupervisor:
        def run_worker(self, request):
            result = _successful_result(
                valid_manifest, request.attempt_id, request.deadline_monotonic
            )
            first, second = result.observations[:2]
            reversed_samples = (
                replace(first, source_sequence=2),
                replace(second, source_sequence=1),
            )
            return replace(result, observations=reversed_samples + result.observations[2:])

    runner, policy, evidence = _runner(
        tmp_path, valid_manifest, NonAdvancingSupervisor(), authorization
    )

    outcome = runner.run_once(valid_manifest, host, authority, authorization, policy)

    assert outcome.status == "failed"
    assert not outcome.required_state_groups_observed
    assert read_lock_record(valid_manifest.lock_path).state == "unknown"
    evidence.close()
