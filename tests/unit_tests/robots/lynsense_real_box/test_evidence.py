from __future__ import annotations

import json
import math
import re
from pathlib import Path
from typing import Any

import pytest

from robots.lynsense_real_box.evidence import EvidenceRecorder
from robots.lynsense_real_box.site_profile import (
    SiteProfile,
    site_profile_hash,
    validate_site_profile,
)
from tests.unit_tests.robots.lynsense_real_box.test_site_profile import valid_profile


@pytest.fixture
def profile() -> SiteProfile:
    return validate_site_profile(valid_profile())


@pytest.fixture
def evidence_path(tmp_path: Path) -> Path:
    return tmp_path / "evidence" / "run.jsonl"


def read_events(path: Path) -> list[dict[str, Any]]:
    raw = path.read_bytes()
    lines = raw.decode("utf-8").splitlines()
    events = [json.loads(line) for line in lines]
    for line, event in zip(lines, events, strict=True):
        canonical = json.dumps(
            event,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        assert line == canonical
    return events


def test_tool_events_have_a_shared_header_and_stable_ids(
    evidence_path: Path,
    profile: SiteProfile,
) -> None:
    recorder = EvidenceRecorder(evidence_path, profile)
    first_result = {"status": "ok"}
    second_result = {"status": "failed"}

    first_id = recorder.record_tool_call(
        "detect_box",
        {"goal": "pickup"},
        first_result,
        "initialized",
        "box_localized",
    )
    second_id = recorder.record_tool_call(
        "nav_to_pose",
        {"goal": "pickup"},
        second_result,
        "box_localized",
        "at_pick_approach",
    )
    recorder.close()

    assert first_id == "evt-000001"
    assert second_id == "evt-000002"
    assert first_result["evidence_id"] == first_id
    assert second_result["evidence_id"] == second_id
    events = read_events(evidence_path)
    assert [event["evidence_id"] for event in events] == [first_id, second_id]
    for event in events:
        assert event["schema_version"] == 1
        assert event["profile_id"] == profile.profile_id
        assert event["profile_sha256"] == site_profile_hash(profile)
        assert re.fullmatch(r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?Z", event["created_at"])
        assert event["tool"]
    assert events[0]["event"] == "tool_call"
    assert events[0]["arguments"] == {"goal": "pickup"}
    assert events[0]["result"] == {
        "status": "ok",
        "evidence_id": first_id,
    }
    assert events[0]["state_before"] == "initialized"
    assert events[0]["state_after"] == "box_localized"


def test_effectful_tool_intent_is_recorded_before_its_completion(
    evidence_path: Path,
    profile: SiteProfile,
) -> None:
    recorder = EvidenceRecorder(evidence_path, profile)
    intent_id = recorder.record_tool_intent(
        "move_dual_arms",
        {"profile_name": "carry"},
        "idle",
    )
    result = {"status": "ok"}
    completion_id = recorder.record_tool_call(
        "move_dual_arms",
        {"profile_name": "carry"},
        result,
        "idle",
        "idle",
        intent_evidence_id=intent_id,
    )
    recorder.close()

    events = read_events(evidence_path)
    assert [event["event"] for event in events] == [
        "tool_intent",
        "tool_call",
    ]
    assert events[0]["evidence_id"] == intent_id
    assert events[1]["evidence_id"] == completion_id
    assert events[1]["intent_evidence_id"] == intent_id
    assert result["intent_evidence_id"] == intent_id


def test_tool_completion_failure_links_to_its_intent(
    evidence_path: Path,
    profile: SiteProfile,
) -> None:
    recorder = EvidenceRecorder(evidence_path, profile)
    intent_id = recorder.record_tool_intent(
        "move_dual_arms",
        {"profile_name": "carry"},
        "idle",
    )
    failure_id = recorder.record_tool_completion_failure(
        "move_dual_arms",
        intent_id,
        "idle",
        "operator_review",
        "completion_evidence_failed",
        "ok",
        None,
    )
    recorder.close()

    events = read_events(evidence_path)
    assert [event["event"] for event in events] == [
        "tool_intent",
        "tool_completion_failure",
    ]
    assert events[1]["evidence_id"] == failure_id
    assert events[1]["intent_evidence_id"] == intent_id
    assert events[1]["reason"] == "completion_evidence_failed"
    assert events[1]["stop_status"] == "ok"
    assert events[1]["stop_reason"] is None


def test_operator_acknowledgement_uses_the_shared_header(
    evidence_path: Path,
    profile: SiteProfile,
) -> None:
    recorder = EvidenceRecorder(evidence_path, profile)
    recorder.record_operator_acknowledgement("site-operator")
    recorder.close()

    events = read_events(evidence_path)
    assert len(events) == 1
    event = events[0]
    assert event["schema_version"] == 1
    assert event["event"] == "operator_acknowledgement"
    assert event["profile_id"] == profile.profile_id
    assert event["profile_sha256"] == site_profile_hash(profile)
    assert event["created_at"]
    assert event["operator"] == "site-operator"


@pytest.mark.parametrize(
    "key",
    [
        "password",
        "PASSWORD",
        "token",
        "api_key",
        "api-key",
        "api key",
        "secret",
        "private_key",
        "Private-Key",
        "ssh_config",
        "SSH CONFIG",
    ],
)
def test_nested_credential_keys_fail_closed(
    evidence_path: Path,
    profile: SiteProfile,
    key: str,
) -> None:
    recorder = EvidenceRecorder(evidence_path, profile)
    accepted_result: dict[str, Any] = {"status": "ok"}
    recorder.record_tool_call("read_state", {}, accepted_result, "initialized", "initialized")
    arguments = {"connection": {key: "credential-value"}}
    result = {"status": "rejected", "details": {key: "credential-value"}}

    with pytest.raises(ValueError, match="credential-like key"):
        recorder.record_tool_call(
            "read_robot_state",
            arguments,
            result,
            "initialized",
            "initialized",
        )
    recorder.close()

    assert accepted_result == {"status": "ok", "evidence_id": "evt-000001"}
    assert result == {"status": "rejected", "details": {key: "credential-value"}}
    events = read_events(evidence_path)
    assert len(events) == 1
    assert events[0]["evidence_id"] == "evt-000001"


@pytest.mark.parametrize("key", ["apiKey", "access_token", "authToken", "Authorization"])
@pytest.mark.parametrize("field", ["arguments", "result"])
def test_credential_key_variants_fail_closed_without_logging_values(
    evidence_path: Path,
    profile: SiteProfile,
    key: str,
    field: str,
) -> None:
    recorder = EvidenceRecorder(evidence_path, profile)
    accepted_result: dict[str, Any] = {"status": "ok"}
    recorder.record_tool_call("read_state", {}, accepted_result, "initialized", "initialized")
    rejected_value = "credential-value"
    arguments: dict[str, Any] = {"task": "read_state"}
    result: dict[str, Any] = {"status": "ok", "robot_state": {"healthy": True}}
    if field == "arguments":
        arguments["robot_state"] = {key: rejected_value}
    else:
        result["robot_state"] = {key: rejected_value}
    before = evidence_path.read_bytes()

    with pytest.raises(ValueError, match="credential-like key"):
        recorder.record_tool_call(
            "read_robot_state",
            arguments,
            result,
            "initialized",
            "initialized",
        )
    recorder.close()

    assert evidence_path.read_bytes() == before
    assert rejected_value not in evidence_path.read_text(encoding="utf-8")
    assert len(read_events(evidence_path)) == 1


def test_non_finite_json_numbers_fail_closed(
    evidence_path: Path,
    profile: SiteProfile,
) -> None:
    recorder = EvidenceRecorder(evidence_path, profile)
    accepted_result: dict[str, Any] = {"status": "ok"}
    recorder.record_tool_call("read_state", {}, accepted_result, "initialized", "initialized")

    with pytest.raises(ValueError, match="finite JSON number"):
        recorder.record_tool_call(
            "detect_box",
            {"distance": math.inf},
            {"status": "ok"},
            "initialized",
            "initialized",
        )
    recorder.close()

    assert len(read_events(evidence_path)) == 1


def test_failed_write_preserves_complete_prior_lines(
    evidence_path: Path,
    profile: SiteProfile,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    recorder = EvidenceRecorder(evidence_path, profile)
    recorder.record_tool_call("read_state", {}, {"status": "ok"}, "initialized", "initialized")
    before = evidence_path.read_bytes()

    def fail_write(data: str) -> int:
        raise OSError("simulated write failure")

    monkeypatch.setattr(recorder._file, "write", fail_write)
    with pytest.raises(OSError, match="simulated write failure"):
        recorder.record_tool_call(
            "detect_box",
            {},
            {"status": "ok"},
            "initialized",
            "initialized",
        )
    recorder.close()

    assert evidence_path.read_bytes() == before
    assert len(read_events(evidence_path)) == 1


def test_close_is_idempotent_and_rejects_new_records(
    evidence_path: Path,
    profile: SiteProfile,
) -> None:
    recorder = EvidenceRecorder(evidence_path, profile)
    recorder.close()
    recorder.close()

    with pytest.raises(ValueError, match="closed"):
        recorder.record_tool_call("read_state", {}, {"status": "ok"}, "initialized", "initialized")
    with pytest.raises(ValueError, match="closed"):
        recorder.record_operator_acknowledgement("site-operator")


def test_reopening_recovers_the_existing_event_high_water_mark(
    evidence_path: Path,
    profile: SiteProfile,
) -> None:
    recorder = EvidenceRecorder(evidence_path, profile)
    recorder.record_tool_call("read_state", {}, {"status": "ok"}, "initialized", "initialized")
    recorder.record_operator_acknowledgement("site-operator")
    recorder.close()

    reopened = EvidenceRecorder(evidence_path, profile)
    result: dict[str, Any] = {"status": "ok"}
    reopened_id = reopened.record_tool_call(
        "detect_box",
        {},
        result,
        "initialized",
        "initialized",
    )
    reopened.close()

    assert reopened_id == "evt-000003"
    assert result["evidence_id"] == reopened_id
    assert [event["evidence_id"] for event in read_events(evidence_path)] == [
        "evt-000001",
        "evt-000002",
        "evt-000003",
    ]


@pytest.mark.parametrize("field", ["profile_id", "profile_sha256"])
def test_reopening_with_a_different_profile_identity_is_refused(
    evidence_path: Path,
    profile: SiteProfile,
    field: str,
) -> None:
    recorder = EvidenceRecorder(evidence_path, profile)
    recorder.record_tool_call("read_state", {}, {"status": "ok"}, "initialized", "initialized")
    recorder.close()
    before = evidence_path.read_bytes()

    event = json.loads(before.decode("utf-8"))
    event[field] = "incompatible-profile-identity"
    evidence_path.write_text(
        json.dumps(
            event,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        + "\n",
        encoding="utf-8",
    )
    incompatible = evidence_path.read_bytes()

    with pytest.raises(ValueError, match="different site profile"):
        EvidenceRecorder(evidence_path, profile)

    assert evidence_path.read_bytes() == incompatible


@pytest.mark.parametrize("corruption", ["truncated", "malformed", "duplicate"])
def test_malformed_existing_evidence_fails_closed_without_rewriting(
    evidence_path: Path,
    profile: SiteProfile,
    corruption: str,
) -> None:
    recorder = EvidenceRecorder(evidence_path, profile)
    recorder.record_tool_call("read_state", {}, {"status": "ok"}, "initialized", "initialized")
    recorder.close()
    valid = evidence_path.read_bytes()
    if corruption == "truncated":
        evidence_path.write_bytes(valid[:-1])
    elif corruption == "malformed":
        evidence_path.write_bytes(valid + b"not-json\n")
    else:
        evidence_path.write_bytes(valid + valid)
    corrupted = evidence_path.read_bytes()

    with pytest.raises(ValueError, match="invalid existing evidence"):
        EvidenceRecorder(evidence_path, profile)

    assert evidence_path.read_bytes() == corrupted


def test_rejected_acknowledgement_does_not_consume_an_event_id(
    evidence_path: Path,
    profile: SiteProfile,
) -> None:
    recorder = EvidenceRecorder(evidence_path, profile)

    with pytest.raises(ValueError, match="operator"):
        recorder.record_operator_acknowledgement("")

    evidence_id = recorder.record_tool_call(
        "read_state",
        {},
        {"status": "ok"},
        "initialized",
        "initialized",
    )
    recorder.close()

    assert evidence_id == "evt-000001"
    events = read_events(evidence_path)
    assert [event["evidence_id"] for event in events] == [evidence_id]
