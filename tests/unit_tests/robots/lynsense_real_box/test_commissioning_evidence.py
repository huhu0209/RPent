from __future__ import annotations

import json

import pytest

from robots.lynsense_real_box.commissioning_evidence import (
    CommissioningEvidenceError, CommissioningEvidenceWriter,
)


def test_writer_emits_canonical_linked_jsonl_and_final_report(tmp_path, valid_manifest):
    writer = CommissioningEvidenceWriter("run-1", valid_manifest, tmp_path / "run.jsonl")
    writer.record("intent", attempt_id="attempt-1", values=[1, 2])
    report_path = writer.write_report({"status": "accepted", "attempt_id": "attempt-1"})
    writer.close()

    line = (tmp_path / "run.jsonl").read_text(encoding="utf-8").strip()
    event = json.loads(line)
    assert line == json.dumps(event, ensure_ascii=False, allow_nan=False, sort_keys=True, separators=(",", ":"))
    assert event["run_id"] == "run-1"
    assert event["manifest_sha256"] == valid_manifest.manifest_sha256
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["status"] == "accepted"
    assert report["run_id"] == "run-1"


@pytest.mark.parametrize("fields", [{"nested": {"apiKey": "secret"}}, {"sample": float("nan")}])
def test_writer_rejects_credentials_and_non_finite_values(tmp_path, valid_manifest, fields):
    writer = CommissioningEvidenceWriter("run-1", valid_manifest, tmp_path / "run.jsonl")
    with pytest.raises(CommissioningEvidenceError):
        writer.record("observation", **fields)
    writer.close()


def test_writer_rejects_truncated_existing_jsonl(tmp_path, valid_manifest):
    path = tmp_path / "run.jsonl"
    path.write_text('{"partial":', encoding="utf-8")
    with pytest.raises(CommissioningEvidenceError, match="truncated"):
        CommissioningEvidenceWriter("run-1", valid_manifest, path)


def test_writer_rejects_report_linkage_override(tmp_path, valid_manifest):
    writer = CommissioningEvidenceWriter("run-1", valid_manifest, tmp_path / "run.jsonl")
    with pytest.raises(CommissioningEvidenceError, match="override"):
        writer.write_report({"run_id": "other-run"})
    writer.close()
