from __future__ import annotations

import dataclasses
import hashlib
import json
from datetime import datetime, timedelta, timezone

import pytest

from robots.lynsense_real_box.commissioning_contract import (
    AuthorizationRecord,
    CommissioningContractError,
    HostEnrollmentAuthority,
    HostEnrollmentRecord,
    HostIdentity,
    TrustedOperatorPolicy,
    endpoint_scope_digest,
    load_commissioning_manifest,
    load_trusted_operator_policy,
    manifest_digest,
    validate_commissioning_manifest,
)
from robots.lynsense_real_box.ownership_protocol import load_ownership_protocol


ARTIFACT_BYTES = (
    '{\n'
    '  "endpoint_authority_root": "/tmp/lynrotcontrol-authority-1000",\n'
    '  "protocol_id": "lynrotcontrol.service-ownership",\n'
    '  "protocol_version": 1,\n'
    '  "read_allowlist_sha256": "38015e139b4b7ae2c0ca7f2261be2a90cb4d40ee12d35420875c6d150962ceee",\n'
    '  "runtime_protocol": 15\n'
    '}\n'
)


def test_manifest_digest_is_self_excluding_and_canonical(tmp_path, valid_manifest_data, write_json):
    data = valid_manifest_data(tmp_path)
    manifest = load_commissioning_manifest(write_json(tmp_path / "manifest.json", data))
    changed = dict(data, manifest_sha256="0" * 64)
    assert manifest_digest(changed) == manifest.manifest_sha256


def test_manifest_enforces_exact_schema_and_domain(tmp_path, valid_manifest_data, write_json):
    data = valid_manifest_data(tmp_path)
    data.pop("execution_host_machine_id")
    with pytest.raises(CommissioningContractError, match="machine identity"):
        load_commissioning_manifest(write_json(tmp_path / "missing.json", data))
    data = valid_manifest_data(tmp_path)
    data["unexpected"] = True
    with pytest.raises(CommissioningContractError, match="unexpected"):
        load_commissioning_manifest(write_json(tmp_path / "extra.json", data))
    data = valid_manifest_data(tmp_path)
    data["ros_domain_id"] = 42
    with pytest.raises(CommissioningContractError, match="ROS_DOMAIN_ID=3"):
        load_commissioning_manifest(write_json(tmp_path / "domain.json", data))


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("required_samples", 1, "at least 2"),
        ("site_id", "", "site_id"),
        ("evidence_dir", "relative/evidence", "absolute"),
        ("authorization_expires_at", "2099-01-01T00:00:00", "timezone-aware"),
    ],
)
def test_manifest_rejects_invalid_required_values(
    tmp_path, valid_manifest_data, write_json, field, value, message
):
    data = valid_manifest_data(tmp_path)
    data[field] = value
    with pytest.raises(CommissioningContractError, match=message):
        load_commissioning_manifest(write_json(tmp_path / f"invalid-{field}.json", data))


def test_manifest_rejects_nonfinite_budget_and_symlink_paths(tmp_path, valid_manifest_data, write_json):
    data = valid_manifest_data(tmp_path)
    data["budgets"]["initialize"] = float("inf")
    with pytest.raises(CommissioningContractError, match="finite"):
        load_commissioning_manifest(write_json(tmp_path / "budget.json", data))

    target = tmp_path / "actual-root"
    target.mkdir()
    link = tmp_path / "linked-root"
    link.symlink_to(target, target_is_directory=True)
    data = valid_manifest_data(tmp_path)
    data["trusted_lock_root"] = str(link)
    with pytest.raises(CommissioningContractError, match="symlink"):
        load_commissioning_manifest(write_json(tmp_path / "symlink.json", data))


def test_manifest_pins_interpreter_budgets_and_acceptance_inventory(
    tmp_path, valid_manifest_data, write_json
):
    data = valid_manifest_data(tmp_path)
    data.pop("execution_interpreter")
    with pytest.raises(CommissioningContractError, match="execution_interpreter"):
        load_commissioning_manifest(write_json(tmp_path / "interpreter.json", data))

    data = valid_manifest_data(tmp_path)
    data["budgets"].pop("read")
    with pytest.raises(CommissioningContractError, match="initialize and read"):
        load_commissioning_manifest(write_json(tmp_path / "budget-keys.json", data))

    data = valid_manifest_data(tmp_path)
    data["expected_identity"]["left_arm"]["side"] = "right"
    with pytest.raises(CommissioningContractError, match="expected_identity"):
        load_commissioning_manifest(write_json(tmp_path / "identity.json", data))

    data = valid_manifest_data(tmp_path)
    data["expected_owned_ros_resources"].append("left-arm-service")
    with pytest.raises(CommissioningContractError, match="unique"):
        load_commissioning_manifest(write_json(tmp_path / "resources.json", data))


def test_manifest_requires_source_and_config_digest_keys_to_name_artifacts(
    tmp_path, valid_manifest_data, write_json
):
    data = valid_manifest_data(tmp_path)
    data["config_sha256"] = {"site-config": "d" * 64}
    data["manifest_sha256"] = manifest_digest(data)
    with pytest.raises(CommissioningContractError, match="normalized paths"):
        load_commissioning_manifest(write_json(tmp_path / "unresolved-config.json", data))


def test_ownership_protocol_artifact_has_exact_schema_and_digest(tmp_path):
    path = tmp_path / "OWNERSHIP_PROTOCOL.json"
    path.write_text(ARTIFACT_BYTES, encoding="utf-8")
    protocol = load_ownership_protocol(path)
    assert protocol.artifact_sha256 == (
        "78c45f6796cd90fdf63f51ac407ac261c04e42ab454bb6d0897749d81904c84a"
    )
    assert protocol.endpoint_authority_root.as_posix() == (
        "/tmp/lynrotcontrol-authority-1000"
    )
    assert protocol.runtime_protocol == 15


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("endpoint_authority_root", "relative/root"),
        ("protocol_id", "wrong"),
        ("protocol_version", 2),
        ("read_allowlist_sha256", "0" * 64),
        ("runtime_protocol", 14),
    ],
)
def test_ownership_protocol_artifact_value_mismatch_is_rejected(
    tmp_path, field, value
):
    import json

    path = tmp_path / "OWNERSHIP_PROTOCOL.json"
    data = json.loads(ARTIFACT_BYTES)
    data[field] = value
    path.write_text(json.dumps(data, sort_keys=True), encoding="utf-8")
    with pytest.raises(ValueError, match="ownership protocol"):
        load_ownership_protocol(path)


def test_ownership_protocol_symlink_and_extra_fields_are_rejected(tmp_path):
    target = tmp_path / "actual.json"
    target.write_text(ARTIFACT_BYTES, encoding="utf-8")
    link = tmp_path / "linked.json"
    link.symlink_to(target)
    with pytest.raises(ValueError, match="cannot read ownership protocol"):
        load_ownership_protocol(link)

    path = tmp_path / "extra.json"
    path.write_text('{"unexpected":true}\n', encoding="utf-8")
    with pytest.raises(ValueError, match="does not match protocol v1"):
        load_ownership_protocol(path)


def test_endpoint_digest_uses_exact_sorted_endpoint_objects(valid_manifest):
    manifest = valid_manifest
    expected = {
        "robot_id": manifest.robot_id,
        "site_id": manifest.site_id,
        "endpoints": [
            {
                "endpoint_id": item.endpoint_id,
                "locator": item.locator,
                "subsystem": item.subsystem,
                "transport": item.transport,
            }
            for item in sorted(
                manifest.endpoints,
                key=lambda item: (item.subsystem, item.endpoint_id, item.transport, item.locator),
            )
        ],
    }
    encoded = json.dumps(expected, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    assert endpoint_scope_digest(manifest) == hashlib.sha256(encoded.encode()).hexdigest()


def test_manifest_pins_exact_ownership_protocol_fields(valid_manifest, valid_manifest_data):
    assert valid_manifest.ownership_protocol_sha256 == (
        "78c45f6796cd90fdf63f51ac407ac261c04e42ab454bb6d0897749d81904c84a"
    )
    assert valid_manifest.ownership_protocol_artifact.name == "OWNERSHIP_PROTOCOL.json"
    assert valid_manifest.ownership_protocol_id == "lynrotcontrol.service-ownership"
    assert valid_manifest.ownership_protocol_version == 1
    assert valid_manifest.ownership_runtime_protocol == 15


def test_manifest_rejects_missing_or_extra_ownership_fields(
    tmp_path, valid_manifest_data, write_json
):
    data = valid_manifest_data(tmp_path)
    data.pop("ownership_protocol_sha256")
    data["manifest_sha256"] = manifest_digest(data)
    with pytest.raises(CommissioningContractError, match="missing required fields"):
        load_commissioning_manifest(write_json(tmp_path / "missing.json", data))

    data = valid_manifest_data(tmp_path)
    data["unexpected_ownership"] = True
    data["manifest_sha256"] = manifest_digest(data)
    with pytest.raises(CommissioningContractError, match="unexpected fields"):
        load_commissioning_manifest(write_json(tmp_path / "extra.json", data))


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("ownership_protocol_artifact", None),
        ("ownership_protocol_sha256", "0" * 64),
        ("ownership_protocol_id", "wrong"),
        ("ownership_protocol_version", 2),
        ("ownership_read_allowlist_sha256", "0" * 64),
        ("endpoint_authority_root", "/tmp/not-approved"),
        ("ownership_runtime_protocol", 14),
    ],
)
def test_manifest_rejects_ownership_fields_mismatch(
    tmp_path, valid_manifest_data, write_json, field, value
):
    data = valid_manifest_data(tmp_path)
    if value is None:
        data["source_artifact_sha256"].pop(data["ownership_protocol_artifact"])
        data["source_artifacts"] = [
            item for item in data["source_artifacts"]
            if item != data["ownership_protocol_artifact"]
        ]
    else:
        data[field] = value
    unsigned = {key: val for key, val in data.items() if key != "manifest_sha256"}
    data["manifest_sha256"] = hashlib.sha256(json.dumps(
        unsigned, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")).hexdigest()
    with pytest.raises(CommissioningContractError, match="ownership"):
        load_commissioning_manifest(write_json(tmp_path / "bad-ownership.json", data))


def test_manifest_rejects_conflicting_ownership_source_digest(
    tmp_path, valid_manifest_data, write_json
):
    data = valid_manifest_data(tmp_path)
    artifact = data["ownership_protocol_artifact"]
    data["source_artifact_sha256"][artifact] = "1" * 64
    unsigned = {key: val for key, val in data.items() if key != "manifest_sha256"}
    data["manifest_sha256"] = hashlib.sha256(json.dumps(
        unsigned, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")).hexdigest()
    with pytest.raises(CommissioningContractError, match="source artifact"):
        load_commissioning_manifest(write_json(tmp_path / "conflict.json", data))


@pytest.mark.parametrize("axes", [0, -1, 1.5, True, ["a", "b"]])
def test_manifest_requires_positive_integer_arm_axes(
    tmp_path, valid_manifest_data, write_json, axes
):
    data = valid_manifest_data(tmp_path)
    data["expected_identity"]["left_arm"]["axes"] = axes
    data["manifest_sha256"] = manifest_digest(data)
    with pytest.raises(CommissioningContractError, match="positive integer"):
        load_commissioning_manifest(write_json(tmp_path / "axes.json", data))


def test_host_enrollment_and_authorization_are_bound(valid_manifest, valid_authority):
    host = valid_authority[1]
    authority = valid_authority[0]
    authorization = valid_authority[2]
    policy = TrustedOperatorPolicy(
        valid_manifest.trusted_lock_root / "operator-policy.json",
        "e" * 64,
        valid_manifest.trusted_lock_root,
        ("operator",),
        {valid_manifest.authorization_id: valid_authority[2].authorization_sha256},
    )
    validate_commissioning_manifest(
        valid_manifest,
        actual_host=host,
        enrollment=authority,
        authorization=authorization,
        policy=policy,
        trusted_lock_root=valid_manifest.trusted_lock_root,
    )

    from robots.lynsense_real_box.commissioning_contract import host_enrollment_digest

    duplicate_records = authority.records + (authority.records[0],)
    duplicate = dataclasses.replace(
        authority,
        authority_sha256=host_enrollment_digest(authority.authority_id, duplicate_records),
        records=duplicate_records,
    )
    duplicate_manifest = dataclasses.replace(
        valid_manifest, host_enrollment_sha256=duplicate.authority_sha256
    )
    with pytest.raises(CommissioningContractError, match="exactly once"):
        validate_commissioning_manifest(
            duplicate_manifest,
            actual_host=host,
            enrollment=duplicate,
            policy=policy,
            trusted_lock_root=valid_manifest.trusted_lock_root,
        )


def test_authorization_expiry_and_root_mismatch_are_rejected(valid_manifest, valid_authority):
    authority, host, authorization = valid_authority
    policy = TrustedOperatorPolicy(
        valid_manifest.trusted_lock_root / "operator-policy.json",
        "e" * 64,
        valid_manifest.trusted_lock_root,
        ("operator",),
        {valid_manifest.authorization_id: authorization.authorization_sha256},
    )
    expired = dataclasses.replace(
        authorization,
        expires_at=datetime.now(timezone.utc) - timedelta(seconds=1),
    )
    with pytest.raises(CommissioningContractError, match="expired"):
        validate_commissioning_manifest(
            valid_manifest,
            actual_host=host,
            enrollment=authority,
            authorization=expired,
            policy=policy,
            trusted_lock_root=valid_manifest.trusted_lock_root,
        )
    with pytest.raises(CommissioningContractError, match="trusted lock root"):
        validate_commissioning_manifest(
            valid_manifest,
            actual_host=host,
            enrollment=authority,
            authorization=authorization,
            policy=policy,
            trusted_lock_root=valid_manifest.trusted_lock_root.parent,
        )

    mismatched = dataclasses.replace(authorization, robot_id="another-robot")
    with pytest.raises(CommissioningContractError, match="authorization does not match"):
        validate_commissioning_manifest(
            valid_manifest,
            actual_host=host,
            enrollment=authority,
            authorization=mismatched,
            policy=policy,
            trusted_lock_root=valid_manifest.trusted_lock_root,
        )

    mismatched_expiry = dataclasses.replace(
        authorization,
        expires_at=authorization.expires_at + timedelta(seconds=1),
    )
    with pytest.raises(CommissioningContractError, match="authorization expiry"):
        validate_commissioning_manifest(
            valid_manifest,
            actual_host=host,
            enrollment=authority,
            authorization=mismatched_expiry,
            policy=policy,
            trusted_lock_root=valid_manifest.trusted_lock_root,
        )

    wrong_interpreter = dataclasses.replace(host, interpreter="/another/python")
    with pytest.raises(CommissioningContractError, match="interpreter"):
        validate_commissioning_manifest(
            valid_manifest,
            actual_host=wrong_interpreter,
            enrollment=authority,
            authorization=authorization,
            policy=policy,
            trusted_lock_root=valid_manifest.trusted_lock_root,
        )

    bad_policy = dataclasses.replace(policy, sha256="f" * 64)
    with pytest.raises(CommissioningContractError, match="policy digest"):
        validate_commissioning_manifest(
            valid_manifest,
            actual_host=host,
            enrollment=authority,
            authorization=authorization,
            policy=bad_policy,
            trusted_lock_root=valid_manifest.trusted_lock_root,
        )

    empty_operator = dataclasses.replace(authorization, operator=" ")
    with pytest.raises(CommissioningContractError, match="operator"):
        validate_commissioning_manifest(
            valid_manifest,
            actual_host=host,
            enrollment=authority,
            authorization=empty_operator,
            policy=policy,
            trusted_lock_root=valid_manifest.trusted_lock_root,
        )


def test_trusted_policy_digest_and_operator_membership_are_verified(valid_manifest, tmp_path):
    policy_path = tmp_path / "policy.json"
    unsigned = {
        "trusted_lock_root": str(valid_manifest.trusted_lock_root),
        "allowed_operators": ["operator"],
        "approved_authorizations": {},
    }
    policy_path.write_text(
        json.dumps({**unsigned, "sha256": hashlib.sha256(
            json.dumps(unsigned, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode()
        ).hexdigest()}),
        encoding="utf-8",
    )
    policy = load_trusted_operator_policy(policy_path)
    assert policy.allowed_operators == ("operator",)

    bad = dataclasses.replace(policy, allowed_operators=("other-operator",))
    with pytest.raises(CommissioningContractError, match="not allowed"):
        validate_commissioning_manifest(
            valid_manifest,
            actual_host=HostIdentity(
                valid_manifest.execution_host_machine_id,
                valid_manifest.execution_hostname,
                valid_manifest.execution_account,
                "/usr/bin/python3",
            ),
            enrollment=HostEnrollmentAuthority(
                "authority-1",
                valid_manifest.host_enrollment_sha256,
                (HostEnrollmentRecord(
                    valid_manifest.site_id,
                    valid_manifest.execution_host_machine_id,
                    valid_manifest.execution_hostname,
                    valid_manifest.execution_account,
                    True,
                ),),
            ),
            authorization=AuthorizationRecord(
                valid_manifest.authorization_id,
                "commissioning_run",
                "a" * 32,
                "f" * 64,
                valid_manifest.manifest_sha256,
                valid_manifest.site_id,
                valid_manifest.robot_id,
                valid_manifest.execution_host_machine_id,
                valid_manifest.host_enrollment_sha256,
                policy.sha256,
                "operator",
                3,
                valid_manifest.authorization_expires_at,
            ),
            policy=bad,
            trusted_lock_root=valid_manifest.trusted_lock_root,
        )


def test_contract_module_has_no_effectful_imports():
    import robots.lynsense_real_box.commissioning_contract as contract

    assert not {"lynrotcontrol", "rclpy", "socket", "subprocess"}.intersection(contract.__dict__)
