"""Offline LynrotControl ownership protocol artifact verification."""

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path


PROTOCOL_ID = "lynrotcontrol.service-ownership"
PROTOCOL_VERSION = 1
RUNTIME_PROTOCOL = 15
READ_ALLOWLIST_SHA256 = (
    "38015e139b4b7ae2c0ca7f2261be2a90cb4d40ee12d35420875c6d150962ceee"
)
ARTIFACT_SHA256 = (
    "78c45f6796cd90fdf63f51ac407ac261c04e42ab454bb6d0897749d81904c84a"
)
AUTHORITY_ROOT = Path("/tmp/lynrotcontrol-authority-1000")
_ARTIFACT_KEYS = {
    "endpoint_authority_root",
    "protocol_id",
    "protocol_version",
    "read_allowlist_sha256",
    "runtime_protocol",
}


@dataclass(frozen=True)
class OwnershipProtocolContract:
    artifact_path: Path
    artifact_sha256: str
    protocol_id: str
    protocol_version: int
    read_allowlist_sha256: str
    endpoint_authority_root: Path
    runtime_protocol: int


def load_ownership_protocol(path: Path) -> OwnershipProtocolContract:
    artifact_path = Path(path)
    try:
        if artifact_path.is_symlink() or not artifact_path.is_file():
            raise OSError("not a regular non-symlink file")
        content = artifact_path.read_bytes()
    except OSError as exc:
        raise ValueError(f"cannot read ownership protocol artifact: {exc}") from exc
    artifact_sha256 = hashlib.sha256(content).hexdigest()
    try:
        value = json.loads(content.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot decode ownership protocol artifact: {exc}") from exc
    expected = {
        "endpoint_authority_root": AUTHORITY_ROOT.as_posix(),
        "protocol_id": PROTOCOL_ID,
        "protocol_version": PROTOCOL_VERSION,
        "read_allowlist_sha256": READ_ALLOWLIST_SHA256,
        "runtime_protocol": RUNTIME_PROTOCOL,
    }
    authority_root = value.get("endpoint_authority_root") if isinstance(value, dict) else None
    if (
        not isinstance(value, dict)
        or set(value) != _ARTIFACT_KEYS
        or value != expected
        or not isinstance(authority_root, str)
    ):
        raise ValueError("ownership protocol artifact does not match protocol v1")
    authority_path = Path(authority_root)
    if not authority_path.is_absolute() or ".." in authority_path.parts:
        raise ValueError("ownership protocol authority root must be absolute")
    if artifact_sha256 != ARTIFACT_SHA256:
        raise ValueError("ownership protocol artifact digest does not match protocol v1")
    return OwnershipProtocolContract(
        artifact_path=artifact_path,
        artifact_sha256=artifact_sha256,
        protocol_id=PROTOCOL_ID,
        protocol_version=PROTOCOL_VERSION,
        read_allowlist_sha256=READ_ALLOWLIST_SHA256,
        endpoint_authority_root=authority_path,
        runtime_protocol=RUNTIME_PROTOCOL,
    )
