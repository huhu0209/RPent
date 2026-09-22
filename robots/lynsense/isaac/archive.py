# Copyright 2026 The RPent Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import hashlib
import os
import tarfile
from dataclasses import dataclass
from pathlib import Path, PurePosixPath

from robots.lynsense.isaac.contracts import IsaacProbeError

MAX_ARCHIVE_ENTRIES = 100_000
MAX_UNCOMPRESSED_BYTES = 2 * 1024**3
_COPY_CHUNK_BYTES = 1024 * 1024
_ROBOT_URDF_NAME = "URDF_robot_1/robot.urdf"


class ArchiveError(IsaacProbeError):
    """A robot archive could not be safely staged."""


@dataclass(frozen=True)
class ArchiveInventory:
    archive_path: Path
    robot_urdf: Path
    file_count: int
    total_bytes: int
    archive_digest: str
    robot_urdf_sha256: str


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file_object:
        for chunk in iter(lambda: file_object.read(_COPY_CHUNK_BYTES), b""):
            digest.update(chunk)
    return f"sha256:{digest.hexdigest()}"


def _member_relative_path(member: tarfile.TarInfo) -> PurePosixPath:
    name = member.name
    if not name or "\\" in name:
        raise ArchiveError(f"unsafe archive member name {name!r}")
    relative_path = PurePosixPath(name)
    if (
        relative_path.is_absolute()
        or not relative_path.parts
        or any(part in {".", ".."} for part in relative_path.parts)
    ):
        raise ArchiveError(f"unsafe archive member name {name!r}")
    return relative_path


def _validate_members(
    members: list[tarfile.TarInfo], max_uncompressed_bytes: int
) -> tuple[list[tuple[tarfile.TarInfo, PurePosixPath]], int, str | None]:
    if len(members) > MAX_ARCHIVE_ENTRIES:
        raise ArchiveError(f"archive contains more than {MAX_ARCHIVE_ENTRIES} entries")

    validated: list[tuple[tarfile.TarInfo, PurePosixPath]] = []
    total_bytes = 0
    robot_urdf_member: tarfile.TarInfo | None = None
    for member in members:
        relative_path = _member_relative_path(member)
        if member.isdir():
            pass
        elif member.isfile():
            if member.size < 0:
                raise ArchiveError(f"negative size for archive member {member.name!r}")
            total_bytes += member.size
            if total_bytes > max_uncompressed_bytes:
                raise ArchiveError(
                    "uncompressed archive content exceeds "
                    f"{max_uncompressed_bytes} bytes; hard limit is 2 GiB"
                )
            if relative_path.as_posix() == _ROBOT_URDF_NAME:
                robot_urdf_member = member
        else:
            kind = {
                tarfile.SYMTYPE: "symlink",
                tarfile.LNKTYPE: "hardlink",
                tarfile.FIFOTYPE: "FIFO",
                tarfile.CHRTYPE: "character device",
                tarfile.BLKTYPE: "block device",
            }.get(member.type, "unknown entry type")
            raise ArchiveError(f"unsupported archive member {member.name!r}: {kind}")
        validated.append((member, relative_path))
    return validated, total_bytes, robot_urdf_member


def extract_archive(
    archive: Path,
    destination: Path,
    *,
    max_uncompressed_bytes: int = MAX_UNCOMPRESSED_BYTES,
) -> ArchiveInventory:
    if type(max_uncompressed_bytes) is not int or max_uncompressed_bytes <= 0:
        raise ArchiveError("max_uncompressed_bytes must be a positive integer")
    if max_uncompressed_bytes > MAX_UNCOMPRESSED_BYTES:
        raise ArchiveError("max_uncompressed_bytes must not exceed 2 GiB")
    if not archive.is_file():
        raise ArchiveError(f"archive is not a regular file: {archive}")

    try:
        with tarfile.open(archive, "r:gz") as tar:
            validated_members, total_bytes, robot_urdf_member = _validate_members(
                tar.getmembers(), max_uncompressed_bytes
            )
            if robot_urdf_member is None:
                raise ArchiveError(f"archive does not contain {_ROBOT_URDF_NAME}")

            resolved_destination = destination.resolve()
            proposed_outputs: list[tuple[tarfile.TarInfo, Path]] = []
            for member, relative_path in validated_members:
                output_path = (resolved_destination / relative_path).resolve()
                if not output_path.is_relative_to(resolved_destination):
                    raise ArchiveError(f"unsafe archive member name {member.name!r}")
                proposed_outputs.append((member, output_path))

            destination.mkdir(mode=0o700, parents=True, exist_ok=True)
            for member, output_path in proposed_outputs:
                if not output_path.resolve().is_relative_to(resolved_destination):
                    raise ArchiveError(f"unsafe archive member name {member.name!r}")
                if member.isdir():
                    output_path.mkdir(mode=0o700, parents=True, exist_ok=True)
                    continue

                output_path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
                extracted_file = tar.extractfile(member)
                if extracted_file is None:
                    raise ArchiveError(f"could not read archive member {member.name!r}")
                try:
                    descriptor = os.open(
                        output_path,
                        os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW,
                        0o600,
                    )
                    with os.fdopen(descriptor, "wb") as output_file, extracted_file:
                        while chunk := extracted_file.read(_COPY_CHUNK_BYTES):
                            output_file.write(chunk)
                    os.chmod(output_path, 0o600)
                except OSError as exc:
                    raise ArchiveError(
                        f"could not extract archive member {member.name!r}"
                    ) from exc
    except (tarfile.TarError, EOFError, OSError) as exc:
        raise ArchiveError(f"could not safely read archive {archive}") from exc

    robot_urdf = destination / _ROBOT_URDF_NAME
    return ArchiveInventory(
        archive_path=archive,
        robot_urdf=robot_urdf,
        file_count=sum(member.isfile() for member, _ in validated_members),
        total_bytes=total_bytes,
        archive_digest=sha256_file(archive),
        robot_urdf_sha256=sha256_file(robot_urdf),
    )
