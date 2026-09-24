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

import io
import tarfile
from pathlib import Path

import pytest

from robots.lynsense.isaac.archive import ArchiveInventory, extract_archive


def _write_archive(path: Path, members: list[tuple[str, bytes]]) -> None:
    with tarfile.open(path, "w:gz") as archive:
        for name, payload in members:
            info = tarfile.TarInfo(name)
            if name.endswith("/"):
                info.type = tarfile.DIRTYPE
                info.mode = 0o700
                archive.addfile(info)
                continue
            info.size = len(payload)
            archive.addfile(info, io.BytesIO(payload))


def test_archive_extracts_expected_robot_and_records_inventory(tmp_path: Path) -> None:
    archive = tmp_path / "robot.tar.gz"
    _write_archive(
        archive,
        [
            ("URDF_robot_1/", b""),
            ("URDF_robot_1/robot.urdf", b"<robot/>"),
            ("URDF_robot_1/pkg/mesh.stl", b"mesh"),
        ],
    )
    destination = tmp_path / "out"
    inventory = extract_archive(archive, destination)

    assert isinstance(inventory, ArchiveInventory)
    assert inventory.file_count == 2
    assert inventory.total_bytes == 12
    assert inventory.archive_digest.startswith("sha256:")
    assert inventory.robot_urdf_sha256.startswith("sha256:")
    assert inventory.robot_urdf == destination / "URDF_robot_1" / "robot.urdf"
    assert inventory.robot_urdf.read_bytes() == b"<robot/>"


def test_archive_rejects_missing_robot_urdf(tmp_path: Path) -> None:
    archive = tmp_path / "robot.tar.gz"
    _write_archive(archive, [("URDF_robot_1/other.txt", b"x")])
    with pytest.raises(Exception, match="robot.urdf"):
        extract_archive(archive, tmp_path / "out")


def _special_archive(path: Path, name: str, kind: str) -> None:
    with tarfile.open(path, "w:gz") as archive:
        info = tarfile.TarInfo(name)
        info.type = {
            "symlink": tarfile.SYMTYPE,
            "hardlink": tarfile.LNKTYPE,
            "device": tarfile.CHRTYPE,
        }[kind]
        info.linkname = "/outside" if kind == "symlink" else "URDF_robot_1/other"
        archive.addfile(info)


@pytest.mark.parametrize("kind", ["symlink", "hardlink", "device"])
def test_archive_rejects_special_entries(tmp_path: Path, kind: str) -> None:
    archive = tmp_path / "robot.tar.gz"
    _special_archive(archive, f"URDF_robot_1/{kind}", kind)
    with pytest.raises(Exception, match=kind):
        extract_archive(archive, tmp_path / "out")


def test_archive_rejects_parent_and_absolute_paths(tmp_path: Path) -> None:
    for name in ("../escape.txt", "/absolute.txt", "URDF_robot_1/a\\b.txt"):
        archive = tmp_path / f"{len(name)}.tar.gz"
        _write_archive(archive, [(name, b"x")])
        with pytest.raises(Exception, match="unsafe"):
            extract_archive(archive, tmp_path / "out")


def test_archive_rejects_entry_and_size_limits(tmp_path: Path, monkeypatch) -> None:
    from robots.lynsense.isaac import archive as archive_module

    archive = tmp_path / "robot.tar.gz"
    _write_archive(archive, [("URDF_robot_1/robot.urdf", b"<robot/>")])
    monkeypatch.setattr(archive_module, "MAX_ARCHIVE_ENTRIES", 0)
    with pytest.raises(Exception, match="entries"):
        extract_archive(archive, tmp_path / "out")

    monkeypatch.setattr(archive_module, "MAX_ARCHIVE_ENTRIES", 100_000)
    with pytest.raises(Exception, match="2 GiB"):
        extract_archive(
            archive,
            tmp_path / "another-out",
            max_uncompressed_bytes=1,
        )
