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

import os
import subprocess
import sys
from pathlib import Path


def test_isaac_readme_documents_scope_commands_and_gates() -> None:
    readme = Path("robots/lynsense/isaac/README.md").read_text(encoding="utf-8")
    assert "Isaac Sim `2023.1.*`" in readme
    assert "--isaac-python" in readme
    assert "--visual-review" in readme
    assert "--verify-milestone" in readme
    assert "`77`" in readme
    assert "not a pass" in readme
    assert "`visual-review.json`" in readme
    assert "Webots baseline" in readme
    assert "URDF_robot_1.tar.gz" in readme
    assert "ROS Domain 3" in readme
    assert ".artifacts/lynsense-isaac" in readme


def test_probe_module_has_a_direct_cli_entrypoint() -> None:
    probe_path = Path("robots/lynsense/isaac/probe.py").resolve()
    completed = subprocess.run(
        [sys.executable, str(probe_path), "--help"],
        capture_output=True,
        text=True,
        env={**os.environ, "PYTHONPATH": str(Path(".").resolve())},
        check=False,
    )
    assert completed.returncode == 0
    assert "--isaac-python" in completed.stdout
