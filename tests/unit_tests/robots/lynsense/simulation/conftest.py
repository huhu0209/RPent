from __future__ import annotations

import sys
from pathlib import Path

PACKAGE_PARENT = (
    Path(__file__).resolve().parents[5]
    / "robots"
    / "lynsense"
    / "simulation"
    / "ros"
    / "lynsense_webots_sim"
)
sys.path.insert(0, str(PACKAGE_PARENT))
