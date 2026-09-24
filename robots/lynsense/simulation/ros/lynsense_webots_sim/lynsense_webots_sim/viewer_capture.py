from __future__ import annotations

import os
from pathlib import Path
from typing import Any


CAMERA_NAME = "viewer_front_camera"
SAMPLE_PERIOD_MS = 96


class ViewerCapture:
    """Publish complete, replaceable image files from the controller thread."""

    def __init__(self, robot: Any, directory: Path) -> None:
        self.robot = robot
        self.directory = directory
        self.directory.mkdir(parents=True, exist_ok=True)
        self.camera = robot.getDevice(CAMERA_NAME)
        if self.camera is None:
            raise RuntimeError(f"missing viewer camera: {CAMERA_NAME}")
        self.camera.enable(SAMPLE_PERIOD_MS)
        self.next_sample_s = SAMPLE_PERIOD_MS / 1000.0
        self.stopped = False

    def sample(self, sim_time_s: float) -> None:
        if self.stopped:
            return
        if (self.directory / "stopped").exists():
            self.camera.disable()
            self.stopped = True
            return
        if sim_time_s < self.next_sample_s:
            return
        self.next_sample_s = sim_time_s + SAMPLE_PERIOD_MS / 1000.0
        if self.camera.getImage() is None:
            return
        for view in ("scene", "robot"):
            temporary = self.directory / f".{view}.jpg"
            if view == "scene":
                self.robot.exportImage(str(temporary), 85)
            elif self.camera.saveImage(str(temporary), 85) != 0:
                raise RuntimeError("first-person camera image export failed")
            if not temporary.is_file() or temporary.stat().st_size == 0:
                raise RuntimeError(f"empty {view} image export")
        for view in ("scene", "robot"):
            os.replace(self.directory / f".{view}.jpg", self.directory / f"{view}.jpg")


def capture_from_environment(robot: Any) -> ViewerCapture | None:
    directory = os.environ.get("LYNSENSE_VIEWER_CAPTURE_DIR")
    return ViewerCapture(robot, Path(directory)) if directory else None
