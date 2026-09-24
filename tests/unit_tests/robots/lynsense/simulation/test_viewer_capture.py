from pathlib import Path

import pytest

from lynsense_webots_sim.viewer_capture import (
    CAMERA_NAME, SAMPLE_PERIOD_MS, ViewerCapture, capture_from_environment,
)


class Camera:
    def enable(self, period):
        self.period = period

    def disable(self):
        self.period = 0

    def getImage(self):
        return b"image"

    def saveImage(self, path, quality):
        Path(path).write_bytes(b"first-person")
        return 0


class Robot:
    def __init__(self):
        self.camera = Camera()
        self.export_count = 0

    def getDevice(self, name):
        assert name == CAMERA_NAME
        return self.camera

    def exportImage(self, path, quality):
        self.export_count += 1
        Path(path).write_bytes(b"3D-only")


def test_no_viewer_environment_means_no_camera_activation(monkeypatch):
    monkeypatch.delenv("LYNSENSE_VIEWER_CAPTURE_DIR", raising=False)
    assert capture_from_environment(object()) is None


def test_export_only_complete_scene_and_robot_frames(tmp_path):
    robot = Robot()
    capture = ViewerCapture(robot, tmp_path)
    assert robot.camera.period == SAMPLE_PERIOD_MS
    capture.sample(0.032)
    assert list(tmp_path.iterdir()) == []
    capture.sample(0.096)
    assert (tmp_path / "scene.jpg").read_bytes() == b"3D-only"
    assert (tmp_path / "robot.jpg").read_bytes() == b"first-person"
    capture.sample(0.128)
    assert robot.export_count == 1
    assert sorted(p.name for p in tmp_path.iterdir()) == ["robot.jpg", "scene.jpg"]


def test_failed_export_does_not_replace_last_complete_frame(tmp_path):
    robot = Robot()
    capture = ViewerCapture(robot, tmp_path)
    capture.sample(0.1)
    robot.exportImage = lambda path, quality: Path(path).write_bytes(b"new-scene")
    robot.camera.saveImage = lambda *args: -1
    with pytest.raises(RuntimeError, match="first-person"):
        capture.sample(0.3)
    assert (tmp_path / "robot.jpg").read_bytes() == b"first-person"
    assert (tmp_path / "scene.jpg").read_bytes() == b"3D-only"


def test_capture_stops_after_terminal_finalization(tmp_path):
    robot = Robot()
    capture = ViewerCapture(robot, tmp_path)
    (tmp_path / "stopped").touch()
    capture.sample(0.1)
    capture.sample(0.3)
    assert robot.export_count == 0
    assert robot.camera.period == 0


def test_missing_camera_fails_explicitly(tmp_path):
    robot = Robot()
    robot.camera = None
    with pytest.raises(RuntimeError, match="missing viewer camera"):
        ViewerCapture(robot, tmp_path)
