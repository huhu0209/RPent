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

from pathlib import Path

import imageio.v3 as iio
import numpy as np
import pytest

from robots.lynsense.isaac.render_evidence import validate_render_evidence


def _write_png(path: Path, pixels: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    iio.imwrite(path, pixels, extension=".png")


def _write_variant_png(path: Path, changed_pixels: int, changed_value: int) -> None:
    pixels = np.zeros((180, 320, 3), dtype=np.uint8)
    pixels.flat[3 : (changed_pixels + 1) * 3 : 3] = changed_value
    _write_png(path, pixels)


def test_three_nonblank_pngs_pass(tmp_path: Path) -> None:
    y, x = np.mgrid[:180, :320]
    pixels = ((x + y) % 256).astype(np.uint8)
    pixels = np.stack((pixels, pixels, pixels), axis=-1)
    for name in ("scene-initial.png", "scene-action.png", "scene-final.png"):
        _write_png(tmp_path / name, pixels)

    result = validate_render_evidence(tmp_path)
    assert result["minimum_pixels"] == 180
    assert result["all_nonblank"] is True


def test_blank_png_fails(tmp_path: Path) -> None:
    pixels = np.full((180, 320, 3), 17, dtype=np.uint8)
    for name in ("scene-initial.png", "scene-action.png", "scene-final.png"):
        _write_png(tmp_path / name, pixels)

    with pytest.raises(Exception, match="blank"):
        validate_render_evidence(tmp_path)


def test_rgba_png_passes_with_color_statistics(tmp_path: Path) -> None:
    y, x = np.mgrid[:180, :320]
    gray = ((x + y) % 256).astype(np.uint8)
    pixels = np.stack((gray, gray, gray, np.full_like(gray, 255)), axis=-1)
    for name in ("scene-initial.png", "scene-action.png", "scene-final.png"):
        _write_png(tmp_path / name, pixels)

    assert validate_render_evidence(tmp_path)["all_nonblank"] is True


def test_missing_or_undersized_image_fails(tmp_path: Path) -> None:
    pixels = np.zeros((160, 320, 3), dtype=np.uint8)
    pixels[0, 0] = 255
    _write_png(tmp_path / "scene-initial.png", pixels)
    _write_png(tmp_path / "scene-action.png", pixels)

    with pytest.raises(Exception, match="scene-final.png"):
        validate_render_evidence(tmp_path)


@pytest.mark.parametrize(
    ("pixels", "message"),
    [
        (np.zeros((180, 320, 2), dtype=np.uint8), "RGB or RGBA"),
        (
            np.full((180, 320, 3), np.nan, dtype=np.float32),
            "invalid pixels",
        ),
        (np.full((180, 320, 3), -1, dtype=np.int16), "invalid pixels"),
    ],
)
def test_invalid_image_data_fails(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    pixels: np.ndarray,
    message: str,
) -> None:
    for name in ("scene-initial.png", "scene-action.png", "scene-final.png"):
        (tmp_path / name).touch()
    monkeypatch.setattr(iio, "imread", lambda _path: pixels)

    with pytest.raises(Exception, match=message):
        validate_render_evidence(tmp_path)


def test_low_std_with_enough_changed_pixels_fails(tmp_path: Path) -> None:
    for name in ("scene-initial.png", "scene-action.png", "scene-final.png"):
        _write_variant_png(tmp_path / name, changed_pixels=1_000, changed_value=1)

    with pytest.raises(Exception, match="blank"):
        validate_render_evidence(tmp_path)


def test_sufficient_std_with_too_few_changed_pixels_fails(
    tmp_path: Path,
) -> None:
    for name in ("scene-initial.png", "scene-action.png", "scene-final.png"):
        _write_variant_png(tmp_path / name, changed_pixels=999, changed_value=255)

    with pytest.raises(Exception, match="blank"):
        validate_render_evidence(tmp_path)
