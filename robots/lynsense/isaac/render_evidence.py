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

from robots.lynsense.isaac.contracts import SCREENSHOT_NAMES, IsaacProbeError

MINIMUM_WIDTH = 320
MINIMUM_HEIGHT = 180
MINIMUM_VARIANT_PIXELS = 1_000
MINIMUM_STANDARD_DEVIATION = 1.0


def validate_render_evidence(run_directory: Path) -> dict[str, object]:
    """Require three decoded screenshots with visible spatial variation."""
    import imageio.v3 as iio
    import numpy as np

    paths = [run_directory / name for name in SCREENSHOT_NAMES]
    missing = [path.name for path in paths if not path.is_file()]
    if missing:
        raise IsaacProbeError(f"required render evidence is missing: {missing[0]}")

    file_results: dict[str, dict[str, object]] = {}
    for name in SCREENSHOT_NAMES:
        path = run_directory / name
        try:
            pixels = iio.imread(path)
        except (OSError, ValueError) as exc:
            raise IsaacProbeError(
                f"render evidence is not a valid image: {name}"
            ) from exc

        if pixels.ndim != 3 or pixels.shape[2] not in (3, 4):
            raise IsaacProbeError(f"render evidence must be RGB or RGBA: {name}")
        height, width = pixels.shape[:2]
        if width < MINIMUM_WIDTH or height < MINIMUM_HEIGHT:
            raise IsaacProbeError(
                f"render evidence is too small: {name} ({width}x{height})"
            )
        if not np.isfinite(pixels).all() or (pixels < 0).any():
            raise IsaacProbeError(f"render evidence has invalid pixels: {name}")

        rgb_pixels = pixels[..., :3]
        upper_left = rgb_pixels[0, 0]
        standard_deviation = float(np.std(rgb_pixels, dtype=np.float64))
        changed_pixel_count = int(
            np.count_nonzero(np.any(rgb_pixels != upper_left, axis=-1))
        )
        if (
            standard_deviation < MINIMUM_STANDARD_DEVIATION
            or changed_pixel_count < MINIMUM_VARIANT_PIXELS
        ):
            raise IsaacProbeError(f"render evidence is blank: {name}")

        file_results[name] = {
            "width": width,
            "height": height,
            "standard_deviation": standard_deviation,
            "changed_pixel_count": changed_pixel_count,
        }

    return {
        "minimum_pixels": MINIMUM_HEIGHT,
        "all_nonblank": True,
        "images": file_results,
    }
