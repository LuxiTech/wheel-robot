"""Independent SDK references for enum-2 pixel/ray conversion, not just shapes."""

from types import SimpleNamespace

import numpy as np
import pytest
from wheel_perception_core.geometry import prepare_input


@pytest.fixture
def reference_case():
    rs = pytest.importorskip("pyrealsense2")
    width, height = 640, 480
    k = np.array(
        [
            [387.5992126464844, 0, 318.66900634765625],
            [0, 387.0912780761719, 240.11669921875],
            [0, 0, 1],
        ]
    )
    coefficients = [
        -0.05630532652139664,
        0.060888826847076416,
        -0.00026793632423505187,
        -0.00022309062478598207,
        -0.019627751782536507,
    ]
    intrinsics = rs.intrinsics()
    intrinsics.width, intrinsics.height = width, height
    intrinsics.fx, intrinsics.fy = k[0, 0], k[1, 1]
    intrinsics.ppx, intrinsics.ppy = k[0, 2], k[1, 2]
    intrinsics.model = rs.distortion.inverse_brown_conrady
    intrinsics.coeffs = coefficients
    yy, xx = np.indices((height, width), dtype=np.float32)
    observation = SimpleNamespace(
        metadata={
            "intrinsics": {
                "K": k.ravel().tolist(),
                "D": coefficients,
                "distortion_model": "realsense_2",
            },
            "depth": {"value_semantics": "color_optical_z"},
            "frame_id": "head_rgb_color_optical",
        },
        # Artificial coordinate ramp, not a captured image: remapping this
        # exposes the exact sampled source coordinates in the first 2 channels.
        rgb=np.dstack([xx, yy, np.zeros_like(xx)]),
        depth=1.0 + (xx + width * yy) * 1e-6,
    )
    return rs, intrinsics, observation, k


def test_rectification_samples_sdk_projected_source_pixels(reference_case):
    rs, intrinsics, observation, k = reference_case
    result = prepare_input(observation)
    for y in np.linspace(25, 454, 10).round().astype(int):
        for x in np.linspace(25, 614, 12).round().astype(int):
            ray = [(x - k[0, 2]) / k[0, 0], (y - k[1, 2]) / k[1, 1], 1.0]
            expected = rs.rs2_project_point_to_pixel(intrinsics, ray)
            # OpenCV INTER_LINEAR has a 1/32-pixel interpolation grid.
            assert np.linalg.norm(result.rgb[y, x, :2] - expected) < 0.025


def test_rectified_pixels_deproject_back_to_their_pinhole_rays(reference_case):
    rs, intrinsics, observation, k = reference_case
    result = prepare_input(observation)
    for x, y in [(30, 30), (100, 200), (500, 200), (610, 450)]:
        sampled = result.rgb[y, x, :2].tolist()
        direction = rs.rs2_deproject_pixel_to_point(intrinsics, sampled, 1.0)
        projected = k @ np.array(direction)
        assert np.linalg.norm(projected[:2] / projected[2] - [x, y]) < 0.03


def test_depth_and_rgb_use_the_same_source_warp(reference_case):
    rs, intrinsics, observation, k = reference_case
    result = prepare_input(observation)
    for x, y in [(100, 200), (500, 200), (300, 400)]:
        ray = [(x - k[0, 2]) / k[0, 0], (y - k[1, 2]) / k[1, 1], 1.0]
        source_x, source_y = np.rint(rs.rs2_project_point_to_pixel(intrinsics, ray)).astype(int)
        assert result.depth[y, x] == observation.depth[source_y, source_x]
