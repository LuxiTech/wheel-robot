"""Depth order and alpha endpoints for the diagnostic solid-mesh overlay."""

import numpy as np
from grasp.render_standard_gripper_overlay import DepthRaster, blend


def test_nearest_surface_wins_independent_of_triangle_submission_order():
    screen = np.array([[1, 1], [7, 1], [1, 7]], float)
    results = []
    for order in [(2.0, 1.0), (1.0, 2.0)]:
        raster = DepthRaster(8, 8)
        for depth in order:
            raster.triangle(
                screen,
                np.full(3, depth),
                [20, 50, 200] if depth == 1 else [200, 50, 20],
                int(depth),
            )
        assert raster.labels[2, 2] == 1
        assert raster.depth[2, 2] == 1
        assert raster.labels[7, 7] == -1
        results.append(raster.color)
    np.testing.assert_array_equal(*results)


def test_interpolated_camera_depth_is_perspective_correct():
    raster = DepthRaster(4, 4)
    raster.triangle(
        np.array([[0, 0], [4, 0], [0, 4]], float), np.array([1.0, 2.0, 4.0]), [1, 2, 3], 0
    )
    # Pixel center(0.5,0.5) has screen barycentrics(0.75,0.125,0.125).
    np.testing.assert_allclose(raster.depth[0, 0], 1 / (0.75 + 0.125 / 2 + 0.125 / 4))
    assert not np.isclose(raster.depth[0, 0], 0.75 + 0.125 * 2 + 0.125 * 4)


def test_opacity_zero_restores_original_pixels_and_clear_regions_are_untouched():
    photo = np.full((2, 2, 3), 95, np.uint8)
    layer = np.zeros((2, 2, 4), np.uint8)
    layer[0, 0] = [20, 100, 220, 255]
    np.testing.assert_array_equal(blend(photo, layer, 0), photo)
    opaque = blend(photo, layer, 1)
    np.testing.assert_array_equal(opaque[0, 0], layer[0, 0, :3])
    np.testing.assert_array_equal(opaque[1, 1], photo[1, 1])
