import numpy as np
from wheel_perception_core.visualization import project_points


def test_projection_extrinsics_and_invalid_depth():
    camera = np.eye(4)
    camera[:3, 3] = [1, 2, 3]
    k = np.array([[100, 0, 50], [0, 100, 40], [0, 0, 1]])
    p = project_points([[1, 2, 5], [2, 2, 5], [1, 2, 2]], camera, k, (80, 100))
    assert p[0]["pixel"] == [50, 40] and p[0]["visible"]
    assert p[1]["pixel"] == [100, 40] and not p[1]["visible"]
    assert p[2]["pixel"] is None and p[2]["reason"] == "behind_camera"


def test_mesh_silhouette_unions_duplicate_and_overlapping_triangles():
    from wheel_perception_core.visualization import rasterize_triangles

    first = np.array([[1, 1], [9, 1], [1, 9]])
    second = np.array([[3, 3], [11, 3], [3, 11]])
    mask = rasterize_triangles([first, first, second], (14, 14, 3))
    assert mask[2, 2] == 255  # Duplicate faces must not cancel their interiors.
    assert mask[4, 4] == 255  # The overlap belongs to the union.
    assert mask[4, 8] == 255
    assert mask[12, 12] == 0
