from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation
from wheel_perception_core.pipeline import refine_surface_pose, surface_quality


def test_refinement_reduces_depth_model_error_without_task_prior():
    k = np.array([[100.0, 0, 40], [0, 100.0, 40], [0, 0, 1]])
    yy, xx = np.indices((80, 80))
    rays = np.stack([xx, yy, np.ones_like(xx)], axis=-1) @ np.linalg.inv(k).T
    truth = np.eye(4)
    truth[:3, 3] = [0.02, 0.01, 1.0]
    truth[:3, :3] = Rotation.from_euler("xyz", [0.3, 0.5, 0.2]).as_matrix()
    origin = -truth[:3, 3] @ truth[:3, :3]
    directions = rays @ truth[:3, :3]
    half = np.array([0.2, 0.15, 0.11])
    a = (-half - origin) / directions
    b = (half - origin) / directions
    near = np.minimum(a, b).max(-1)
    far = np.maximum(a, b).min(-1)
    mask = (near > 0) & (near < far)
    depth = np.where(mask, near, 0)
    inp = SimpleNamespace(k=k, depth=depth)
    colliders = [dict(position=[0, 0, 0], size=(2 * half).tolist())]
    guess = truth.copy()
    guess[:3, 3] += [0.008, -0.007, 0.006]
    guess[:3, :3] = Rotation.from_rotvec([0.02, -0.02, 0.015]).as_matrix() @ guess[:3, :3]
    before = surface_quality(inp, mask, guess, colliders)["surface_rms_m"]
    result, details = refine_surface_pose(inp, mask, guess, colliders)
    after = surface_quality(inp, mask, result, colliders)["surface_rms_m"]
    assert after < before * 0.1 and after < 0.001
    assert np.linalg.norm(result[:3, 3] - truth[:3, 3]) < 0.002


def test_quality_uses_whole_mask_instead_of_overweighting_boundary_sample():
    depth = np.ones((100, 100))
    depth[0, 0] = 1.2
    inp = SimpleNamespace(k=np.array([[100.0, 0, 50], [0, 100.0, 50], [0, 0, 1]]), depth=depth)
    result = surface_quality(
        inp,
        np.ones_like(depth, dtype=bool),
        np.eye(4),
        [dict(position=[0, 0, 1.001], size=[20, 20, 0.002])],
    )
    assert result["points"] == 10000
    assert abs(result["surface_rms_m"] - 0.198 / 100) < 1e-9
    assert result["accepted"]
