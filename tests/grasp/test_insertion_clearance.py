import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from wheel_perception_core.insertion import insertion_clearance


def evaluate(rim, rail, up, **kwargs):
    return insertion_clearance(
        rim,
        rail,
        up,
        clearance_m=0.01,
        uncertainty_m=0.002,
        rim_confirmed=True,
        geometry_confirmed=True,
        depth_semantics_confirmed=True,
        **kwargs,
    )


def test_world_height_is_invariant_to_camera_tilt():
    rim = np.tile([0.0, 0.0, 0.3], (40, 1))
    rail = np.array([[0.0, 0.0, 0.2], [0.02, 0, 0.21]])
    up = np.array([0.0, 0, -1])
    expected = evaluate(rim, rail, up)
    rotation = Rotation.from_euler("xyz", [12, -18, 9], degrees=True).as_matrix()
    actual = evaluate(rim @ rotation.T, rail @ rotation.T, rotation @ up)
    assert expected["gap_rail_above_rim_m"] == pytest.approx(0.09)
    assert actual["remaining_downward_travel_m"] == pytest.approx(0.078)
    assert actual["descent_geometry_accepted"] is True


def test_downward_translation_reaches_stop_before_contact():
    rim = np.tile([0.0, 0.0, 0.3], (40, 1))
    rail = np.array([[0, 0, 0.21]])
    # Moving the camera and rail down makes the fixed rim approach the camera.
    result = evaluate(rim - [0, 0, 0.079], rail, [0, 0, -1])
    assert result["candidate_height_condition_reached"] is True
    assert result["descent_geometry_accepted"] is False
    assert result["closure_permitted"] is False


def test_candidate_correspondence_does_not_authorize_motion():
    result = insertion_clearance(
        np.tile([0, 0, 0.3], (40, 1)),
        [[0, 0, 0.21]],
        [0, 0, -1],
        clearance_m=0.01,
        uncertainty_m=0.002,
    )
    assert len(result["blockers"]) == 3
    assert result["descent_geometry_accepted"] is False


def test_wall_or_mixed_depths_fail_height_spread_gate():
    rim = np.column_stack([np.zeros(40), np.zeros(40), np.linspace(0.3, 0.48, 40)])
    result = evaluate(rim, [[0, 0, 0.2]], [0, 0, -1])
    assert "rim_height_spread_too_large" in result["blockers"]
    assert result["descent_geometry_accepted"] is False


@pytest.mark.parametrize("bad_depth", [float("nan"), float("inf"), 0.0, -0.1])
def test_invalid_rim_depth_is_rejected(bad_depth):
    rim = np.tile([0.0, 0.0, 0.3], (40, 1))
    rim[0, 2] = bad_depth
    with pytest.raises(ValueError):
        evaluate(rim, [[0, 0, 0.2]], [0, 0, -1])
