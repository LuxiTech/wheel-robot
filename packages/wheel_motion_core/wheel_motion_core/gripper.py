"""Official v1 linear approximation, with explicit motor coordinates and limits."""

import math


def finger_displacement(
    feedback_rad,
    *,
    offset_rad,
    opening_sign,
    bounds,
    closed_feedback_rad=None,
    closed_displacement_m=None,
):
    """Convert motor feedback to nominal travel with explicit closed references.

    SDK feedback + offset is raw motor radians for metadata. With a supplied
    closed reference, travel depends on SDK angle increments about that
    reference; the raw motor offset is not added to the closed anchor.
    The legacy no-anchor branch alone assumes raw motor zero is closed.
    opening_sign is +1 for increasing angle on opening, -1 otherwise.
    """
    if (
        opening_sign not in (-1, 1)
        or not all(math.isfinite(v) for v in [feedback_rad, offset_rad, *bounds])
        or bounds[0] > bounds[1]
    ):
        raise ValueError("Invalid gripper mapping inputs")
    raw = feedback_rad + offset_rad
    if (closed_feedback_rad is None) != (closed_displacement_m is None):
        raise ValueError("Both closed reference coordinates required")
    if closed_feedback_rad is not None:
        if not all(math.isfinite(v) for v in [closed_feedback_rad, closed_displacement_m]):
            raise ValueError("Invalid closed reference")
        displacement = (
            closed_displacement_m
            + 0.044 * opening_sign * (feedback_rad - closed_feedback_rad) / 1.0472
        )
        assumption = "User-provided approximate SDK closed reference; CAD closed displacement is nominal, not measured"
    else:
        displacement = 0.044 * opening_sign * raw / 1.0472
        assumption = (
            "Legacy diagnostic: raw motor zero corresponds to official URDF displacement zero"
        )
    return dict(
        feedback_rad=feedback_rad,
        offset_rad=offset_rad,
        raw_motor_rad=raw,
        opening_sign=opening_sign,
        estimated_finger_displacement_m=displacement,
        within_urdf_limits=bounds[0] <= displacement <= bounds[1],
        urdf_bounds_m=list(bounds),
        approximate=True,
        closed_feedback_rad=closed_feedback_rad,
        closed_displacement_m=closed_displacement_m,
        assumption=assumption,
        source="https://github.com/enactic/openarm_ros2/pull/49/files",
    )
