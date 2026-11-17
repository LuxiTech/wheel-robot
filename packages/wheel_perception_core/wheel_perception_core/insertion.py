"""Signed rim/rail clearances in one optical frame; no robot control."""

import numpy as np


def insertion_clearance(
    rim_points_camera,
    rail_points_camera,
    camera_up,
    *,
    clearance_m,
    uncertainty_m,
    rim_confirmed=False,
    geometry_confirmed=False,
    depth_semantics_confirmed=False,
    maximum_rim_span_m=0.01,
):
    """Positive travel is downward until the lowest rail point reaches clearance.

    Rim points must describe the local TOP lip, not the wall or box floor.
    Rail points must bound the actual rail surface. Optical Z alone is not
    height. A positive gap is diagnostic until all correspondence gates pass.
    """
    rim = np.asarray(rim_points_camera, float)
    rail = np.asarray(rail_points_camera, float)
    up = np.asarray(camera_up, float)
    if (
        rim.ndim != 2
        or rail.ndim != 2
        or rim.shape[1:] != (3,)
        or rail.shape[1:] != (3,)
        or len(rim) < 30
        or len(rail) < 1
        or not np.isfinite(rim).all()
        or not np.isfinite(rail).all()
        or np.any(rim[:, 2] <= 0)
        or up.shape != (3,)
        or not np.isfinite(up).all()
        or not np.isclose(np.linalg.norm(up), 1)
    ):
        raise ValueError("Finite optical-frame geometry and unit up vector required")
    if any(
        type(value) is not bool
        for value in (rim_confirmed, geometry_confirmed, depth_semantics_confirmed)
    ):
        raise ValueError("Correspondence gates must be explicit booleans")
    if (
        not np.isfinite([clearance_m, uncertainty_m, maximum_rim_span_m]).all()
        or clearance_m < 0
        or uncertainty_m < 0
        or maximum_rim_span_m <= 0
    ):
        raise ValueError("Invalid insertion margins")
    low, high = np.percentile(rim @ up, [5, 95])
    rail_lowest = float(np.min(rail @ up))
    # Highest plausible local lip / lowest rail is conservative for collision.
    gap = float(rail_lowest - high)
    remaining = gap - clearance_m - uncertainty_m
    blockers = []
    for accepted, reason in (
        (rim_confirmed, "local_top_rim_correspondence_unconfirmed"),
        (geometry_confirmed, "optical_to_rail_correspondence_unconfirmed"),
        (depth_semantics_confirmed, "aligned_depth_metric_semantics_unconfirmed"),
        (bool(high - low <= maximum_rim_span_m), "rim_height_spread_too_large"),
    ):
        if accepted is not True:
            blockers.append(reason)
    return dict(
        gap_rail_above_rim_m=gap,
        remaining_downward_travel_m=float(remaining),
        rim_height_p05_p95_camera_m=[float(low), float(high)],
        lowest_rail_height_camera_m=rail_lowest,
        clearance_m=float(clearance_m),
        uncertainty_m=float(uncertainty_m),
        candidate_height_condition_reached=bool(remaining <= 0),
        descent_geometry_accepted=bool(not blockers and remaining > 0),
        closure_permitted=False,
        blockers=blockers,
        closure_note="Rail height alone does not establish hook insertion, bilateral alignment or capture",
    )
