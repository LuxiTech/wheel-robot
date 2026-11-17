"""Explicit handoff from vision inference to conventional motion planning."""

import numpy as np
from wheel_task_core.grasp import assign_candidates
from wheel_task_core.transforms import pose

from wheel_perception_core.geometry import rim_targets


def visual_target(
    *,
    target_id,
    pair_id,
    object_pose_origin,
    rules,
    height_um,
    quality,
    source_blockers,
    observed_at_monotonic=None,
    max_p95_depth_error_m=0.020,
):
    """Return serializable hook poses in SDK arm_origin, preserving all blockers.

    Pose is CAD bottom/opening-up. Its yaw determines opposite walls; +Y in
    SDK coordinates determines hand assignment. Pixel A/B is not an arm label.
    """
    if not target_id or not pair_id or not np.isfinite(height_um):
        raise ValueError("Target, image pair and measured lift height required")
    if not np.isfinite(max_p95_depth_error_m) or not 0 < max_p95_depth_error_m <= 0.03:
        raise ValueError("Invalid visual depth-tail tolerance")
    options = dict(rules)
    options["tool_convention"] = "pinch_y"
    grasp, pre = rim_targets(pose(object_pose_origin), **options)
    assigned = assign_candidates(grasp, pre)
    rim_top = {}
    object_frame = pose(object_pose_origin)
    for side, item in assigned.items():
        local = np.linalg.inv(object_frame) @ item["grasp"][:, 3]
        rim_top[side] = (
            item["grasp"][:3, 3]
            + object_frame[:3, 2] * options["below_top"]
            + object_frame[:3, 0] * options["inset"] * np.sign(local[0])
        ).tolist()
    # Grasp points still come from the selected CAD/rules. Only orientation
    # and approach direction change: tool X is world up, pinch Y is horizontal
    # across the selected rim. Do not apply this rotation to the wrist directly.
    for item in assigned.values():
        tool = item["grasp"]
        y = tool[:3, 1].copy()
        y[2] = 0.0
        if np.linalg.norm(y) < 1e-6:
            raise ValueError("Selected rim has no horizontal pinch direction")
        y /= np.linalg.norm(y)
        # Supplied CAD rail underside normal is tool -X, not tool -Z.
        # Thus hooks descend along -X while the rail remains horizontal.
        x = np.array([0.0, 0.0, 1.0])
        tool[:3, :3] = np.column_stack((x, y, np.cross(x, y)))
        item["pregrasp"] = tool.copy()
        item["pregrasp"][2, 3] += float(options.get("clearance", 0.06))
    blockers = list(source_blockers)
    if "surface_rms_m" in quality:
        rms = quality.get("surface_rms_m")
        p95 = quality.get("surface_p95_m")
        accepted = (
            quality.get("accepted") is True
            and rms is not None
            and p95 is not None
            and np.isfinite([rms, p95]).all()
            and 0 <= rms <= 0.006
            and 0 <= p95 <= max_p95_depth_error_m
        )
    else:
        median = quality.get("median_depth_error_m")
        p95 = quality.get("p95_depth_error_m")
        iou = quality.get("silhouette_iou")
        accepted = (
            median is not None
            and p95 is not None
            and iou is not None
            and np.isfinite([median, p95, iou]).all()
            and 0 <= median <= 0.006
            and median <= p95 <= max_p95_depth_error_m
            and 0.5 <= iou <= 1
        )
    if not accepted:
        blockers.append("visual_surface_quality_rejected")
    if quality.get("metric_depth_semantics_validated") is not True:
        blockers.append("metric_depth_semantics_unvalidated")
    if observed_at_monotonic is None:
        blockers.append("saved_target_is_replay_not_live")
    elif not np.isfinite(observed_at_monotonic):
        raise ValueError("Invalid local monotonic observation time")
    blockers = list(dict.fromkeys(blockers))
    return dict(
        schema="wheel_visual_grasp_target_v1",
        target_id=target_id,
        pair_id=pair_id,
        frame_id="arm_origin",
        units="metres",
        tool_frame="cad_contact_midpoint_pinch_y",
        orientation_policy="ground_vertical_pinch_y",
        approach_axis_origin=[0.0, 0.0, -1.0],
        tool_reference="CAD contact midpoint; not an independently measured hook insertion tip",
        observed_at_monotonic=observed_at_monotonic,
        height_um=float(height_um),
        object_pose=pose(object_pose_origin).tolist(),
        rim_top=rim_top,
        insertion_reference="sliding_rail_underside",
        rail_stop_clearance_m=0.01,
        pregrasp_vertical_clearance_m=0.03,
        alignment_clearance_m=0.03,
        quality=dict(quality),
        assignment={side: value["candidate"] for side, value in assigned.items()},
        grasp={side: value["grasp"].tolist() for side, value in assigned.items()},
        pregrasp={side: value["pregrasp"].tolist() for side, value in assigned.items()},
        rules=options,
        diagnostic_only=bool(blockers),
        execution_blockers=blockers,
    )
