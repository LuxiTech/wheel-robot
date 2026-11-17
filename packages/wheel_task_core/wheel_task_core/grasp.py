"""Arm assignment shared by vision and motion; no robot access."""

from wheel_task_core.transforms import pose

SIDES = ("left", "right")


def assign_candidates(grasp, pregrasp):
    """Assign opposite candidates in SDK arm_origin by +Y=left; never image labels."""
    if len(grasp) != 2 or set(grasp) != set(pregrasp):
        raise ValueError("Exactly two matching grasp/pregrasp candidates required")
    keys = sorted(grasp, key=lambda key: pose(grasp[key])[1, 3], reverse=True)
    if pose(grasp[keys[0]])[1, 3] - pose(grasp[keys[1]])[1, 3] < 0.05:
        raise ValueError("Opposite rims do not establish a distinct SDK left/right assignment")
    return {
        side: dict(candidate=key, grasp=pose(grasp[key]), pregrasp=pose(pregrasp[key]))
        for side, key in zip(SIDES, keys)
    }
