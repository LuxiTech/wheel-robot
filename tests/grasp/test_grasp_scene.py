"""CAD material and frame checks; no hardware acceptance or motor calls."""

import hashlib
import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from test_ik_executor import Q
from wheel_motion_core.collision import Box
from wheel_motion_core.grasp_scene import CADMaterial, GraspSceneCollision, fixed_body_exclusions
from wheel_motion_core.kinematics import RobotModel
from wheel_motion_core.sdk_kinematics import SDKKinematics

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def material():
    mesh = ROOT / "assets/objects/crate/model.obj"
    transform = np.diag([1.0, -1.0, -1.0, 1.0])
    transform[2, 3] = 0.18
    return CADMaterial(mesh, hashlib.sha256(mesh.read_bytes()).hexdigest(), transform)


def box_at(point, half=(0.01, 0.01, 0.005)):
    transform = np.eye(4)
    transform[:3, 3] = point
    return Box("probe", transform, half)


def test_open_crate_cavity_is_not_filled_bounding_box(material):
    assert not material.overlaps(box_at([0, 0, 0.1]), np.eye(4), 0.0)
    assert material.overlaps(box_at([0, 0, 0.005]), np.eye(4), 0.0)
    assert not material.overlaps(box_at([0, 0, 0.3]), np.eye(4), 0.0)


def test_cad_collision_is_invariant_to_common_rigid_transform(material):
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler("xyz", [13, 29, 37], degrees=True).as_matrix()
    transform[:3, 3] = [0.2, -0.15, 0.4]
    for point in [[0, 0, 0.005], [0, 0, 0.1]]:
        original = box_at(point)
        moved = Box(original.name, transform @ original.transform, original.half)
        assert material.overlaps(original, np.eye(4), 0.005) == material.overlaps(
            moved, transform, 0.005
        )


def test_material_hash_rejected_before_mesh_loading():
    with pytest.raises(ValueError, match="fingerprint"):
        CADMaterial(ROOT / "assets/objects/crate/model.obj", "wrong", np.eye(4))


def test_only_same_or_adjacent_rigid_groups_are_excluded():
    def joint(parent, child, kind):
        return dict(parent=parent, child=child, kind=kind)

    joints = dict(
        shell=joint("base", "cover", "fixed"),
        arm=joint("cover", "arm", "revolute"),
        palm=joint("arm", "palm", "fixed"),
        finger_a=joint("palm", "a", "prismatic"),
        finger_b=joint("palm", "b", "prismatic"),
    )
    ignored = {frozenset(pair) for pair in fixed_body_exclusions(joints)}
    assert frozenset(("base", "cover")) in ignored
    assert frozenset(("base", "palm")) in ignored
    assert frozenset(("a", "b")) not in ignored
    assert frozenset(("base", "a")) not in ignored


def test_scene_uses_measured_opening_and_preserves_asset_wheel_joints(material):
    urdf = ROOT / "assets/robots/r680_lift_openarm_claw1.4/r680_lift_openarm_robot_viewer.urdf"
    chains = ROOT / "config/robots/calibration/sdk_arm_chains_claw14.json"
    mapping = json.loads((ROOT / "config/robots/calibration/gripper_claw14.json").read_text())[
        "sides"
    ]
    target_pose = np.eye(4)
    target_pose[:3, 3] = [10, 10, 10]
    target = dict(frame_id="arm_origin", object_pose=target_pose, height_um=100000)
    scene = GraspSceneCollision(urdf, chains, material, target, mapping)
    scene.refresh(Q)
    assert scene.fingers["openarm_right_finger_joint1"] != 0
    scene(SDKKinematics.from_sdk_vector(Q) | {"lift_joint": 0.1})
    assert scene.checked_configurations == 1
    assert len(scene.wheel_sweeps) == 3
    assert RobotModel(urdf).joints["front_wheel_joint"]["kind"] == "continuous"
    for name, geometry in scene.robot_check.parts:
        if name in scene.wheel_sweeps:
            assert np.allclose(geometry.half, geometry.half[0])
    assert not scene(SDKKinematics.from_sdk_vector(Q) | {"lift_joint": 0.101})
    assert scene.last_collision == ("fixed_height_context_changed",)
