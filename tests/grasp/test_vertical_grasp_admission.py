"""Vertical CAD tool targets and read-only descent admission checks."""

import ast
import copy
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from test_visual_grasp_flow import planner
from wheel_perception_core.targets import visual_target

ROOT = Path(__file__).resolve().parents[2]


def test_tilted_box_preserves_contact_xy_but_tool_and_descent_are_ground_vertical():
    box = np.eye(4)
    box[:3, :3] = Rotation.from_euler("xyz", [0.09, -0.07, 1.5]).as_matrix()
    target = visual_target(
        target_id="box",
        pair_id="frame",
        object_pose_origin=box,
        rules=dict(width=0.445, height=0.18, inset=0.007, below_top=0.001),
        height_um=0,
        quality={},
        source_blockers=[],
    )
    p = planner()
    resolved = p.resolve_target(target)
    for side in ("left", "right"):
        begin = np.array(resolved["pregrasp"][side])
        end = np.array(resolved["grasp"][side])
        np.testing.assert_allclose(begin[:3, 0], [0, 0, 1], atol=1e-12)
        np.testing.assert_allclose(begin[:2, 3], end[:2, 3], atol=1e-12)
        np.testing.assert_allclose(begin[:3, :3], end[:3, :3], atol=1e-12)
        assert begin[2, 3] > end[2, 3]
        # CAD rail underside -X must face the ground, not a generic TCP Z.
        palm = p.model.urdf.transform(
            f"openarm_{side}_ee_base_link", {}, base=f"openarm_{side}_link7"
        )
        normal_tool = p.mounts[side][:3, :3].T @ palm[:3, :3] @ np.array([-1.0, 0.0, 0.0])
        np.testing.assert_allclose(begin[:3, :3] @ normal_tool, [0, 0, -1], atol=1e-12)
        # Wrist installation must be inverted, not dropped to make wrists vertical.
        wrist = p.wrist_targets(target, "pregrasp", 0)[side]
        np.testing.assert_allclose(wrist @ p.mounts[side], begin, atol=1e-12)


def validator():
    # Execute the actual gateway method without importing unavailable ROS bindings.
    path = ROOT / "ros2_ws/src/wheel_moveit_control/scripts/planning_gateway.py"
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "Gateway")
    method = next(
        n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "validate_insertion"
    )
    namespace = dict(np=np, copy=copy, json=json)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), "exec"), namespace)
    return namespace["validate_insertion"]


@pytest.mark.parametrize("fail_at", [None, 2])
def test_descent_checks_opening_and_all_ik_samples_before_acceptance(tmp_path, fail_at):
    fingers = [f"openarm_{s}_finger_joint1" for s in ("left", "right")]
    state = SimpleNamespace(
        joint_state=SimpleNamespace(name=["j"] + fingers, position=[0.0, 0.02, 0.02])
    )
    calls = []

    def ik(previous, targets):
        calls.append(targets)
        if len(calls) == fail_at:
            raise RuntimeError("NO_IK_SOLUTION")
        result = copy.deepcopy(previous)
        result.joint_state.position[0] += 0.01
        return result

    checked = []
    gateway = SimpleNamespace(
        names=["j"],
        ik=ik,
        valid=lambda state: checked.append(copy.deepcopy(state)),
        current_plan_run=tmp_path,
    )
    begin = np.eye(4)
    end = begin.copy()
    end[2, 3] = -0.006
    data = dict(
        targets={s: begin.tolist() for s in ("left", "right")},
        insertion_check=dict(
            targets={s: end.tolist() for s in ("left", "right")},
            open_fingers=dict.fromkeys(fingers, 0.04),
        ),
    )
    if fail_at:
        with pytest.raises(RuntimeError, match="sample 2/3"):
            validator()(gateway, state, data)
        assert not (tmp_path / "insertion_validation.json").exists()
    else:
        assert validator()(gateway, state, data)["accepted"]
        assert len(calls) == 3
    assert len(checked) == 9
    assert state.joint_state.position == [0.0, 0.02, 0.02]


def test_opening_collision_rejects_before_descent_ik(tmp_path):
    fingers = [f"openarm_{s}_finger_joint1" for s in ("left", "right")]
    state = SimpleNamespace(
        joint_state=SimpleNamespace(name=["j"] + fingers, position=[0.0, 0.02, 0.02])
    )

    def valid(candidate):
        if candidate.joint_state.position[1] > 0.025:
            raise RuntimeError("opening collision")

    def ik(*args):
        pytest.fail("IK should not run after an opening collision")

    gateway = SimpleNamespace(names=["j"], ik=ik, valid=valid, current_plan_run=tmp_path)
    matrix = np.eye(4).tolist()
    data = dict(
        targets=dict(left=matrix, right=matrix),
        insertion_check=dict(
            targets=dict(left=matrix, right=matrix), open_fingers=dict.fromkeys(fingers, 0.04)
        ),
    )
    with pytest.raises(RuntimeError, match="opening collision"):
        validator()(gateway, state, data)
    assert not (tmp_path / "insertion_validation.json").exists()
