"""Offline checks use the supplied model; no ROS or motor commands."""

import importlib.util
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml
from test_ik_executor import HIGH, LOW, Q
from wheel_execution.moveit_driver import MoveItTrajectoryDriver
from wheel_motion_core.live_kinematics import SDKAlignedModel

ROOT = Path(__file__).resolve().parents[2]


def test_bilateral_targets_are_relative_to_their_own_sdk_base():
    model = SDKAlignedModel(
        ROOT / "assets/robots/r680_lift_openarm_claw1.4/r680_lift_openarm_robot_viewer.urdf",
        ROOT / "config/robots/calibration/sdk_arm_chains_claw14.json",
        LOW,
        HIGH,
    )
    driver = MoveItTrajectoryDriver.__new__(MoveItTrajectoryDriver)
    driver.planner = SimpleNamespace(model=model)
    height = -0.137159
    values = model.values(Q, height)
    targets = {s: model.transform(f"openarm_{s}_ee_base_link", values) for s in ("left", "right")}
    relative = driver.base_targets(targets, height)
    for side in targets:
        expected = model.urdf.transform(
            f"openarm_{side}_link7", values, base=f"openarm_{side}_base_link"
        )
        np.testing.assert_allclose(relative[side], expected, atol=1e-12)


def test_generated_model_uses_actual_sdk_limits_and_preserves_original(tmp_path):
    spec = importlib.util.spec_from_file_location(
        "moveit_config_generator", ROOT / "tools/grasp/generate_moveit_config.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    selected = json.loads((ROOT / "config/robots/robot_model.json").read_text())
    source = ROOT / selected["urdf"]
    original = source.read_bytes()
    low, high = list(LOW), list(HIGH)
    for index in (6, 14):
        low[index], high[index] = -np.deg2rad(52), np.deg2rad(52)
    state = dict(
        arms=dict(position=list(Q)),
        lift=dict(current_um=-137159),
        arm_startup=dict(joint_control=dict(lower=low, upper=high)),
    )
    folder = module.generate(tmp_path / "config", state)
    assert source.read_bytes() == original
    robot = ET.parse(folder / "robot.urdf").getroot()
    for side in ("left", "right"):
        limit = robot.find(f"joint[@name='openarm_{side}_joint7']/limit")
        assert float(limit.get("upper")) == np.deg2rad(52)
        assert float(limit.get("lower")) == -np.deg2rad(52)
    original_robot = ET.fromstring(original)
    base = robot.find("link[@name='base_link']")
    assert base.find("collision/geometry/box") is None
    collision_file = base.find("collision/geometry/mesh").get("filename")
    assert collision_file.endswith("derived_collision/base/r680_base_collision.stl")
    assert base.find("visual/geometry/mesh").get("filename").endswith("meshes/visual/r680_base.stl")
    assert base.find("collision/origin").attrib == base.find("visual/origin").attrib
    # Every other collision shape and installation pose is unchanged. Only
    # generated file:// resource paths are normalized for this comparison.
    for link in original_robot.findall("link"):
        if link.get("name") == "base_link":
            continue
        generated = robot.find(f"link[@name='{link.get('name')}']")
        for expected, actual in zip(
            link.findall("collision"), generated.findall("collision"), strict=True
        ):
            for mesh in expected.findall(".//mesh"):
                mesh.set("filename", "file://" + str(source.parent / mesh.get("filename")))
            assert ET.tostring(expected).strip() == ET.tostring(actual).strip()
    assert robot.find("ros2_control/hardware/plugin").text == "mock_components/GenericSystem"
    controller = yaml.safe_load((folder / "controllers.yaml").read_text())["arm_controller"][
        "ros__parameters"
    ]
    assert len(set(controller["joints"])) == 14
    assert controller["open_loop_control"] is False
    assert all(controller["constraints"][n]["trajectory"] > 0 for n in controller["joints"])
    context = json.loads((folder / "context.json").read_text())
    assert abs(context["passive_joint_positions"]["lift_joint"] - 0.062841) < 1e-12


def test_gripper_stop_remains_sdk_scoped_and_does_not_call_moveit(monkeypatch):
    import threading

    import pytest
    import trimesh
    from wheel_sdk_bridge.http_client import SDKError

    posts = []
    joints = SimpleNamespace(
        lock=threading.RLock(),
        pending_id="gripper-owned",
        snapshot=lambda: {"arm_startup": {"joint_control": {"request_id": "gripper-owned"}}},
        http=SimpleNamespace(request=lambda *args: posts.append(args) or {"success": True}),
    )
    monkeypatch.setattr(trimesh, "load", lambda *a, **k: SimpleNamespace(vertices=[], faces=[]))
    driver = MoveItTrajectoryDriver(
        joints,
        lambda: {},
        SimpleNamespace(),
        gateway_url="http://unused.invalid",
        crate_mesh="unused",
    )
    driver.trajectory.stop("gripper-owned")
    assert len(posts) == 1
    assert posts[0][:2] == ("POST", "/api/arm_startup")
    assert json.loads(posts[0][2]["detail"]) == {"request_id": "gripper-owned"}
    with pytest.raises(SDKError, match="unowned"):
        driver.trajectory.stop("another-request")
    assert len(posts) == 1


def test_moveit_failure_stop_does_not_depend_on_valid_feedback(monkeypatch):
    import trimesh

    posts = []
    joints = SimpleNamespace(
        pending_id=None,
        http=SimpleNamespace(request=lambda *args: posts.append(args) or {"success": True}),
    )
    monkeypatch.setattr(trimesh, "load", lambda *a, **k: SimpleNamespace(vertices=[], faces=[]))
    driver = MoveItTrajectoryDriver(
        joints,
        lambda: {},
        SimpleNamespace(),
        gateway_url="http://unused.invalid",
        crate_mesh="unused",
    )
    driver.moveit_request_id = driver.request_id = "wheel-jtc-owned"
    driver.hold()
    assert json.loads(posts[0][2]["detail"]) == {"request_id": "wheel-jtc-owned"}
    assert driver.cancelled.is_set()
