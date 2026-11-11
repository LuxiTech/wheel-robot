import hashlib
import json
from pathlib import Path

import numpy as np
from _robot_model import capture_model, selected_model
from wheel_motion_core.kinematics import RobotModel
from wheel_motion_core.sdk_kinematics import SDKKinematics

ROOT = Path(__file__).resolve().parents[2]


def test_selected_urdf_matches_new_archive_manifest_and_old_asset_is_preserved():
    resources = selected_model(ROOT)
    source = json.loads(resources["source"].read_text())
    assert "claw1.4" in str(resources["urdf"])
    assert hashlib.sha256(resources["urdf"].read_bytes()).hexdigest() == source["urdf_sha256"]
    old = json.loads((ROOT / "config/robots/r680_lift_openarm_claw1.3.source.json").read_text())
    assert hashlib.sha256((ROOT / old["urdf"]).read_bytes()).hexdigest() == old["urdf_sha256"]


def test_historical_capture_keeps_original_model_while_new_capture_retains_selection():
    old = json.loads((ROOT / "config/robots/r680_lift_openarm_claw1.3.source.json").read_text())
    resources = capture_model(ROOT, {"urdf_sha256": old["urdf_sha256"]})
    assert resources["urdf"] == ROOT / old["urdf"]
    assert resources["sdk_chains"].name == "sdk_arm_chains.json"
    selected = selected_model(ROOT)
    saved = {key: str(path.relative_to(ROOT)) for key, path in selected.items()}
    assert capture_model(ROOT, {"model_resources": saved}) == selected


def test_new_sdk_snapshot_preserves_left_origin_and_uses_both_urdf_shoulders():
    resources = selected_model(ROOT)
    robot = RobotModel(resources["urdf"])
    sdk = SDKKinematics(resources["sdk_chains"])
    old = SDKKinematics(ROOT / "config/robots/calibration/sdk_arm_chains.json")
    carriage_T_origin = robot.transform(
        "openarm_left_base_link", {}, base="lift_carriage_link"
    ) @ np.linalg.inv(old.transform("openarm_left_base_link"))
    for side in ("left", "right"):
        urdf_base = robot.transform(f"openarm_{side}_base_link", {}, base="lift_carriage_link")
        expected = np.linalg.inv(carriage_T_origin) @ urdf_base
        np.testing.assert_allclose(sdk.transform(f"openarm_{side}_base_link"), expected, atol=1e-12)
    np.testing.assert_allclose(
        sdk.transform("openarm_left_base_link"), old.transform("openarm_left_base_link"), atol=1e-12
    )


def test_only_wrist_camera_joints_changed_between_supplied_versions():
    old = RobotModel(
        ROOT / "assets/robots/r680_lift_openarm_claw1.3/r680_lift_openarm_robot_viewer.urdf"
    )
    new = RobotModel(selected_model(ROOT)["urdf"])
    assert old.joints.keys() == new.joints.keys()
    changed = []
    for name, joint in old.joints.items():
        if not np.array_equal(joint["t"], new.joints[name]["t"]):
            changed.append(name)
        np.testing.assert_array_equal(joint["axis"], new.joints[name]["axis"])
        assert joint["bounds"] == new.joints[name]["bounds"]
    assert set(changed) == {"left_wrist_eb435i_mount_joint", "right_wrist_eb435i_mount_joint"}
