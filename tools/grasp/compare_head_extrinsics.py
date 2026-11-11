#!/usr/bin/env python3
"""Compare calibrated projections on post-zero, paired recorded RGB and joint feedback."""

import argparse
import hashlib
import html
import json
import sys
import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path

import cv2
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from _bootstrap import configure

configure(ROOT)
from wheel_motion_core.gripper import finger_displacement
from wheel_motion_core.kinematics import RobotModel
from wheel_motion_core.sdk_kinematics import SDKKinematics
from wheel_motion_core.unified_model import registered_camera
from wheel_perception_core.extrinsics import HeadExtrinsics
from wheel_perception_core.geometry import prepare_input
from wheel_perception_core.rgbd import Observation
from wheel_perception_core.visualization import rasterize_triangles


def fixed_origin(element):
    result = np.eye(4)
    if element is not None:
        result[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
        result[:3, :3] = Rotation.from_euler(
            "xyz", np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
        ).as_matrix()
    return result


def ideal_pixels(pixels, intrinsics):
    """Inverse Brown deprojection, for measured RGB corners, at unit optical Z."""
    k = np.asarray(intrinsics["K"]).reshape(3, 3)
    pixels = np.asarray(pixels)
    x, y = ((pixels - [k[0, 2], k[1, 2]]) / [k[0, 0], k[1, 1]]).T
    k1, k2, p1, p2, k3 = intrinsics["D"]
    radius = x * x + y * y
    radial = 1 + k1 * radius + k2 * radius**2 + k3 * radius**3
    assert intrinsics["distortion_model"] == "realsense_2"
    undistorted = np.array(
        [
            x * radial + 2 * p1 * x * y + p2 * (radius + 2 * x * x),
            y * radial + p1 * (radius + 2 * y * y) + 2 * p2 * x * y,
        ]
    ).T
    return undistorted * [k[0, 0], k[1, 1]] + [k[0, 2], k[1, 2]]


def project(points, transform, k):
    camera_points = (transform @ np.c_[points, np.ones(len(points))].T).T[:, :3]
    homogeneous = (k @ camera_points.T).T
    return camera_points, homogeneous[:, :2] / homogeneous[:, 2, None]


def statistics(values):
    return {
        "count": len(values),
        "rms_px": float(np.sqrt(np.mean(np.square(values)))),
        "median_px": float(np.median(values)),
        "p95_px": float(np.percentile(values, 95)),
        "max_px": float(np.max(values)),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--session", type=Path, default=ROOT / "outputs/calibration-rezero/20261103/left_j3"
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        default=ROOT / "config/robots/calibration/head_camera_left_j3_rezero_20261103.json",
    )
    parser.add_argument(
        "--baseline",
        type=Path,
        default=ROOT / "config/robots/calibration/head_camera_rezero_20261102.json",
    )
    parser.add_argument("--max-poses", type=int, default=6)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    calibration = HeadExtrinsics(args.calibration)
    baseline = HeadExtrinsics(args.baseline)
    pairs_path = args.session / "pairs.json"
    assert (
        hashlib.sha256(pairs_path.read_bytes()).hexdigest()
        == calibration.config["source"]["sha256"]
    )
    pairs = json.loads(pairs_path.read_text())
    frames = json.loads((args.session / "frames.json").read_text())
    rebase_path = args.session / "coordinate_rebase.json"
    rebase = json.loads(rebase_path.read_text()) if rebase_path.exists() else None
    joint_note = (
        f"历史左J3已按写零前后实测差值+{rebase['delta_deg']:.6f}°换算到新零点；"
        "其余15个角度保持原值，没有按图拟合关节偏移，也不将该换算用于当前SDK反馈。"
        if rebase
        else "关节角不置零、不反号、不加补偿。"
    )
    urdf = ROOT / "assets/robots/r680_lift_openarm_claw1.3/r680_lift_openarm_robot_viewer.urdf"
    model = RobotModel(urdf)
    # Recorded measured poses can exceed command limits. Evaluate unchanged
    # angles in a separate offline geometry instance, never clip or alter q.
    geometry_model = deepcopy(model)
    for specification in geometry_model.joints.values():
        specification["bounds"] = None
    sdk = SDKKinematics(ROOT / "config/robots/calibration/sdk_arm_chains.json")
    profile = json.loads((ROOT / "config/robots/calibration/gripper_v1.json").read_text())
    xml = ET.parse(urdf).getroot()
    board = np.array([[x * 0.01, y * 0.01, 0] for y in range(8) for x in range(11)])
    mount = np.asarray(calibration.config["T_ee_board"])
    measurements = {"new": [], "baseline": []}
    fk_errors = []
    outside_limits = {}
    rows = []
    for pair in pairs:
        frame = frames[pair["frame_index"]]
        values = sdk.from_sdk_vector(frame["q"])
        for name, value in values.items():
            low, high = model.joints[name]["bounds"]
            if not low <= value <= high:
                outside_limits.setdefault(name, []).append(
                    {"frame_index": pair["frame_index"], "measured_rad": value}
                )
        wrist = sdk.transform("openarm_left_base_link") @ geometry_model.transform(
            "openarm_left_link7", values, base="openarm_left_base_link"
        )
        fk_errors.append(float(np.max(np.abs(wrist - np.array(pair["A"])))))
        observed = ideal_pixels(pair["corners"], pair["intrinsics"])
        k = np.asarray(pair["intrinsics"]["K"]).reshape(3, 3)
        row = {"frame_index": pair["frame_index"], "group": pair["group"]}
        for key, selected in [("new", calibration), ("baseline", baseline)]:
            camera = selected.camera_to_origin(pair["height_m"] * 1e6)
            _, predicted = project(board, np.linalg.inv(camera) @ wrist @ mount, k)
            errors = np.linalg.norm(predicted - observed, axis=1)
            measurements[key].extend(errors.tolist())
            row[key + "_rms_px"] = float(np.sqrt(np.mean(errors**2)))
        rows.append(row)
    assert max(fk_errors) < 1e-12
    meshes = {}
    for side in ["left", "right"]:
        for suffix in ["link4", "link5", "link6", "ee_base_link", "ee_link1", "ee_link2", "camera"]:
            name = f"{side}_wrist_eb435i_link" if suffix == "camera" else f"openarm_{side}_{suffix}"
            meshes[name] = []
            for visual in xml.findall(f"link[@name='{name}']/visual"):
                element = visual.find("geometry/mesh")
                if element is None:
                    continue
                geometry = trimesh.load(
                    urdf.parent / element.get("filename"), force="scene"
                ).to_geometry()
                vertices = geometry.vertices * np.fromstring(element.get("scale", "1 1 1"), sep=" ")
                meshes[name].append((vertices, geometry.faces, fixed_origin(visual.find("origin"))))
    groups = sorted({p["group"] for p in pairs})
    groups = [
        groups[i]
        for i in np.unique(
            np.rint(np.linspace(0, len(groups) - 1, min(args.max_poses, len(groups)))).astype(int)
        )
    ]
    samples = []
    page_samples = []
    for group in groups:
        candidates = [p for p in pairs if p["group"] == group]
        pair = candidates[len(candidates) // 2]
        frame = frames[pair["frame_index"]]
        rgb = cv2.cvtColor(cv2.imread(str(args.session / frame["image"])), cv2.COLOR_BGR2RGB)
        # This replay only needs RGB geometry; no depth is invented or used for a 3D measurement.
        observation = Observation(
            {
                "intrinsics": frame["intrinsics"],
                "frame_id": frame["frame"],
                "depth": {"value_semantics": "unavailable_rgb_only_replay"},
                "replay": True,
            },
            rgb,
            np.zeros(rgb.shape[:2], np.float32),
            0,
        )
        prepared = prepare_input(observation)
        photo = cv2.cvtColor(prepared.rgb, cv2.COLOR_RGB2BGR)
        values = sdk.from_sdk_vector(frame["q"])
        height_um = pair["height_m"] * 1e6
        values["lift_joint"] = height_um * 1e-6 + 0.2
        mappings = {}
        for side, index in [("left", 15), ("right", 7)]:
            config = profile["sides"][side]
            mappings[side] = finger_displacement(
                frame["q"][index],
                offset_rad=config["offset_rad"],
                opening_sign=config["opening_sign"],
                closed_feedback_rad=config.get("closed_feedback_rad"),
                closed_displacement_m=config.get("closed_displacement_m"),
                bounds=model.joints[f"openarm_{side}_finger_joint1"]["bounds"],
            )
            if mappings[side]["within_urdf_limits"]:
                values[f"openarm_{side}_finger_joint1"] = mappings[side][
                    "estimated_finger_displacement_m"
                ]
        filenames = {}
        for key, selected in [("new", calibration), ("baseline", baseline)]:
            camera = registered_camera(
                geometry_model, sdk, selected, height_um, values["lift_joint"]
            )
            image = photo.copy()
            for name, parts in meshes.items():
                side = "left" if "left" in name else "right"
                if "ee_link" in name and not mappings[side]["within_urdf_limits"]:
                    continue
                color = (
                    (0, 160, 255)
                    if "ee_base" in name
                    else (255, 90, 230)
                    if "eb435i" in name
                    else (255, 200, 0)
                    if "ee_link" in name
                    else (0, 210, 90)
                )
                transform = np.linalg.inv(camera) @ geometry_model.transform(name, values)
                mask = np.zeros(photo.shape[:2], np.uint8)
                for vertices, faces, local in parts:
                    points, pixels = project(vertices, transform @ local, prepared.k)
                    valid = np.all(points[faces, 2] > 0.01, axis=1) & np.all(
                        np.abs(pixels[faces]) < 10000, axis=(1, 2)
                    )
                    mask |= rasterize_triangles(
                        np.rint(pixels[faces[valid]]).astype(np.int32), photo.shape
                    )
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(image, contours, -1, color, 1)
            filename = f"group_{group:02d}_{key}.jpg"
            cv2.imwrite(str(args.output / filename), image)
            filenames[key] = filename
        raw_name = f"group_{group:02d}_photo.jpg"
        cv2.imwrite(str(args.output / raw_name), photo)
        sample = {
            "group": group,
            "frame_index": pair["frame_index"],
            "source_image": str(args.session / frame["image"]),
            "height_um": height_um,
            "q": frame["q"],
            "opening_mapping": mappings,
            "omitted_fingers": [s for s in mappings if not mappings[s]["within_urdf_limits"]],
            "images": filenames,
            "photo": raw_name,
        }
        samples.append(sample)
        page_samples.append(
            f'<h3>姿态组 {group}，原始帧 {pair["frame_index"]}</h3><div class="grid">'
            f'<figure><figcaption>旧外参文件对照</figcaption><img src="{filenames["baseline"]}"></figure>'
            f'<figure><figcaption>新外参：标零后参考</figcaption><img src="{filenames["new"]}"></figure></div>'
        )
        print("RENDERED", group, flush=True)
    metrics = {key: statistics(value) for key, value in measurements.items()}
    report = {
        "commands_sent": False,
        "replay": True,
        "session": str(args.session),
        "calibration": str(calibration.path),
        "calibration_sha256": hashlib.sha256(calibration.path.read_bytes()).hexdigest(),
        "baseline": str(baseline.path),
        "baseline_scope": "Previous calibration with current-epoch joint angles; file-switch diagnostic, not historical fit accuracy",
        "coordinate_rebase": rebase,
        "board_mount_for_both": "Same newly fitted T_ee_board; reference diagnostic, not independent metrology",
        "board_corner_metrics_rectified": metrics,
        "paired_frames": len(pairs),
        "urdf_vs_recorded_sdk_max_error": max(fk_errors),
        "offline_measured_angles_outside_command_limits": outside_limits,
        "recorded_angles_clipped": False,
        "samples": samples,
        "whole_robot_registration": "Selected SDK extrinsic registered through left base; right CAD base relationship remains an installation assumption",
        "acceptance": "Visual comparison only; complete physical gripper alignment and grasp accuracy are not automatically accepted",
    }
    (args.output / "report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    )
    description = f"""<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>标零后新外参夹爪叠图检查</title>
<style>body{{font:17px/1.65 sans-serif;margin:30px auto;max-width:1350px;padding:0 24px;color:#17212d}}
.grid{{display:grid;grid-template-columns:1fr 1fr;gap:18px}}figure{{margin:0}}img{{width:100%}}.note{{background:#fff2d4;padding:16px}}
@media(max-width:850px){{.grid{{grid-template-columns:1fr}}}}pre{{overflow:auto;background:#f4f6fa;padding:15px}}</style>
<h1>新外参是否改善夹爪叠图</h1><p>使用保存的同期图像、实测关节角、升降反馈和每帧内参。
{html.escape(joint_note)}没有发送真机指令。两列使用完全相同的姿态和几何，只替换相机外参。</p>
<p>绿线：J4～J6原始外观网格；橙线：固定夹爪主体；紫线：腕部相机外壳；青线：换算有效的夹指。</p>
<div class="note">左列旧外参仅作文件切换对照，不等同于它在原始角度上的拟合误差。
左夹指若开合换算超出URDF范围则不画，不以假定张开量代替反馈。
右肩关系沿用整机URDF安装定义，相机通过左臂根注册；这不是右臂独立实物验收。</div>
<h2>同一棋盘格安装关系下的参考重投影</h2><p>{len(pairs)}张标零后配对图像、每张88个角点。
旧文件RMS {metrics["baseline"]["rms_px"]:.2f}px；新文件RMS {metrics["new"]["rms_px"]:.2f}px。
使用新标定拟合的同一个棋盘格安装矩阵，属于拟合内参考，不能替代夹爪轮廓或抓取TCP的实物验收。</p>
<p>新参考高度179494μm；各样本按各自升降高度计算，未重复加升降量。</p>
{"".join(page_samples)}<h2>追溯记录</h2><p><a href="report.json">原始角度、外参指纹、缺省夹指和计算记录</a></p>
<pre>{html.escape(str(calibration.path))}</pre></html>"""
    (args.output / "report.html").write_text(description)
    print(json.dumps({"output": str(args.output), "metrics": metrics}, ensure_ascii=False))


if __name__ == "__main__":
    main()
