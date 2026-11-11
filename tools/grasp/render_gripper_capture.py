"""Current stationary pose overlay, preserving measured angles and estimated opening."""

import argparse
import hashlib
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
from _robot_model import capture_model

configure(ROOT)
from wheel_motion_core.kinematics import RobotModel
from wheel_motion_core.sdk_kinematics import SDKKinematics
from wheel_motion_core.unified_model import registered_camera
from wheel_perception_core.extrinsics import HeadExtrinsics
from wheel_perception_core.geometry import prepare_input
from wheel_perception_core.rgbd import load_observation
from wheel_perception_core.visualization import rasterize_triangles


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--show-unclipped-fingers",
        action="store_true",
        help="Show finite approximate finger coordinates outside CAD limits without clipping; diagnostic only",
    )
    parser.add_argument(
        "--omit-wrist-cameras",
        action="store_true",
        help="Display the physically removed wrist cameras as absent; source URDF unchanged",
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        help="Explicit diagnostic candidate; active configuration is unchanged",
    )
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    capture = args.capture.resolve()
    report = json.loads((capture / "pose_report.json").read_text())
    state = json.loads((capture / "paired_joint_state.json").read_text())
    calibration = HeadExtrinsics(args.calibration or report["calibration_path"])
    resources = capture_model(ROOT, report)
    urdf = resources["urdf"]
    model = RobotModel(urdf)
    geometry = deepcopy(model)
    for joint in geometry.joints.values():
        joint["bounds"] = None
    sdk = SDKKinematics(resources["sdk_chains"])
    values = sdk.from_sdk_vector(state["arms"]["position"])
    height = report["height_um"]
    values["lift_joint"] = height * 1e-6 + 0.2
    camera = registered_camera(geometry, sdk, calibration, height, values["lift_joint"])
    prepared = prepare_input(load_observation(capture / "observation.npz"))
    photo = cv2.cvtColor(prepared.rgb, cv2.COLOR_RGB2BGR)
    overlay = photo.copy()
    xml = ET.parse(urdf).getroot()
    opening_notes = {}
    for side in ["left", "right"]:
        mapping = report["arms"][side]["official_approximation"]
        displacement = mapping["estimated_finger_displacement_m"]
        low, high = model.joints[f"openarm_{side}_finger_joint1"]["bounds"]
        distance = max(low - displacement, displacement - high, 0)
        # Source hardware zero is quantized near -0.000190738 rad; it maps to
        # about 8 micrometres below the nominal CAD closed limit on the left.
        # Show the unmodified computed value only within 20 micrometres of
        # that display boundary. This never permits a command or clips q.
        visible = bool(
            distance <= 0.00002 or (args.show_unclipped_fingers and np.isfinite(displacement))
        )
        opening_notes[side] = {
            "mapping": mapping,
            "displayed": visible,
            "distance_outside_nominal_limits_m": distance,
            "estimated_displacement_modified": False,
            "unclipped_outside_nominal_range_display": bool(visible and distance > 0.00002),
        }
        if visible:
            values[f"openarm_{side}_finger_joint1"] = displacement
        for suffix in ["link4", "link5", "link6", "ee_base_link", "ee_link1", "ee_link2", "camera"]:
            if args.omit_wrist_cameras and suffix == "camera":
                continue
            if suffix.startswith("ee_link") and not visible:
                continue
            name = f"{side}_wrist_eb435i_link" if suffix == "camera" else f"openarm_{side}_{suffix}"
            transform = np.linalg.inv(camera) @ geometry.transform(name, values)
            mask = np.zeros(photo.shape[:2], np.uint8)
            for visual in xml.findall(f"link[@name='{name}']/visual"):
                element = visual.find("geometry/mesh")
                if element is None:
                    continue
                mesh = trimesh.load(
                    urdf.parent / element.get("filename"), force="scene"
                ).to_geometry()
                local = np.eye(4)
                origin = visual.find("origin")
                if origin is not None:
                    local[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
                    local[:3, :3] = Rotation.from_euler(
                        "xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
                    ).as_matrix()
                vertices = mesh.vertices * np.fromstring(element.get("scale", "1 1 1"), sep=" ")
                points = (transform @ local @ np.c_[vertices, np.ones(len(vertices))].T).T[:, :3]
                homogeneous = (prepared.k @ points.T).T
                pixels = homogeneous[:, :2] / homogeneous[:, 2, None]
                faces = mesh.faces
                valid = np.all(points[faces, 2] > 0.01, axis=1) & np.all(
                    np.abs(pixels[faces]) < 10000, axis=(1, 2)
                )
                mask |= rasterize_triangles(
                    np.rint(pixels[faces[valid]]).astype(np.int32), photo.shape
                )
            color = (
                (0, 160, 255)
                if suffix == "ee_base_link"
                else (220, 70, 220)
                if suffix == "camera"
                else (40, 70, 240)
                if suffix.startswith("ee_link")
                else (50, 210, 70)
            )
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(overlay, contours, -1, color, 1)
    cv2.imwrite(str(output / "current_overlay.png"), overlay)
    cv2.imwrite(str(output / "current_photo.png"), photo)
    delta = calibration.camera_to_origin(height)[:3, 3] - calibration.reference[:3, 3]
    reference_height = calibration.config["reference_height_um"]
    calibration_label = "诊断候选外参" if args.calibration else "当前选定外参"
    opening_descriptions = []
    for side, label, index in [("left", "左", 15), ("right", "右", 7)]:
        note = opening_notes[side]
        displacement_mm = note["mapping"]["estimated_finger_displacement_m"] * 1000
        visibility = (
            f"按原估值绘制，超出CAD名义范围{note['distance_outside_nominal_limits_m'] * 1000:.3f}mm"
            if note["unclipped_outside_nominal_range_display"]
            else "已绘制夹指"
            if note["displayed"]
            else "超出名义范围，省略夹指"
        )
        opening_descriptions.append(
            f"{label}夹爪电机反馈{state['arms']['position'][index]:.6f}rad，"
            f"模型单指位移{displacement_mm:.3f}mm，{visibility}。"
        )
    summary = {
        "commands_sent": False,
        "capture_time_utc": report["created_utc"],
        "calibration_path": str(calibration.path),
        "calibration_sha256": hashlib.sha256(calibration.path.read_bytes()).hexdigest(),
        "height_um": height,
        "reference_height_um": calibration.config["reference_height_um"],
        "lift_compensation_translation_m": delta.tolist(),
        "formula": f"t(h) = t0 - lift_axis*(h_um-{reference_height})*1e-6; R(h)=R0",
        "lift_direction_and_scale": "Nominal unchanged unit-scale relationship; shaft projection trials do not establish metric TCP accuracy",
        "lift_axis_in_origin": calibration.axis.tolist(),
        "calibration_validation": calibration.config.get("validation_summary", {}),
        "whole_robot_camera_registration": "Left-base correspondence; candidate lift joint offset+0.2m cancels in camera/arm relative projection",
        "joint_feedback": state["arms"]["position"],
        "joint_convention": "Current measured SDK angles used directly; no historical zero conversion",
        "urdf_sha256": report["urdf_sha256"],
        "capture_source_sha256": report["source_sha256"],
        "opening": opening_notes,
        "pairing": report["pairing"],
        "image_max_age_ms": report["image_max_age_ms"],
        "geometry_display_only": True,
        "wrist_camera_meshes_omitted": args.omit_wrist_cameras,
        "show_unclipped_fingers": args.show_unclipped_fingers,
        "height_extrapolation_diagnostic": calibration.config.get(
            "height_extrapolation_diagnostic"
        ),
        "source_capture": str(capture),
    }
    (output / "current_report.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n"
    )
    extrapolation = calibration.config.get("height_extrapolation_diagnostic")
    diagnostic_note = (
        "<p><strong>当前高度超出原标定范围：本图按原升降公式外推，仅供诊断；正式标定范围未扩大。</strong></p>"
        if extrapolation
        else ""
    )
    if args.omit_wrist_cameras:
        diagnostic_note += "<p>腕部相机已拆除，本图不绘制其外壳；源URDF保持不变。</p>"
    page = f"""<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>夹爪叠图：{calibration_label}</title>
<style>body{{font:18px/1.65 sans-serif;color:#17212d;max-width:1400px;margin:28px auto;padding:0 20px}}.grid{{display:grid;grid-template-columns:1fr 1fr;gap:20px}}img{{width:100%}}figure{{margin:0}}.note{{background:#edf4ff;padding:16px}}code{{font-size:15px}}@media(max-width:900px){{.grid{{grid-template-columns:1fr}}}}</style>
<h1>保存位姿重新叠图：{calibration_label}</h1><p>采集UTC：{report["created_utc"]}。使用配对的图像、SDK实测关节角和升降反馈；本渲染工具只读保存数据，不发送运动指令。</p>
{diagnostic_note}
<div class="note">升降补偿沿用所选文件的轴向和单位比例，参考高度为{reference_height}μm。
当前高度为{height:.0f}μm，本次相机相对SDK臂原点的补偿为{(delta * 1000).round(3).tolist()}mm。
少量轴心投影检查不代表完整夹爪或抓取精度验收；显式候选文件不会覆盖当前配置。</div>
<p><code>t(h) = t0 − lift_axis × (h_um − {reference_height}) × 10⁻⁶；R(h) = R0</code></p>
<div class="grid"><figure><figcaption>本次无叠加实拍</figcaption><a href="current_photo.png"><img src="current_photo.png"></a></figure><figure><figcaption>本次计算叠图（点击放大）</figcaption><a href="current_overlay.png"><img src="current_overlay.png"></a></figure></div>
<p>绿线：机械臂原始外观网格；橙线：固定夹爪主体；紫线：腕部相机外壳；红线：夹指。</p>
<p>{"".join(opening_descriptions)}开合由已有传动近似换算，位移未按画面调整。
{"本图显式显示未裁剪的开合估值；超出CAD名义范围的部分只作几何诊断。" if args.show_unclipped_fingers else "允许最多0.020mm的名义边界偏差用于显示。"}原始角度和位移没有裁剪，不改变控制判断。</p>
<details><summary>参数与配对记录</summary><p>外参文件：{calibration.path}。</p><p>当前SDK实测角直接输入对应URDF关节，不再叠加历史录包的J3零点换算。</p><p>最近关节反馈与图像接收时间差{report["pairing"]["nearest_joint_image_delta_ms"]:.2f}ms，保持静止与反馈有效性检查；图像上限{report["image_max_age_ms"]:g}ms仅用于本次只读诊断。</p></details>
<p><a href="current_report.json">完整计算与升降补偿记录</a></p></html>"""
    (output / "report.html").write_text(page)
    print(
        json.dumps(
            {
                "report": str(output / "report.html"),
                "height_um": height,
                "compensation_mm": (delta * 1000).tolist(),
            },
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()
