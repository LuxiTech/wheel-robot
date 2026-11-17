#!/usr/bin/env python3
"""Fit a diagnostic seven-joint sine correction to one saved stationary pose.

A single pose cannot identify 21 coefficients per arm. The existing weak prior
selects a local solution; no fitted values are sent to hardware or production.
"""

import argparse
import base64
import copy
import datetime
import json
import sys
import xml.etree.ElementTree as ET
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
from grasp import compare_residual_models as comparison
from grasp import validate_residual_models as validation
from grasp.render_standard_gripper_overlay import DepthRaster, blend
from wheel_motion_core.kinematics import RobotModel
from wheel_motion_core.sdk_kinematics import SDKKinematics
from wheel_motion_core.unified_model import registered_camera
from wheel_perception_core.extrinsics import HeadExtrinsics


def save(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def replace_masks(row, output):
    """Fit the visible red jaw only; freeze its mesh identity before fitting."""
    # Current photo: left ee_link2 and right ee_link1 are the red inner jaws.
    # The other jaw overlaps yellow hardware; it is still drawn in solid mode
    # but has no independently verified pixel correspondence in this capture.
    index = 1 if row["side"] == "left" else 0
    row["fitted_mesh_link"] = f"openarm_{row['side']}_ee_link{index + 1}"
    row["parts"] = [row["parts"][index]]
    raster = DepthRaster(*row["photo"].shape[:2])
    for points, faces in row["parts"]:
        raster.mesh(points, faces, row["k"], [255, 100, 0], 1)
    covered = raster.labels >= 0
    edge = covered & ~cv2.erode(covered.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
    yy, xx = np.where(edge)
    camera_points = np.c_[xx + 0.5, yy + 0.5, np.ones(len(xx))] @ np.linalg.inv(row["k"]).T
    camera_points *= raster.depth[yy, xx, None]
    row["contour_points"] = validation.evenly(camera_points)
    # Reuse all three original red segmentation policies and the broad ROI.
    edge = validation.boundary(row["observed"]["nominal"])
    yy, xx = np.where(edge)
    row["observed_points"] = validation.evenly(np.c_[xx, yy])
    row["distance_to_observed"] = cv2.distanceTransform(
        (~edge).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE
    )
    roi = np.zeros(row["photo"].shape[:2], bool)
    x0, y0, x1, y1 = row["roi_xyxy"]
    roi[y0:y1, x0:x1] = True
    photo = row["photo"].copy()
    photo[~roi] //= 3
    contours, _ = cv2.findContours(
        row["observed"]["nominal"].astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    cv2.drawContours(photo, contours, -1, (0, 255, 0), 1)
    cv2.imwrite(str(output / (row["side"] + "_segmentation.png")), photo)


def render_layers(capture, calibration, models, output):
    record = json.loads((capture / "pose_report.json").read_text())
    state = json.loads((capture / "paired_joint_state.json").read_text())
    resources = capture_model(ROOT, record)
    geometry = RobotModel(resources["urdf"])
    for joint in geometry.joints.values():
        joint["bounds"] = None
    sdk = SDKKinematics(resources["sdk_chains"])
    xml = ET.parse(resources["urdf"]).getroot()
    values = sdk.from_sdk_vector(state["arms"]["position"])
    values["lift_joint"] = record["height_um"] * 1e-6 + 0.2
    camera = registered_camera(
        geometry, sdk, calibration, record["height_um"], values["lift_joint"]
    )
    from wheel_perception_core.geometry import prepare_input
    from wheel_perception_core.rgbd import load_observation

    prepared = prepare_input(load_observation(capture / "observation.npz"))
    photo = cv2.cvtColor(prepared.rgb, cv2.COLOR_RGB2BGR)
    height, width = photo.shape[:2]
    for side, offset in (("right", 0), ("left", 8)):
        model = models.get(side)
        if model is not None:
            data = dict(
                q=np.array([state["arms"]["position"][offset : offset + 7]]),
                h=np.array([record["height_um"] * 1e-6]),
                rays=np.zeros((1, 88, 2)),
                focal=np.ones((1, 2)),
            )
            fitted = model.predict(data)["virtual_q"][0]
            for index, angle in enumerate(fitted):
                values[f"openarm_{side}_joint{index + 1}"] = float(angle)
        values[f"openarm_{side}_finger_joint1"] = record["arms"][side]["official_approximation"][
            "estimated_finger_displacement_m"
        ]
    intrinsics = prepared.k.copy()
    intrinsics[:2] *= 2
    full, gripper = DepthRaster(height * 2, width * 2), DepthRaster(height * 2, width * 2)
    for side, side_id in (("right", 0), ("left", 1)):
        for suffix in [*(f"link{i}" for i in range(1, 8)), "ee_base_link", "ee_link1", "ee_link2"]:
            link = f"openarm_{side}_{suffix}"
            camera_link = np.linalg.inv(camera) @ geometry.transform(link, values)
            for visual in xml.findall(f"link[@name='{link}']/visual"):
                node = visual.find("geometry/mesh")
                if node is None:
                    continue
                mesh = trimesh.load(
                    resources["urdf"].parent / node.get("filename"), force="scene"
                ).to_geometry()
                local = np.eye(4)
                origin = visual.find("origin")
                if origin is not None:
                    local[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
                    local[:3, :3] = Rotation.from_euler(
                        "xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
                    ).as_matrix()
                vertices = mesh.vertices * np.fromstring(node.get("scale", "1 1 1"), sep=" ")
                points = validation.transform(vertices, camera_link @ local)
                color = (40, 150, 255) if suffix.startswith("ee_") else (220, 190, 40)
                full.mesh(points, mesh.faces, intrinsics, color, side_id)
                if suffix.startswith("ee_"):
                    gripper.mesh(points, mesh.faces, intrinsics, color, side_id)
    output.mkdir(exist_ok=True)
    cv2.imwrite(str(output / "photo.png"), photo)
    for name, raster in (("full", full), ("gripper", gripper)):
        layer = raster.rgba(width, height)
        cv2.imwrite(str(output / (name + "_layer.png")), layer)
        cv2.imwrite(str(output / (name + "_overlay.png")), blend(photo, layer, 0.45))
    shoulder_z = geometry.transform("openarm_left_link1", values)[2, 3]
    actual = (camera[2, 3] - shoulder_z) * 1000
    assert abs(actual - (325.742 - record["height_um"] / 1000)) < 1e-8
    return actual


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(exist_ok=False)
    calibration = HeadExtrinsics(args.calibration)
    rows = validation.load_capture(
        args.capture.resolve(), "train", calibration, output / "segmentation"
    )
    resources = capture_model(ROOT, json.loads((args.capture / "pose_report.json").read_text()))
    sdk = SDKKinematics(resources["sdk_chains"])
    context = comparison.Context(calibration.config, sdk)
    fitted_models, results = {}, []
    for row in rows:
        replace_masks(row, output)
        side = row["side"]
        side_context = copy.copy(context)
        side_context.chain = sdk.chains[side]
        side_context.joint_names = [f"openarm_{side}_joint{i + 1}" for i in range(7)]
        spec = comparison.Spec(side + "_current_sine", joint="sine")
        baseline = comparison.Model(side_context, comparison.Spec("baseline"))
        baseline.value = np.zeros(0)
        before = validation.evaluate_silhouettes([row], baseline, output / "before", side)[0]
        model, fit = validation.fit_contour([row], side_context, spec)
        after = validation.evaluate_silhouettes([row], model, output / "after", side)[0]
        fitted_models[side] = model
        offset = 8 if side == "left" else 0
        measured = np.array(row["measured_q16"])[offset : offset + 7]
        a = model.at(model.value, "joint_offsets")
        b = model.at(model.value, "joint_gains")
        c = model.at(model.value, "joint_sine_amplitudes")
        delta = a + b * measured + c * np.sin(measured)
        results.append(
            dict(
                side=side,
                fitted_mesh_link=row["fitted_mesh_link"],
                fit=fit,
                model=model.export(),
                before=before,
                after=after,
                measured_q_rad=measured.tolist(),
                virtual_delta_q_rad=delta.tolist(),
                coefficients=dict(a_rad=a.tolist(), b=b.tolist(), c_rad=c.tolist()),
                coefficient_basis_rank_per_joint=1,
                independent_poses=1,
                held_out_poses=0,
                robustness_validated=False,
            )
        )
        save(output / "results.json", results)
        print(
            side,
            "rank",
            fit["data_rank"],
            "/21",
            "before",
            before["metrics"]["nominal"],
            "after",
            after["metrics"]["nominal"],
            flush=True,
        )
    camera_height = render_layers(args.capture, calibration, {}, output / "baseline-solid")
    render_layers(args.capture, calibration, fitted_models, output / "fitted-solid")
    photo = rows[0]["photo"].copy()
    panels = []
    for prefix in ("before", "after"):
        panel = photo.copy()
        for row in rows:
            side = row["side"]
            if prefix == "after":
                model = fitted_models[side]
            else:
                model = comparison.Model(fitted_models[side].ctx, comparison.Spec("baseline"))
                model.value = np.zeros(0)
            predicted = validation.full_mask(row, model, side)
            for mask, color in (
                (row["observed"]["nominal"].astype(np.uint8), (0, 255, 0)),
                (predicted, (0, 150, 255)),
            ):
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(panel, contours, -1, color, 1)
        cv2.imwrite(str(output / (prefix + ".png")), panel)
        cv2.putText(
            panel,
            "Before fit" if prefix == "before" else "After sine fit (single pose)",
            (12, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )
        panels.append(panel)
    cv2.imwrite(str(output / "comparison.png"), np.hstack(panels))
    assets = {}
    for name, file in {
        "photo": "baseline-solid/photo.png",
        "before": "before.png",
        "after": "after.png",
        "full_before": "baseline-solid/full_layer.png",
        "full_after": "fitted-solid/full_layer.png",
        "gripper_before": "baseline-solid/gripper_layer.png",
        "gripper_after": "fitted-solid/gripper_layer.png",
        "seg_left": "left_segmentation.png",
        "seg_right": "right_segmentation.png",
    }.items():
        assets[name] = (
            "data:image/png;base64," + base64.b64encode((output / file).read_bytes()).decode()
        )
    summary = dict(
        source_capture=str(args.capture.resolve()),
        height_um=rows[0]["height_um"],
        camera_above_shoulder_mm=camera_height,
        formula="q_fit=q+a+b*q+c*sin(q)",
        formula_input="Current measured SDK joint angles in radians, no historical J7 offsets",
        independent_poses=1,
        held_out_poses=0,
        robustness_validated=False,
        diagnostic_only=True,
        deployable=False,
        robot_commands_sent=False,
        segmentation="Visible red inner jaw only: left ee_link2, right ee_link1; three existing red segmentation policies; other geometry is display-only",
        opening_policy="Left negative opening from prior measured confirmation; existing approximate scale. Both estimates exceed URDF opening bounds; evaluated without clipping only in offline geometry.",
        source_hashes={
            str(p.relative_to(ROOT)): comparison.sha(p)
            for p in [
                resources["urdf"],
                resources["sdk_chains"],
                resources["gripper"],
                ROOT / "config/robots/calibration/head_camera_j4_rezero_20261104.json",
                Path(__file__),
            ]
        },
        results=results,
    )
    save(output / "summary.json", summary)
    table = "".join(
        f"<tr><td>{r['side']}</td><td>{r['before']['metrics']['nominal']['symmetric_boundary_mean_px']:.2f} → {r['after']['metrics']['nominal']['symmetric_boundary_mean_px']:.2f}</td><td>{r['before']['metrics']['nominal']['symmetric_boundary_p95_px']:.2f} → {r['after']['metrics']['nominal']['symmetric_boundary_p95_px']:.2f}</td><td>{r['fit']['data_rank']}/21</td></tr>"
        for r in results
    )
    page = """<!doctype html><html lang="zh"><meta charset="utf-8"><title>当前高度 · 七关节 sin 拟合叠图</title><style>body{font:16px system-ui;background:#111820;color:#e3edf6;max-width:1200px;margin:30px auto;padding:0 20px}button,select{padding:9px;margin:5px;background:#263b50;color:white;border:1px solid #58718b;border-radius:6px}button.active{background:#086b76}#stage{position:relative;width:min(100%,960px);aspect-ratio:4/3}#stage img{position:absolute;width:100%}table{border-collapse:collapse}td,th{padding:10px;border:1px solid #506475}.note{color:#ffd58a}a{color:#7fd9ec}</style><h1>当前固定高度：七关节 sin 局部拟合</h1><p>SDK升降 HEIGHT mm · 相机高出肩轴 CAM mm · CAPTURE_TIME 北京时间</p><p>q拟合 = q + a + bq + c sin(q)，左右臂分别拟合；原相机旋转与XY保持。</p><p class="note">仅一个独立位姿，每臂21个参数不可唯一辨识；当前图参与训练，没有多姿态留出验收。没有控制机器人运动。</p><button onclick="pick('before')">拟合前轮廓</button><button onclick="pick('after')">拟合后轮廓</button><button onclick="pick('photo')">实拍</button><button onclick="pick('solid_before')">拟合前实体</button><button onclick="pick('solid_after')">拟合后实体</button><select id="parts" onchange="draw()"><option value="gripper">仅夹爪</option><option value="full">机械臂与夹爪</option></select><label>实体透明度 <input id="opacity" type="range" min="0" max="100" value="45" oninput="draw()"></label><p>绿色：实拍红色夹指；橙色：对应模型夹指。实体模式另绘出其余夹爪，供观察未参与拟合的部位。</p><div id="stage"><img id="photo"><img id="layer"></div><table><tr><th>机械臂</th><th>平均轮廓距离 px 前→后</th><th>P95 px 前→后</th><th>数据秩/参数数</th></tr>TABLE</table><p class="note">左爪开合方向按11月2日实测“负向打开”作诊断对照，未修改生产映射。开合比例仍为旧近似；两爪当前估计位移都超过URDF名义上限，不裁剪。仅左ee_link2、右ee_link1红色夹指参与拟合；其余部位没有本帧独立同名像素验收。颜色分割含遮挡和外形差异，不能换算成TCP精度。</p><button onclick="pick('seg_left')">左爪分割</button><button onclick="pick('seg_right')">右爪分割</button><p><a href="summary.json">完整参数、三组分割门限指标与输入指纹</a> · <a href="../original-solid/overlay_gripper_45.png">原开合映射叠图</a></p><script>const A=ASSETS;let mode='after';function pick(m){mode=m;draw()}function draw(){const solid=mode.startsWith('solid');document.getElementById('photo').src=solid?A.photo:A[mode];const layer=document.getElementById('layer');layer.style.display=solid?'block':'none';if(solid){layer.src=A[document.getElementById('parts').value+'_'+mode.slice(6)];layer.style.opacity=document.getElementById('opacity').value/100}document.body.dataset.mode=mode}draw()</script></html>"""
    page = (
        page.replace("HEIGHT", f"{rows[0]['height_um'] / 1000:.3f}")
        .replace("CAM", f"{camera_height:.3f}")
        .replace("TABLE", table)
        .replace("ASSETS", json.dumps(assets))
    )
    recorded = json.loads((args.capture / "pose_report.json").read_text())
    capture_time = datetime.datetime.fromisoformat(recorded["created_utc"]).astimezone(
        datetime.timezone(datetime.timedelta(hours=8))
    )
    page = page.replace("CAPTURE_TIME", capture_time.strftime("%Y-%m-%d %H:%M:%S"))
    (output / "report.html").write_text(page)


if __name__ == "__main__":
    main()
