#!/usr/bin/env python3
"""Compare URDF RGB optical-center height with the selected extrinsic; GET only."""

import argparse
import datetime
import hashlib
import html
import json
import sys
import urllib.request
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from _bootstrap import configure
from _robot_model import selected_model

configure(ROOT)
from wheel_motion_core.kinematics import RobotModel
from wheel_motion_core.sdk_kinematics import SDKKinematics
from wheel_perception_core.extrinsics import HeadExtrinsics


def write_json(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--lift-offset-m", type=float, default=0.2)
    parser.add_argument(
        "--calibration",
        type=Path,
        default=ROOT / "config/robots/calibration/head_camera_j4_rezero_20261104.json",
    )
    args = parser.parse_args()
    if not np.isfinite(args.lift_offset_m):
        raise ValueError("Finite URDF lift offset required")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    resources = selected_model(ROOT)
    model = RobotModel(resources["urdf"])
    sdk = SDKKinematics(resources["sdk_chains"])
    calibration = HeadExtrinsics(args.calibration)
    lift_config = json.loads((ROOT / "config/robots/lift_control.json").read_text())
    endpoint = lift_config["base_url"].rstrip("/") + "/api/state"
    samples = []
    for index in range(3):
        with urllib.request.urlopen(endpoint, timeout=5) as response:
            state = json.load(response)
        write_json(output / f"sdk_state_{index}.json", state)
        samples.append(state)
    state = samples[-1]
    lift = state["lift"]
    if lift.get("valid") is not True or lift.get("mock") is not False:
        raise ValueError("Real, valid SDK lift feedback required")
    height_um = float(lift["current_um"])
    values = {name: 0.0 for name in model.joints}
    values["lift_joint"] = height_um * 1e-6 + args.lift_offset_m
    # J1 origin is unchanged by its own rotation; no arm-angle substitution
    # or command is needed to calculate shoulder-center height.
    cad_camera = model.transform("head_d455_color_optical_frame", values)
    world_origin = model.transform("openarm_left_base_link", values) @ np.linalg.inv(
        sdk.transform("openarm_left_base_link")
    )
    reference_um = float(calibration.config["reference_height_um"])
    relative_camera = calibration.reference.copy()
    relative_camera[:3, 3] -= calibration.axis * ((height_um - reference_um) * 1e-6)
    # Diagnostic evaluation is explicitly recorded outside the formal range;
    # the selected calibration and its normal range guard are unchanged.
    fitted_camera = world_origin @ relative_camera
    shoulder = {
        side: model.transform(f"openarm_{side}_link1", values) for side in ("left", "right")
    }
    differences = (cad_camera[:3, 3] - fitted_camera[:3, 3]) * 1000
    angle = np.rad2deg(
        Rotation.from_matrix(cad_camera[:3, :3].T @ fitted_camera[:3, :3]).magnitude()
    )
    offset_cases = []
    for offset in (0.0, 0.1, 0.2):
        candidate = dict(values, lift_joint=height_um * 1e-6 + offset)
        candidate_origin = model.transform("openarm_left_base_link", candidate) @ np.linalg.inv(
            sdk.transform("openarm_left_base_link")
        )
        candidate_fit = candidate_origin @ relative_camera
        candidate_shoulder = model.transform("openarm_left_link1", candidate)
        offset_cases.append(
            dict(
                lift_offset_m=offset,
                urdf_camera_above_J1_mm=(cad_camera[2, 3] - candidate_shoulder[2, 3]) * 1000,
                fitted_camera_above_J1_mm=(candidate_fit[2, 3] - candidate_shoulder[2, 3]) * 1000,
            )
        )
    inside = bool(calibration.height_range[0] <= height_um <= calibration.height_range[1])
    evidence = {}
    for name, path in {
        "urdf": resources["urdf"],
        "sdk_chains": resources["sdk_chains"],
        "calibration": args.calibration,
    }.items():
        evidence[name] = dict(
            path=str(path), sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest()
        )
    report = dict(
        created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        commands_sent=False,
        active_resources_modified=False,
        sdk_endpoint=endpoint,
        sdk_lift=lift,
        sdk_lift_status=state.get("lift_status"),
        sample_height_um=[sample["lift"]["current_um"] for sample in samples],
        resources=evidence,
        lift_mapping=dict(
            formula="URDF lift_joint_m = SDK current_um * 1e-6 + lift_offset_m",
            lift_offset_m=args.lift_offset_m,
            absolute_offset_physically_accepted=False,
        ),
        camera_frame="head_d455_color_optical_frame",
        urdf_world_T_camera=cad_camera.tolist(),
        fitted_world_T_camera=fitted_camera.tolist(),
        reference_height_um=reference_um,
        reference_camera_z_mm=float(calibration.reference[2, 3] * 1000),
        calibration_height_range_um=calibration.height_range.tolist(),
        within_recorded_height_range=inside,
        evaluation_only_extrapolation=not inside,
        sides={
            side: dict(
                shoulder_world_z_mm=float(transform[2, 3] * 1000),
                urdf_camera_above_J1_mm=float((cad_camera[2, 3] - transform[2, 3]) * 1000),
                fitted_camera_above_J1_mm=float((fitted_camera[2, 3] - transform[2, 3]) * 1000),
            )
            for side, transform in shoulder.items()
        },
        urdf_minus_fitted_world_position_mm=differences.tolist(),
        urdf_vs_fitted_rotation_deg=float(angle),
        offset_cases=offset_cases,
        conclusion="Model disagreement identified; neither model is independent physical metrology.",
    )
    write_json(output / "audit.json", report)
    left = report["sides"]["left"]
    height_mm = height_um / 1000
    cad_relative = left["urdf_camera_above_J1_mm"]
    fit_relative = left["fitted_camera_above_J1_mm"]
    report_html = f"""<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<title>相机相对肩关节高度核查</title><style>
body{{font:18px/1.7 sans-serif;max-width:1080px;margin:30px auto;padding:0 20px;color:#182c40}}
table{{border-collapse:collapse;width:100%}}td,th{{border:1px solid #bac6cf;padding:10px;text-align:left}}
.note{{background:#fff3d6;padding:16px}}code{{white-space:normal}}svg{{width:100%;max-height:340px}}
</style><h1>相机相对肩关节高度核查</h1>
<p>读取时间：{html.escape(report["created_utc"])}；SDK 实测升降高度：<b>{height_mm:.3f} mm</b>。
仅 GET 读取反馈；没有运动、写零、改外参或改 URDF。</p>
<table><tr><th>同一 SDK 高度下</th><th>RGB 光心高出左 J1 中心</th><th>高出右 J1 中心</th></tr>
<tr><td>URDF 头部 RGB optical frame</td><td>{cad_relative:.3f} mm</td><td>{report["sides"]["right"]["urdf_camera_above_J1_mm"]:.3f} mm</td></tr>
<tr><td>当前叠图使用的标定外参</td><td>{fit_relative:.3f} mm</td><td>{report["sides"]["right"]["fitted_camera_above_J1_mm"]:.3f} mm</td></tr>
<tr><td>两者高度差</td><td colspan="2"><b>{differences[2]:.3f} mm</b></td></tr></table>
<svg viewBox="0 0 900 290" role="img" aria-label="同一肩轴水平线上的两种相机高度计算">
<line x1="50" x2="850" y1="240" y2="240" stroke="#333" stroke-width="3"/>
<text x="50" y="275">肩关节 J1 转轴中心：统一作为相对高度 0</text>
<line x1="270" x2="270" y1="240" y2="{240 - cad_relative:.2f}" stroke="#1673a8" stroke-width="4"/>
<circle cx="270" cy="{240 - cad_relative:.2f}" r="9" fill="#1673a8"/>
<text x="70" y="45">URDF 光心：+{cad_relative:.3f} mm</text>
<line x1="650" x2="650" y1="240" y2="{240 - fit_relative:.2f}" stroke="#df6b20" stroke-width="4"/>
<circle cx="650" cy="{240 - fit_relative:.2f}" r="9" fill="#df6b20"/>
<text x="490" y="45">外参光心：+{fit_relative:.3f} mm</text></svg>
<h2>怎么计算</h2>
<p>URDF：相对高度 = 光心整机 Z − J1 转轴中心整机 Z。当前光心 Z = {cad_camera[2, 3] * 1000:.6f} mm；
J1 Z = {left["shoulder_world_z_mm"]:.6f} mm。相机链包含 head_d455_mount_joint →
head_d455_color_joint → head_d455_color_optical_frame_joint，使用光心 frame，未使用外壳顶部。</p>
<p>叠图外参：相对高度 = {calibration.reference[2, 3] * 1000:.6f} −
({height_mm:.3f} − {reference_um / 1000:.3f}) = <b>{fit_relative:.6f} mm</b>。
前一项是外参拟合值；后一项是相对标定参考高度的升降补偿。</p>
<p>在整机坐标系中，URDF 相机位置减外参相机位置：
X {differences[0]:+.3f}、Y {differences[1]:+.3f}、Z {differences[2]:+.3f} mm；
相机朝向还相差 <b>{angle:.3f}°</b>。这不是仅一个高度数不同。</p>
<p class="note">本次 URDF 肩高沿用现有映射“SDK 升降米值 + {args.lift_offset_m:.3f} m”。
这个绝对零点映射尚未独立实物验收，所以两种模型的差值不能直接认定为真实外参误差。
原外参叠图中，+200 mm 同时用于手臂和相机的整机注册，会抵消；但与固定在底盘的 URDF 相机比较时不会抵消。
URDF 结果是结构模型基准，外参结果是视觉拟合基准；模型相互比较不能代替实物验收。</p>
<p>外参记录高度范围 {calibration.height_range[0] / 1000:.3f}～{calibration.height_range[1] / 1000:.3f} mm；
本次是否在范围内：{"是" if inside else "否，仅按原公式作诊断外推，没有放宽正式范围"}。</p>
<p><a href="audit.json">完整反馈、资源 SHA256、矩阵和不同升降零点映射的对照</a></p></html>"""
    (output / "report.html").write_text(report_html)
    print(
        json.dumps(
            dict(
                output=str(output),
                **report["sides"],
                difference_mm=differences.tolist(),
                rotation_deg=float(angle),
            ),
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
