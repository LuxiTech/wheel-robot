#!/usr/bin/env python3
"""Independent raw-URDF FK and a standalone, rotatable gripper pose inspection."""

import argparse
import base64
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np
import plotly.graph_objects as go
import trimesh

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from _bootstrap import configure
from _robot_model import capture_model

configure(ROOT)
from wheel_motion_core.kinematics import RobotModel
from wheel_perception_core.geometry import prepare_input
from wheel_perception_core.rgbd import load_observation
from wheel_perception_core.visualization import rasterize_triangles


def axis_rotation(axis, angle):
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * (skew @ skew)


def origin_matrix(element):
    result = np.eye(4)
    if element is not None:
        result[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
        roll, pitch, yaw = np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
        result[:3, :3] = (
            axis_rotation(np.array([0.0, 0.0, 1.0]), yaw)
            @ axis_rotation(np.array([0.0, 1.0, 0.0]), pitch)
            @ axis_rotation(np.array([1.0, 0.0, 0.0]), roll)
        )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", type=Path, required=True)
    parser.add_argument("--unified-report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    capture = json.loads((args.capture / "pose_report.json").read_text())
    urdf = capture_model(ROOT, capture)["urdf"]
    xml = ET.parse(urdf).getroot()
    parents = {joint.find("child").get("link"): joint for joint in xml.findall("joint")}
    joints = {joint.get("name"): joint for joint in xml.findall("joint")}
    state = json.loads((args.capture / "paired_joint_state.json").read_text())
    report = json.loads(args.unified_report.read_text())
    # Deliberately avoid the production SDK-vector mapping in this recomputation.
    values = {"lift_joint": report["urdf_lift_m"]}
    for side, start in [("right", 0), ("left", 8)]:
        for index in range(7):
            values[f"openarm_{side}_joint{index + 1}"] = state["arms"]["position"][start + index]
        approximation = report["opening_mapping"][side]
        if not approximation["within_urdf_limits"]:
            raise ValueError("Cannot display unknown finger displacement as measured")
        values[f"openarm_{side}_finger_joint1"] = approximation["estimated_finger_displacement_m"]

    def joint_value(joint):
        mimic = joint.find("mimic")
        if mimic is not None:
            return float(mimic.get("multiplier", "1")) * joint_value(
                joints[mimic.get("joint")]
            ) + float(mimic.get("offset", "0"))
        return values[joint.get("name")]

    def independent_fk(link):
        if link == "base_footprint":
            return np.eye(4)
        joint = parents[link]
        motion = np.eye(4)
        kind = joint.get("type")
        if kind != "fixed":
            axis_element = joint.find("axis")
            axis = np.fromstring(axis_element.get("xyz", "1 0 0"), sep=" ")
            if kind == "prismatic":
                motion[:3, 3] = axis / np.linalg.norm(axis) * joint_value(joint)
            else:
                motion[:3, :3] = axis_rotation(axis, joint_value(joint))
        return (
            independent_fk(joint.find("parent").get("link"))
            @ origin_matrix(joint.find("origin"))
            @ motion
        )

    production = RobotModel(urdf)
    observation = prepare_input(load_observation(args.capture / "observation.npz"))
    camera_inverse = np.linalg.inv(report["base_footprint_T_camera"])
    photo = cv2.cvtColor(observation.rgb, cv2.COLOR_RGB2BGR)
    model_image = np.full_like(photo, 245)
    overlay = photo.copy()
    figure = go.Figure()
    checks = {}
    for side in ["left", "right"]:
        wrist = independent_fk(f"openarm_{side}_link7")
        checks[side] = {
            "independent_vs_mujoco_wrist_error": float(
                np.max(np.abs(wrist - np.asarray(report["wrists"][side])))
            ),
            "parts": {},
        }
        for suffix in ["link4", "link5", "link6", "ee_base_link", "ee_link1", "ee_link2"]:
            link = f"openarm_{side}_{suffix}"
            transform = independent_fk(link)
            checks[side]["parts"][link] = {
                "independent_vs_production_error": float(
                    np.max(np.abs(transform - production.transform(link, values)))
                ),
                "base_T_part": transform.tolist(),
            }
            kind = "collision" if suffix in ["link4", "link5", "link6"] else "visual"
            color = (
                "#555b66"
                if kind == "collision"
                else ("#e7b522" if suffix == "ee_base_link" else "#d64646")
            )
            for visual in xml.findall(f"link[@name='{link}']/{kind}"):
                mesh = visual.find("geometry/mesh")
                if mesh is None:
                    continue
                geometry = trimesh.load(urdf.parent / mesh.get("filename"), force="mesh")
                vertices = geometry.vertices * np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
                world = (
                    transform
                    @ origin_matrix(visual.find("origin"))
                    @ np.c_[vertices, np.ones(len(vertices))].T
                ).T[:, :3]
                faces = geometry.faces
                figure.add_trace(
                    go.Mesh3d(
                        x=world[:, 0] * 1000,
                        y=world[:, 1] * 1000,
                        z=world[:, 2] * 1000,
                        i=faces[:, 0],
                        j=faces[:, 1],
                        k=faces[:, 2],
                        color=color,
                        name=link,
                        flatshading=True,
                        showlegend=True,
                    )
                )
                points = (camera_inverse @ np.c_[world, np.ones(len(world))].T).T[:, :3]
                homogeneous = (observation.k @ points.T).T
                pixels = homogeneous[:, :2] / homogeneous[:, 2, None]
                valid = np.all(points[faces, 2] > 0.01, axis=1) & np.all(
                    np.abs(pixels[faces]) < 10000, axis=(1, 2)
                )
                mask = rasterize_triangles(
                    np.rint(pixels[faces[valid]]).astype(np.int32), photo.shape
                )
                bgr = (
                    (80, 80, 80)
                    if kind == "collision"
                    else ((0, 180, 240) if suffix == "ee_base_link" else (70, 70, 210))
                )
                model_image[mask > 0] = bgr
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(overlay, contours, -1, bgr, 1)
        for index, axis_color in enumerate(["red", "green", "blue"]):
            endpoints = np.array([wrist[:3, 3], wrist[:3, 3] + 0.05 * wrist[:3, index]]) * 1000
            figure.add_trace(
                go.Scatter3d(
                    x=endpoints[:, 0],
                    y=endpoints[:, 1],
                    z=endpoints[:, 2],
                    mode="lines",
                    line={"color": axis_color, "width": 6},
                    name=f"{side} wrist {'XYZ'[index]}",
                )
            )
    max_error = max(
        item["independent_vs_production_error"]
        for side in checks.values()
        for item in side["parts"].values()
    )
    if (
        max_error > 1e-10
        or max(side["independent_vs_mujoco_wrist_error"] for side in checks.values()) > 1e-10
    ):
        raise ValueError("Independent 3D FK mismatch")
    figure.update_layout(
        title="计算出的三维姿态：可拖动旋转；单位mm；不使用相机外参",
        scene={
            "aspectmode": "data",
            "xaxis_title": "整机X (mm)",
            "yaxis_title": "整机Y (mm)",
            "zaxis_title": "整机Z (mm)",
        },
        height=700,
    )
    cv2.imwrite(str(args.output / "real.png"), photo)
    cv2.imwrite(str(args.output / "model_camera_view.png"), model_image)
    cv2.imwrite(str(args.output / "overlay.png"), overlay)
    panels = []
    for title, image in [
        ("实物照片", photo),
        ("计算模型在相机视角下的样子（此面板使用待核对外参）", model_image),
        ("两者叠加（此面板使用待核对外参）", overlay),
    ]:
        encoded = base64.b64encode(cv2.imencode(".png", image)[1]).decode()
        panels.append(
            f'<figure><figcaption>{title}</figcaption><img src="data:image/png;base64,{encoded}"></figure>'
        )
    content = '<!doctype html><meta charset="utf-8"><style>body{font:16px sans-serif;margin:24px}figure{display:inline-block;margin:8px}img{width:580px;max-width:100%}</style><h1>夹爪三维位姿排查</h1>'
    content += f"<p>使用保存的实测反馈：{capture['created_utc']}。灰：臂段；黄：固定夹爪；红：夹指。没有发送运动命令。</p>"
    content += figure.to_html(full_html=False, include_plotlyjs=True)
    content += (
        "<p>上方三维图独立于相机；下方相机视角图涉及外参。三维图是计算结果，没有独立实测三维姿态覆盖，因此不能仅凭它宣布实物位姿已正确。</p>"
        + "".join(panels)
    )
    (args.output / "report.html").write_text(content)
    (args.output / "independent_fk.json").write_text(
        json.dumps(
            {
                "commands_sent": False,
                "uses_camera_for_3d_fk": False,
                "max_independent_fk_error": max_error,
                "checks": checks,
                "limits": [
                    "Independent implementation verifies computation from supplied URDF, not physical encoder/mount correspondence.",
                    "0.2m lift offset and CAD closed displacement remain nominal.",
                    "3D image is calculated geometry; no measured real-world 3D pose is available.",
                ],
            },
            indent=2,
        )
        + "\n"
    )
    print(args.output / "report.html")


if __name__ == "__main__":
    main()
