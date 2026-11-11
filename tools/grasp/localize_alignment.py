#!/usr/bin/env python3
"""Segmented offline wrist/body overlays; hypotheses are never motion calibration."""

import argparse
import hashlib
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

configure(ROOT)
from wheel_motion_core.kinematics import RobotModel
from wheel_motion_core.sdk_kinematics import SDKKinematics
from wheel_perception_core.extrinsics import CalibratedHeadTransform, HeadExtrinsics
from wheel_perception_core.geometry import prepare_input
from wheel_perception_core.rgbd import load_observation
from wheel_perception_core.visualization import rasterize_triangles


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--capture", type=Path, required=True)
    p.add_argument("--calibration", type=Path, required=True, help="Match the capture's zero epoch")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument(
        "--hypothesis",
        type=Path,
        help="Previously recorded constant J7 offsets; diagnostic comparison only",
    )
    p.add_argument(
        "--cad-bases-from-left",
        action="store_true",
        help="Diagnostic hypothesis: register carriage by left base, then use CAD right base",
    )
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    urdf = ROOT / "assets/robots/r680_lift_openarm_claw1.3/r680_lift_openarm_robot_viewer.urdf"
    m = RobotModel(urdf)
    sdk = SDKKinematics(ROOT / "config/robots/calibration/sdk_arm_chains.json")
    xml = ET.parse(urdf).getroot()
    obs = load_observation(a.capture / "observation.npz")
    inp = prepare_input(obs)
    context = json.loads((a.capture / "lift_context.json").read_text())
    camera, _ = CalibratedHeadTransform(HeadExtrinsics(a.calibration)).resolve(obs, context)
    q = sdk.from_sdk_vector(
        json.loads((a.capture / "paired_joint_state.json").read_text())["arms"]["position"]
    )
    report = dict(
        commands_sent=False,
        executable=False,
        capture=str(a.capture),
        urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),
        camera_transform=camera.tolist(),
        segments={},
        hypothesis_applied_to_runtime=False,
    )
    hypothesis = (
        None if a.hypothesis is None else json.loads(a.hypothesis.read_text())["offset_rad"]
    )
    bases = {side: sdk.transform(f"openarm_{side}_base_link") for side in ["left", "right"]}
    if a.cad_bases_from_left:
        registration = bases["left"] @ np.linalg.inv(
            m.transform("openarm_left_base_link", {}, base="lift_carriage_link")
        )
        bases["right"] = registration @ m.transform(
            "openarm_right_base_link", {}, base="lift_carriage_link"
        )
    report["root_hypothesis"] = a.cad_bases_from_left
    report["arm_origin_T_bases"] = {s: t.tolist() for s, t in bases.items()}

    def draw(image, side, frame, values, color, geometry):
        t = bases[side] @ m.transform(frame, values, base=f"openarm_{side}_base_link")
        c = np.linalg.inv(camera) @ t
        for visual in xml.findall(f"link[@name='{frame}']/{geometry}"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            v = np.eye(4)
            origin = visual.find("origin")
            if origin is not None:
                v[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
                v[:3, :3] = Rotation.from_euler(
                    "xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
                ).as_matrix()
            asset = trimesh.load(urdf.parent / mesh.get("filename"), force="mesh")
            vertices = asset.vertices * np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
            pts = (c @ v @ np.c_[vertices, np.ones(len(vertices))].T).T[:, :3]
            h = (inp.k @ pts.T).T
            uv = h[:, :2] / h[:, 2, None]
            faces = asset.faces
            good = np.all(pts[faces, 2] > 0.01, axis=1) & np.all(
                np.abs(uv[faces]) < 10000, axis=(1, 2)
            )
            polygons = list(np.rint(uv[faces[good]]).astype(np.int32))
            mask = rasterize_triangles(polygons, image.shape)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(image, contours, -1, color, 1)
        return t

    for stage in ["arm", "body", "combined", "hypothesis"]:
        if stage == "hypothesis" and hypothesis is None:
            continue
        image = cv2.cvtColor(inp.rgb, cv2.COLOR_RGB2BGR)
        for side in ["left", "right"]:
            vals = dict(q)
            if stage == "hypothesis":
                vals[f"openarm_{side}_joint7"] += hypothesis[side]
            if stage in ["arm", "combined", "hypothesis"]:
                for i in [4, 5, 6]:
                    t = draw(image, side, f"openarm_{side}_link{i}", vals, (0, 255, 0), "collision")
                    report["segments"][f"{side}_link{i}"] = t.tolist()
            if stage in ["body", "combined", "hypothesis"]:
                draw(image, side, f"openarm_{side}_ee_base_link", vals, (0, 180, 255), "visual")
            for i in [6, 7]:
                t = bases[side] @ m.transform(
                    f"openarm_{side}_link{i}", vals, base=f"openarm_{side}_base_link"
                )
                point = (np.linalg.inv(camera) @ t)[:3, 3]
                if point[2] > 0.01:
                    pixel = inp.k @ point
                    xy = tuple(np.rint(pixel[:2] / pixel[2]).astype(int))
                    cv2.drawMarker(image, xy, (255, 0, 255), cv2.MARKER_CROSS, 12, 1)
                    cv2.putText(
                        image, side[0] + str(i), xy, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1
                    )
        cv2.putText(
            image,
            stage + ": green=arm STL orange=fixed body magenta=frames",
            (5, 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (255, 255, 255),
            1,
        )
        cv2.imwrite(str(a.output / (stage + ".png")), image)
    if hypothesis:
        report["diagnostic_offsets_rad"] = hypothesis
        report["scope"] = (
            "Old fixed-offset hypothesis applied unchanged for comparison; not a fitted or physically accepted calibration. J7 zero offset and rotation of its downstream mount about the same axis cannot be separated by this test."
        )
    (a.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(a.output)


if __name__ == "__main__":
    main()
