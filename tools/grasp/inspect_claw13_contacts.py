#!/usr/bin/env python3
"""Extract CAD contact candidates and mounting discrepancy; never commands hardware."""

import argparse
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from _bootstrap import configure
from _robot_model import selected_model

configure(ROOT)
from wheel_motion_core.kinematics import RobotModel
from wheel_motion_core.sdk_kinematics import SDKKinematics


def extract(output):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    resources = selected_model(ROOT)
    source = resources["source"]
    manifest = json.loads(source.read_text())
    urdf = ROOT / manifest["urdf"]
    digest = hashlib.sha256(urdf.read_bytes()).hexdigest()
    if digest != manifest["urdf_sha256"]:
        raise ValueError("Model changed; recheck contact selection")
    model = RobotModel(urdf)
    xml = ET.parse(urdf).getroot()
    sdk = SDKKinematics(resources["sdk_chains"])
    report = dict(
        schema="wheel_claw13_contact_candidates_v1",
        executable=False,
        hardware_validated=False,
        urdf_sha256=digest,
        sides={},
        contact_definition="Opposing distal planar faces: long-finger lip and short-finger flat. CAD candidate only.",
        motor_angle_to_width=None,
    )
    transfers = {}
    plot_data = {}
    for side in ("left", "right"):
        joint = f"openarm_{side}_finger_joint1"
        low, high = model.joints[joint]["bounds"]
        wrist = f"openarm_{side}_link7"
        surfaces = []
        plot_data[side] = []
        for number in (1, 2):
            link = f"openarm_{side}_ee_link{number}"
            visual = xml.find(f"link[@name='{link}']/visual")
            origin = visual.find("origin")
            if not np.allclose(np.fromstring(origin.get("xyz"), sep=" "), 0) or not np.allclose(
                np.fromstring(origin.get("rpy"), sep=" "), 0
            ):
                raise ValueError("Contact extraction expects identity mesh origin")
            path = urdf.parent / visual.find("geometry/mesh").get("filename")
            mesh = trimesh.load(path, force="mesh")
            sign = 1 if number == 1 else -1
            candidates = []
            for index, faces in enumerate(mesh.facets):
                vertices = mesh.vertices[np.unique(mesh.faces[faces])]
                if (
                    np.dot(mesh.facets_normal[index], [0, sign, 0]) > 0.999
                    and mesh.facets_area[index] > 0.0005
                    and vertices[:, 0].max() < -0.04
                ):
                    candidates.append((index, vertices))
            if not candidates:
                raise ValueError("No distal contact plane")
            index, vertices = max(candidates, key=lambda item: sign * item[1][:, 1].mean())
            # A shared X/Z location within the overlapping flat surfaces, in mesh axes.
            point = np.array([-0.096, vertices[:, 1].mean(), 0.0565])
            patch = mesh.submesh([mesh.facets[index]], append=True)
            _, distance, _ = trimesh.proximity.closest_point_naive(patch, [point])
            if distance[0] > 1e-7:
                raise ValueError("Selected reference is outside the actual contact triangles")
            t = model.transform(link, {joint: high}, base=wrist)
            p = (t @ np.r_[point, 1])[:3]
            normal = t[:3, :3] @ mesh.facets_normal[index]
            surfaces.append(
                dict(
                    link=link,
                    mesh_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    facet_index=int(index),
                    area_mm2=float(mesh.facets_area[index] * 1e6),
                    point_in_finger_m=point.tolist(),
                    point_in_wrist_at_max_open_m=p.tolist(),
                    inward_normal_in_wrist=normal.tolist(),
                    surface_distance_m=float(distance[0]),
                )
            )
            plot_data[side].append((mesh, point, number, high))
        p1, p2 = [np.array(s["point_in_wrist_at_max_open_m"]) for s in surfaces]
        direction = -np.array(surfaces[0]["inward_normal_in_wrist"])
        width = float(np.dot(p1 - p2, direction))
        offset = width - 2 * high
        sweep_errors = []
        for opening in np.linspace(low, high, 21):
            points = []
            for surface in surfaces:
                t = model.transform(surface["link"], {joint: opening}, base=wrist)
                points.append((t @ np.r_[surface["point_in_finger_m"], 1])[:3])
            sweep_errors.append(
                abs(np.dot(points[0] - points[1], direction) - (2 * opening + offset))
            )
            if not np.allclose((points[0] + points[1]) / 2, (p1 + p2) / 2, atol=1e-9):
                raise ValueError("Contact midpoint changes across opening range")
        if max(sweep_errors) > 1e-9:
            raise ValueError("Opening formula does not reproduce URDF")
        # The two planar contact normals must face each other.
        if not np.allclose(
            surfaces[0]["inward_normal_in_wrist"],
            -np.array(surfaces[1]["inward_normal_in_wrist"]),
            atol=1e-7,
        ):
            raise ValueError("Nonparallel contact planes")
        mount = model.transform(f"openarm_{side}_base_link", {}, base="lift_carriage_link")
        transfers[side] = mount @ np.linalg.inv(sdk.transform(f"openarm_{side}_base_link"))
        report["sides"][side] = dict(
            contacts=surfaces,
            finger_displacement_limits_m=[low, high],
            opening_formula=dict(
                variable="URDF single-finger displacement q in metres, NOT motor radians",
                width_m_equals="2*q + offset_m",
                offset_m=offset,
                width_at_limits_mm=[(2 * low + offset) * 1000, width * 1000],
            ),
            contact_midpoint_in_wrist_m=((p1 + p2) / 2).tolist(),
            midpoint_orientation=None,
            opening_sweep_max_error_m=float(max(sweep_errors)),
            carriage_T_sdk_origin_if_this_base_corresponds=transfers[side].tolist(),
        )
    difference = np.linalg.inv(transfers["left"]) @ transfers["right"]
    report["bilateral_registration"] = dict(
        translation_disagreement_mm=float(np.linalg.norm(difference[:3, 3]) * 1000),
        rotation_disagreement_deg=float(
            np.rad2deg(Rotation.from_matrix(difference[:3, :3]).magnitude())
        ),
        accepted=False,
        interpretation="Left-base and right-base correspondence imply different common transforms. Do not apply one arm registration to both arms.",
    )
    output.mkdir(parents=True, exist_ok=False)
    (output / "contacts.json").write_text(json.dumps(report, indent=2) + "\n")
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for ax, side in zip(axes, ("left", "right")):
        for mesh, point, number, q in plot_data[side]:
            # Common unrotated mechanism view: joint offsets +/-5mm; opening +/-q.
            shift = (1 if number == 1 else -1) * (0.005 + q)
            p = point.copy()
            p[1] -= shift
            section = mesh.section(plane_normal=[0, 0, 1], plane_origin=[0, 0, point[2]])
            for contour in section.discrete:
                contour = contour.copy()
                contour[:, 1] -= shift
                ax.plot(
                    contour[:, 0] * 1000,
                    contour[:, 1] * 1000,
                    color="C" + str(number - 1),
                    linewidth=1,
                )
            ax.plot(p[0] * 1000, p[1] * 1000, "rx")
            ax.annotate(
                f"contact {number}", p[:2] * 1000, xytext=(5, 5), textcoords="offset points"
            )
        ax.set(
            title=f"{side}: contact-plane section, q=44 mm",
            xlabel="finger local X (mm)",
            ylabel="mechanism Y (mm)",
        )
        ax.axis("equal")
        ax.grid(True)
    fig.suptitle(
        "Red marks: lip-to-opposing-flat measurement reference; hardware alignment unverified"
    )
    fig.tight_layout()
    fig.savefig(output / "contact_reference.png", dpi=160)
    plt.close(fig)
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = extract(args.output)
    print(json.dumps(result, indent=2))
