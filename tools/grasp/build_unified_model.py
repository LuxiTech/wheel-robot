#!/usr/bin/env python3
"""Generate a portable, kinematics-only MJCF from the supplied whole-robot URDF."""

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


def numbers(v):
    return " ".join(format(float(x), ".17g") for x in np.ravel(v))


def build(urdf, output, *, sdk_reference_chains=None, mesh_directory="meshes"):
    if Path(mesh_directory).name != mesh_directory or mesh_directory in ("", ".", ".."):
        raise ValueError("Mesh directory must be one relative directory name")
    output.mkdir(parents=True, exist_ok=False)
    (output / mesh_directory).mkdir()
    source = ET.parse(urdf).getroot()
    robot = RobotModel(urdf)
    version = urdf.parent.name.replace("r680_lift_openarm_", "").replace(".", "")
    root = ET.Element("mujoco", model=f"wheel_{version}_unified")
    ET.SubElement(root, "compiler", angle="radian", fusestatic="false", inertiafromgeom="false")
    ET.SubElement(root, "option", gravity="0 0 0")
    asset = ET.SubElement(root, "asset")
    world = ET.SubElement(root, "worldbody")
    equality = ET.SubElement(root, "equality")
    links = {link.get("name"): link for link in source.findall("link")}
    bodies = {}
    files = {}
    dummy = []

    def placement(t):
        quat = Rotation.from_matrix(t[:3, :3]).as_quat()
        return dict(pos=numbers(t[:3, 3]), quat=numbers(quat[[3, 0, 1, 2]]))

    def origin(element):
        t = np.eye(4)
        if element is not None:
            t[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
            t[:3, :3] = Rotation.from_euler(
                "xyz", np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
            ).as_matrix()
        return t

    def add(name, parent, joint=None):
        t = np.eye(4) if joint is None else joint["t"].copy()
        reference = None
        if (
            sdk_reference_chains
            and joint
            and "finger_joint" in joint["name"]
            and joint["kind"] == "prismatic"
        ):
            # Native SDK IK synchronizes the fourteen arm joints only. Give
            # unused finger coordinates a valid nominal closed initial pose.
            # Moving the joint frame by ref and using q-ref preserves URDF FK
            # at every q; it does not change the URDF or motor zero.
            reference = joint["bounds"][0]
            shift = np.eye(4)
            shift[:3, 3] = joint["axis"] * reference
            t = t @ shift
        body = ET.SubElement(parent, "body", name=name, **placement(t))
        bodies[name] = body
        if joint and joint["kind"] != "fixed":
            attrs = dict(
                name=joint["name"],
                type="slide" if joint["kind"] == "prismatic" else "hinge",
                axis=numbers(joint["axis"]),
            )
            if joint["bounds"]:
                attrs.update(limited="true", range=numbers(joint["bounds"]))
            if reference is not None:
                attrs["ref"] = numbers([reference])
            ET.SubElement(body, "joint", **attrs)
            if joint["mimic"]:
                src, mult, offset = joint["mimic"]
                ET.SubElement(
                    equality,
                    "joint",
                    joint1=joint["name"],
                    joint2=src,
                    polycoef=numbers([offset, mult, 0, 0, 0]),
                )
        inertial = links[name].find("inertial")
        if inertial is not None:
            mass = float(inertial.find("mass").get("value"))
            item = inertial.find("inertia")
            o = origin(inertial.find("origin"))
            tensor = np.array(
                [
                    [
                        float(item.get("i" + a + b))
                        if item.get("i" + a + b) is not None
                        else float(item.get("i" + b + a))
                        for b in "xyz"
                    ]
                    for a in "xyz"
                ]
            )
            tensor = o[:3, :3] @ tensor @ o[:3, :3].T
            ET.SubElement(
                body,
                "inertial",
                pos=numbers(o[:3, 3]),
                mass=str(mass),
                fullinertia=numbers(
                    [
                        tensor[0, 0],
                        tensor[1, 1],
                        tensor[2, 2],
                        tensor[0, 1],
                        tensor[0, 2],
                        tensor[1, 2],
                    ]
                ),
            )
        elif joint and joint["kind"] != "fixed":
            # Empty URDF joint frames carry mass in fixed descendants. MuJoCo
            # requires positive local inertia; this epsilon is declared, not a
            # claim that dynamics were validated.
            ET.SubElement(
                body, "inertial", pos="0 0 0", mass="1e-8", diaginertia="1e-10 1e-10 1e-10"
            )
            dummy.append(name)
        ET.SubElement(body, "site", name="frame_" + name, size=".002", rgba="0 0 0 0")
        for kind, group in [("visual", "2"), ("collision", "3")]:
            for i, visual in enumerate(links[name].findall(kind)):
                attrs = dict(
                    name=f"{name}_{kind}_{i}",
                    group=group,
                    contype="0",
                    conaffinity="0",
                    **placement(origin(visual.find("origin"))),
                )
                geom = visual.find("geometry")
                mesh = geom.find("mesh")
                if mesh is not None:
                    path = urdf.parent / mesh.get("filename")
                    scale = np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
                    key = (str(path), tuple(scale))
                    identifier = hashlib.sha256(repr(key).encode()).hexdigest()[:20]
                    if identifier not in files:
                        loaded = trimesh.load(path, force="scene")
                        m = loaded.to_geometry() if isinstance(loaded, trimesh.Scene) else loaded
                        m.apply_scale(scale)
                        dest = output / mesh_directory / (identifier + ".obj")
                        m.export(dest)
                        ET.SubElement(
                            asset,
                            "mesh",
                            name="mesh_" + identifier,
                            file=mesh_directory + "/" + dest.name,
                        )
                        files[identifier] = dict(
                            source=str(path),
                            source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                            scale=scale.tolist(),
                            output_sha256=hashlib.sha256(dest.read_bytes()).hexdigest(),
                        )
                    attrs.update(type="mesh", mesh="mesh_" + identifier)
                elif geom.find("box") is not None:
                    attrs.update(
                        type="box",
                        size=numbers(np.fromstring(geom.find("box").get("size"), sep=" ") / 2),
                    )
                elif geom.find("sphere") is not None:
                    attrs.update(type="sphere", size=geom.find("sphere").get("radius"))
                elif geom.find("cylinder") is not None:
                    g = geom.find("cylinder")
                    attrs.update(
                        type="cylinder",
                        size=numbers([float(g.get("radius")), float(g.get("length")) / 2]),
                    )
                else:
                    raise ValueError("Unsupported URDF geometry")
                color = visual.find("material/color")
                if color is not None:
                    attrs["rgba"] = color.get("rgba")
                ET.SubElement(body, "geom", **attrs)
        for spec in robot.joints.values():
            if spec["parent"] == name:
                add(spec["child"], body, spec)

    roots = robot.links - set(robot.parents)
    if roots != {"base_footprint"}:
        raise ValueError("Unexpected root")
    add("base_footprint", world)
    sdk_origin = np.eye(4)
    origin_definition = "arm_origin aliases lift_carriage_link; NOT the old bimanual SDK origin"
    if sdk_reference_chains:
        from wheel_motion_core.sdk_kinematics import SDKKinematics

        sdk = SDKKinematics(sdk_reference_chains)
        sdk_origin = robot.transform(
            "openarm_left_base_link", {}, base="lift_carriage_link"
        ) @ np.linalg.inv(sdk.transform("openarm_left_base_link"))
        origin_definition = (
            "Legacy arm_origin registered through the unchanged left arm base; "
            "full URDF right shoulder retained. This frame moves with the carriage. "
            "Native SDK wrist FK/IK relative to this frame is independent of lift q."
        )
    ET.SubElement(
        bodies["lift_carriage_link"],
        "site",
        name="arm_origin",
        size=".003",
        **placement(sdk_origin),
    )
    for side in ["left", "right"]:
        ET.SubElement(
            bodies[f"openarm_{side}_link7"], "site", name=side + "_ee_control_point", size=".004"
        )
    ET.indent(root)
    ET.ElementTree(root).write(output / "robot.xml", encoding="utf-8", xml_declaration=True)
    manifest = dict(
        urdf=str(urdf),
        urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),
        meshes=files,
        massless_moving_frames_with_declared_epsilon=dummy,
        kinematics_only=True,
        contacts_enabled=False,
        mesh_directory=mesh_directory,
        sdk_origin_definition=origin_definition,
        carriage_T_arm_origin=sdk_origin.tolist(),
        sdk_reference_chains=str(sdk_reference_chains) if sdk_reference_chains else None,
        hardware_deployment_approved=False,
    )
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return robot


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument(
        "--urdf",
        type=Path,
        default=selected_model(ROOT)["urdf"],
    )
    p.add_argument(
        "--sdk-reference-chains", type=Path, help="Preserve the calibrated legacy left-base frame"
    )
    p.add_argument("--mesh-directory", default="meshes")
    a = p.parse_args()
    build(
        a.urdf,
        a.output,
        sdk_reference_chains=a.sdk_reference_chains,
        mesh_directory=a.mesh_directory,
    )
    print(a.output / "robot.xml")
