"""PhysX post-processing: free rolling omni-wheel contacts and real mimic joints."""

import json
import math
import xml.etree.ElementTree as ET
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
from common import ROOT


def build_robot(imported, cfg=None, out=None):
    cfg = cfg or json.loads((ROOT / "config/robot_config.json").read_text())
    source = ROOT / "assets/robots" / cfg["source_urdf"]
    urdf = ET.parse(source).getroot()
    out = out or ROOT / "assets/robots/generated"
    stage = Usd.Stage.CreateInMemory()
    root = UsdGeom.Xform.Define(stage, "/Robot").GetPrim()
    root.GetReferences().AddReference(str(imported))
    stage.SetDefaultPrim(root)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    # Flatten composition before adding bodies so the final robot is self-contained.
    flat = stage.Flatten()
    stage = Usd.Stage.Open(flat)
    stage.SetDefaultPrim(stage.GetPrimAtPath("/Robot"))
    joints = {
        p.GetName(): p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint)
    }
    bodies = {
        p.GetName(): p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)
    }
    original_mass = sum(
        UsdPhysics.MassAPI(p).GetMassAttr().Get() for p in bodies.values()
    )
    meta = {
        "source_urdf": cfg["source_urdf"],
        "solver_iterations": {"position": 64, "velocity": 16},
        "original_mass": original_mass,
        "wheels": [],
        "actuated_joints": [],
        "mimic_joints": [],
        "root_path": None,
    }
    for p in stage.Traverse():
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            art = PhysxSchema.PhysxArticulationAPI.Apply(p)
            art.CreateEnabledSelfCollisionsAttr(cfg["self_collision"])
            art.CreateSolverPositionIterationCountAttr(64)
            art.CreateSolverVelocityIterationCountAttr(16)
            meta["root_path"] = str(p.GetPath()).removeprefix("/Robot")
    for xml in urdf.findall("joint"):
        name = xml.attrib["name"]
        if name not in joints:
            continue
        p = joints[name]
        lim = xml.find("limit")
        mimic = xml.find("mimic")
        is_wheel = xml.attrib["type"] == "continuous"
        is_finger = "finger" in name or "gripper" in name
        is_lift = xml.attrib["type"] == "prismatic"
        unit_scale = 1.0 if is_lift else math.pi / 180
        info = {
            "name": name,
            "path": str(p.GetPath()).removeprefix("/Robot"),
            "effort": float(lim.attrib["effort"]),
            "velocity": float(lim.attrib["velocity"]),
            "lower": float(lim.attrib.get("lower", "-inf")),
            "upper": float(lim.attrib.get("upper", "inf")),
        }
        if mimic is not None:
            # Isaac 6 import writes NewtonMimicAPI; author the PhysX coupling explicitly.
            p.RemoveAPI(UsdPhysics.DriveAPI, "angular")
            for attr in ("stiffness", "damping", "targetPosition", "targetVelocity"):
                p.RemoveProperty("drive:angular:physics:" + attr)
            api = PhysxSchema.PhysxMimicJointAPI.Apply(p, "rotX")
            api.CreateReferenceJointRel().SetTargets(
                [joints[mimic.attrib["joint"]].GetPath()]
            )
            api.CreateGearingAttr(-float(mimic.attrib.get("multiplier", 1)))
            api.CreateOffsetAttr(math.degrees(float(mimic.attrib.get("offset", 0))))
            info["reference"] = mimic.attrib["joint"]
            meta["mimic_joints"].append(info)
            continue
        drive = UsdPhysics.DriveAPI.Apply(p, "linear" if is_lift else "angular")
        drive.CreateTypeAttr("force")
        drive.CreateMaxForceAttr(
            min(info["effort"], cfg["base"]["wheel_torque_limit"])
            if is_wheel
            else info["effort"]
        )
        # Angular drives use degrees; linear drives retain meters and N/m.
        gains = cfg["lift" if is_lift else ("gripper" if is_finger else "arm")]
        drive.CreateStiffnessAttr(0 if is_wheel else gains["stiffness"] * unit_scale)
        drive.CreateDampingAttr(
            (cfg["base"]["wheel_velocity_damping"] if is_wheel else gains["damping"])
            * unit_scale
        )
        drive.CreateTargetPositionAttr(cfg["home"].get(name, 0) / unit_scale)
        drive.CreateTargetVelocityAttr(0)
        PhysxSchema.PhysxJointAPI.Apply(p).CreateMaxJointVelocityAttr(
            info["velocity"] / unit_scale
        )
        info["kind"] = (
            "lift" if is_lift else
            "wheel" if is_wheel else
            "gripper" if is_finger else "arm"
        )
        # No non-standard JSON infinities for continuous joints.
        if is_wheel:
            info["lower"] = info["upper"] = None
        meta["actuated_joints"].append(info)
        if is_wheel:
            wheel_link = xml.find("child").attrib["link"]
            wheel_xml = urdf.find(f"link[@name='{wheel_link}']")
            r = float(wheel_xml.find("collision/geometry/cylinder").attrib["radius"])
            axis = [float(v) for v in xml.find("axis").attrib["xyz"].split()]
            pos = [float(v) for v in xml.find("origin").attrib["xyz"].split()]
            meta["wheels"].append(
                {
                    "joint": name,
                    "link": wheel_link,
                    "radius": r,
                    "axis": axis,
                    "position": pos,
                    "max_velocity": info["velocity"],
                }
            )
    # Bake the home pose into body transforms before physics starts. Drive targets
    # alone leave long arms hanging through the floor at the URDF zero pose.
    def forward_kinematics(home):
        poses = {"base_footprint": Gf.Matrix4d(1)}
        pending = list(urdf.findall("joint"))
        while pending:
            remaining = []
            for xml in pending:
                parent = xml.find("parent").get("link")
                if parent not in poses:
                    remaining.append(xml)
                    continue
                origin = xml.find("origin")
                xyz = list(map(float, origin.get("xyz", "0 0 0").split())) if origin is not None else [0, 0, 0]
                rpy = list(map(float, origin.get("rpy", "0 0 0").split())) if origin is not None else [0, 0, 0]
                rotation = (Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(rpy[2])).GetQuat()
                            * Gf.Rotation(Gf.Vec3d(0, 1, 0), math.degrees(rpy[1])).GetQuat()
                            * Gf.Rotation(Gf.Vec3d(1, 0, 0), math.degrees(rpy[0])).GetQuat())
                local = Gf.Matrix4d().SetRotate(rotation)
                local.SetTranslateOnly(Gf.Vec3d(*xyz))
                mimic = xml.find("mimic")
                value = home.get(xml.get("name"), 0.0)
                if mimic is not None:
                    value = home.get(mimic.get("joint"), 0.0) * float(mimic.get("multiplier", 1)) + float(mimic.get("offset", 0))
                motion = Gf.Matrix4d(1)
                if xml.get("type") != "fixed":
                    axis = Gf.Vec3d(*map(float, xml.find("axis").get("xyz").split()))
                    if xml.get("type") == "prismatic":
                        motion.SetTranslate(axis * value)
                    else:
                        motion.SetRotate(Gf.Rotation(axis, math.degrees(value)))
                poses[xml.find("child").get("link")] = motion * local * poses[parent]
            if len(remaining) == len(pending):
                raise RuntimeError("Invalid URDF tree")
            pending = remaining
        return poses

    zero = forward_kinematics({})
    home = forward_kinematics(cfg["home"])
    initial = {name: UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0) for name, p in bodies.items()}
    for name, p in bodies.items():
        world_pose = initial[name] * zero[name].GetInverse() * home[name]
        parent_pose = UsdGeom.Xformable(p.GetParent()).ComputeLocalToWorldTransform(0)
        UsdGeom.Xformable(p).MakeMatrixXform().Set(world_pose * parent_pose.GetInverse())
    meta["home"] = cfg["home"]
    # Restore fixed reference/camera frames under their actual moving rigid body.
    # In particular wrist cameras follow the wrists, and carriage frames the lift.
    frames = {name: str(p.GetPath()) for name, p in bodies.items()}
    fixed = [j for j in urdf.findall("joint") if j.get("type") == "fixed"]
    while fixed:
        pending = []
        for xml in fixed:
            parent = xml.find("parent").get("link")
            child = xml.find("child").get("link")
            if parent not in frames:
                pending.append(xml)
                continue
            if child in frames:
                continue
            frame = UsdGeom.Xform.Define(stage, frames[parent] + "/Frames/" + child)
            origin = xml.find("origin")
            xyz = list(map(float, origin.get("xyz", "0 0 0").split())) if origin is not None else [0, 0, 0]
            rpy = list(map(float, origin.get("rpy", "0 0 0").split())) if origin is not None else [0, 0, 0]
            frame.AddTranslateOp().Set(Gf.Vec3d(*xyz))
            q = (Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(rpy[2])).GetQuat()
                 * Gf.Rotation(Gf.Vec3d(0, 1, 0), math.degrees(rpy[1])).GetQuat()
                 * Gf.Rotation(Gf.Vec3d(1, 0, 0), math.degrees(rpy[0])).GetQuat())
            frame.AddOrientOp().Set(Gf.Quatf(q))
            frames[child] = str(frame.GetPath())
        if len(pending) == len(fixed):
            raise RuntimeError("Cannot restore fixed frames: " + str([j.get("name") for j in pending]))
        fixed = pending
    # Compatibility reference: rigid base mounting frame, not the moving carriage.
    if "openarm_mount_link" not in frames:
        mount = UsdGeom.Xform.Define(stage, frames["base_link"] + "/Frames/openarm_mount_link")
        frames["openarm_mount_link"] = str(mount.GetPath())
    meta["frames"] = {name: path.removeprefix("/Robot") for name, path in frames.items()}
    # One high-friction material for rollers; lateral mobility comes from joints,
    # not from making wheels frictionless or teleporting the robot root.
    mat = UsdShade.Material.Define(stage, "/Robot/OmniRubber")
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(cfg["rollers"]["friction"])
    pm.CreateDynamicFrictionAttr(cfg["rollers"]["friction"])
    pm.CreateRestitutionAttr(0)
    for wheel in meta["wheels"]:
        body = bodies[wheel["link"]]
        for child in Usd.PrimRange(body):
            if child.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(child).CreateCollisionEnabledAttr(False)
        n = cfg["rollers"]["per_row"]
        nr = cfg["rollers"]["rows"]
        total = n * nr
        roller_mass = cfg["rollers"]["total_mass_per_wheel"] / total
        mass = UsdPhysics.MassAPI(body)
        mass.CreateMassAttr(1 - cfg["rollers"]["total_mass_per_wheel"])
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(0.0016))
        a = Gf.Vec3d(*wheel["axis"]).GetNormalized()
        tangent = Gf.Cross(a, Gf.Vec3d(0, 0, 1)).GetNormalized()
        wheel_matrix = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        orientation = wheel_matrix.ExtractRotationQuat()
        for row in range(nr):
            for i in range(n):
                phi = 2 * math.pi * (i + row / nr) / n
                radial = Gf.Vec3d(0, 0, -1) * math.cos(phi) + tangent * math.sin(phi)
                spin_axis = Gf.Cross(a, radial).GetNormalized()
                local_pos = radial * (
                    wheel["radius"] - cfg["rollers"]["radius"]
                ) + a * ((2 * row - (nr - 1)) * cfg["rollers"]["row_offset"])
                path = f"/Robot/Rollers/{wheel['link']}_{row}_{i:02}"
                roller = UsdGeom.Xform.Define(stage, path)
                roller.AddTranslateOp().Set(wheel_matrix.Transform(local_pos))
                roller.AddOrientOp().Set(Gf.Quatf(orientation))
                UsdPhysics.RigidBodyAPI.Apply(roller.GetPrim())
                ma = UsdPhysics.MassAPI.Apply(roller.GetPrim())
                ma.CreateMassAttr(roller_mass)
                ma.CreateDiagonalInertiaAttr(Gf.Vec3f(roller_mass * 0.0001))
                cap = UsdGeom.Capsule.Define(stage, path + "/Contact")
                cap.CreateRadiusAttr(cfg["rollers"]["radius"])
                cap.CreateHeightAttr(cfg["rollers"]["length"])
                cap.CreateAxisAttr("Z")
                cap.AddOrientOp().Set(
                    Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 0, 1), spin_axis).GetQuat())
                )
                cap.CreateVisibilityAttr("invisible")
                UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
                PhysxSchema.PhysxCollisionAPI.Apply(
                    cap.GetPrim()
                ).CreateContactOffsetAttr(0.001)
                PhysxSchema.PhysxCollisionAPI(cap.GetPrim()).CreateRestOffsetAttr(0)
                UsdShade.MaterialBindingAPI.Apply(cap.GetPrim()).Bind(
                    mat, materialPurpose="physics"
                )
                hinge = UsdPhysics.RevoluteJoint.Define(stage, path + "/Bearing")
                hinge.CreateAxisAttr("X")
                hinge.CreateBody0Rel().SetTargets([body.GetPath()])
                hinge.CreateBody1Rel().SetTargets([roller.GetPath()])
                hinge.CreateLocalPos0Attr(Gf.Vec3f(local_pos))
                hinge.CreateLocalPos1Attr(Gf.Vec3f(0))
                rotation = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0, 0), spin_axis).GetQuat())
                hinge.CreateLocalRot0Attr(rotation)
                hinge.CreateLocalRot1Attr(rotation)
                hinge.CreateCollisionEnabledAttr(False)
    # Apply collision margins to mesh proxies; avoid cm-wide invisible gaps.
    colliders = 0
    for p in stage.Traverse():
        if p.HasAPI(UsdPhysics.CollisionAPI):
            colliders += 1
            api = PhysxSchema.PhysxCollisionAPI.Apply(p)
            api.CreateContactOffsetAttr(0.001)
            api.CreateRestOffsetAttr(0)
    meta["passive_roller_joints"] = (
        len(meta["wheels"]) * cfg["rollers"]["rows"] * cfg["rollers"]["per_row"]
    )
    meta["colliders"] = colliders
    meta["total_mass"] = sum(
        UsdPhysics.MassAPI(p).GetMassAttr().Get()
        for p in stage.Traverse()
        if p.HasAPI(UsdPhysics.RigidBodyAPI)
    )
    meta["urdf_mass"] = sum(float(m.get("value")) for m in urdf.findall("link/inertial/mass"))
    assert abs(meta["total_mass"] - meta["urdf_mass"]) < 0.002, meta["total_mass"]
    expected = sum(j.get("type") != "fixed" and j.find("mimic") is None for j in urdf.findall("joint"))
    expected_mimic = len(urdf.findall("joint/mimic"))
    assert len(meta["actuated_joints"]) == expected and len(meta["mimic_joints"]) == expected_mimic
    if stage.GetCompositionErrors():
        raise RuntimeError(str(stage.GetCompositionErrors()))
    stage.Export(str(out / "robot.usdc"))
    (out / "robot_metadata.json").write_text(
        json.dumps(meta, indent=2, allow_nan=False)
    )
    print(
        "ROBOT_PHYSICS_READY",
        json.dumps(
            {
                "mass": meta["total_mass"],
                "actuated": len(meta["actuated_joints"]),
                "mimic": expected_mimic,
                "passive_rollers": meta["passive_roller_joints"],
            }
        ),
        flush=True,
    )
