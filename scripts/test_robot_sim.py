"""Actual contact-driven robot verification inside the industrial environment."""

import json
import math
import sys
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import ASSETS, OUTPUTS, load_config
from robot_control import RobotControl, load_robot_data
from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "multi_gpu": False})
try:
    import numpy as np
    import omni.usd
    import omni.kit.app
    from pxr import UsdGeom, UsdPhysics, Gf

    omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
        "isaacsim.core.api", True
    )
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation

    cfg, meta = load_robot_data()
    c = load_config()
    omni.usd.get_context().open_stage(str(ASSETS / "main.usda"))
    for _ in range(15):
        app.update()
    stage = omni.usd.get_context().get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    dt = c["physics"]["dt"]
    world = World(
        physics_dt=dt,
        rendering_dt=dt,
        stage_units_in_meters=1.0,
        backend="numpy",
        device="cpu",
    )
    robot = world.scene.add(
        SingleArticulation("/World/Robot" + meta["root_path"], name="r680")
    )
    world.reset()
    ctl = RobotControl(stage)
    dofs = robot.dof_names
    print("ROBOT_DOFS", len(dofs), dofs, flush=True)
    readings = {}
    checks = {}

    def run(seconds):
        for i in range(round(seconds / dt)):
            ctl.update(dt)
            world.step(render=(i % 4 == 0))

    def pose():
        p, q = robot.get_world_pose()
        return p.copy(), q.copy()

    def yaw(q):
        return math.atan2(
            2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        )

    def reset():
        world.reset()
        ctl.home()
        ctl.sent = ctl.targets.copy()
        run(2.0)

    reset()
    p, q = pose()
    readings["rest_pose"] = {"p": p.tolist(), "q": q.tolist()}
    checks["mass_preserved"] = abs(meta["total_mass"] - meta["urdf_mass"]) < 0.002
    checks["actuated_count_mimic_2"] = (
        len(meta["actuated_joints"]) == (20 if "lift_joint" in dofs else 19) and len(meta["mimic_joints"]) == 2
    )
    checks["stable_upright"] = bool(abs(p[2]) < 0.06 and abs(q[1]) + abs(q[2]) < 0.12)
    if "lift_joint" in dofs:
        lift_index = dofs.index("lift_joint")
        checks["lift_starts_lowest"] = bool(abs(robot.get_joint_positions()[lift_index]) < 0.003)

        def frame_position(name):
            return np.array(UsdGeom.Xformable(stage.GetPrimAtPath(
                "/World/Robot" + meta["frames"][name]
            )).ComputeLocalToWorldTransform(0).ExtractTranslation())

        bottom = frame_position("lift_bottom_reference_link")
        shoulder = frame_position("openarm_left_base_link")
        checks["shoulder_low_106mm"] = bool(abs((shoulder - bottom)[2] - 0.106) < 0.003)
        head0 = frame_position("head_d455_link")
        wrist0 = frame_position("left_wrist_eb435i_link")
        ctl.set_joint("lift_joint", 0.8)  # Controller must clamp to the upper stop.
        checks["lift_target_clamped"] = ctl.targets["lift_joint"] == 0.4
        peak_speed = 0.0
        for _ in range(round(9.5 / dt)):
            ctl.update(dt)
            world.step(render=(_ % 4 == 0))
            peak_speed = max(peak_speed, abs(float(robot.get_joint_velocities()[lift_index])))
        high = float(robot.get_joint_positions()[lift_index])
        shoulder_high = float((frame_position("openarm_left_base_link") - frame_position("lift_bottom_reference_link"))[2])
        checks["lift_reaches_400mm"] = abs(high - 0.4) < 0.003
        checks["lift_speed_50mm_s"] = peak_speed <= 0.055
        checks["shoulder_high_506mm"] = abs(shoulder_high - 0.506) < 0.003
        checks["head_camera_fixed"] = bool(np.linalg.norm(frame_position("head_d455_link") - head0) < 0.015)
        checks["wrist_camera_follows_lift"] = bool(abs((frame_position("left_wrist_eb435i_link") - wrist0)[2] - 0.4) < 0.015)
        ctl.set_joint("lift_joint", -0.1)
        run(9.5)
        low = float(robot.get_joint_positions()[lift_index])
        checks["lift_returns_lowest"] = abs(low) < 0.003
        readings["lift"] = {"high_m": high, "low_m": low, "peak_speed_m_s": peak_speed, "shoulder_high_m": shoulder_high}
        print("PHASE lift", readings["lift"], flush=True)
    for name, cmd in [
        ("forward", [0.2, 0, 0]),
        ("lateral", [0, 0.2, 0]),
        ("yaw", [0, 0, 0.35]),
    ]:
        reset()
        p0, q0 = pose()
        ctl.set_twist(*cmd)
        run(2.5)
        ctl.stop()
        run(0.6)
        p1, q1 = pose()
        d = p1 - p0
        dyaw = math.atan2(math.sin(yaw(q1) - yaw(q0)), math.cos(yaw(q1) - yaw(q0)))
        readings[name] = {
            "delta": d.tolist(),
            "yaw_delta": dyaw,
            "wheel_velocities": robot.get_joint_velocities()[
                [dofs.index(w["joint"]) for w in meta["wheels"]]
            ].tolist(),
        }
        if name == "forward":
            checks["forward_contact_drive"] = bool(
                d[0] > 0.25 and abs(d[1]) < 0.15 and abs(dyaw) < 0.2
            )
        elif name == "lateral":
            checks["lateral_contact_drive"] = bool(
                d[1] > 0.25 and abs(d[0]) < 0.15 and abs(dyaw) < 0.2
            )
        else:
            checks["yaw_contact_drive"] = bool(
                dyaw > 0.45 and np.linalg.norm(d[:2]) < 0.15
            )
        checks[name + "_upright"] = bool(
            abs(q1[1]) + abs(q1[2]) < 0.15 and abs(p1[2]) < 0.06
        )
        print("PHASE", name, readings[name], flush=True)
    reset()
    arm_specs = [s for s in meta["actuated_joints"] if s["kind"] == "arm"]
    for s in arm_specs:
        ctl.set_joint(s["name"], cfg["home"].get(s["name"], 0) + 0.10)
    ctl.gripper("left", True)
    ctl.gripper("right", True)
    run(3.0)
    positions = robot.get_joint_positions()
    errors = {
        s["name"]: float(positions[dofs.index(s["name"])] - ctl.targets[s["name"]])
        for s in arm_specs
    }
    readings["arm_tracking_errors"] = errors
    checks["fourteen_arm_joints_track"] = max(abs(x) for x in errors.values()) < 0.12
    readings["gripper_open"] = {
        s["name"]: float(positions[dofs.index(s["name"])])
        for s in meta["actuated_joints"] + meta["mimic_joints"]
        if "finger" in s["name"]
    }
    for side, sign in [("left", 1), ("right", -1)]:
        a = positions[dofs.index(f"openarm_{side}_finger_joint1")]
        b = positions[dofs.index(f"openarm_{side}_finger_joint2")]
        checks[side + "_gripper_opens"] = bool(sign * a > 0.6)
        checks[side + "_physical_mimic"] = bool(abs(a - b) < 0.02)
    ctl.gripper("left", False)
    ctl.gripper("right", False)
    run(2.0)
    positions = robot.get_joint_positions()
    checks["grippers_close"] = bool(
        max(
            abs(positions[dofs.index(s["name"])])
            for s in meta["actuated_joints"] + meta["mimic_joints"]
            if "finger" in s["name"]
        )
        < 0.03
    )
    reset()
    # Fixed obstacle in a test-only session layer: wheel drive must not pass through it.
    obstacle = UsdGeom.Cube.Define(stage, "/World/RobotTestObstacle")
    obstacle.CreateSizeAttr(1)
    obstacle.AddTranslateOp().Set(Gf.Vec3d(-1.7, 0, 0.5))
    obstacle.AddScaleOp().Set(Gf.Vec3d(0.2, 1.5, 1.0))
    obstacle.CreateVisibilityAttr("invisible")
    UsdPhysics.CollisionAPI.Apply(obstacle.GetPrim())
    for _ in range(10):
        app.update()
    ctl.set_twist(0.2, 0, 0)
    run(7.0)
    ctl.stop()
    run(0.6)
    p, q = pose()
    readings["obstacle_stop"] = {"p": p.tolist(), "q": q.tolist()}
    checks["blocked_by_static_obstacle"] = bool(
        -2.7 < p[0] < -2.04 and abs(q[1]) + abs(q[2]) < 0.2
    )
    stage.RemovePrim("/World/RobotTestObstacle")
    reset()
    p, q = pose()
    if "lift_joint" in dofs:
        checks["reset_lift_lowest"] = bool(abs(robot.get_joint_positions()[lift_index]) < 0.003)
    checks["reset_restores_robot"] = bool(
        np.linalg.norm(p[:2] - np.array(c["robot"]["position"][:2])) < 0.06
    )
    import omni.replicator.core as rep
    from PIL import Image

    cam = rep.create.camera(
        position=(-1.3, -2.0, 1.5),
        look_at=(-3, 0, 0.65),
        focal_length=28,
        horizontal_aperture=36,
    )
    rp = rep.create.render_product(cam, (1280, 960))
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb.attach(rp)
    for _ in range(16):
        rep.orchestrator.step(rt_subframes=4)
    Image.fromarray(rgb.get_data()).save(str(OUTPUTS / "robot_preview.png"))
    report = {
        "source_urdf": cfg["source_urdf"],
        "solver_iterations": meta.get("solver_iterations"),
        "passed": all(checks.values()),
        "checks": checks,
        "measurements": readings,
        "dof_count": len(dofs),
        "note": "Wheel motors and passive roller contacts only; no root pose/velocity forcing. No payload/grasp certification.",
    }
    (OUTPUTS / "robot_sim_report.json").write_text(json.dumps(report, indent=2))
    print("ROBOT_TEST_REPORT", json.dumps(report), flush=True)
    if not report["passed"]:
        raise RuntimeError("Robot physics checks failed; inspect robot_sim_report.json")
except Exception:
    traceback.print_exc()
    app.close(exit_code=1)
else:
    app.close()
