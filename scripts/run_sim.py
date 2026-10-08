"""Headless environment runner and actual PhysX drop/reset regression checks."""

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import ASSETS, OUTPUTS, box_start, load_config
from setup_task import PlacementTask

parser = argparse.ArgumentParser()
parser.add_argument("--steps", type=int, default=600, help="0 runs until stopped")
parser.add_argument("--smoke-test", action="store_true")
parser.add_argument("--capture", action="store_true")
args = parser.parse_args()
c = load_config()
if not (ASSETS / "main.usda").is_file():
    raise SystemExit("Run ./scripts/simctl build-scene first")
if json.loads((ASSETS / "scene_config.json").read_text()) != c:
    raise SystemExit("Config changed: run build-scene again")
from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "width": 1280, "height": 800})
try:
    import numpy as np
    import omni.usd
    import omni.kit.app
    import omni.physx
    from omni.physx.bindings._physx import ContactEventType
    from pxr import PhysxSchema, PhysicsSchemaTools

    # Version-pinned compatibility API provides deterministic explicit dt stepping.
    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("isaacsim.core.api", True)
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleRigidPrim
    from isaacsim.core.utils.rotations import quat_to_rot_matrix

    omni.usd.get_context().open_stage(str(ASSETS / "main.usda"))
    for _ in range(20):
        app.update()
    stage = omni.usd.get_context().get_stage()
    dt = c["physics"]["dt"]
    world = World(
        physics_dt=dt,
        rendering_dt=dt,
        stage_units_in_meters=1.0,
        backend="numpy",
        device="cpu",
    )
    boxes = []
    for i in range(c["box"]["count"]):
        path = f"/World/Boxes/Box_{i:02}"
        PhysxSchema.PhysxContactReportAPI.Apply(
            stage.GetPrimAtPath(path)
        ).CreateThresholdAttr(0.0)
        box = world.scene.add(SingleRigidPrim(path, name=f"box_{i}"))
        box.set_default_state(
            position=np.array(box_start(c, i)),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        )
        boxes.append(box)
    contact = {"target": False}
    target_path = "/World/Collisions/" + c["task"]["target"] + "_Top"

    def on_contacts(headers, data):
        for h in headers:
            paths = {
                str(PhysicsSchemaTools.intToSdfPath(h.actor0)),
                str(PhysicsSchemaTools.intToSdfPath(h.actor1)),
            }
            if "/World/Boxes/Box_00" in paths and target_path in paths:
                if h.type == ContactEventType.CONTACT_LOST:
                    contact["target"] = False
                elif h.num_contact_data > 0:
                    contact["target"] = True

    subscription = (
        omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(
            on_contacts
        )
    )
    world.reset()
    task = PlacementTask(c)
    started = time.monotonic()
    steps = 0

    def step():
        global steps
        world.step(render=True)
        steps += 1
        p, q = boxes[0].get_world_pose()
        extents = np.abs(quat_to_rot_matrix(q)) @ (np.array(c["box"]["size"]) / 2)
        # No gripper is attached in the environment smoke test. Robot controllers
        # must explicitly supply release state instead of assuming release.
        released = True if args.smoke_test or not c["robot"]["usd_path"] else None
        return task.update(
            position=p,
            half_extents=extents,
            linear_velocity=boxes[0].get_linear_velocity(),
            angular_velocity=boxes[0].get_angular_velocity(),
            released=released,
            supported=contact["target"],
            dt=dt,
        )

    checks = {}
    if args.smoke_test:
        for _ in range(360):
            step()
        p = boxes[0].get_world_pose()[0]
        initial = box_start(c, 0, drop=0)
        checks["source_drop_stable"] = bool(
            np.linalg.norm(p - np.array(initial)) < 0.02
            and np.linalg.norm(boxes[0].get_linear_velocity()) < 0.03
        )
        checks["source_is_not_success"] = not task.success
        # This teleport initializes a target drop test; it is NOT a pick/place demo.
        boxes[0].set_world_pose(
            np.array(box_start(c, 0, target=True, drop=0.25)),
            np.array([1.0, 0.0, 0.0, 0.0]),
        )
        boxes[0].set_linear_velocity(np.zeros(3))
        boxes[0].set_angular_velocity(np.zeros(3))
        task.reset()
        saw_contact = False
        for _ in range(360):
            step()
            saw_contact |= contact["target"]
        checks["target_contact_detected"] = bool(saw_contact)
        checks["target_drop_success"] = bool(task.success)
        world.reset()
        task.reset()
        contact["target"] = False
        for _ in range(240):
            step()
        checks["reset_restores_source"] = bool(
            np.linalg.norm(boxes[0].get_world_pose()[0] - np.array(initial)) < 0.02
        )
        checks["reset_clears_success"] = not task.success
    else:
        while app.is_running() and (args.steps == 0 or steps < args.steps):
            step()
    if args.capture:
        import omni.replicator.core as rep
        from PIL import Image

        camera = rep.create.camera(
            position=c["camera"]["showcase"],
            look_at=c["camera"]["look_at"],
            focal_length=22,
            horizontal_aperture=36,
        )
        product = rep.create.render_product(camera, (1280, 800))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach(product)
        for _ in range(24):
            rep.orchestrator.step(rt_subframes=4)
        data = rgb.get_data()
        if data.size == 0:
            raise RuntimeError("Empty camera frame")
        Image.fromarray(data).save(str(OUTPUTS / "isaac_showcase.png"))
        camera2 = rep.create.camera(
            position=(-3.6, -2.0, 1.35), look_at=(-5, -3.5, 0.86), focal_length=20
        )
        product2 = rep.create.render_product(camera2, (960, 600))
        rgb2 = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb2.attach(product2)
        for _ in range(12):
            rep.orchestrator.step(rt_subframes=4)
        Image.fromarray(rgb2.get_data()).save(str(OUTPUTS / "isaac_task_view.png"))
    report = {
        "mode": "physics_smoke" if args.smoke_test else "environment",
        "checks": checks,
        "passed": all(checks.values()) if checks else None,
        "steps": steps,
        "sim_seconds": steps * dt,
        "wall_seconds": time.monotonic() - started,
        "box_position": boxes[0].get_world_pose()[0].tolist(),
        "robot_loaded": bool(c["robot"]["usd_path"]),
        "note": "Target drop is test initialization; robot grasp/transport not tested.",
    }
    (OUTPUTS / "simulation_report.json").write_text(json.dumps(report, indent=2))
    print("SIMULATION_REPORT", json.dumps(report), flush=True)
    if args.smoke_test and not report["passed"]:
        raise RuntimeError("Physics regression failed; see simulation_report.json")
except Exception:
    import traceback

    traceback.print_exc()
    sys.stdout.flush()
    sys.stderr.flush()
    app.close(exit_code=1)
    raise
else:
    app.close()
