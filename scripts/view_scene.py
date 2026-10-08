"""Local interactive viewer. All edits remain in an in-memory USD session layer."""

import asyncio
import json
import sys
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import ASSETS, OUTPUTS, load_config

c = load_config()
if not (ASSETS / "main.usda").is_file():
    raise SystemExit("Run ./scripts/simctl build-scene first")
if json.loads((ASSETS / "scene_config.json").read_text()) != c:
    raise SystemExit("Config changed: run ./scripts/simctl build-scene first")

from isaacsim import SimulationApp

app = SimulationApp(
    {
        "headless": False,
        "width": 1280,
        "height": 800,
        "window_width": 1600,
        "window_height": 1000,
        "multi_gpu": False,
        "extra_args": [
            "--/app/window/title=Wheel Robot - Industrial Simulation",
            "--/app/runLoops/main/rateLimitEnabled=true",
            "--/app/runLoops/main/rateLimitFrequency=60",
        ],
    }
)
try:
    import omni.timeline
    import omni.usd
    import omni.ui as ui
    from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
    from pxr import Gf, UsdGeom, PhysxSchema

    context = omni.usd.get_context()
    if not context.open_stage(str(ASSETS / "main.usda")):
        raise RuntimeError("Failed to open main.usda")
    for _ in range(30):
        app.update()
    stage = context.get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    scene = stage.GetPrimAtPath("/World/PhysicsScene")
    PhysxSchema.PhysxSceneAPI.Apply(scene).CreateTimeStepsPerSecondAttr(
        round(1 / c["physics"]["dt"])
    )
    # Match World() regression tests: stiff lift drives need the TGS solver.
    scene_api = PhysxSchema.PhysxSceneAPI.Apply(scene)
    scene_api.CreateSolverTypeAttr("TGS")
    scene_api.CreateEnableGPUDynamicsAttr(False)
    scene_api.CreateBroadphaseTypeAttr("MBP")
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("No viewport available")
    camera = UsdGeom.Camera.Define(stage, "/World/ViewerCamera")
    camera.CreateFocalLengthAttr(22)
    camera.CreateHorizontalApertureAttr(36)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 500))
    transform = camera.MakeMatrixXform()
    viewport.camera_path = camera.GetPath()

    def view(position, target):
        transform.Set(
            Gf.Matrix4d()
            .SetLookAt(Gf.Vec3d(*position), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1))
            .GetInverse()
        )
        viewport.camera_path = camera.GetPath()

    view(c["camera"]["showcase"], c["camera"]["look_at"])
    timeline = omni.timeline.get_timeline_interface()
    pending = {"reset": False}

    def toggle():
        if timeline.is_playing():
            timeline.pause()
        else:
            timeline.play()

    def reset():
        pending["reset"] = True

    window = ui.Window("Wheel Robot Controls", width=330, height=190)
    with window.frame:
        with ui.VStack(spacing=8):
            ui.Label("Industrial box environment", height=22)
            with ui.HStack(height=30):
                ui.Button("Play / Pause", clicked_fn=toggle)
                ui.Button("Reset scene", clicked_fn=reset)
            with ui.HStack(height=30):
                ui.Button(
                    "Overview",
                    clicked_fn=lambda: view(
                        c["camera"]["showcase"], c["camera"]["look_at"]
                    ),
                )
                ui.Button(
                    "Box view",
                    clicked_fn=lambda: view((-3.6, -2, 1.35), (-5, -3.5, 0.86)),
                )
            ui.Label("Alt + left drag: orbit | Wheel: zoom", height=22)
            ui.Label("Middle drag: pan | Select object + F: focus", height=22)
    robot_control = None
    world = None
    if c["robot"]["usd_path"] == "robots/generated/robot.usdc":
        import omni.physx
        from robot_control import RobotControl
        from robot_ui import RobotPanel

        import omni.kit.app
        import numpy as np

        omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
            "isaacsim.core.api", True
        )
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation

        robot_control = RobotControl(stage)
        world = World(
            physics_dt=c["physics"]["dt"], rendering_dt=c["physics"]["dt"],
            stage_units_in_meters=1.0, backend="numpy", device="cpu",
        )
        robot = world.scene.add(SingleArticulation(
            "/World/Robot" + robot_control.meta["root_path"], name="r680"
        ))
        world.reset()
        robot.set_joints_default_state(positions=np.array([
            robot_control.cfg["home"].get(name, 0.0) for name in robot.dof_names
        ]))
        world.reset()


        def focus_robot():
            p = (
                UsdGeom.Xformable(
                    stage.GetPrimAtPath(
                        "/World/Robot" + robot_control.meta["root_path"]
                    )
                )
                .ComputeLocalToWorldTransform(0)
                .ExtractTranslation()
            )
            view((p[0] + 1.7, p[1] - 2.0, p[2] + 1.5), (p[0], p[1], p[2] + 0.65))

        robot_panel = RobotPanel(robot_control, focus_robot)
        robot_subscription = (
            omni.physx.get_physx_interface().subscribe_physics_step_events(
                robot_control.update
            )
        )
        focus_robot()
    timeline.play()
    for _ in range(90):
        app.update()

    # Kit may restore its saved layout after custom windows are constructed.
    # Dock and show them after that restore so controls cannot disappear off-screen.
    window.visible = True
    if robot_control:
        robot_panel.window.visible = True
        target = ui.Workspace.get_window("Stage")
        panel = ui.Workspace.get_window("R680 + OpenArm controls")
        if target and panel:
            panel.dock_in(target, ui.DockPosition.SAME)
        controls = ui.Workspace.get_window("Wheel Robot Controls")
        content = ui.Workspace.get_window("Content")
        if controls and content:
            controls.dock_in(content, ui.DockPosition.SAME)
        print("ROBOT_GUI_PANELS_READY", flush=True)
    for _ in range(5):
        app.update()

    # Capture the actual interactive viewport, not a separate render product.
    async def screenshot():
        capture = capture_viewport_to_file(viewport, str(OUTPUTS / "gui_viewport.png"))
        await capture.wait_for_result()
        print("GUI_VIEWPORT_CAPTURED", flush=True)

    capture_task = asyncio.ensure_future(screenshot())
    print("GUI_READY: scene loaded; native viewport and controls available", flush=True)
    while app.is_running():
        if pending["reset"]:
            timeline.stop()
            if robot_control:
                robot_panel.home()
                robot_control.sent = robot_control.targets.copy()
                robot_control.update(c["physics"]["dt"])
            app.update()
            if world is not None:
                world.reset()
            timeline.play()
            pending["reset"] = False
        app.update()
except Exception:
    traceback.print_exc()
    app.close(exit_code=1)
    raise
else:
    app.close()
