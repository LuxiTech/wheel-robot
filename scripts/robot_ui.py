"""Small native Isaac panel for the R680's independently driven joints."""

import math
import omni.ui as ui


class RobotPanel:
    def __init__(self, control, focus):
        self.control = control
        self.models = {}
        self.window = ui.Window("R680 + OpenArm controls", width=410, height=700)
        v = control.cfg["base"]["gui_linear_speed"]
        w = control.cfg["base"]["gui_angular_speed"]

        def pulse(vx, vy, wz):
            control.set_twist(vx, vy, wz, duration=0.8)

        with self.window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=5, height=0):
                    ui.Label("Body frame: +X front wheel, +Y left, +Z up", height=22)
                    ui.Label("Base: click for a short motion (0.8 s)", height=22)
                    with ui.HStack(height=30):
                        ui.Button("Left +Y", clicked_fn=lambda: pulse(0, v, 0))
                        ui.Button("Forward +X", clicked_fn=lambda: pulse(v, 0, 0))
                        ui.Button("Right -Y", clicked_fn=lambda: pulse(0, -v, 0))
                    with ui.HStack(height=30):
                        ui.Button("Yaw +", clicked_fn=lambda: pulse(0, 0, w))
                        ui.Button("Backward -X", clicked_fn=lambda: pulse(-v, 0, 0))
                        ui.Button("Yaw -", clicked_fn=lambda: pulse(0, 0, -w))
                    with ui.HStack(height=32):
                        ui.Button("STOP BASE", clicked_fn=control.stop)
                        ui.Button("Robot view", clicked_fn=focus)
                        ui.Button("Robot home", clicked_fn=self.home)
                    ui.Separator(height=10)
                    if "lift_joint" in control.targets:
                        ui.Label("Lift / mm (0 = lowest; 50 mm/s)", height=22)
                        with ui.HStack(height=26):
                            slider = ui.FloatSlider(min=0.0, max=400.0)
                            slider.model.set_value(control.targets["lift_joint"] * 1000)
                            slider.model.add_value_changed_fn(
                                lambda m: control.set_joint("lift_joint", m.as_float / 1000)
                            )
                            self.models["lift_joint"] = slider.model
                        ui.Separator(height=10)
                    for side in ("left", "right"):
                        with ui.CollapsableFrame(
                            side.title() + " arm / degrees", collapsed=False
                        ):
                            with ui.VStack(spacing=4):
                                for j in range(1, 8):
                                    name = f"openarm_{side}_joint{j}"
                                    s = control.specs[name]
                                    with ui.HStack(height=24):
                                        ui.Label("J" + str(j), width=30)
                                        slider = ui.FloatSlider(
                                            min=math.degrees(s["lower"]),
                                            max=math.degrees(s["upper"]),
                                        )
                                        slider.model.set_value(
                                            math.degrees(control.targets[name])
                                        )
                                        slider.model.add_value_changed_fn(
                                            lambda m, n=name: control.set_joint(
                                                n, math.radians(m.as_float)
                                            )
                                        )
                                        self.models[name] = slider.model
                                with ui.HStack(height=28):
                                    ui.Button(
                                        "Open " + side,
                                        clicked_fn=lambda s=side: control.gripper(
                                            s, True
                                        ),
                                    )
                                    ui.Button(
                                        "Close " + side,
                                        clicked_fn=lambda s=side: control.gripper(
                                            s, False
                                        ),
                                    )
                    ui.Label(
                        "Joint commands are limited and slew-rate controlled.",
                        height=20,
                    )
                    ui.Label("Self-collision OFF; payload not validated.", height=20)

    def home(self):
        self.control.home()
        for name, model in self.models.items():
            value = self.control.targets[name]
            model.set_value(value * 1000 if name == "lift_joint" else math.degrees(value))
