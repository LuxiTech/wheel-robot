"""Native Isaac operator console: four live USD cameras and explicit task control."""
import math
import carb.settings
import omni.ui as ui
from pxr import Gf, UsdGeom
from omni.kit.widget.viewport import ViewportWidget
from common import OUTPUTS


class OperatorConsole:
    def __init__(self, stage, metadata, capture_events=False, demo=False):
        self.capture_events = capture_events
        self.demo = demo
        self.captured_phases = set()
        self.runtime = None
        self.start_requested = False
        self.reset_requested = False
        self.state = 'READY'
        self.widgets = []
        self.multi_camera = not demo
        self.cameras = []
        for name, frame, fov in (
            ('Head camera / D455', 'head_d455_color_optical_frame', 87),
            ('Left wrist / EB435i', 'left_wrist_eb435i_color_optical_frame', 84),
            ('Right wrist / EB435i', 'right_wrist_eb435i_color_optical_frame', 84),
        ):
            path = '/World/Robot'+metadata['frames'][frame]+'/OperatorCamera'
            camera = UsdGeom.Camera.Define(stage,path)
            # ROS optical: +Z forward, +Y down. USD camera: -Z forward, +Y up.
            camera.AddRotateXOp().Set(180)
            camera.CreateHorizontalApertureAttr(36.)
            camera.CreateVerticalApertureAttr(36.*9/16)
            camera.CreateFocalLengthAttr(18/math.tan(math.radians(fov/2)))
            camera.CreateClippingRangeAttr(Gf.Vec2f(.025,100.))
            self.cameras.append((name,path))
        camera = UsdGeom.Camera.Define(stage,'/World/OperatorCameras/Overview')
        camera.CreateProjectionAttr('perspective' if demo else 'orthographic')
        camera.CreateHorizontalApertureAttr(36. if demo else 220.)
        camera.CreateVerticalApertureAttr(27. if demo else 165.)
        if demo:
            camera.CreateFocalLengthAttr(24.)
        camera.CreateClippingRangeAttr(Gf.Vec2f(.1,100.))
        camera.MakeMatrixXform().Set(Gf.Matrix4d().SetLookAt(
            Gf.Vec3d(6.8,.7,3.1) if demo else Gf.Vec3d(0,0,5.15),Gf.Vec3d(4.5,-2.1,.55) if demo else Gf.Vec3d(0,0,0),Gf.Vec3d(0,0,1) if demo else Gf.Vec3d(0,1,0)).GetInverse())
        self.cameras.insert(1,('Global / workbench' if demo else 'Global / overhead',str(camera.GetPath())))
        self.window = ui.Window('Robot Operator Console',width=1240,height=750,flags=ui.WINDOW_FLAGS_NO_COLLAPSE | ui.WINDOW_FLAGS_NO_CLOSE)
        with self.window.frame:
            with ui.VStack(spacing=6,style={'background_color':0xff201a16,'font_size':16}):
                with ui.HStack(height=40,spacing=12):
                    ui.Label('R680 | 3-BOX DEMO' if demo else 'R680  |  SLAM + IK',width=240,style={'font_size':20})
                    self.start_button = ui.Button('START',width=180,clicked_fn=self.request_start,
                                                 style={'background_color':0xff547e28,'font_size':20})
                    self.reset_button = ui.Button('RESET',width=180,clicked_fn=self.request_reset,
                                                 style={'background_color':0xff69533b,'font_size':20})
                    self.view_button = ui.Button('4 CAMERAS' if demo else 'OVERVIEW',width=130,clicked_fn=self.toggle_views)
                    status = ui.StringField(read_only=True)
                    self.status_model = status.model
                    self.status_model.add_value_changed_fn(self.on_phase)
                ui.Label('3 floor boxes -> workbench | START: complete batch | RESET: restore all boxes' if demo else 'START: one delivery   |   RESET: restore robot, box and map   |   Release, then reverse 0.50 m',height=22)
                self.views_frame = ui.Frame()
                self.views_frame.set_build_fn(self.build_views)
        # Fill the main workspace, retaining the native app's window management.
        viewport = ui.Workspace.get_window('Viewport')
        if viewport is not None:
            viewport.visible = True
        self.window.deferred_dock_in('Viewport',ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)
        self.window.focus()

    def build_views(self):
        for widget in self.widgets:
            widget.destroy()
        self.widgets = []
        self.active_cameras = self.cameras if self.multi_camera else [self.cameras[1]]
        with ui.VStack(spacing=6):
            for row in range(2 if self.multi_camera else 1):
                with ui.HStack(spacing=6):
                    for name, path in self.active_cameras[row*2:row*2+2]:
                        with ui.VStack(spacing=2):
                            ui.Label(name,height=24,style={'font_size':18})
                            widget = ViewportWidget(camera_path=path, resolution=(640,480) if 'Global' in name else (640,360))
                            widget.viewport_api.resolution_scale = 1.0
                            widget.viewport_api.display_render_var = 'LdrColor'
                            self.widgets.append(widget)
                            for guide in ('grid','axis','selection'):
                                carb.settings.get_settings().set_bool(
                                    f'/persistent/app/viewport/{widget.viewport_api.id}/guide/{guide}/visible',False)

    def toggle_views(self):
        self.multi_camera = not self.multi_camera
        self.view_button.text = 'OVERVIEW' if self.multi_camera else '4 CAMERAS'
        self.views_frame.rebuild()

    def activate(self):
        # Stage and Hydra initialization may finish after widgets are built.
        # Rebind once rendering is ready, and keep the demo tab in front.
        for widget, (_, path) in zip(self.widgets, self.active_cameras):
            alternate = self.cameras[1][1] if path == self.cameras[0][1] else self.cameras[0][1]
            widget.viewport_api.camera_path = alternate
            widget.viewport_api.camera_path = path
            widget.viewport_api.resolution_scale = 1.0
        if self.demo:
            for name in ('Stage', 'Property', 'Content', 'Content Browser', 'Console', 'Render Settings'):
                window = ui.Workspace.get_window(name)
                if window is not None:
                    window.visible = False
        # Restore the native scene tab even if a previous demo hid it.
        # Keep it alongside the four-camera console for scene inspection.
        viewport = ui.Workspace.get_window('Viewport')
        if viewport is not None and not viewport.visible:
            viewport.visible = True
        self.window.visible = True
        self.window.focus()

    def on_phase(self, model):
        if self.demo and self.capture_events and self.runtime is not None:
            key = (self.runtime.box_index, self.runtime.phase)
            if key[1] in ('CARRY', 'RETREAT_COMPLETE') and key not in self.captured_phases:
                self.captured_phases.add(key)
                self.capture(f'console_box{key[0]+1}_{key[1].lower()}')
            return
        if self.capture_events and model.as_string in ('CARRY', 'RETREAT_COMPLETE'):
            self.capture('console_'+model.as_string.lower())

    def update_views(self, frame):
        # Keep the overview live; auxiliary views refresh every third frame.
        for widget, (name, _) in zip(self.widgets, self.active_cameras):
            widget.viewport_api.updates_enabled = 'Global' in name or frame % 3 == 0

    def request_start(self):
        if self.state == 'READY' and not self.reset_requested:
            self.start_requested = True
            self.start_button.enabled = False

    def request_reset(self):
        self.reset_requested = True
        self.start_requested = False
        self.start_button.enabled = False
        self.status_model.set_value('RESETTING')
        if self.runtime is not None:
            self.runtime.reset_requested = True
            self.runtime.ctl.stop()

    def set_state(self,state):
        self.state = state
        if state != 'RUNNING':
            for widget in self.widgets:
                widget.viewport_api.updates_enabled = True
        if state == 'READY':
            self.captured_phases.clear()
        self.start_button.enabled = state == 'READY'
        if state == 'COMPLETE' and self.demo and self.runtime is not None:
            self.status_model.set_value(f'3/3 COMPLETE | {self.runtime.time:.1f}s | RESET to restart')
            return
        self.status_model.set_value({
            'READY':'READY - press START',
            'COMPLETE':'COMPLETE - released and backed away; RESET to restart'
        }.get(state,state))

    def capture(self,name):
        self.window.focus()
        import omni.kit.renderer.capture
        import omni.appwindow
        capture = omni.kit.renderer.capture.acquire_renderer_capture_interface()
        capture.capture_next_frame_swapchain(str(OUTPUTS/(('demo_' if self.demo else '')+name+'.png')),
                                             omni.appwindow.get_default_app_window())
