"""Force-driven handoff pusher and pose-based simulated occupancy sensors."""
import numpy as np

ROOT = '/World/HandoffPusher'


def geometry(config):
    left, right = config['handoff']['slots']
    sx, sy, sz = config['crate']['size']
    thickness = .025
    home = np.array([left[0]-sx/2-thickness/2-.035, left[1], config['handoff']['size'][2]+.075])
    stroke = right[0]-sx/2-thickness/2-home[0]
    return home, stroke, (thickness, sy-.02, .12)


def build(stage, config):
    from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
    home, stroke, size = geometry(config)
    UsdGeom.Xform.Define(stage, ROOT)

    def cube(path, position, size, color, collision=True):
        obj = UsdGeom.Cube.Define(stage, path)
        obj.CreateSizeAttr(1.)
        obj.AddTranslateOp().Set(Gf.Vec3d(*map(float, position)))
        obj.AddScaleOp().Set(Gf.Vec3d(*map(float, size)))
        obj.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        if collision:
            UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
            api = PhysxSchema.PhysxCollisionAPI.Apply(obj.GetPrim())
            api.CreateContactOffsetAttr(.001)
            api.CreateRestOffsetAttr(0.)
        return obj

    body = UsdGeom.Xform.Define(stage, ROOT+'/Slider')
    body.AddTranslateOp().Set(Gf.Vec3d(*map(float, home)))
    UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    mass = UsdPhysics.MassAPI.Apply(body.GetPrim())
    mass.CreateMassAttr(2.)
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(.015,.015,.015))
    PhysxSchema.PhysxContactReportAPI.Apply(body.GetPrim()).CreateThresholdAttr(0.)
    cube(ROOT+'/Slider/Plate', (0,0,0), size, (.95,.35,.025))
    cube(ROOT+'/Slider/Bracket', (0,.22,.04), (.025,.2,.025), (.22,.25,.28))
    cube(ROOT+'/Slider/Carriage', (0,.30,0), (.10,.08,.10), (.95,.35,.025))
    cx, cy = config['handoff']['center']
    height = config['handoff']['size'][2]
    cube(ROOT+'/Rail', (cx,cy+.30,height+.035), (1.30,.045,.04), (.22,.25,.28), False)
    for sign in (-1,1):
        cube(ROOT+('/GuideFront' if sign<0 else '/GuideRear'),
             (cx,cy+sign*(config['crate']['size'][1]/2+.02),height+.022),
             (1.28,.015,.04), (.4,.45,.5))
    joint = UsdPhysics.PrismaticJoint.Define(stage, ROOT+'/SlideJoint')
    joint.CreateBody1Rel().SetTargets([body.GetPath()])
    joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float,home)))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0,0,0))
    joint.CreateAxisAttr('X')
    joint.CreateLowerLimitAttr(0.)
    joint.CreateUpperLimitAttr(float(stroke))
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), 'linear')
    drive.CreateTypeAttr('force')
    drive.CreateStiffnessAttr(5000.)
    drive.CreateDampingAttr(160.)
    drive.CreateMaxForceAttr(config['handoff']['pusher']['max_force'])
    drive.CreateTargetPositionAttr(0.)
    drive.CreateTargetVelocityAttr(0.)


class HandoffPusher:
    def __init__(self, stage, config, slider, crates):
        from pxr import UsdPhysics
        self.config, self.slider, self.crates = config, slider, crates
        self.home, self.stroke, _ = geometry(config)
        self.settings = config['handoff']['pusher']
        self.target_attr = UsdPhysics.DriveAPI(stage.GetPrimAtPath(ROOT+'/SlideJoint'), 'linear').GetTargetPositionAttr()
        # Standalone environment keeps its original automatic behavior. A relay
        # runtime installs these callbacks before allowing robot motion.
        self.start_permitted = lambda index: True
        self.motion_permitted = lambda: True
        self.sensor_period = 0.
        self.reset()

    def reset(self):
        self.state = 'WAITING'
        self.target = self.timer = self.elapsed = 0.
        self.selected = None
        self.completed = 0
        self.left_present = self.right_present = False
        self.sensor_elapsed = float('inf')
        self.cached_left = self.cached_right = []
        self.target_attr.Set(0.)

    def occupants(self, slot, exclude=None):
        # Conservative oriented-box AABB overlap, including boxes partly in a slot.
        from pxr import Gf
        sx, sy, sz = self.config['crate']['size']
        cx, cy = self.config['handoff']['slots'][slot]
        height = self.config['handoff']['size'][2]
        found = []
        for i, crate in enumerate(self.crates):
            if i == exclude:
                continue
            p, q = crate.get_world_pose()
            rotation = np.array(Gf.Matrix3d(Gf.Quatd(float(q[0]),Gf.Vec3d(*map(float,q[1:])))))
            center = p + np.array([0,0,sz/2]) @ rotation
            extent = np.array([sx,sy,sz])/2 @ np.abs(rotation)
            if (abs(center[0]-cx)<extent[0]+sx/2+.025 and
                abs(center[1]-cy)<extent[1]+sy/2+.025 and
                center[2]+extent[2]>height+.01 and center[2]-extent[2]<height+sz+.06):
                found.append(i)
        return found

    def ready(self, index):
        crate = self.crates[index]
        p, q = crate.get_world_pose()
        left = self.config['handoff']['slots'][0]
        return (np.linalg.norm(p[:2]-left)<.035 and
                abs(p[2]-self.config['handoff']['size'][2])<.015 and
                max(abs(q[0]),abs(q[3]))>.998 and np.linalg.norm(crate.get_linear_velocity())<.025 and
                np.linalg.norm(crate.get_angular_velocity())<.05)

    def update(self, dt):
        self.sensor_elapsed += dt
        if self.sensor_elapsed >= self.sensor_period:
            self.cached_left, self.cached_right = self.occupants(0), self.occupants(1)
            self.sensor_elapsed = 0.
        left = self.cached_left
        right = [i for i in self.cached_right if self.state == 'WAITING' or i != self.selected]
        self.left_present, self.right_present = bool(left), bool(self.cached_right)
        actual = float(self.slider.get_world_pose()[0][0]-self.home[0])
        if self.state == 'WAITING':
            candidate = left[0] if len(left)==1 and not right and self.ready(left[0]) and self.start_permitted(left[0]) else None
            self.timer = self.timer+dt if candidate is not None and candidate==self.selected else 0.
            self.selected = candidate
            if candidate is not None and self.timer >= self.settings['settle_seconds']:
                self.state, self.elapsed, self.timer = 'PUSHING', 0., 0.
        elif self.state in ('PUSHING', 'BLOCKED'):
            self.elapsed += dt
            if right or not self.motion_permitted():
                self.state = 'BLOCKED'
                self.target = max(0.,min(actual,self.stroke))
            else:
                self.state = 'PUSHING'
                self.target = min(self.stroke,self.target+self.settings['speed']*dt)
                if self.target>=self.stroke and abs(actual-self.stroke)<.008:
                    self.timer += dt
                    if self.timer>.5:
                        self.state = 'RETURNING'
                        p = self.crates[self.selected].get_world_pose()[0]
                        if np.linalg.norm(p[:2]-self.config['handoff']['slots'][1])<.035:
                            self.completed += 1
                        else:
                            self.state = 'FAULT'
                elif self.elapsed>20. and self.state!='BLOCKED':
                    self.state = 'FAULT'
                    self.target = max(0.,min(actual,self.stroke))
        elif self.state == 'RETURNING':
            if not self.motion_permitted():
                self.target = max(0.,min(actual,self.stroke))
                self.target_attr.Set(float(self.target))
                return
            self.target = max(0.,self.target-self.settings['return_speed']*dt)
            if self.target==0. and abs(actual)<.006:
                self.state, self.selected, self.timer = 'WAITING', None, 0.
        self.target_attr.Set(float(self.target))
