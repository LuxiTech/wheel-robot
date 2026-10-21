"""Shared capture/execution scene. Ground truth stays in the simulation layer."""
import json
import math
from pathlib import Path
import numpy as np


def build_scene(cfg, measured=None, base_state=None):
    import omni.usd
    from pxr import Gf, UsdGeom, UsdPhysics, UsdLux, PhysxSchema
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation
    from scipy.spatial.transform import Rotation
    s = cfg['simulation']
    meta = json.loads(Path(s['robot_metadata']).read_text())
    if Path(cfg['urdf']).resolve() != (Path('assets/robots')/meta['source_urdf']).resolve():
        raise ValueError('Simulation robot and perception URDF mismatch')
    world = World(physics_dt=1/120, rendering_dt=1/120, stage_units_in_meters=1.)
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    def cube(path, position, size, color):
        obj = UsdGeom.Cube.Define(stage,path)
        obj.CreateSizeAttr(1.)
        obj.AddTranslateOp().Set(Gf.Vec3d(*position))
        obj.AddScaleOp().Set(Gf.Vec3d(*size))
        obj.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
    cube('/World/Ground',[0,0,-.05],[20,20,.1],[.24,.25,.27])
    base_z = s['base_height']
    xyz = np.asarray(s['crate_position'],float)+[0,0,base_z]
    # Support is part of the actual sim scene, not inferred from a pose result.
    cube('/World/Support',[xyz[0],xyz[1],xyz[2]/2],[.32,1.3,xyz[2]],[.4,.32,.24])
    light = UsdLux.DomeLight.Define(stage,'/World/Light')
    light.CreateIntensityAttr(1200.)
    root = stage.DefinePrim('/World/Robot','Xform')
    root.GetReferences().AddReference(str(Path(s['robot_usd']).resolve()))
    robot = world.scene.add(SingleArticulation('/World/Robot'+meta['root_path'],name='sim_robot'))
    robot.set_default_state(position=np.array(base_state['position'] if base_state else [0.,0.,base_z]),
                            orientation=np.array(base_state['orientation'] if base_state else [1.,0.,0.,0.]))
    target = UsdGeom.Xform.Define(stage,'/World/Target')
    target.GetPrim().GetReferences().AddReference(str(Path(s['crate_usd']).resolve()))
    UsdPhysics.RigidBodyAPI(target.GetPrim()).CreateKinematicEnabledAttr(not s.get('carry',{}).get('enabled',False))
    target.AddTranslateOp().Set(Gf.Vec3d(*xyz))
    target.AddRotateZOp().Set(math.degrees(s['crate_yaw']))
    for prim in stage.Traverse():
        if str(prim.GetPath()).startswith('/World/Robot') and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
        if str(prim.GetPath()).startswith('/World/Robot') and prim.IsA(UsdPhysics.RevoluteJoint):
            name = prim.GetName()
            gains = s.get('arm_drive_gains')
            if gains and name.startswith('openarm_') and '_joint' in name and 'finger' not in name:
                if any(not math.isfinite(gains[k]) or gains[k] <= 0 for k in ('stiffness','damping')):
                    raise ValueError('Invalid simulation arm drive gains')
                drive = UsdPhysics.DriveAPI(prim,'angular')
                drive.CreateStiffnessAttr(gains['stiffness']*math.pi/180)
                drive.CreateDampingAttr(gains['damping']*math.pi/180)
                # Keep the original URDF effort and velocity limits.
    world.reset()
    names = list(robot.dof_names)
    q = np.asarray(robot.get_joint_positions()).copy()
    requested = s['initial_joints'] if measured is None else measured
    for name,value in requested.items():
        q[names.index(name)] = value
    for name in names:
        if 'finger_joint' in name:
            q[names.index(name)] = math.pi/4 if 'left' in name else -math.pi/4
    robot.set_joint_positions(q)
    robot.set_joint_velocities(np.zeros(len(names)))
    robot.set_linear_velocity(np.zeros(3));robot.set_angular_velocity(np.zeros(3))
    truth = np.eye(4)
    truth[:3,:3] = Rotation.from_euler('z',s['crate_yaw']).as_matrix()
    truth[:3,3] = s['crate_position']
    return world,stage,robot,meta,q,truth
