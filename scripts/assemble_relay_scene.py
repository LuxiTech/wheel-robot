"""Assemble Blender meshes, compound tote collisions and two articulated robots."""
import json
import math
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import ROOT
from relay_layout import slot_position, pallet_yaw
from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'multi_gpu': False})
try:
    import numpy as np
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade, UsdLux, PhysxSchema
    out = ROOT / 'assets/generated/relay'
    c = json.loads((ROOT / 'config/relay_config.json').read_text())
    if c != json.loads((out / 'scene_config.json').read_text()):
        raise RuntimeError('Relay config changed: rebuild Blender assets')

    def make_stage(path, root):
        stage = Usd.Stage.CreateNew(str(path))
        stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, root).GetPrim())
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.)
        return stage

    def shape(stage, path, part, material):
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.)
        cube.AddTranslateOp().Set(Gf.Vec3d(*part['position']))
        cube.AddScaleOp().Set(Gf.Vec3d(*part['size']))
        cube.CreateVisibilityAttr('invisible')
        prim = cube.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        api.CreateContactOffsetAttr(.001)
        api.CreateRestOffsetAttr(0.)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose='physics')

    crate = make_stage(out / 'crate.usda', '/Crate')
    root = crate.GetDefaultPrim()
    UsdPhysics.RigidBodyAPI.Apply(root)
    api = PhysxSchema.PhysxRigidBodyAPI.Apply(root)
    api.CreateSolverPositionIterationCountAttr(32)
    api.CreateSolverVelocityIterationCountAttr(8)
    api.CreateMaxDepenetrationVelocityAttr(1.)
    PhysxSchema.PhysxContactReportAPI.Apply(root).CreateThresholdAttr(0.)
    parts = json.loads((out / 'crate_colliders.json').read_text())
    # Volume-weighted compound approximation; includes small overlapping ribs.
    volumes = np.array([np.prod(p['size']) for p in parts])
    masses = c['crate']['mass'] * volumes / volumes.sum()
    centers = np.array([p['position'] for p in parts])
    center = (centers * masses[:, None]).sum(axis=0) / masses.sum()
    diagonal = np.zeros(3)
    for part, mass, pos in zip(parts, masses, centers):
        sx, sy, sz = part['size']
        r = pos-center
        diagonal += mass*np.array([sy*sy+sz*sz, sx*sx+sz*sz, sx*sx+sy*sy])/12
        diagonal += mass*(np.dot(r,r)-r*r)
    ma = UsdPhysics.MassAPI.Apply(root)
    ma.CreateMassAttr(c['crate']['mass'])
    ma.CreateCenterOfMassAttr(Gf.Vec3f(*center))
    ma.CreateDiagonalInertiaAttr(Gf.Vec3f(*diagonal))
    mat = UsdShade.Material.Define(crate, '/Crate/PhysicsMaterial')
    material = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    material.CreateStaticFrictionAttr(c['crate']['static_friction'])
    material.CreateDynamicFrictionAttr(c['crate']['dynamic_friction'])
    material.CreateRestitutionAttr(0.)
    UsdGeom.Xform.Define(crate, '/Crate/Visuals').GetPrim().GetReferences().AddReference('crate_visual.usdc')
    for part in parts:
        shape(crate, '/Crate/Collisions/'+part['name'], part, mat)
    crate.GetRootLayer().Save()

    stage = make_stage(out / 'main.usda', '/World')
    UsdGeom.Xform.Define(stage, '/World/Environment').GetPrim().GetReferences().AddReference('environment.usdc')
    ground_mat = UsdShade.Material.Define(stage, '/World/Materials/Ground')
    m = UsdPhysics.MaterialAPI.Apply(ground_mat.GetPrim())
    m.CreateStaticFrictionAttr(.8)
    m.CreateDynamicFrictionAttr(.6)
    m.CreateRestitutionAttr(0.)
    for part in json.loads((out / 'static_colliders.json').read_text()):
        shape(stage, '/World/Collisions/'+part['name'], part, ground_mat)
    from relay_pusher import build as build_pusher
    build_pusher(stage, c)
    ps = UsdPhysics.Scene.Define(stage, '/World/PhysicsScene')
    ps.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1))
    ps.CreateGravityMagnitudeAttr(c['physics']['gravity'])
    phys = PhysxSchema.PhysxSceneAPI.Apply(ps.GetPrim())
    phys.CreateSolverTypeAttr('TGS')
    phys.CreateTimeStepsPerSecondAttr(round(1/c['physics']['dt']))
    meta = json.loads((ROOT / 'assets/robots/generated/relay/robot_metadata.json').read_text())
    for robot in c['robots']:
        prim = UsdGeom.Xform.Define(stage, '/World/Robots/'+robot['id'])
        prim.GetPrim().GetReferences().AddReference('../../robots/generated/relay/robot.usdc')
        prim.AddTranslateOp().Set(Gf.Vec3d(*robot['position']))
        prim.AddRotateZOp().Set(math.degrees(robot['yaw']))
        prim.GetPrim().SetCustomDataByKey('relay_role', robot['role'])
    initial = []
    stack = c['source_stack']
    px, py = c['pallet']['source']
    for layer in range(stack['layers']):
        for row in range(stack['rows']):
            for col in range(stack['columns']):
                index = len(initial)
                pos = slot_position(c, 'source', col, row, layer)
                root = f'/World/Crates/Crate_{index:02d}'
                obj = UsdGeom.Xform.Define(stage, root)
                obj.GetPrim().GetReferences().AddReference('crate.usda')
                obj.AddTranslateOp().Set(Gf.Vec3d(*pos))
                obj.AddRotateZOp().Set(math.degrees(pallet_yaw(c)))
                obj.GetPrim().SetCustomDataByKey('stack_layer', layer)
                initial.append(dict(path=root, position=pos, yaw=pallet_yaw(c), layer=layer, row=row, column=col))
    for i, pos in enumerate(c['handoff']['slots']):
        frame = UsdGeom.Xform.Define(stage, f'/World/TaskFrames/Handoff_{i}')
        frame.AddTranslateOp().Set(Gf.Vec3d(*pos, c['handoff']['size'][2]))
        frame.GetPrim().SetCustomDataByKey('occupancy', 'empty')
    UsdGeom.Xform.Define(stage, '/World/TaskFrames/TargetPallet').AddTranslateOp().Set(Gf.Vec3d(*c['pallet']['target'], c['pallet']['size'][2]))
    dome = UsdLux.DomeLight.Define(stage, '/World/Lighting/Ambient')
    dome.CreateIntensityAttr(350.)
    dome.CreateColorAttr(Gf.Vec3f(.9,.94,1.))
    for i, (x,y) in enumerate(((-2,-1), (2,-1), (0,2))):
        light = UsdLux.RectLight.Define(stage, f'/World/Lighting/Panel{i}')
        light.AddTranslateOp().Set(Gf.Vec3d(x,y,4))
        light.CreateWidthAttr(3.)
        light.CreateHeightAttr(2.)
        light.CreateIntensityAttr(1400.)
    camera = UsdGeom.Camera.Define(stage, '/World/Overview')
    matrix = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*c['camera']['position']), Gf.Vec3d(*c['camera']['target']), Gf.Vec3d(0,0,1)).GetInverse()
    camera.AddTransformOp().Set(matrix)
    camera.CreateFocalLengthAttr(38.)
    camera.CreateHorizontalApertureAttr(36.)
    stage.GetRootLayer().Save()
    (out/'initial_state.json').write_text(json.dumps({'crates':initial, 'robot_root_path':meta['root_path'], 'crate_com':center.tolist()},indent=2))
    check = Usd.Stage.Open(str(out/'main.usda'))
    assert not check.GetCompositionErrors(), check.GetCompositionErrors()
    assert len([p for p in check.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]) == 2
    print('RELAY_SCENE_READY', out/'main.usda', 'crates',len(initial),flush=True)
except Exception:
    import traceback
    traceback.print_exc()
    app.close(exit_code=1)
    raise
else:
    app.close()
