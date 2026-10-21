"""Render a receiving-slot fixture through B's calibrated URDF camera."""
import json
from pathlib import Path
from common import ROOT, OUTPUTS
from isaacsim import SimulationApp

app=SimulationApp({'headless':True,'multi_gpu':False})
try:
    import omni.usd
    import omni.replicator.core as rep
    from pxr import Gf, UsdGeom, UsdPhysics
    from PIL import Image
    scene=json.loads((ROOT/'config/relay_config.json').read_text())
    meta=json.loads((ROOT/'assets/robots/generated/relay/robot_metadata.json').read_text())
    calibration=json.loads((ROOT/'config/grasp/runtime.sim.json').read_text())['simulation']['camera']
    omni.usd.get_context().open_stage(str(ROOT/'assets/generated/relay/main.usda'))
    for _ in range(12):app.update()
    stage=omni.usd.get_context().get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
        if prim.IsA(UsdPhysics.Joint) or prim.IsA(UsdPhysics.Scene):prim.SetActive(False)
    # A separate diagnostic fixture, never written into the production scene.
    box=stage.GetPrimAtPath('/World/Crates/Crate_12')
    transform=Gf.Matrix4d(1)
    transform.SetTranslateOnly(Gf.Vec3d(*scene['handoff']['slots'][1],scene['handoff']['size'][2]))
    UsdGeom.Xformable(box).MakeMatrixXform().Set(transform)
    camera=UsdGeom.Camera.Define(stage,'/World/Robots/Robot_B'+meta['frames']['head_d455_color_optical_frame']+'/ViewValidation')
    camera.AddRotateXOp().Set(180.)
    camera.CreateFocalLengthAttr(24.)
    camera.CreateHorizontalApertureAttr(24*calibration['width']/calibration['fx'])
    camera.CreateVerticalApertureAttr(24*calibration['height']/calibration['fy'])
    camera.CreateClippingRangeAttr(Gf.Vec2f(.03,10.))
    out=OUTPUTS/'handoff_long_edges'
    out.mkdir(exist_ok=True)
    for name,path in [('robot_b_receiving_box',str(camera.GetPath())),('overview','/World/Overview')]:
        product=rep.create.render_product(path,(calibration['width'],calibration['height']))
        rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach(product)
        for _ in range(8):rep.orchestrator.step(rt_subframes=4,delta_time=0.)
        Image.fromarray(rgb.get_data()).save(out/(name+'.png'))
        rgb.detach(product);product.destroy()
    print('HANDOFF_VIEW_RENDERED',out,flush=True)
finally:
    app.close()
