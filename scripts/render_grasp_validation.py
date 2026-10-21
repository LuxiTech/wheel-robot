"""Render a retained diagnostic physics snapshot without advancing simulation."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import ROOT, OUTPUTS
from relay_state_ipc import StateChannel
p=argparse.ArgumentParser();p.add_argument('--case',required=True);a=p.parse_args()
folder=OUTPUTS/'pinch_grasp_validation'/a.case
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'multi_gpu':False})
try:
    import omni.usd
    import omni.replicator.core as rep
    from pxr import Gf,UsdGeom,UsdPhysics
    from PIL import Image
    omni.usd.get_context().open_stage(str(ROOT/'assets/generated/relay/main.usda'))
    for _ in range(8):app.update()
    stage=omni.usd.get_context().get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
        if prim.IsA(UsdPhysics.Joint):prim.SetActive(False)
        if prim.IsA(UsdPhysics.Scene):prim.SetActive(False)
    channel=StateChannel(folder/'state.bin');state=channel.read();channel.close()
    for path,pose in state['poses']:
        xf=UsdGeom.Xformable(stage.GetPrimAtPath(path));op=xf.MakeMatrixXform();xf.SetResetXformStack(True)
        t=Gf.Matrix4d(1);t.SetRotate(Gf.Quatd(pose[3],Gf.Vec3d(*pose[4:])))
        t.SetTranslateOnly(Gf.Vec3d(*pose[:3]));op.Set(t)
    report=json.loads((folder/'relay_demo_report.json').read_text())
    target=report['hand_goals']['A']['left'];focus=[target[i][3] for i in range(3)]
    for name,delta in [('front',[.65,-.75,.35]),('side',[.05,-.85,.05])]:
        camera=UsdGeom.Camera.Define(stage,'/World/Review_'+name)
        camera.MakeMatrixXform().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*[x+d for x,d in zip(focus,delta)]),Gf.Vec3d(*focus),Gf.Vec3d(0,0,1)).GetInverse())
        camera.CreateFocalLengthAttr(50);camera.CreateHorizontalApertureAttr(36);camera.CreateClippingRangeAttr(Gf.Vec2f(.01,100))
        product=rep.create.render_product(str(camera.GetPath()),(1200,900));rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach(product)
        for _ in range(8):rep.orchestrator.step(rt_subframes=4,delta_time=0.)
        Image.fromarray(rgb.get_data()).save(folder/(name+'.png'));rgb.detach(product);product.destroy()
finally:
    app.close()
