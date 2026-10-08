"""Convert the supplied visualization URDF into an editable floating-base USD."""

import json
import hashlib
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import ROOT
from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "multi_gpu": False})
try:
    from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig
    from pxr import Usd, UsdPhysics

    robot_cfg = json.loads((ROOT / "config/robot_config.json").read_text())
    source = ROOT / "assets/robots" / robot_cfg["source_urdf"]
    output = ROOT / "assets/robots/generated"
    config = URDFImporterConfig(
        urdf_path=str(source),
        usd_path=str(output / "imported"),
        fix_base=False,
        merge_fixed_joints=True,
        collision_type="Convex Decomposition",
        allow_self_collision=False,
        joint_drive_type="force",
        joint_target_type="position",
        override_joint_stiffness=400.0,
        override_joint_damping=40.0,
    )
    if "--reuse-import" in sys.argv:
        cached = json.loads((output / "import_summary.json").read_text())
        if cached.get("source_sha256") != hashlib.sha256(source.read_bytes()).hexdigest():
            raise RuntimeError("Imported URDF changed: rebuild without --reuse-import")
        result = cached["asset"]
    else:
        result = URDFImporter(config).import_urdf()
    if not result:
        raise RuntimeError("URDF importer returned no asset")
    stage = Usd.Stage.Open(result)
    summary = {
        "asset": str(result),
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "default_prim": str(stage.GetDefaultPrim().GetPath()),
        "bodies": [],
        "joints": [],
    }
    for p in stage.Traverse():
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            summary["bodies"].append(
                {
                    "path": str(p.GetPath()),
                    "mass": UsdPhysics.MassAPI(p).GetMassAttr().Get(),
                }
            )
        if p.IsA(UsdPhysics.Joint):
            summary["joints"].append(
                {
                    "path": str(p.GetPath()),
                    "type": p.GetTypeName(),
                    "body0": [
                        str(x) for x in UsdPhysics.Joint(p).GetBody0Rel().GetTargets()
                    ],
                    "body1": [
                        str(x) for x in UsdPhysics.Joint(p).GetBody1Rel().GetTargets()
                    ],
                }
            )
    (output / "import_summary.json").write_text(json.dumps(summary, indent=2))
    from robot_asset import build_robot

    build_robot(result)
    print("ROBOT_IMPORTED", result, flush=True)
except Exception:
    import traceback

    traceback.print_exc()
    app.close(exit_code=1)
else:
    app.close()
