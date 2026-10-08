"""Import the separate relay URDF and apply the existing PhysX robot preparation."""
import json
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import ROOT
from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'multi_gpu': False})
try:
    from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig
    from robot_asset import build_robot
    cfg = json.loads((ROOT / 'config/relay_robot_config.json').read_text())
    out = ROOT / 'assets/robots/generated/relay'
    out.mkdir(parents=True, exist_ok=True)
    config = URDFImporterConfig(urdf_path=str(ROOT / 'assets/robots' / cfg['source_urdf']),
        usd_path=str(out / 'imported'), fix_base=False, merge_fixed_joints=True,
        collision_type='Convex Decomposition', allow_self_collision=False,
        joint_drive_type='force', joint_target_type='position',
        override_joint_stiffness=400.0, override_joint_damping=40.0)
    result = URDFImporter(config).import_urdf()
    if not result:
        raise RuntimeError('Relay URDF import failed')
    build_robot(result, cfg=cfg, out=out)
except Exception:
    import traceback
    traceback.print_exc()
    app.close(exit_code=1)
    raise
else:
    app.close()
