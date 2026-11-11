"""Resolve the selected robot assets without changing historical model files."""

import json
from pathlib import Path


def selected_model(root):
    root = Path(root).resolve()
    config = json.loads((root / "config/robots/robot_model.json").read_text())
    return {
        name: root / config[name]
        for name in ("urdf", "source", "sdk_chains", "gripper", "contacts")
    }


def capture_model(root, report):
    """Saved captures retain the model version that generated their poses."""
    root = Path(root).resolve()
    if report.get("model_resources"):
        return {name: root / path for name, path in report["model_resources"].items()}
    for source in (root / "config/robots").glob("r680_lift_openarm_claw*.source.json"):
        manifest = json.loads(source.read_text())
        if manifest["urdf_sha256"] == report["urdf_sha256"]:
            return dict(
                urdf=root / manifest["urdf"],
                source=source,
                sdk_chains=root / "config/robots/calibration/sdk_arm_chains.json",
                gripper=root / "config/robots/calibration/gripper_v1.json",
            )
    raise ValueError("No matching model source for this saved capture")
