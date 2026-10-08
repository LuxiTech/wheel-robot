"""Dependency-free configuration shared by Blender and Isaac Python."""

import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config/scene_config.yaml"
ASSETS = ROOT / "assets/generated"
OUTPUTS = ROOT / "outputs"


def load_config(path=CONFIG):
    # JSON is a YAML 1.2 subset; avoid installing PyYAML in either runtime.
    c = json.loads(Path(path).read_text())
    if c["schema_version"] != 1:
        raise ValueError("Unsupported schema_version")
    for values in [c["room"]["size"], c["box"]["size"]]:
        if len(values) != 3 or not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError("Dimensions must be finite, positive XYZ triples")
    if c["box"]["count"] not in (1, 2, 3):
        raise ValueError("box.count must be 1, 2 or 3")
    if c["box"]["mass"] <= 0 or c["physics"]["dt"] <= 0:
        raise ValueError("Mass and timestep must be positive")
    if c["task"]["source"] == c["task"]["target"]:
        raise ValueError("Source and target must be different tables")
    if not 0 < c["room"]["aisle_width"] < c["room"]["size"][1]:
        raise ValueError("Invalid aisle width")
    if any(c["box"][k] < 0 for k in ("static_friction", "dynamic_friction")):
        raise ValueError("Friction must be nonnegative")
    if not 0 <= c["box"]["restitution"] <= 1:
        raise ValueError("Restitution must be between zero and one")
    ids = [t["id"] for t in c["tables"]]
    if len(ids) != len(set(ids)):
        raise ValueError("Duplicate table ID")
    for key in ["source", "target"]:
        if c["task"][key] not in ids:
            raise ValueError(f"Unknown task {key}")
    for t in c["tables"]:
        if not t["id"].isidentifier() or any(v <= 0 for v in t["size"]):
            raise ValueError("Invalid table ID or size")
        if abs(t["xy"][1]) - t["size"][1] / 2 < c["room"]["aisle_width"] / 2:
            raise ValueError("Table intrudes into central aisle")
        for axis in (0, 1):
            if (
                abs(t["xy"][axis]) + t["size"][axis] / 2
                >= c["room"]["size"][axis] / 2 - 0.1
            ):
                raise ValueError("Table intersects a room wall")
    for key in ("source", "target"):
        t = table_by_id(c, c["task"][key])
        occupied_width = (
            c["box"]["count"] * c["box"]["size"][0] + (c["box"]["count"] - 1) * 0.08
        )
        if (
            occupied_width > t["size"][0] - 0.04
            or c["box"]["size"][1] > t["size"][1] - 0.04
        ):
            raise ValueError("Boxes do not fit on the source/target table")
    return c


def table_by_id(c, name):
    return next(t for t in c["tables"] if t["id"] == name)


def box_start(c, i=0, target=False, drop=0.04):
    t = table_by_id(c, c["task"]["target" if target else "source"])
    spacing = c["box"]["size"][0] + 0.08
    return [
        t["xy"][0] + (i - (c["box"]["count"] - 1) / 2) * spacing,
        t["xy"][1],
        t["size"][2] + c["box"]["size"][2] / 2 + drop,
    ]
