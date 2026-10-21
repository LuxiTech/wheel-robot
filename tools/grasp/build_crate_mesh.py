"""Generate a known crate mesh from existing local collider assets, in metres.

This is a commissioning model, not a claim that every real crate is identical.
"""
import argparse
import json
from pathlib import Path
import trimesh
p=argparse.ArgumentParser()
p.add_argument('--colliders',default='assets/generated/relay/crate_colliders.json')
p.add_argument('--output',default='assets/generated/grasp/crate.obj')
a=p.parse_args()
parts=[]
for item in json.loads(Path(a.colliders).read_text()):
    part=trimesh.creation.box(extents=item['size']);part.apply_translation(item['position']);parts.append(part)
out=Path(a.output)
if out.exists(): raise SystemExit('Output already exists; select a new filename')
out.parent.mkdir(parents=True,exist_ok=True)
trimesh.util.concatenate(parts).export(out)
print(out)
