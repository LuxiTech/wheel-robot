"""Blender source assets for the two-robot relay cell, in metres / Z up."""
import json
import math
import sys
from pathlib import Path
import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'assets/generated/relay'
OUT.mkdir(parents=True, exist_ok=True)
C = json.loads((ROOT / 'config/relay_config.json').read_text())
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)
scene = bpy.context.scene
scene.unit_settings.system = 'METRIC'
scene.unit_settings.scale_length = 1
materials = {}
for name, color, metallic, roughness in [
    ('ToteGray', (.34, .36, .39, 1), .05, .48),
    ('PalletBlue', (.018, .085, .48, 1), .0, .4),
    ('Floor', (.52, .54, .56, 1), .05, .65),
    ('Wall', (.75, .78, .8, 1), 0, .75),
    ('Wood', (.23, .12, .065, 1), 0, .48),
    ('Steel', (.055, .06, .07, 1), .55, .3),
    ('White', (.9, .92, .95, 1), 0, .5),
    ('Amber', (.95, .47, .04, 1), 0, .5),
    ('Teal', (.02, .5, .43, 1), 0, .5),
]:
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = color
    mat.use_nodes = True
    node = mat.node_tree.nodes.get('Principled BSDF')
    node.inputs['Base Color'].default_value = color
    node.inputs['Metallic'].default_value = metallic
    node.inputs['Roughness'].default_value = roughness
    materials[name] = mat


def cube(name, position, size, material, manifest=None, bevel=.001):
    bpy.ops.mesh.primitive_cube_add(size=1, location=position)
    ob = bpy.context.object
    ob.name = name
    ob.dimensions = size
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    ob.data.materials.append(materials[material])
    if bevel:
        mod = ob.modifiers.new('MouldedEdges', 'BEVEL')
        mod.width = min(bevel, min(size)/5)
        mod.segments = 2
        ob.modifiers.new('SurfaceNormals', 'WEIGHTED_NORMAL')
    if manifest is not None:
        manifest.append(dict(name=name, position=list(position), size=list(size)))
    return ob


def export(path, root, objects):
    bpy.ops.object.select_all(action='DESELECT')
    for ob in objects:
        ob.select_set(True)
    bpy.ops.wm.usd_export(filepath=str(path), root_prim_path=root,
        selected_objects_only=True, export_materials=True, export_lights=False,
        export_cameras=False, relative_paths=True, generate_preview_surface=True)


# Crate root is the centre of its bottom support plane. It is truly hollow.
crate_parts = []
x, y, h = C['crate']['size']
w, b, rim, rh = [C['crate'][k] for k in ('wall', 'floor', 'rim_width', 'rim_height')]
wx, wy = x/2-.01-w/2, y/2-.01-w/2
cube('FloorPlate', (0, 0, .004+b/2), (x-.02, y-.02, b), 'ToteGray', crate_parts)
for sign in (-1, 1):
    suffix = 'N' if sign < 0 else 'P'
    cube('WallX'+suffix, (sign*wx, 0, (h+.008)/2), (w, y-.02, h-.008), 'ToteGray', crate_parts)
    cube('WallY'+suffix, (0, sign*wy, (h+.008)/2), (x-.02-2*w, w, h-.008), 'ToteGray', crate_parts)
    cube('RimX'+suffix, (sign*(x/2-rim/2), 0, h-rh/2), (rim, y, rh), 'ToteGray', crate_parts)
    cube('RimY'+suffix, (0, sign*(y/2-rim/2), h-rh/2), (x-2*rim, rim, rh), 'ToteGray', crate_parts)
    cube('StackFoot'+suffix, (sign*(x/2-.017), 0, .002), (.014, y-.024, .004), 'ToteGray', crate_parts)
    # Vertical corner pairs and side ribs recreate the video's moulded tote.
    for j, xx in enumerate((-x*.43, -x*.33, 0, x*.33, x*.43)):
        cube(f'RibY{suffix}_{j}', (xx, sign*(y/2-.006), h/2), (.004, .008, h-.022), 'ToteGray', crate_parts)
    for j, yy in enumerate((-y*.41, -y*.29, y*.29, y*.41)):
        cube(f'RibX{suffix}_{j}', (sign*(x/2-.006), yy, h/2), (.008, .004, h-.022), 'ToteGray', crate_parts)
    for j, zz in enumerate((.027, h*.47, h-.033)):
        cube(f'BandY{suffix}_{j}', (0, sign*(y/2-.006), zz), (x-.01, .008, .003), 'ToteGray', crate_parts)
        cube(f'BandX{suffix}_{j}', (sign*(x/2-.006), 0, zz), (.008, y-.01, .003), 'ToteGray', crate_parts)
crate_objects = list(scene.objects)
export(OUT / 'crate_visual.usdc', '/CrateVisual', crate_objects)
bpy.ops.wm.save_as_mainfile(filepath=str(OUT / 'tote.blend'))
(OUT / 'crate_colliders.json').write_text(json.dumps(crate_parts, indent=2))
# Keep the reusable crate source out of the environment export.
for ob in crate_objects:
    bpy.data.objects.remove(ob, do_unlink=True)

static_parts = []
room = C['room']
floor_x, floor_y, floor_z = room['floor_size']
wall_y, wall_h, wall_t = (room[k] for k in ('back_wall_y', 'wall_height', 'wall_thickness'))
cube('Floor', (0, 0, -floor_z/2), (floor_x, floor_y, floor_z), 'Floor', static_parts, .0)
cube('BackWall', (0, wall_y, wall_h/2), (floor_x, wall_t, wall_h), 'Wall', static_parts)


def label(name, body, position, size=.18, material='White', vertical=False):
    bpy.ops.object.text_add(location=position, rotation=(math.pi/2, 0, 0) if vertical else (0, 0, 0))
    ob = bpy.context.object
    ob.name = name
    ob.data.body = body
    ob.data.size = size
    ob.data.align_x = 'CENTER'
    ob.data.extrude = .0005
    ob.data.materials.append(materials[material])
    bpy.ops.object.convert(target='MESH')


def outline(name, center, size, material):
    cx, cy = center
    sx, sy = size
    for i, (dx, dy, a, b) in enumerate(((-sx/2, 0, .025, sy), (sx/2, 0, .025, sy), (0, -sy/2, sx, .025), (0, sy/2, sx, .025))):
        cube(name+str(i), (cx+dx, cy+dy, .002), (a, b, .004), material, bevel=0)


px, py, ph = C['pallet']['size']
for station, material in (('source', 'Amber'), ('target', 'Teal')):
    previous_objects = set(scene.objects)
    previous_colliders = len(static_parts)
    cx, cy = C['pallet'][station]
    stem = station.title()+'Pallet'
    # A physical continuous deck supports crates; narrow top ribs show the grid.
    cube(stem+'Deck', (cx, cy, ph-.012), (px, py, .024), 'PalletBlue', static_parts)
    for i in range(13):
        xx = -px/2+.015+i*(px-.03)/12
        cube(stem+f'GridX{i}', (cx+xx, cy, ph+.001), (.012, py, .002), 'PalletBlue')
    for i in range(11):
        yy = -py/2+.015+i*(py-.03)/10
        cube(stem+f'GridY{i}', (cx, cy+yy, ph+.001), (px, .012, .002), 'PalletBlue')
    for ix in (-1, 0, 1):
        for iy in (-1, 0, 1):
            cube(stem+f'Block{ix+1}{iy+1}', (cx+ix*(px/2-.08), cy+iy*(py/2-.07), (ph-.024)/2), (.13, .12, ph-.024), 'PalletBlue', static_parts)
    for ix in (-1, 0, 1):
        cube(stem+f'Runner{ix+1}', (cx+ix*(px/2-.08), cy, .009), (.13, py, .018), 'PalletBlue', static_parts)
    outline(stem+'Zone', (cx, cy), (px+.23, py+.23), material)
    label(stem+'Label', 'SOURCE / A' if station == 'source' else 'TARGET / B', (cx, cy-py/2-.24, .007), .14, material)

    angle = math.radians(C['pallet'].get('yaw_degrees', 0.))
    ca, sa = math.cos(angle), math.sin(angle)
    for ob in set(scene.objects)-previous_objects:
        dx, dy = ob.location.x-cx, ob.location.y-cy
        ob.location.x, ob.location.y = cx+ca*dx-sa*dy, cy+sa*dx+ca*dy
        ob.rotation_euler.z += angle
    if not math.isclose(angle % (math.pi/2), 0., abs_tol=1e-8):
        raise ValueError('Pallet collision export supports quarter turns only')
    for part in static_parts[previous_colliders:]:
        dx, dy = part['position'][0]-cx, part['position'][1]-cy
        part['position'][:2] = [cx+ca*dx-sa*dy, cy+sa*dx+ca*dy]
        sx, sy = part['size'][:2]
        part['size'][:2] = [abs(ca)*sx+abs(sa)*sy, abs(sa)*sx+abs(ca)*sy]

tx, ty = C['handoff']['center']
sx, sy, top = C['handoff']['size']
cube('HandoffTop', (tx, ty, top-.025), (sx, sy, .05), 'Wood', static_parts, .008)
for ix in (-1, 1):
    for iy in (-1, 1):
        cube(f'TableLeg{ix+1}{iy+1}', (tx+ix*(sx/2-.08), ty+iy*(sy/2-.08), (top-.05)/2), (.035, .035, top-.05), 'Steel', static_parts)
for i, (cx, cy) in enumerate(C['handoff']['slots']):
    for j, (dx, dy, a, b) in enumerate(((-.225,0,.008,.35),(.225,0,.008,.35),(0,-.175,.45,.008),(0,.175,.45,.008))):
        cube(f'HandoffSlot{i}_{j}', (cx+dx,cy+dy,top+.001), (a,b,.002), 'Amber' if i==0 else 'Teal', bevel=0)
label('TableLabel', 'HANDOFF', (tx, ty-sy/2-.002, top-.045), .075, vertical=True)
label('CellTitle', 'DUAL ROBOT  /  TOTE RELAY', (0, wall_y-wall_t/2-.015, 2.1), .3, 'Steel', vertical=True)
label('CellSubtitle', 'SOURCE   >   HANDOFF   >   TARGET', (0, wall_y-wall_t/2-.02, 1.8), .12, 'Steel', vertical=True)
for robot, text_value, material in zip(C['robots'], ('ROBOT A', 'ROBOT B'), ('Amber', 'Teal')):
    cx, cy, _ = robot['position']
    label('Floor'+text_value.replace(' ', ''), text_value, (cx, cy-.65, .007), .12, material)
environment_objects = list(scene.objects)
export(OUT / 'environment.usdc', '/Environment', environment_objects)
(OUT / 'static_colliders.json').write_text(json.dumps(static_parts, indent=2))
(OUT / 'scene_config.json').write_text(json.dumps(C, indent=2))
# Robots and crate instances are composed later in the Isaac USD assembly.
bpy.ops.wm.save_as_mainfile(filepath=str(OUT / 'relay_environment.blend'))
print('RELAY_BLENDER_READY', OUT, 'crate shapes', len(crate_parts), 'static shapes', len(static_parts))
