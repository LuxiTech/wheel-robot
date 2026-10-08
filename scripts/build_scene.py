"""Run with Blender 4.5.3. Visual geometry and collision manifest share dimensions."""

import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy
from mathutils import Vector
from common import ASSETS, OUTPUTS, load_config, box_start

c = load_config()
ASSETS.mkdir(parents=True, exist_ok=True)
OUTPUTS.mkdir(parents=True, exist_ok=True)
bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
scene = bpy.context.scene
scene.unit_settings.system = "METRIC"
scene.unit_settings.scale_length = 1.0
materials = {}
for name, color, metallic, roughness in [
    ("Floor", (0.25, 0.30, 0.34, 1), 0.25, 0.36),
    ("Wall", (0.68, 0.73, 0.76, 1), 0.0, 0.65),
    ("Steel", (0.055, 0.08, 0.11, 1), 0.75, 0.3),
    ("Blue", (0.025, 0.22, 0.42, 1), 0.5, 0.28),
    ("Yellow", (1, 0.62, 0.035, 1), 0.1, 0.4),
    ("White", (0.8, 0.86, 0.88, 1), 0.1, 0.4),
    ("Cardboard", (0.48, 0.27, 0.10, 1), 0, 0.85),
    ("Screen", (0.015, 0.08, 0.09, 1), 0.2, 0.22),
]:
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = color
    mat.use_nodes = True
    p = mat.node_tree.nodes.get("Principled BSDF")
    p.inputs["Base Color"].default_value = color
    p.inputs["Metallic"].default_value = metallic
    p.inputs["Roughness"].default_value = roughness
    materials[name] = mat
colliders = []


def cube(name, pos, size, material="Steel", collision=True, bevel=0.015):
    bpy.ops.mesh.primitive_cube_add(size=1, location=pos)
    ob = bpy.context.object
    ob.name = name
    ob.dimensions = size
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    ob.data.materials.append(materials[material])
    if bevel:
        mod = ob.modifiers.new("EdgeHighlights", "BEVEL")
        mod.width = min(bevel, min(size) / 4)
        mod.segments = 2
        ob.modifiers.new("Normals", "WEIGHTED_NORMAL")
    if collision:
        colliders.append({"name": name, "position": list(pos), "size": list(size)})
    return ob


def text(name, value, pos, size=0.22, rotation=(math.pi / 2, 0, 0), material="White"):
    bpy.ops.object.text_add(location=pos, rotation=rotation)
    ob = bpy.context.object
    ob.name = name
    ob.data.body = value
    ob.data.size = size
    ob.data.align_x = "CENTER"
    ob.data.extrude = 0.001
    ob.data.materials.append(materials[material])
    bpy.ops.object.convert(target="MESH")


w, d, h = c["room"]["size"]
cube("Floor", (0, 0, -0.12), (w, d, 0.24), "Floor", bevel=0)
# Full rear and side walls; front wall has a 4m loading entrance.
cube("WallBack", (0, d / 2, h / 2), (w, 0.18, h), "Wall")
for x in [-w / 2, w / 2]:
    cube("WallSide" + ("L" if x < 0 else "R"), (x, 0, h / 2), (0.18, d, h), "Wall")
for x in [-(w + 4) / 4, (w + 4) / 4]:
    cube(
        "FrontWall" + ("L" if x < 0 else "R"),
        (x, -d / 2, h / 2),
        ((w - 4) / 2, 0.18, h),
        "Wall",
    )
cube("DoorLintel", (0, -d / 2, h - 0.6), (4, 0.18, 1.2), "Wall")
for x in [-9, -3, 3, 9]:
    for y in [-7.25, 7.25]:
        cube(
            f"Column_{x}_{y}".replace("-", "m").replace(".", "_"),
            (x, y, h / 2),
            (0.24, 0.3, h),
        )
    cube(
        f"RoofBeam_{x}".replace("-", "m"),
        (x, 0, h - 0.25),
        (0.20, d, 0.30),
        collision=False,
    )
for y in [-5, 0, 5]:
    cube(
        f"CableTray_{y}".replace("-", "m"),
        (0, y, h - 0.5),
        (w, 0.18, 0.10),
        collision=False,
    )
    for x in [-6, 0, 6]:
        cube(
            f"LightHousing_{x}_{y}".replace("-", "m"),
            (x, y, h - 0.65),
            (2.4, 0.32, 0.10),
            "White",
            False,
        )
        bpy.ops.object.light_add(type="AREA", location=(x, y, h - 0.72))
        light = bpy.context.object
        light.name = f"CeilingLight_{x}_{y}".replace("-", "m")
        light.data.energy = 600
        light.data.shape = "RECTANGLE"
        light.data.size = 2.4
        light.data.size_y = 0.8
# Continuous main lane and dashed centerline.
for y in [-c["room"]["aisle_width"] / 2, c["room"]["aisle_width"] / 2]:
    cube(
        "LaneEdge" + ("L" if y < 0 else "R"),
        (0, y, 0.004),
        (w - 0.8, 0.08, 0.008),
        "Yellow",
        False,
        0,
    )
for x in range(-8, 9, 2):
    cube(
        f"LaneDash_{x}".replace("-", "m"),
        (x, 0, 0.006),
        (0.75, 0.05, 0.008),
        "White",
        False,
        0,
    )
text("LaneLabel", "LOGISTICS  /  03", (0, -0.8, 0.018), 0.40, (0, 0, 0))
for t in c["tables"]:
    name = t["id"]
    x, y = t["xy"]
    sx, sy, z = t["size"]
    cube(name + "_Top", (x, y, z - 0.045), (sx, sy, 0.09), "Steel")
    for dx in [-sx / 2 + 0.10, sx / 2 - 0.10]:
        for dy in [-sy / 2 + 0.10, sy / 2 - 0.10]:
            cube(
                name + f"_Leg_{dx}_{dy}".replace("-", "m").replace(".", "_"),
                (x + dx, y + dy, (z - 0.09) / 2),
                (0.08, 0.08, z - 0.09),
                "Blue",
            )
    cube(name + "_Shelf", (x, y, 0.18), (sx - 0.15, sy - 0.15, 0.05), "Blue")
    cube(
        name + "_Fascia",
        (x, y - sy / 2 - 0.01, z - 0.17),
        (sx, 0.035, 0.19),
        "Blue",
        False,
    )
    text(
        name + "_Label",
        name.replace("_", "  /  "),
        (x, y - sy / 2 - 0.032, z - 0.20),
        0.12,
    )
    for dy in [-0.38, 0.38]:
        cube(
            name + f"_Mark{dy}".replace("-", "m").replace(".", "_"),
            (x, y + dy, z + 0.002),
            (sx - 0.2, 0.035, 0.004),
            "Yellow",
            False,
            0,
        )
    for dx in [-sx / 2 - 0.35, sx / 2 + 0.35]:
        cube(
            name + f"_Zone{dx}".replace("-", "m").replace(".", "_"),
            (x + dx, y, 0.006),
            (0.06, sy + 0.6, 0.012),
            "Yellow",
            False,
            0,
        )
# Storage racks with open shelves and separate colliders.
for r, x in enumerate([-6, -2, 2, 6]):
    y = 6.35
    for dx in [-1.2, 1.2]:
        for dy in [-0.5, 0.5]:
            cube(
                f"Rack{r}_Post{dx}_{dy}".replace("-", "m").replace(".", "_"),
                (x + dx, y + dy, 1.8),
                (0.08, 0.08, 3.6),
                "Blue",
            )
    for level, z in enumerate([0.3, 1.4, 2.5, 3.5]):
        cube(f"Rack{r}_Shelf{level}", (x, y, z), (2.5, 1.1, 0.08), "Steel")
        cube(
            f"Rack{r}_Rail{level}", (x, y - 0.56, z), (2.5, 0.05, 0.13), "Yellow", False
        )
        if level < 3:
            for j in [-0.75, 0, 0.75]:
                cube(
                    f"Rack{r}_Stock{level}_{j}".replace("-", "m").replace(".", "_"),
                    (x + j, y, z + 0.28),
                    (0.55, 0.65, 0.48),
                    "Cardboard",
                )
    text(f"RackLabel{r}", f"R-{r + 1:02}", (x, y - 0.59, 3.27), 0.18)
# Fixed machines, cabinet panels and safety fence along south wall.
for i, x in enumerate([-7, -3.5, 3.5, 7]):
    y = -6.2
    cube(f"Machine{i}", (x, y, 1.15), (2.25, 1.6, 2.3), "Blue")
    cube(f"Machine{i}_Door", (x, y + 0.81, 1.3), (1.8, 0.025, 1.55), "Steel", False)
    cube(
        f"Machine{i}_Screen",
        (x - 0.4, y + 0.835, 1.6),
        (0.5, 0.025, 0.32),
        "Screen",
        False,
    )
    cube(
        f"Machine{i}_Handle",
        (x + 0.6, y + 0.87, 1.3),
        (0.035, 0.06, 0.3),
        "White",
        False,
    )
    text(
        f"Machine{i}_Label",
        f"CELL {i + 1:02}",
        (x, y + 0.85, 2.03),
        0.16,
        (math.pi / 2, 0, math.pi),
    )
for x in [-8.5, -5.5, 5.5, 8.5]:
    cube(
        f"Bollard{x}".replace("-", "m").replace(".", "_"),
        (x, -4.95, 0.45),
        (0.12, 0.12, 0.9),
        "Yellow",
    )
for x in [-7, 7]:
    for z in [0.35, 0.8]:
        cube(
            f"Guard{x}_{z}".replace("-", "m").replace(".", "_"),
            (x, -4.95, z),
            (3, 0.055, 0.055),
            "Yellow",
        )
# Presentation camera is inside the building for an unobstructed industrial view.
bpy.ops.object.camera_add(location=c["camera"]["showcase"])
cam = bpy.context.object
cam.name = "ShowcaseCamera"
cam.rotation_euler = (
    (Vector(c["camera"]["look_at"]) - cam.location).to_track_quat("-Z", "Y").to_euler()
)
cam.data.lens = 22
scene.camera = cam
scene.world.color = (0.15, 0.15, 0.15)
# Export only environment; dynamic box lives in its own USD asset.
bpy.ops.wm.usd_export(
    filepath=str(ASSETS / "environment.usdc"),
    root_prim_path="/Environment",
    export_materials=True,
    export_lights=True,
    export_cameras=True,
    export_textures=True,
    relative_paths=True,
    generate_preview_surface=True,
)
(ASSETS / "colliders.json").write_text(json.dumps(colliders, indent=2))
(ASSETS / "scene_config.json").write_text(json.dumps(c, indent=2))
# Box is included in the editable Blender preview, excluded from environment USD.
for i in range(c["box"]["count"]):
    p = box_start(c, i, drop=0.002)
    cube(f"PreviewBox{i}", p, c["box"]["size"], "Cardboard", False, 0.006)
    cube(
        f"PreviewTape{i}",
        (p[0], p[1], p[2] + c["box"]["size"][2] / 2 + 0.001),
        (0.05, c["box"]["size"][1], 0.002),
        "Yellow",
        False,
        0,
    )
scene.render.engine = "CYCLES"
scene.cycles.samples = 24
scene.render.resolution_x = 1280
scene.render.resolution_y = 800
scene.render.resolution_percentage = 100
bpy.ops.wm.save_as_mainfile(filepath=str(ASSETS / "industrial_scene.blend"))
if "--render-preview" in sys.argv:
    scene.render.filepath = str(OUTPUTS / "blender_preview.png")
    bpy.ops.render.render(write_still=True)
print(f"ENVIRONMENT_EXPORTED: {len(colliders)} static colliders -> {ASSETS}")
