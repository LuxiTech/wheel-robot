"""Compose portable visual, collision, dynamic-box and task layers inside Isaac."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from isaacsim import SimulationApp

app = SimulationApp(
    {
        "headless": True,
        "width": 1280,
        "height": 800,
        "extra_args": ["--/app/asyncRendering=false"],
    }
)
try:
    import json
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade, UsdLux
    from common import ASSETS, load_config, box_start

    c = load_config()
    if json.loads((ASSETS / "scene_config.json").read_text()) != c:
        raise RuntimeError(
            "Config changed: regenerate Blender environment before assembly"
        )

    def stage(path, root):
        s = Usd.Stage.CreateNew(str(path))
        p = UsdGeom.Xform.Define(s, root).GetPrim()
        s.SetDefaultPrim(p)
        UsdGeom.SetStageUpAxis(s, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(s, 1.0)
        return s

    def cube(s, path, pos, size):
        g = UsdGeom.Cube.Define(s, path)
        g.CreateSizeAttr(1)
        g.AddTranslateOp().Set(Gf.Vec3d(*pos))
        g.AddScaleOp().Set(Gf.Vec3d(*size))
        return g.GetPrim()

    physics = stage(ASSETS / "physics.usda", "/World")
    UsdGeom.Scope.Define(physics, "/World/Collisions")
    for item in json.loads((ASSETS / "colliders.json").read_text()):
        p = cube(
            physics, "/World/Collisions/" + item["name"], item["position"], item["size"]
        )
        UsdGeom.Imageable(p).CreateVisibilityAttr("invisible")
        UsdPhysics.CollisionAPI.Apply(p)
    ps = UsdPhysics.Scene.Define(physics, "/World/PhysicsScene")
    ps.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    ps.CreateGravityMagnitudeAttr(c["physics"]["gravity"])
    physics.GetRootLayer().Save()

    b = stage(ASSETS / "box.usda", "/Box")
    rb = UsdPhysics.RigidBodyAPI.Apply(b.GetDefaultPrim())
    rb.CreateRigidBodyEnabledAttr(True)
    mass = UsdPhysics.MassAPI.Apply(b.GetDefaultPrim())
    m = c["box"]["mass"]
    x, y, z = c["box"]["size"]
    mass.CreateMassAttr(m)
    mass.CreateCenterOfMassAttr(Gf.Vec3f(0))
    mass.CreateDiagonalInertiaAttr(
        Gf.Vec3f(
            m * (y * y + z * z) / 12, m * (x * x + z * z) / 12, m * (x * x + y * y) / 12
        )
    )
    p = cube(b, "/Box/Geometry", (0, 0, 0), (x, y, z))
    UsdPhysics.CollisionAPI.Apply(p)
    mat = UsdShade.Material.Define(b, "/Box/Material")
    shader = UsdShade.Shader.Define(b, "/Box/Material/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    from pxr import Sdf

    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.48, 0.27, 0.10)
    )
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.85)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(p).Bind(mat)
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(c["box"]["static_friction"])
    pm.CreateDynamicFrictionAttr(c["box"]["dynamic_friction"])
    pm.CreateRestitutionAttr(c["box"]["restitution"])
    UsdShade.MaterialBindingAPI(p).Bind(mat, materialPurpose="physics")
    b.GetRootLayer().Save()

    main = stage(ASSETS / "main.usda", "/World")
    main.GetRootLayer().subLayerPaths = ["physics.usda"]
    env = UsdGeom.Xform.Define(main, "/World/Environment").GetPrim()
    env.GetReferences().AddReference("environment.usdc", "/Environment")
    # Blender's photometric export does not match RTX exposure out of the box.
    # Author native lights in the assembly layer, leaving the visual asset intact.
    dome = UsdLux.DomeLight.Define(main, "/World/Lighting/Ambient")
    dome.CreateIntensityAttr(11200.0)
    dome.CreateColorAttr(Gf.Vec3f(0.82, 0.90, 1.0))
    for ix, x in enumerate((-6, 0, 6)):
        for iy, y in enumerate((-5, 0, 5)):
            lamp = UsdLux.RectLight.Define(main, f"/World/Lighting/Panel_{ix}_{iy}")
            lamp.AddTranslateOp().Set(Gf.Vec3d(x, y, c["room"]["size"][2] - 0.8))
            lamp.CreateWidthAttr(2.4)
            lamp.CreateHeightAttr(0.8)
            lamp.CreateIntensityAttr(28800.0)
            lamp.CreateColorAttr(Gf.Vec3f(1.0, 0.94, 0.85))
    for i in range(c["box"]["count"]):
        p = UsdGeom.Xform.Define(main, f"/World/Boxes/Box_{i:02}")
        p.GetPrim().GetReferences().AddReference("box.usda", "/Box")
        p.AddTranslateOp().Set(Gf.Vec3d(*box_start(c, i)))
    robot = c["robot"]["usd_path"]
    if robot:
        import os

        robot_path = Path(robot)
        if not robot_path.is_absolute():
            robot_path = ASSETS.parent / robot_path
        if not robot_path.is_file():
            raise FileNotFoundError(robot_path)
        p = UsdGeom.Xform.Define(main, "/World/Robot")
        p.GetPrim().GetReferences().AddReference(os.path.relpath(robot_path, ASSETS))
        p.AddTranslateOp().Set(Gf.Vec3d(*c["robot"]["position"]))
    main.GetRootLayer().Save()
    # Reopen the final stage, fail on composition errors or invalid scaling.
    check = Usd.Stage.Open(str(ASSETS / "main.usda"))
    if check.GetCompositionErrors():
        raise RuntimeError(str(check.GetCompositionErrors()))
    assert check.GetPrimAtPath("/World/Environment/Floor")
    assert UsdGeom.GetStageMetersPerUnit(check) == 1.0
    print("SCENE_ASSEMBLED:", ASSETS / "main.usda", flush=True)
except Exception:
    import traceback

    traceback.print_exc()
    sys.stdout.flush()
    sys.stderr.flush()
    app.close(exit_code=1)
    raise
else:
    app.close()
