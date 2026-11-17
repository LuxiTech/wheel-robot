"""Reduced collision geometry must retain surfaces and real empty space."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest
import trimesh

pytest.importorskip("pymeshlab")
fcl = pytest.importorskip("fcl")
ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    "base_reducer", ROOT / "tools/grasp/generate_base_collision_model.py"
)
reducer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reducer)


def test_reduced_ring_preserves_surface_and_empty_cavity():
    original = trimesh.creation.annulus(r_min=0.02, r_max=0.04, height=0.02, sections=64)
    reduced, forward, reverse = reducer.reduce_component(original, 0.0005)
    assert forward <= 0.0005 and reverse <= 0.0005
    assert len(reduced.faces) <= len(original.faces)
    shape = fcl.BVHModel()
    shape.beginModel(len(reduced.vertices), len(reduced.faces))
    shape.addSubModel(np.asarray(reduced.vertices), np.asarray(reduced.faces, dtype=np.int32))
    shape.endModel()
    body = fcl.CollisionObject(shape)
    for location, expected in [([0, 0, 0], False), ([0.04, 0, 0], True)]:
        probe = fcl.CollisionObject(
            fcl.Sphere(0.002), fcl.Transform(np.array(location, dtype=float))
        )
        hit = fcl.collide(body, probe, fcl.CollisionRequest(), fcl.CollisionResult())
        assert bool(hit) is expected


def test_thin_component_cannot_disappear_under_face_budget():
    original = trimesh.creation.box(extents=[0.03, 0.02, 0.0002])
    reduced, forward, reverse = reducer.reduce_component(original, 0.0005)
    np.testing.assert_array_equal(reduced.triangles, original.triangles)
    assert forward == reverse == 0
