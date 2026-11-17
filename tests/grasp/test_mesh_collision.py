"""Original material refinement must retain true intersections and containment."""

import numpy as np
import pytest
import trimesh
from scipy.spatial.transform import Rotation

pytest.importorskip("fcl")

from wheel_motion_core.collision import Box, overlaps
from wheel_motion_core.mesh_collision import FCLNarrowPhase


def scene(tmp_path, *, open_mesh=False):
    mesh = trimesh.creation.annulus(r_min=0.02, r_max=0.04, height=0.1, sections=32)
    if open_mesh:
        mesh.update_faces(np.arange(len(mesh.faces) - 1))
    mesh.export(tmp_path / "shell.stl")
    urdf = tmp_path / "shell.urdf"
    urdf.write_text(
        '<robot name="test"><link name="shell"><collision><geometry>'
        '<mesh filename="shell.stl"/></geometry></collision></link>'
        '<link name="probe"><collision><geometry><sphere radius="0.0025"/>'
        "</geometry></collision></link></robot>"
    )
    shell = Box("shell:0", np.eye(4), np.array([0.04, 0.04, 0.05]))
    probe = Box("probe:0", np.eye(4), np.full(3, 0.0025))
    narrow = FCLNarrowPhase(urdf, [("shell", shell), ("probe", probe)])
    return narrow, shell, probe


def test_box_overlap_inside_empty_cavity_is_refined_clear(tmp_path):
    narrow, shell, probe = scene(tmp_path)
    assert overlaps(shell, probe, 0.005)
    assert not narrow(shell, probe, 0.005)
    assert narrow.last_detail["reason"] == "broad_phase_false_positive"


def test_contained_part_inside_solid_wall_is_not_missed(tmp_path):
    narrow, shell, probe = scene(tmp_path)
    probe.transform[0, 3] = 0.03
    assert narrow(shell, probe, 0)
    assert narrow.last_detail["reason"] == "original_material_containment"


def test_surface_intersection_is_rejected(tmp_path):
    narrow, shell, probe = scene(tmp_path)
    probe.transform[0, 3] = 0.02
    assert narrow(shell, probe, 0)
    assert narrow.last_detail["reason"] == "original_geometry_intersection"


def test_original_two_body_margin_is_retained(tmp_path):
    narrow, shell, probe = scene(tmp_path)
    assert narrow(shell, probe, 0.01)
    assert narrow.last_detail["reason"] == "original_geometry_within_margin"


def test_common_rotation_and_translation_do_not_change_result(tmp_path):
    narrow, shell, probe = scene(tmp_path)
    common = np.eye(4)
    common[:3, :3] = Rotation.from_euler("xyz", [0.3, -0.5, 0.8]).as_matrix()
    common[:3, 3] = [0.8, -1.2, 0.3]
    shell.transform = common.copy()
    probe.transform = common.copy()
    assert not narrow(shell, probe, 0.005)
    offset = np.eye(4)
    offset[0, 3] = 0.03
    probe.transform = common @ offset
    assert narrow(shell, probe, 0)


def test_open_source_mesh_retains_broad_phase_rejection(tmp_path):
    narrow, shell, probe = scene(tmp_path, open_mesh=True)
    assert narrow(shell, probe, 0.005)
    assert narrow.last_detail["reason"] == "open_mesh_retains_broad_phase_rejection"
