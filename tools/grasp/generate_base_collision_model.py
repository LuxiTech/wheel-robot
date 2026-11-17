#!/usr/bin/env python3
"""Offline component-preserving QEM reduction of the supplied base surface.

Uses the collision tooling venv (trimesh, scipy, rtree, pymeshlab).
Distances below are sampled surface errors, not a continuous Hausdorff proof.
No source geometry or joint transforms are modified.
"""

import argparse
import hashlib
import json
import time
from importlib.metadata import version
from pathlib import Path

import numpy as np
import pymeshlab
import trimesh

ROOT = Path(__file__).resolve().parents[2]
SOURCE = (
    ROOT
    / "assets/robots/r680_lift_openarm_claw1.4/resources/r680_lift_openarm_description/meshes/visual/r680_base.stl"
)


def surface_points(mesh):
    triangles = mesh.triangles
    return np.concatenate(
        [
            mesh.vertices,
            triangles.mean(axis=1),
            (triangles[:, 0] + triangles[:, 1]) / 2,
            (triangles[:, 1] + triangles[:, 2]) / 2,
            (triangles[:, 2] + triangles[:, 0]) / 2,
        ]
    )


def max_distance(mesh, points, threshold=None):
    largest = 0.0
    for start in range(0, len(points), 1000):
        distances = trimesh.proximity.closest_point(mesh, points[start : start + 1000])[1]
        if not np.isfinite(distances).all():
            raise ValueError("Non-finite proximity result")
        largest = max(largest, float(distances.max(initial=0)))
        if threshold is not None and largest > threshold:
            break
    return largest


def reduce_component(original, tolerance):
    # Independent CAD components must not vanish under a global face budget.
    target = max(12, int(len(original.faces) * 0.03))
    points = surface_points(original)
    while target < len(original.faces):
        meshes = pymeshlab.MeshSet()
        meshes.add_mesh(pymeshlab.Mesh(original.vertices, original.faces))
        meshes.meshing_decimation_quadric_edge_collapse(
            targetfacenum=target,
            preserveboundary=True,
            preservetopology=True,
            preservenormal=True,
            planarquadric=True,
            qualitythr=0.3,
        )
        mesh = meshes.current_mesh()
        vertices, faces = mesh.vertex_matrix(), mesh.face_matrix()
        if len(faces):
            candidate = trimesh.Trimesh(vertices, faces, process=False)
            forward = max_distance(candidate, points, tolerance)
            if forward <= tolerance:
                reverse = max_distance(original, surface_points(candidate), tolerance)
                if reverse <= tolerance:
                    return candidate, forward, reverse
        target = max(target + 1, target * 2)
    return original, 0.0, 0.0


def generate(output, tolerance=0.0005):
    if not 0 < tolerance <= 0.001:
        raise ValueError("Sampled error budget must be between 0 and 1 mm")
    output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    source = trimesh.load(SOURCE, force="mesh")
    components = trimesh.graph.connected_components(
        source.face_adjacency, nodes=np.arange(len(source.faces)), engine="scipy"
    )
    results, records = [], []
    for index, faces in enumerate(components):
        original = source.submesh([faces], append=True, repair=False)
        reduced, forward, reverse = reduce_component(original, tolerance)
        results.append(reduced)
        records.append(
            dict(
                component=index,
                source_faces=len(original.faces),
                output_faces=len(reduced.faces),
                forward_max_m=forward,
                reverse_max_m=reverse,
            )
        )
        if index % 50 == 0:
            print(f"Component {index + 1}/{len(components)}", flush=True)
    result = trimesh.util.concatenate(results)
    path = output / "r680_base_collision.stl"
    result.export(path)
    metadata = dict(
        source=str(SOURCE.relative_to(ROOT)),
        source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        output_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        source_faces=len(source.faces),
        source_vertices=len(source.vertices),
        output_faces=len(result.faces),
        output_vertices=len(result.vertices),
        component_count=len(components),
        sampled_error_budget_m=tolerance,
        sampling="Every vertex, triangle center and edge midpoint, both directions, separately for every source component; not a continuous Hausdorff guarantee",
        source_watertight=bool(source.is_watertight),
        original_coordinates_preserved=True,
        source_bounds=source.bounds.tolist(),
        output_bounds=result.bounds.tolist(),
        algorithm="Per-component topology/boundary/normal-preserving planar quadric reduction with doubling face budget on sampled-error failure; retain original component on failure",
        pymeshlab_version=version("pymeshlab"),
        trimesh_version=trimesh.__version__,
        generation_s=time.monotonic() - started,
        components=records,
    )
    (output / "provenance.json").write_text(json.dumps(metadata, indent=2))
    print(json.dumps({k: v for k, v in metadata.items() if k != "components"}), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--tolerance", type=float, default=0.0005)
    args = parser.parse_args()
    generate(args.output, args.tolerance)
