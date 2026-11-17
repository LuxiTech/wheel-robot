"""Refine OBB candidates with original triangle material, retaining containment.

FCL is an optional local dependency. Missing/invalid geometry never clears a
candidate. Existing pair exclusions, source geometry and safety margin remain.
"""

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import trimesh

from wheel_motion_core.collision import origin


def winding_contains(triangles, point):
    """Closed oriented material; cavities retain their inward-facing surfaces."""
    vectors = triangles - np.asarray(point)
    a, b, c = vectors[:, 0], vectors[:, 1], vectors[:, 2]
    la, lb, lc = (np.linalg.norm(item, axis=1) for item in (a, b, c))
    determinant = np.einsum("ij,ij->i", a, np.cross(b, c))
    denominator = (
        la * lb * lc
        + np.einsum("ij,ij->i", a, b) * lc
        + np.einsum("ij,ij->i", b, c) * la
        + np.einsum("ij,ij->i", c, a) * lb
    )
    return abs(float((2 * np.arctan2(determinant, denominator)).sum())) > 2 * np.pi


@dataclass
class MeshGeometry:
    shape: object
    triangles: object
    probes: np.ndarray
    closed: bool
    local_pose: np.ndarray


class FCLNarrowPhase:
    def __init__(self, urdf, parts, *, conservative_box_links=()):
        import fcl

        self.fcl = fcl
        self.geometry = {}
        self.box_local = {}
        self.checked_pairs = 0
        self.refined_clear_pairs = 0
        self.last_detail = None
        self.world_geometry = {}
        path = Path(urdf)
        root = ET.parse(path).getroot()
        for link, box in parts:
            self.box_local[box.name] = box.transform.copy()
            index = int(box.name.rsplit(":", 1)[1])
            node = root.find(f"link[@name='{link}']").findall("collision")[index]
            pose = origin(node.find("origin"))
            geometry = node.find("geometry")
            mesh_node = geometry.find("mesh")
            if link in conservative_box_links:
                # The caller's equal half-extents enclose the rotation-swept
                # sphere of an unknown wheel angle. Use that same sphere for
                # narrow phase, retaining every possible wheel orientation.
                if not np.allclose(box.half, box.half[0]):
                    raise ValueError("Rotation-swept wheel envelope must be spherical")
                shape = fcl.Sphere(float(box.half[0]))
                item = MeshGeometry(shape, None, np.zeros((1, 3)), True, box.transform.copy())
            elif mesh_node is not None:
                mesh = trimesh.load(path.parent / mesh_node.get("filename"), force="mesh")
                scale = np.fromstring(mesh_node.get("scale", "1 1 1"), sep=" ")
                mesh = mesh.copy()
                mesh.apply_scale(scale)
                item = self.from_mesh(mesh, pose)
            else:
                if geometry.find("box") is not None:
                    size = np.fromstring(geometry.find("box").get("size"), sep=" ")
                    shape = fcl.Box(*size)
                elif geometry.find("sphere") is not None:
                    shape = fcl.Sphere(float(geometry.find("sphere").get("radius")))
                elif geometry.find("cylinder") is not None:
                    cylinder = geometry.find("cylinder")
                    shape = fcl.Cylinder(
                        float(cylinder.get("radius")), float(cylinder.get("length"))
                    )
                else:
                    raise ValueError("Unsupported original collision geometry: " + box.name)
                item = MeshGeometry(shape, None, np.zeros((1, 3)), True, pose)
            self.geometry[box.name] = item

    def from_mesh(self, mesh, local_pose=None):
        vertices = np.asarray(mesh.vertices)
        faces = np.asarray(mesh.faces, dtype=np.int32)
        if not len(vertices) or not len(faces) or not np.isfinite(vertices).all():
            raise ValueError("Invalid original mesh")
        shape = self.fcl.BVHModel()
        shape.beginModel(len(vertices), len(faces))
        shape.addSubModel(vertices, faces)
        shape.endModel()
        components = trimesh.graph.connected_components(
            mesh.face_adjacency, nodes=np.arange(len(faces))
        )
        probes = np.asarray([vertices[faces[component[0], 0]] for component in components])
        return MeshGeometry(
            shape,
            vertices[faces],
            probes,
            bool(mesh.is_watertight and mesh.is_winding_consistent),
            np.eye(4) if local_pose is None else local_pose,
        )

    def _object(self, geometry, pose):
        return self.fcl.CollisionObject(
            geometry.shape, self.fcl.Transform(pose[:3, :3], pose[:3, 3])
        )

    def _collides(self, a, pose_a, b, pose_b, margin):
        self.checked_pairs += 1
        # Open or inconsistent supplied meshes lack a reliable material inside.
        if not a.closed or not b.closed:
            self.last_detail = dict(reason="open_mesh_retains_broad_phase_rejection")
            return True
        obj_a, obj_b = self._object(a, pose_a), self._object(b, pose_b)
        collision = self.fcl.CollisionResult()
        if self.fcl.collide(obj_a, obj_b, self.fcl.CollisionRequest(), collision):
            self.last_detail = dict(reason="original_geometry_intersection")
            return True
        distance = float(
            self.fcl.distance(obj_a, obj_b, self.fcl.DistanceRequest(), self.fcl.DistanceResult())
        )
        if not np.isfinite(distance) or distance <= 2 * margin:
            self.last_detail = dict(reason="original_geometry_within_margin", distance_m=distance)
            return True
        # Triangle/triangle surface distance alone misses a wholly contained part.
        # Check a vertex from every disconnected component against closed material.
        for inner, inner_pose, outer, outer_pose in (
            (a, pose_a, b, pose_b),
            (b, pose_b, a, pose_a),
        ):
            if outer.triangles is None:
                # FCL's primitive/mesh and primitive/primitive queries treat the
                # primitive as solid, including triangles contained in it.
                continue
            relative = np.linalg.inv(outer_pose) @ inner_pose
            probes = inner.probes @ relative[:3, :3].T + relative[:3, 3]
            if any(winding_contains(outer.triangles, point) for point in probes):
                self.last_detail = dict(reason="original_material_containment")
                return True
        self.refined_clear_pairs += 1
        self.last_detail = dict(reason="broad_phase_false_positive", distance_m=distance)
        return False

    def __call__(self, a, b, margin):
        geometry_a, geometry_b = self.geometry[a.name], self.geometry[b.name]
        pose_a = a.transform @ np.linalg.inv(self.box_local[a.name]) @ geometry_a.local_pose
        pose_b = b.transform @ np.linalg.inv(self.box_local[b.name]) @ geometry_b.local_pose
        return self._collides(geometry_a, pose_a, geometry_b, pose_b, margin)

    def world_mesh_collision(self, a, mesh, name, world_pose, margin):
        if name not in self.world_geometry:
            self.world_geometry[name] = self.from_mesh(mesh)
        geometry = self.geometry[a.name]
        pose = a.transform @ np.linalg.inv(self.box_local[a.name]) @ geometry.local_pose
        return self._collides(geometry, pose, self.world_geometry[name], world_pose, margin)
