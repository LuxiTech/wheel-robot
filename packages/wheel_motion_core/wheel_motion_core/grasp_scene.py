"""Conservative robot OBB / supplied CAD material collision checking.

The open crate is voxelized as material, not a solid outer bounding box. Checks
cover the robot and selected crate only; scene coverage and physical registration
remain independent execution gates. This module never commands hardware.
"""

import copy
import hashlib
import itertools
from pathlib import Path

import numpy as np
import trimesh
from wheel_task_core.transforms import pose

from wheel_motion_core.collision import Box, URDFCollision
from wheel_motion_core.kinematics import RobotModel
from wheel_motion_core.sdk_kinematics import SDKKinematics


def fixed_body_exclusions(joints):
    """Same rigid body and immediately joint-connected bodies share geometry."""
    parents = {}

    def find(name):
        parents.setdefault(name, name)
        while parents[name] != name:
            name = parents[name]
        return name

    for joint in joints.values():
        if joint["kind"] == "fixed":
            parents[find(joint["child"])] = find(joint["parent"])
    links = {j[key] for j in joints.values() for key in ("parent", "child")}
    adjacent = {
        frozenset((find(j["parent"]), find(j["child"])))
        for j in joints.values()
        if j["kind"] != "fixed"
    }
    return [
        (a, b)
        for a, b in itertools.combinations(sorted(links), 2)
        if find(a) == find(b) or frozenset((find(a), find(b))) in adjacent
    ]


class CADMaterial:
    def __init__(self, mesh_path, expected_sha256, source_to_object, *, pitch_m=0.005):
        path = Path(mesh_path)
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected_sha256:
            raise ValueError("Crate mesh fingerprint mismatch")
        if not np.isfinite(pitch_m) or not 0.002 <= pitch_m <= 0.01:
            raise ValueError("Unsupported CAD collision voxel pitch")
        mesh = trimesh.load(path, force="mesh")
        if not mesh.is_watertight:
            raise ValueError("Closed CAD material mesh required for interior filling")
        mesh.apply_transform(pose(source_to_object))
        self.mesh = mesh.copy()
        voxels = mesh.voxelized(pitch_m).fill()
        self.matrix = np.asarray(voxels.matrix, bool)
        self.origin = np.asarray(voxels.transform[:3, 3])
        self.pitch = float(pitch_m)
        if not np.allclose(voxels.transform[:3, :3], np.eye(3) * self.pitch):
            raise ValueError("Unexpected CAD voxel coordinate convention")
        # Cover voxelization uncertainty rather than shrinking thin surfaces.
        self.half = self.pitch * (0.5 + np.sqrt(3) / 2)
        self.sha256 = expected_sha256

    def overlaps(self, box, object_pose, margin_m):
        if not np.isfinite(margin_m) or margin_m < 0:
            raise ValueError("Nonnegative finite collision margin required")
        object_box = np.linalg.inv(pose(object_pose)) @ box.transform
        rotation, centre = object_box[:3, :3], object_box[:3, 3]
        half = box.half + margin_m
        extent = np.abs(rotation) @ half + self.half
        low = np.maximum(np.floor((centre - extent - self.origin) / self.pitch).astype(int), 0)
        high = np.minimum(
            np.ceil((centre + extent - self.origin) / self.pitch).astype(int),
            np.array(self.matrix.shape) - 1,
        )
        if np.any(low > high):
            return False
        subset = self.matrix[tuple(slice(a, b + 1) for a, b in zip(low, high))]
        indices = np.argwhere(subset) + low
        if not len(indices):
            return False
        delta = indices * self.pitch + self.origin - centre
        axes = [*np.eye(3), *rotation.T]
        axes.extend(np.cross(a, b) for a in np.eye(3) for b in rotation.T)
        for axis in axes:
            length = np.linalg.norm(axis)
            if length < 1e-10:
                continue
            axis = axis / length
            radius = np.sum(half * np.abs(rotation.T @ axis)) + self.half * np.sum(np.abs(axis))
            delta = delta[np.abs(delta @ axis) <= radius]
            if not len(delta):
                return False
        return True


class GraspSceneCollision:
    def __init__(
        self,
        urdf,
        sdk_chains,
        material,
        target,
        opening_mapping,
        *,
        margin_m=0.005,
        mesh_narrow_phase=False,
    ):
        if not np.isfinite(margin_m) or margin_m < 0:
            raise ValueError("Nonnegative finite scene margin required")
        self.robot = RobotModel(urdf)
        # Geometry must evaluate the supplied measured opening, even outside
        # nominal CAD travel. Motion limits remain enforced by SDKAlignedModel.
        for joint in self.robot.joints.values():
            joint["bounds"] = None
        self.sdk = SDKKinematics(sdk_chains)
        self.material, self.opening_mapping = material, copy.deepcopy(opening_mapping)
        self.margin = margin_m
        self.wheel_sweeps = []
        for name in ("front_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint"):
            if name in self.robot.joints:
                joint = self.robot.joints[name]
                if joint["kind"] != "continuous":
                    raise ValueError("Unexpected wheel joint convention")
                self.wheel_sweeps.append(joint["child"])
                joint["kind"] = "fixed"
        self.robot_check = URDFCollision(
            urdf,
            self.robot,
            [],
            allowed_pairs=fixed_body_exclusions(self.robot.joints),
            margin=margin_m,
        )
        # No wheel angle feedback is invented. Each unknown rotation is
        # covered by a sphere at the wheel pivot, enclosed by this OBB.
        for link, local in self.robot_check.parts:
            if link in self.wheel_sweeps:
                corners = np.asarray(list(itertools.product((-1, 1), repeat=3))) * local.half
                corners = corners @ local.transform[:3, :3].T + local.transform[:3, 3]
                radius = float(np.linalg.norm(corners, axis=1).max())
                local.transform = np.eye(4)
                local.half = np.full(3, radius)
        if mesh_narrow_phase:
            from wheel_motion_core.mesh_collision import FCLNarrowPhase

            self.robot_check.narrow_phase = FCLNarrowPhase(
                urdf, self.robot_check.parts, conservative_box_links=self.wheel_sweeps
            )
        self.execution_blockers = [
            "surrounding_scene_not_certified",
            "gripper_opening_scale_not_hardware_accepted",
        ]
        self.target = None
        self.fingers = None
        self.last_collision = None
        self.checked_configurations = 0
        self.update_target(target)

    def update_target(self, target):
        if target.get("frame_id") != "arm_origin" or not np.isfinite(target["height_um"]):
            raise ValueError("Scene target requires same-capture SDK arm_origin pose/height")
        self.target = dict(target)
        self.object_origin = pose(target["object_pose"]).copy()

    def refresh(self, q16):
        SDKKinematics.from_sdk_vector(q16)  # Reject invalid input, never clip.
        self.fingers = {}
        for side, index in (("left", 15), ("right", 7)):
            config = self.opening_mapping[side]
            sign = config["opening_sign"]
            if sign not in (-1, 1):
                raise ValueError("Explicit measured opening direction required")
            self.fingers[f"openarm_{side}_finger_joint1"] = (
                config["closed_displacement_m"]
                + 0.044 * sign * (q16[index] - config["closed_feedback_rad"]) / 1.0472
            )

    def __call__(self, values):
        self.checked_configurations += 1
        self.last_collision = None
        if self.fingers is None:
            raise ValueError("Measured gripper feedback required before scene checking")
        height = float(values["lift_joint"])
        if abs(height * 1e6 - self.target["height_um"]) > 200:
            self.last_collision = ("fixed_height_context_changed",)
            return False
        joints = {**values, **self.fingers, "lift_joint": height + 0.2}
        if not self.robot_check(joints):
            self.last_collision = self.robot_check.last_collision
            return False
        world_origin = self.robot.transform("openarm_left_base_link", joints) @ np.linalg.inv(
            self.sdk.transform("openarm_left_base_link")
        )
        world_object = world_origin @ self.object_origin
        for link, local in self.robot_check.parts:
            world_box = Box(
                local.name, self.robot.transform(link, joints) @ local.transform, local.half
            )
            if self.material.overlaps(world_box, world_object, self.margin):
                narrow = self.robot_check.narrow_phase
                if narrow is not None and not narrow.world_mesh_collision(
                    world_box,
                    self.material.mesh,
                    "selected_crate_CAD_material",
                    world_object,
                    self.margin,
                ):
                    continue
                self.last_collision = (local.name, "selected_crate_CAD_material")
                return False
        return True

    def describe(self):
        return dict(
            scope="robot self-collision and selected crate CAD; surrounding scene not certified",
            robot_geometry=(
                "original URDF material meshes/primitives after OBB broad phase; containment retained"
                if self.robot_check.narrow_phase is not None
                else "conservative URDF local mesh bounding boxes"
            ),
            self_collision_exclusions="same rigid group and immediately joint-connected groups; physical acceptance not inferred",
            crate_geometry="filled CAD material voxels; open cavity retained",
            pitch_m=self.material.pitch,
            voxel_half_extent_with_discretization_m=self.material.half,
            material_cells=int(self.material.matrix.sum()),
            crate_mesh_sha256=self.material.sha256,
            margin_m=self.margin,
            checked_configurations=self.checked_configurations,
            last_collision=self.last_collision,
            mesh_narrow_phase_detail=(
                self.robot_check.narrow_phase.last_detail
                if self.robot_check.narrow_phase is not None
                else None
            ),
            finger_displacements_m=self.fingers,
            opening_mapping_hardware_precision_accepted=False,
            wheel_geometry="rotation-swept spheres enclosed in boxes; no wheel feedback assumed",
            wheel_links=self.wheel_sweeps,
        )
