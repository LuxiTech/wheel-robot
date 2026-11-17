"""Vision tool poses -> fixed-lift SDK wrist trajectories, without motor writes."""

import copy
import hashlib
import xml.etree.ElementTree as ET
from dataclasses import asdict
from functools import lru_cache
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from wheel_task_core.transforms import pose

from wheel_motion_core.collision import origin
from wheel_motion_core.kinematics import RobotModel
from wheel_motion_core.live_kinematics import SDKAlignedModel
from wheel_motion_core.planning import plan_pair

SIDES = ("left", "right")


def nominal_hook_mounts(robot, contacts):
    """Explicit CAD contact midpoint frame; not the board mount or link7 origin.

    Tool X follows wrist X, tool Y follows the finger slide axis, Z=X cross Y.
    The point is the recorded contact midpoint, not a measured insertion tip.
    """
    result = {}
    for side in SIDES:
        wrist = f"openarm_{side}_link7"
        finger = robot.joints[f"openarm_{side}_finger_joint1"]
        palm = robot.transform(f"openarm_{side}_ee_base_link", {}, base=wrist)
        y = palm[:3, :3] @ finger["t"][:3, :3] @ finger["axis"]
        y /= np.linalg.norm(y)
        x = np.array([1.0, 0, 0])
        x -= (x @ y) * y
        x /= np.linalg.norm(x)
        mount = np.eye(4)
        mount[:3, :3] = np.column_stack((x, y, np.cross(x, y)))
        mount[:3, 3] = contacts["sides"][side]["contact_midpoint_in_wrist_m"]
        result[side] = pose(mount)
    return result


@lru_cache(maxsize=8)
def _rail_vertices(urdf_path, source_sha256, mesh_fingerprints):
    """Cache immutable CAD extraction, keyed by content as well as path."""
    urdf = Path(urdf_path)
    if hashlib.sha256(urdf.read_bytes()).hexdigest() != source_sha256:
        raise ValueError("Rail geometry changed during loading")
    for path, digest in mesh_fingerprints:
        if hashlib.sha256(Path(path).read_bytes()).hexdigest() != digest:
            raise ValueError("Rail mesh changed during loading")
    xml = ET.parse(urdf).getroot()
    rails = {}
    for side in SIDES:
        visual = xml.find(f"link[@name='openarm_{side}_ee_base_link']/visual")
        origin = visual.find("origin")
        if origin is not None and (
            not np.allclose(np.fromstring(origin.get("xyz", "0 0 0"), sep=" "), 0)
            or not np.allclose(np.fromstring(origin.get("rpy", "0 0 0"), sep=" "), 0)
        ):
            raise ValueError("Rail mesh origin differs from supplied CAD convention")
        mesh_node = visual.find("geometry/mesh")
        if not np.allclose(np.fromstring(mesh_node.get("scale", "1 1 1"), sep=" "), 1):
            raise ValueError("Unexpected rail mesh scale")
        mesh = trimesh.load(Path(urdf).parent / mesh_node.get("filename"), force="mesh")
        candidates = []
        facets, normals, mesh_faces, mesh_vertices = (
            mesh.facets,
            mesh.facets_normal,
            mesh.faces,
            mesh.vertices,
        )
        for index, faces in enumerate(facets):
            vertices = mesh_vertices[np.unique(mesh_faces[faces])]
            if (
                normals[index][0] < -0.999
                and np.ptp(vertices[:, 1]) > 0.15
                and vertices[:, 2].mean() < -0.08
            ):
                candidates.append(vertices)
        if not candidates:
            raise ValueError("CAD rail underside not identified")
        lowest = min(float(v[:, 0].mean()) for v in candidates)
        vertices = np.vstack([v for v in candidates if abs(v[:, 0].mean() - lowest) < 1e-6])
        vertices.setflags(write=False)
        rails[side] = vertices
    return rails


def lowest_rotation_sweep(points, start_rotation, end_rotation):
    """Exact minimum Z of fixed vertices along the shortest rotation arc."""
    vector = Rotation.from_matrix(start_rotation.T @ end_rotation).as_rotvec()
    angle = float(np.linalg.norm(vector))
    if angle < 1e-10:
        return float((points @ start_rotation.T)[:, 2].min())
    if abs(angle - np.pi) < 1e-7:
        # At exactly pi both arcs are equivalent; cover either convention.
        return -float(np.linalg.norm(points, axis=1).max())
    axis = vector / angle
    parallel = np.outer(points @ axis, axis)
    a = ((points - parallel) @ start_rotation.T)[:, 2]
    b = (np.cross(axis, points) @ start_rotation.T)[:, 2]
    c = (parallel @ start_rotation.T)[:, 2]
    lowest = np.minimum(a + c, a * np.cos(angle) + b * np.sin(angle) + c)
    critical = (np.arctan2(b, a) + np.pi) % (2 * np.pi)
    inside = critical <= angle
    lowest[inside] = c[inside] - np.hypot(a[inside], b[inside])
    return float(lowest.min())


class VisualGraspPlanner:
    def __init__(
        self,
        urdf,
        sdk_chains,
        contacts,
        lower,
        upper,
        *,
        collision_check=None,
        gripper_config=None,
        position_tolerance_m=0.15,
    ):
        if hashlib.sha256(Path(urdf).read_bytes()).hexdigest() != contacts["urdf_sha256"]:
            raise ValueError("Hook contact/URDF fingerprint mismatch")
        self.model = SDKAlignedModel(urdf, sdk_chains, lower, upper)
        self.ik = self.model.ik(position_tolerance_m=position_tolerance_m)
        self.mounts = nominal_hook_mounts(self.model.urdf, contacts)
        self.collision_check = collision_check
        self.gripper_config = copy.deepcopy(gripper_config)
        self.gripper_geometry = RobotModel(urdf)
        for name, joint in self.gripper_geometry.joints.items():
            if "finger_joint" in name:
                joint["bounds"] = None
        self.hand_meshes = {side: [] for side in SIDES}
        xml = ET.parse(urdf).getroot()
        for side in SIDES:
            for suffix in ("ee_base_link", "ee_link1", "ee_link2"):
                link = f"openarm_{side}_{suffix}"
                for visual in xml.findall(f"link[@name='{link}']/visual"):
                    node = visual.find("geometry/mesh")
                    if node is None:
                        raise ValueError("Supplied gripper mesh required")
                    mesh = trimesh.load(Path(urdf).parent / node.get("filename"), force="mesh")
                    points = np.asarray(mesh.vertices) * np.fromstring(
                        node.get("scale", "1 1 1"), sep=" "
                    )
                    local = origin(visual.find("origin"))
                    self.hand_meshes[side].append((link, points @ local[:3, :3].T + local[:3, 3]))
        self.rail_points_tool = {}
        xml = ET.parse(urdf).getroot()
        mesh_paths = [
            Path(urdf).parent
            / xml.find(f"link[@name='openarm_{side}_ee_base_link']/visual/geometry/mesh").get(
                "filename"
            )
            for side in SIDES
        ]
        mesh_fingerprints = tuple(
            (str(path.resolve()), hashlib.sha256(path.read_bytes()).hexdigest())
            for path in mesh_paths
        )
        rails = _rail_vertices(
            str(Path(urdf).resolve()), contacts["urdf_sha256"], mesh_fingerprints
        )
        for side, vertices in rails.items():
            wrist_palm = self.model.urdf.transform(
                f"openarm_{side}_ee_base_link", {}, base=f"openarm_{side}_link7"
            )
            tool_palm = np.linalg.inv(self.mounts[side]) @ wrist_palm
            self.rail_points_tool[side] = vertices @ tool_palm[:3, :3].T + tool_palm[:3, 3]

    def gripper_envelope(self, side, q16=None):
        """CAD palm/fingers covering current and requested-open widths, in wrist axes."""
        joint_name = f"openarm_{side}_finger_joint1"
        low, high = self.model.urdf.joints[joint_name]["bounds"]
        openings = [low, high]
        if self.gripper_config is not None:
            config = self.gripper_config[side]
            feedback = [config["configured_open_rad"]]
            if q16 is not None:
                feedback.append(q16[15 if side == "left" else 7])
            openings.extend(
                config["closed_displacement_m"]
                + 0.044 * config["opening_sign"] * (angle - config["closed_feedback_rad"]) / 1.0472
                for angle in feedback
            )
        points = []
        for opening in (min(openings), max(openings)):
            for link, vertices in self.hand_meshes[side]:
                transform = self.gripper_geometry.transform(
                    link, {joint_name: opening}, base=f"openarm_{side}_link7"
                )
                points.append(vertices @ transform[:3, :3].T + transform[:3, 3])
        return np.vstack(points)

    def resolve_target(self, target, q16=None):
        result = copy.deepcopy(target)
        if target.get("insertion_reference") != "sliding_rail_underside":
            return result
        clearance = float(target["rail_stop_clearance_m"])
        approach = float(
            target.get("alignment_clearance_m", target.get("pregrasp_vertical_clearance_m", 0.03))
        )
        if (
            not np.isfinite([clearance, approach]).all()
            or not 0 <= clearance <= 0.02
            or not 0 < approach <= 0.04
        ):
            raise ValueError("Invalid rail/alignment margins")
        raw = target.get("visual_rim_reference_poses", target["grasp"])
        result["visual_rim_reference_poses"] = copy.deepcopy(raw)
        highest_rim = max(float(point[2]) for point in target["rim_top"].values())
        result["required_downward_m"] = {}
        for side in SIDES:
            tool = pose(raw[side]).copy()
            rotated_rail = self.rail_points_tool[side] @ tool[:3, :3].T
            tool[2, 3] = target["rim_top"][side][2] + clearance - float(rotated_rail[:, 2].min())
            result["grasp"][side] = tool.tolist()
            inverse_mount = np.linalg.inv(self.mounts[side])
            wrist_points = self.gripper_envelope(side, q16)
            tool_points = wrist_points @ inverse_mount[:3, :3].T + inverse_mount[:3, 3]
            lowest = float((tool_points @ tool[:3, :3].T)[:, 2].min())
            alignment_z = highest_rim + approach - lowest
            result["required_downward_m"][side] = max(0.0, alignment_z - tool[2, 3])
            tool[2, 3] = alignment_z
            result["pregrasp"][side] = tool.tolist()
        result["rail_target_resolved"] = True
        result["alignment_reference"] = (
            "lowest_gripper_geometry_at_current_and_open_width_above_rim"
        )
        return result

    def alignment_targets(self, target, q16, height_m):
        """Raise in place, align XY/orientation at clearance, then settle vertically."""
        measured = self.model.values(q16, height_m)
        current = self.ik.fk(measured)
        destination = self.wrist_targets(target, "pregrasp", height_m, q16=q16)
        raised, aligned = {}, {}
        highest_rim = max(float(point[2]) for point in target["rim_top"].values()) + height_m
        margin = float(
            target.get("alignment_clearance_m", target.get("pregrasp_vertical_clearance_m", 0.03))
        )
        for side in SIDES:
            points = self.gripper_envelope(side, q16)
            lowest = lowest_rotation_sweep(points, current[side][:3, :3], destination[side][:3, :3])
            safe_z = max(
                current[side][2, 3], destination[side][2, 3], highest_rim + margin - lowest
            )
            raised[side] = current[side].copy()
            raised[side][2, 3] = safe_z
            aligned[side] = destination[side].copy()
            aligned[side][2, 3] = safe_z
        return dict(clearance=raised, align=aligned, pregrasp=destination)

    def wrist_targets(self, target, stage, height_m, *, q16=None):
        if target.get("frame_id") != "arm_origin":
            raise ValueError("Visual tool poses must use SDK arm_origin")
        if target.get("tool_frame") != "cad_contact_midpoint_pinch_y":
            raise ValueError("Explicit hook reference and orientation convention required")
        target = self.resolve_target(target, q16)
        shift = np.eye(4)
        shift[2, 3] = height_m
        return {
            side: shift @ pose(target[stage][side]) @ np.linalg.inv(self.mounts[side])
            for side in SIDES
        }

    def stage(self, targets, q16, height_m, *, steps=20):
        measured = self.model.values(q16, height_m)
        if hasattr(self.collision_check, "refresh"):
            self.collision_check.refresh(q16)
        trajectory = plan_pair(
            self.ik,
            targets,
            measured,
            fixed_lift=height_m,
            steps=steps,
            speed_limits={name: 0.03 for name in self.ik.names if name != "lift_joint"},
            collision_check=self.collision_check,
        )
        result = asdict(trajectory)
        result["frame_id"] = self.model.base
        result["fixed_lift_m"] = height_m
        result["targets"] = {side: pose(t).tolist() for side, t in targets.items()}
        result["initial_q16"] = list(q16)
        result["arm_targets"] = [
            self.model.sdk_target(point["joints"], q16) for point in trajectory.points
        ]
        return result

    def plan(self, target, q16, height_m, *, steps=20):
        result = dict(
            target_id=target["target_id"],
            pair_id=target["pair_id"],
            stages={},
            errors={},
            execution_blockers=list(target.get("execution_blockers", [])),
            position_tolerance_m=self.ik.ptol,
        )
        if hasattr(self.collision_check, "update_target"):
            self.collision_check.update_target(target)
        result["execution_blockers"].extend(getattr(self.collision_check, "execution_blockers", []))
        current = list(q16)
        goals = {}
        if "rim_top" in target:
            goals.update(self.alignment_targets(target, current, height_m))
        else:
            goals["pregrasp"] = self.wrist_targets(target, "pregrasp", height_m, q16=current)
        goals["grasp"] = self.wrist_targets(target, "grasp", height_m, q16=current)
        for stage, goal in goals.items():
            try:
                planned = self.stage(goal, current, height_m, steps=steps)
                result["stages"][stage] = planned
                current = planned["arm_targets"][-1]
            except ValueError as exc:
                result["errors"][stage] = str(exc)
                break
        if self.collision_check is None:
            result["execution_blockers"].append("scene_collision_checker_not_connected")
        if result["errors"]:
            reason = (
                "collision_path_rejected"
                if any("collision" in message.lower() for message in result["errors"].values())
                else "IK_path_infeasible"
            )
            result["execution_blockers"].append(reason)
        if hasattr(self.collision_check, "describe"):
            result["collision_check"] = self.collision_check.describe()
        result["execution_ready"] = not result["execution_blockers"]
        # A successful plan never changes a diagnostic visual target to accepted.
        return result
