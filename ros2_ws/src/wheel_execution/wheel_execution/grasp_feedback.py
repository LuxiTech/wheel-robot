"""Concrete read-only wrist stopping and visual trial-lift verification sources."""

import hashlib
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import trimesh
from wheel_motion_core.kinematics import RobotModel
from wheel_perception_core.geometry import prepare_input
from wheel_perception_core.insertion import insertion_clearance
from wheel_perception_core.rgbd import RGBDClient

from wheel_execution.reuse_existing import REUSE_EXISTING, runtime_blockers


class HeadModelClearanceSource:
    """Existing head rim pose + measured FK + supplied rail mesh, in SDK coordinates."""

    supports_motion = True
    resource_policy = REUSE_EXISTING

    def __init__(self, driver, *, clearance_m, uncertainty_m):
        self.driver = driver
        self.clearance_m = clearance_m
        self.uncertainty_m = uncertainty_m

    def __call__(self, target):
        if runtime_blockers(target.get("execution_blockers", []), REUSE_EXISTING):
            raise ValueError("Head rim observation has a runtime failure")
        state, lift = self.driver.measured()
        height_m = lift["current_um"] * 1e-6
        planner = self.driver.planner
        actual = planner.ik.fk(planner.model.values(state["arms"]["position"], height_m))
        remaining, wrist_z, measurements = {}, {}, {}
        for side in ("left", "right"):
            tool = actual[side] @ planner.mounts[side]
            # 复用已有导轨网格与SDK关节链，不另设相机外参、不加关节角偏置。
            rails = planner.rail_points_tool[side] @ tool[:3, :3].T + tool[:3, 3]
            rim_z = float(target["rim_top"][side][2]) + height_m
            rail_z = float(rails[:, 2].min())
            gap = rail_z - rim_z
            remaining[side] = gap - self.clearance_m - self.uncertainty_m
            wrist_z[side] = float(actual[side][2, 3])
            measurements[side] = dict(
                rail_lowest_z_m=rail_z, rim_top_z_m=rim_z, remaining_downward_m=remaining[side]
            )
        if not np.isfinite(list(remaining.values())).all():
            raise ValueError("Nonfinite measured rail/rim clearance")
        result = dict(
            accepted=True,
            blockers=[],
            source="head_rim_and_measured_gripper_model",
            observed_at_monotonic=target["observed_at_monotonic"],
            pair_ids={side: target["pair_id"] for side in remaining},
            image_receive_mono_ns=target["image_receive_mono_ns"],
            sdk_wrist_z_m=wrist_z,
            remaining_downward_m=remaining,
            measurements=measurements,
        )
        trace = getattr(self.driver, "trace", None)
        if trace is not None:
            trace.record(
                "measured_gripper_clearance",
                result=result,
                measured_joint_stamp=state["arms"]["receive_mono_ns"],
            )
        return result


class WristClearanceSource:
    def __init__(
        self,
        driver,
        urdf,
        expected_urdf_sha256,
        base_url,
        *,
        clearance_m,
        uncertainty_m,
        geometry_validated=False,
        metric_depth_validated=False,
        resource_policy="strict",
        clock=time.monotonic,
    ):
        self.driver, self.clock = driver, clock
        if hashlib.sha256(urdf.read_bytes()).hexdigest() != expected_urdf_sha256:
            raise ValueError("Wrist/rail geometry fingerprint mismatch")
        self.robot = RobotModel(urdf)
        self.client = RGBDClient(base_url, max_age_ms=2000)
        self.geometry_validated = geometry_validated
        self.metric_depth_validated = metric_depth_validated
        self.resource_policy = resource_policy
        self.clearance_m, self.uncertainty_m = clearance_m, uncertainty_m
        xml = ET.parse(urdf).getroot()
        self.rails = {}
        for side in ("left", "right"):
            visual = xml.find(f"link[@name='openarm_{side}_ee_base_link']/visual")
            origin = visual.find("origin")
            if origin is not None and (
                not np.allclose(np.fromstring(origin.get("xyz", "0 0 0"), sep=" "), 0)
                or not np.allclose(np.fromstring(origin.get("rpy", "0 0 0"), sep=" "), 0)
            ):
                raise ValueError("Rail mesh origin must match supplied CAD")
            mesh = trimesh.load(
                urdf.parent / visual.find("geometry/mesh").get("filename"), force="mesh"
            )
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
                raise ValueError("Supplied sliding-rail underside not identified")
            lowest_x = min(float(vertices[:, 0].mean()) for vertices in candidates)
            self.rails[side] = np.vstack(
                [v for v in candidates if abs(v[:, 0].mean() - lowest_x) < 1e-6]
            )

    def paired_state(self, obs, before, after, lift):
        image_stamp = int(obs.metadata["receive_mono_ns"])
        if (
            self.driver.last_arrival_robot_ns is not None
            and image_stamp < self.driver.last_arrival_robot_ns
        ):
            raise ValueError("Wrist image predates measured waypoint arrival")
        if (
            np.max(np.abs(np.asarray(before["arms"]["position"]) - after["arms"]["position"]))
            > 0.006
        ):
            raise ValueError("Arm moved across wrist capture")
        if max(abs(v) for state in (before, after) for v in state["arms"]["velocity"]) > 0.08:
            raise ValueError("Wrist capture requires settled arms")
        if abs(int(after["arms"]["receive_mono_ns"]) - image_stamp) > 200_000_000:
            raise ValueError("Wrist image/joint feedback pairing too old")
        return after, lift

    def __call__(self, target):
        remaining, pairs, rows, blockers = {}, {}, {}, []
        image_stamps, wrist_z = [], {}
        started = self.clock()
        for side in ("left", "right"):
            before, lift = self.driver.measured()
            obs = self.client.receive(side + "_wrist_rgb")
            after, _ = self.driver.measured()
            after, lift = self.paired_state(obs, before, after, lift)
            image_stamps.append(int(obs.metadata["receive_mono_ns"]))
            sdk_values = self.driver.planner.model.values(
                after["arms"]["position"], lift["current_um"] * 1e-6
            )
            wrist_z[side] = float(self.driver.planner.ik.fk(sdk_values)[side][2, 3])
            values = self.driver.planner.model.values(
                after["arms"]["position"], lift["current_um"] * 1e-6
            )
            # SDK encoder lift and CAD lift have different zero references.
            values["lift_joint"] += 0.2
            camera_frame = side + "_wrist_eb435i_color_optical_frame"
            world_camera = self.robot.transform(camera_frame, values)
            origin_camera = self.driver.planner.model.sdk.gripper_transform(
                self.robot, side, camera_frame, values
            )
            point = (np.linalg.inv(origin_camera) @ np.r_[target["rim_top"][side], 1])[:3]
            if point[2] <= 0:
                raise ValueError("Local rim is behind wrist optical camera")
            prepared = prepare_input(obs)
            projected = prepared.k @ point
            x, y = np.rint(projected[:2] / projected[2]).astype(int)
            h, w = prepared.depth.shape
            if not 20 <= x < w - 20 or not 20 <= y < h - 20:
                raise ValueError("Local top rim is outside a full wrist ROI")
            depth = prepared.depth[y - 18 : y + 19, x - 18 : x + 19]
            hsv = cv2.cvtColor(prepared.rgb[y - 18 : y + 19, x - 18 : x + 19], cv2.COLOR_RGB2HSV)
            valid = (depth > 0) & (np.abs(depth - point[2]) < 0.02)
            valid &= (hsv[:, :, 0] >= 80) & (hsv[:, :, 0] <= 125) & (hsv[:, :, 1] >= 30)
            yy, xx = np.where(valid)
            if len(xx) < 30:
                raise ValueError("Insufficient local blue-rim depth near predicted top lip")
            points = np.c_[xx + x - 18, yy + y - 18, np.ones(len(xx))] @ np.linalg.inv(prepared.k).T
            points *= depth[yy, xx, None]
            camera_palm = np.linalg.inv(world_camera) @ self.robot.transform(
                f"openarm_{side}_ee_base_link", values
            )
            rails = self.rails[side] @ camera_palm[:3, :3].T + camera_palm[:3, 3]
            row = insertion_clearance(
                points,
                rails,
                world_camera[:3, :3].T @ [0, 0, 1],
                clearance_m=self.clearance_m,
                uncertainty_m=self.uncertainty_m,
                rim_confirmed=self.geometry_validated,
                geometry_confirmed=self.geometry_validated,
                depth_semantics_confirmed=self.metric_depth_validated,
            )
            remaining[side] = row["remaining_downward_travel_m"]
            pairs[side] = obs.metadata["pair_id"]
            rows[side] = row
            blockers.extend(side + ":" + reason for reason in row["blockers"])
        effective_blockers = []
        for side in ("left", "right"):
            effective_blockers.extend(
                side + ":" + reason
                for reason in runtime_blockers(rows[side]["blockers"], self.resource_policy)
            )
        return dict(
            accepted=not effective_blockers,
            blockers=effective_blockers,
            resource_warnings=blockers,
            resource_policy=self.resource_policy,
            pair_ids=pairs,
            observed_at_monotonic=started,
            remaining_downward_m=remaining,
            measurements=rows,
            image_receive_mono_ns=min(image_stamps),
            sdk_wrist_z_m=wrist_z,
        )


class MovingWristClearanceSource(WristClearanceSource):
    """Use bracketed measured arm angles while the continuous path is running.

    Pairing is at SDK image receive time, not claimed hardware exposure sync.
    Geometry/depth acceptance flags and source-specific uncertainty still apply.
    """

    supports_motion = True

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        if not callable(getattr(self.driver, "state_at", None)):
            raise ValueError("Moving wrist feedback requires a measured joint history")
        self.client = RGBDClient(self.client.base_url, max_age_ms=2000, timeout=0.3)

    def paired_state(self, obs, before, after, lift):
        del before, after, lift
        stamp = int(obs.metadata["receive_mono_ns"])
        motion_end = getattr(self.driver, "last_motion_end_robot_ns", None)
        if motion_end is None:
            motion_end = self.driver.last_arrival_robot_ns
        if motion_end is not None and stamp < motion_end:
            raise ValueError("Wrist image predates measured waypoint arrival")
        return self.driver.state_at(stamp)


class VisualTestLiftVerifier:
    def __init__(self, observe, driver, contact_policy, *, clock=time.monotonic):
        if contact_policy.get("hardware_validated") is not True:
            raise ValueError("Accepted bilateral contact policy required for test lift")
        self.observe, self.driver, self.policy, self.clock = observe, driver, contact_policy, clock

    def __call__(self, target, closure):
        fresh = self.observe(target["target_id"])
        age = self.clock() - fresh["observed_at_monotonic"]
        if (
            fresh["target_id"] != target["target_id"]
            or fresh["pair_id"] == target["pair_id"]
            or fresh.get("frame_id") != "arm_origin"
            or fresh.get("diagnostic_only") is not False
            or fresh.get("execution_blockers")
            or not 0 <= age <= 2
        ):
            raise ValueError("Fresh accepted same-object trial-lift observation required")
        delta = np.asarray(fresh["object_pose"])[:3, 3] - np.asarray(target["object_pose"])[:3, 3]
        state, _ = self.driver.measured()
        contacts = []
        for side, index in (("left", 15), ("right", 7)):
            p = self.policy[side]
            effort = abs(state["arms"]["effort"][index])
            angle = state["arms"]["position"][index]
            contacts.append(
                p["contact_effort_min_nm"] <= effort < p["hard_effort_max_nm"]
                and p["contact_angle_min_rad"] <= angle <= p["contact_angle_max_rad"]
            )
        accepted = (
            closure.get("bilateral_contact_verified") is True
            and all(contacts)
            and 0.005 <= delta[2] <= 0.015
            and np.linalg.norm(delta[:2]) <= 0.01
        )
        return dict(
            box_followed_both_hands=bool(accepted),
            observed_box_translation_m=delta.tolist(),
            current_contact_verified=all(contacts),
            pair_id=fresh["pair_id"],
        )
