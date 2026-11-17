"""Fixed visual goals -> board MoveIt planner -> standard trajectory action.

The notebook never generates the 100 Hz motor references in this mode.
Existing gripper operations and measured endpoint checks remain in the task layer.
"""

import copy
import json
import threading
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import numpy as np
from wheel_sdk_bridge.trajectory_client import JointTrajectoryClient
from wheel_task_core.transforms import pose

from wheel_execution.phase_driver import SDKTrajectoryDriver


class MoveItClient:
    def __init__(self, url):
        self.url = url.rstrip("/")

    def call(self, path="/", data=None, timeout=180):
        body = None if data is None else json.dumps(data).encode()
        request = Request(self.url + path, data=body, headers={"Content-Type": "application/json"})
        try:
            with urlopen(request, timeout=timeout) as response:
                return json.load(response)
        except HTTPError as error:
            detail = json.load(error)
            failure = RuntimeError(
                "MoveIt " + path + " failed: " + str(detail.get("error", detail))
            )
            failure.plan_id = detail.get("plan_id")
            raise failure from error

    def require_supported(self):
        state = self.call(timeout=2)
        if state.get("protocol") != 1 or not state.get("ready") or state.get("mock"):
            raise RuntimeError("Real MoveIt gateway is not ready")
        return state

    def stop(self, request_id):
        return self.call("/cancel", {"plan_id": request_id}, timeout=5)


class MoveItTrajectoryDriver(SDKTrajectoryDriver):
    # Compatibility with the existing grasp state machine, not a claim of visual servoing.
    pure_visual_servo = True
    execution_mode = "moveit_trajectory"

    def __init__(self, *args, gateway_url, crate_mesh, **kwargs):
        self.gateway = MoveItClient(gateway_url)
        super().__init__(*args, trajectory_client=self.gateway, **kwargs)
        self.sdk_phases = JointTrajectoryClient(self.joints)
        # Parent gripper helpers still stop native SDK requests, not MoveIt plans.
        self.trajectory = SimpleNamespace(
            require_supported=self.gateway.require_supported,
            stop=self.sdk_phases.stop,
        )
        self.moveit_request_id = None
        import trimesh

        mesh = trimesh.load(crate_mesh, force="mesh", process=False)
        self.mesh_vertices = np.asarray(mesh.vertices).tolist()
        self.mesh_triangles = np.asarray(mesh.faces).tolist()
        self.external_plan = None
        self.visual_events = []

    def record(self, kind, **detail):
        # summary.json stays compact/JSON-native; full arrays go to ExecutionTrace.
        self.visual_events.append(dict(kind=kind, time=self.clock(), phase=detail.get("phase")))
        trace = getattr(self, "trace", None)
        if trace is not None:
            trace.record(kind, **detail)

    def measured(self):
        # A new JTC session changes SDK generation. Adopt only the exact session
        # reported by the gateway for our current plan, never a global owner.
        if self.external_plan:
            state = self.joints.snapshot()
            if state["arm_startup"]["control_generation"] != self.generation:
                deadline = self.clock() + 0.5
                receipt = None
                while self.clock() < deadline:
                    gateway = self.gateway.call(timeout=2)
                    receipt = gateway.get("owned_request") or gateway.get("last_owned_request")
                    if receipt and receipt["plan_id"] == self.external_plan:
                        break
                    self.sleep(0.01)
                if not receipt or receipt["plan_id"] != self.external_plan:
                    raise RuntimeError("MoveIt SDK ownership not acknowledged")
                control = state["arm_startup"]["joint_control"]
                if control["request_id"] != receipt["request_id"]:
                    raise RuntimeError("MoveIt SDK session superseded")
                self.generation = receipt["generation"]
                self.request_id = receipt["request_id"]
                self.moveit_request_id = self.request_id
        return super().measured()

    def scene(self, target, state, lift):
        model = self.planner.model
        values = model.values(state["arms"]["position"], lift["current_um"] * 1e-6)
        values["lift_joint"] += 0.2  # Existing encoder -> CAD lift convention.
        mapping = self.planner.gripper_config
        for side, index in (("right", 7), ("left", 15)):
            item = mapping[side]
            values[f"openarm_{side}_finger_joint1"] = (
                item["closed_displacement_m"]
                + 0.044
                * item["opening_sign"]
                * (state["arms"]["position"][index] - item["closed_feedback_rad"])
                / 1.0472
            )
        world_origin = model.urdf.transform("openarm_left_base_link", values) @ np.linalg.inv(
            model.sdk.transform("openarm_left_base_link")
        )
        obj = dict(
            id="selected_crate",
            frame_id="base_footprint",
            vertices=self.mesh_vertices,
            triangles=self.mesh_triangles,
            pose=(world_origin @ pose(target["object_pose"])).tolist(),
        )
        passive = {n: v for n, v in values.items() if n == "lift_joint" or "finger_joint" in n}
        attached_pose = np.linalg.inv(
            model.urdf.transform("openarm_right_link7", values)
        ) @ np.asarray(obj["pose"])
        return obj, passive, attached_pose.tolist()

    def base_targets(self, targets, height):
        result = {}
        for side, matrix in targets.items():
            origin = self.planner.model.bases[side].copy()
            origin[2, 3] += height
            result[side] = (np.linalg.inv(origin) @ pose(matrix)).tolist()
        return result

    def follow(self, observations, goal, *, name, complete=None, timeout_s=90, stable_s=0.5):
        del stable_s
        deadline = self.clock() + timeout_s
        # Replanning is limited to wrist-triggered insertion stops. Alignment
        # freezes its accepted target and never waits for a newer image.
        for attempt in range(4):
            target = observations.latest()
            if target is None:
                raise ValueError("Fresh visual target required at trajectory admission")
            state, lift = self.measured()
            packet = copy.deepcopy(goal(target, state, lift))
            current = self.planner.ik.fk(
                self.planner.model.values(state["arms"]["position"], lift["current_um"] * 1e-6)
            )
            obj, passive, attached_pose = self.scene(target, state, lift)
            request = dict(
                targets=self.base_targets(packet["targets"], lift["current_um"] * 1e-6),
                start_targets=self.base_targets(current, lift["current_um"] * 1e-6),
                cartesian=name != "align",
                phase=name,
                image_receive_mono_ns=packet["image_receive_mono_ns"],
                pair_id=packet["pair_id"],
                height_um=lift["current_um"],
                attached_object_pose=attached_pose,
                collision_objects=[obj],
                passive_joint_positions=passive,
            )
            if name == "align" and target.get("orientation_policy") == "ground_vertical_pinch_y":
                # Admit alignment only if the next descent is reachable from
                # the actual planned endpoint, with the fingers opened.
                height = lift["current_um"] * 1e-6
                endings = self.planner.wrist_targets(
                    target, "grasp", height, q16=state["arms"]["position"]
                )
                for side, ending in endings.items():
                    beginning = pose(packet["targets"][side])
                    if not np.allclose(beginning[:2, 3], ending[:2, 3], atol=1e-8):
                        raise ValueError("Pregrasp must be directly above grasp")
                    if not np.allclose(beginning[:3, :3], ending[:3, :3], atol=1e-8):
                        raise ValueError("Vertical insertion must preserve claw orientation")
                    if not 0 <= beginning[2, 3] - ending[2, 3] <= 0.2:
                        raise ValueError("Vertical insertion exceeds configured 0.2 m bound")
                opened = {}
                for side, item in self.planner.gripper_config.items():
                    opened[f"openarm_{side}_finger_joint1"] = (
                        item["closed_displacement_m"]
                        + 0.044
                        * item["opening_sign"]
                        * (item["configured_open_rad"] - item["closed_feedback_rad"])
                        / 1.0472
                    )
                request["insertion_check"] = dict(
                    targets=self.base_targets(endings, height), open_fingers=opened
                )
            self.record("phase_start", phase=name, target=target, packet=packet)
            try:
                plan = self.gateway.call("/plan", request)
            except Exception as error:
                self.record("phase_failed", phase=name, error=str(error))
                if getattr(error, "plan_id", None):
                    self.copy_artifacts(error.plan_id)
                raise
            if "insertion_check" in request and not plan.get("insertion_validation", {}).get(
                "accepted"
            ):
                raise RuntimeError("MoveIt gateway did not validate the subsequent insertion")
            self.record("moveit_plan", phase=name, plan=plan)
            self.external_plan = plan["plan_id"]
            outcome = {}

            def execute():
                try:
                    outcome["result"] = self.gateway.call("/execute", {"plan_id": plan["plan_id"]})
                except Exception as error:
                    outcome["error"] = error

            worker = threading.Thread(target=execute, daemon=True)
            worker.start()
            replan = False
            try:
                while worker.is_alive():
                    if self.cancelled.is_set() or self.clock() > deadline:
                        raise TimeoutError("MoveIt phase cancelled or timed out")
                    state, lift = self.measured()
                    if name == "insert":
                        latest = observations.latest()
                        if latest is None:
                            raise RuntimeError("Wrist insertion source stale")
                        updated = goal(latest, state, lift)
                        # Freeze the admitted path. Only an explicit wrist stop
                        # interrupts it; image jitter must not trigger replanning.
                        if (
                            set(updated.get("held_sides", [])) != set(packet.get("held_sides", []))
                            or updated.get("rail_arrived") is True
                        ):
                            self.gateway.stop(plan["plan_id"])
                            replan = True
                            break
                    self.sleep(0.05)
                worker.join(timeout=5)
                if worker.is_alive():
                    raise TimeoutError("MoveIt action did not return after stop")
                if "error" in outcome and not replan:
                    raise outcome["error"]
                receipt = outcome.get("result", {}).get("owned_request")
                if receipt and receipt.get("plan_id") == plan["plan_id"]:
                    self.generation = receipt["generation"]
                    self.request_id = receipt["request_id"]
                    self.moveit_request_id = self.request_id
                self.record(
                    "moveit_result", phase=name, result=outcome.get("result"), replan=replan
                )
            finally:
                if worker.is_alive():
                    self.gateway.stop(plan["plan_id"])
                    worker.join(timeout=5)
                self.external_plan = None
                self.copy_artifacts(plan["plan_id"])
            state, lift = self.measured()
            if replan:
                settle_until = self.clock() + 3
                stable_since = None
                while self.clock() < settle_until:
                    state, lift = self.measured()
                    speed = max(
                        abs(state["arms"]["velocity"][i]) for i in [*range(7), *range(8, 15)]
                    )
                    if speed <= 0.03:
                        stable_since = self.clock() if stable_since is None else stable_since
                        if self.clock() - stable_since >= 0.2:
                            break
                    else:
                        stable_since = None
                    self.sleep(0.02)
                else:
                    raise TimeoutError("Measured arms did not settle after wrist stop")
                if complete is not None and complete(latest, state, lift, updated):
                    self.record("phase_completed", phase=name, reason="bilateral_wrist_stop")
                    self.last_arrival_robot_ns = int(state["arms"]["receive_mono_ns"])
                    return
                continue
            if complete is not None and not complete(
                target, state, lift, goal(target, state, lift) if name == "insert" else packet
            ):
                raise RuntimeError("Measured grasp phase endpoint not confirmed")
            errors = self.endpoint_errors(packet["targets"], state, lift)
            if name != "insert" and any(
                e["position_m"] > self.position_tolerance_m
                or e["angle_rad"] > self.angle_tolerance_rad
                for e in errors.values()
            ):
                raise RuntimeError("Measured wrist endpoint outside tolerance")
            self.last_arrival_robot_ns = int(state["arms"]["receive_mono_ns"])
            self.record("phase_completed", phase=name, errors=errors)
            return
        raise RuntimeError("Insertion exhausted bounded trajectory replans")

    def copy_artifacts(self, plan_id):
        trace = getattr(self, "trace", None)
        if trace is None:
            return
        try:
            files = self.gateway.call("/artifacts/" + plan_id, timeout=10)
            folder = trace.directory / "moveit" / plan_id
            folder.mkdir(parents=True, exist_ok=True)
            for filename, text in files.items():
                if "/" not in filename and filename not in (".", ".."):
                    (folder / filename).write_text(text)
        except Exception as error:
            self.record("moveit_artifact_copy_failed", error=str(error), plan_id=plan_id)

    def hold(self):
        self.cancelled.set()
        self.finish()
        gateway_error = None
        if self.external_plan:
            try:
                self.gateway.stop(self.external_plan)
            except Exception as error:
                gateway_error = str(error)
        # SDK checks this exact request ID itself. This fallback also works when
        # feedback validation or the planning gateway has failed; no global stop.
        if self.moveit_request_id and self.request_id == self.moveit_request_id:
            result = self.joints.http.request(
                "POST",
                "/api/arm_startup",
                dict(
                    command="joint_stop", detail=json.dumps(dict(request_id=self.moveit_request_id))
                ),
            )
            self.record("moveit_scoped_stop", response=result, gateway_error=gateway_error)
            if result.get("success") is not True:
                raise RuntimeError("SDK rejected the owned MoveIt stop: " + str(result))
            return result
        if self.joints.pending_id is not None:
            return self.sdk_phases.stop(self.joints.pending_id)
        if gateway_error:
            raise RuntimeError("MoveIt cancellation failed: " + gateway_error)
        return dict(commands_sent=False, reason="No owned active phase")
