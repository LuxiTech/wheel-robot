#!/usr/bin/env python3
"""HTTP admission + MoveIt planning + standard JTC action; not a trajectory controller."""

import argparse
import copy
import hashlib
import json
import os
import threading
import time
import uuid
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.request import urlopen

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SetHardwareComponentState, SwitchController
from geometry_msgs.msg import Point, Pose
from lifecycle_msgs.msg import State
from luxi_interfaces.msg import ArmState
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    PlanningScene,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPositionIK, GetStateValidity
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from scipy.spatial.transform import Rotation, Slerp
from sensor_msgs.msg import JointState
from shape_msgs.msg import Mesh, MeshTriangle


class Gateway(Node):
    def __init__(self, folder):
        super().__init__("wheel_planning_gateway")
        self.folder = folder
        self.config_context = json.loads((folder / "context.json").read_text())
        self.mock = self.config_context["hardware"] == "mock_components/GenericSystem"
        if not self.mock:
            with urlopen("http://127.0.0.1:18030/api/state", timeout=2) as response:
                sdk = json.load(response)
            if (
                sdk["arm_startup"]["joint_control"].get("joint_stream_planned_trajectory_protocol")
                != 1
            ):
                raise RuntimeError("SDK planned trajectory capability is not deployed")
        self.names = [f"openarm_{side}_joint{i}" for side in ("right", "left") for i in range(1, 8)]
        robot = ET.parse(folder / "robot.urdf").getroot()
        self.limits = {}
        for name in self.names:
            limit = robot.find(f"joint[@name='{name}']/limit")
            self.limits[name] = (float(limit.get("lower")), float(limit.get("upper")))
        self.latest = None
        self.latest_at = 0.0
        self.plans = {}
        self.service_timing = {}
        self.current_plan_run = None
        self.active = None
        self.executing_plan = None
        self.cancel_requested = threading.Event()
        self.owned_request = None
        self.last_owned_request = None
        self.operation = threading.Lock()
        self.rpc_clients = {
            name: self.create_client(kind, path)
            for name, kind, path in [
                ("ik", GetPositionIK, "/compute_ik"),
                ("plan", GetMotionPlan, "/plan_kinematic_path"),
                ("valid", GetStateValidity, "/check_state_validity"),
                ("scene", ApplyPlanningScene, "/apply_planning_scene"),
                ("retime", GetMotionPlan, "/wheel/retime"),
                (
                    "hardware",
                    SetHardwareComponentState,
                    "/controller_manager/set_hardware_component_state",
                ),
            ]
        }
        self.rpc_clients["switch"] = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self.action = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        if self.mock:
            self.subscription = self.create_subscription(
                JointState, "/joint_states", self.feedback, 20
            )
        else:
            self.subscription = self.create_subscription(
                ArmState,
                "/luxi/arms/state",
                self.sdk_feedback,
                1,
                callback_group=ReentrantCallbackGroup(),
            )
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(0.05, self.publish_passive)
        self.passive = dict(self.config_context["passive_joint_positions"])
        self.log_dir = folder.parent / "executions"
        self.log_dir.mkdir(exist_ok=True)

    def feedback(self, message):
        if all(n in message.name for n in self.names):
            self.latest = message
            self.latest_at = time.monotonic()

    def sdk_feedback(self, state):
        # An inactive controller can retain old state interfaces. Only the SDK's
        # original receive time is authoritative for real measured-state age.
        age = (time.monotonic_ns() - state.receive_mono_ns) * 1e-9
        indices = [*range(7), *range(8, 15)]
        if not 0 <= age <= 0.15 or not all(state.valid[i] for i in indices):
            return
        if not np.isfinite([state.position[i] for i in indices]).all():
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = self.names
        message.position = [state.position[i] for i in indices]
        message.velocity = [state.velocity[i] for i in indices]
        message.effort = [state.effort[i] for i in indices]
        self.latest = message
        self.latest_at = state.receive_mono_ns * 1e-9

    def publish_passive(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.passive)
        msg.position = list(self.passive.values())
        if not self.mock and self.latest is not None:
            age = time.monotonic() - self.latest_at
            if 0 <= age <= 0.15:
                # MoveIt needs scene updates, not every 100 Hz hardware sample.
                # Publish at 20 Hz outside the SDK callback to prevent scene/DDS
                # backpressure from ageing the independent measured-state cache.
                msg.header.stamp = (self.get_clock().now() - Duration(seconds=age)).to_msg()
                msg.name = list(self.latest.name) + msg.name
                msg.position = list(self.latest.position) + list(msg.position)
        self.publisher.publish(msg)

    def wait(self, future, timeout=30):
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout):
            raise TimeoutError("ROS operation deadline exceeded")
        return future.result()

    def call(self, name, request):
        client = self.rpc_clients[name]
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError("ROS service missing: " + name)
        began = time.monotonic()
        try:
            return self.wait(client.call_async(request))
        finally:
            elapsed = time.monotonic() - began
            item = self.service_timing.setdefault(name, dict(calls=0, total_s=0.0, max_s=0.0))
            item["calls"] += 1
            item["total_s"] += elapsed
            item["max_s"] = max(item["max_s"], elapsed)

    def state(self):
        if self.latest is None or time.monotonic() - self.latest_at > 0.15:
            raise RuntimeError("Measured joint states stale")
        msg = RobotState()
        msg.joint_state = copy.deepcopy(self.latest)
        for name, value in self.passive.items():
            if name not in msg.joint_state.name:
                msg.joint_state.name.append(name)
                msg.joint_state.position.append(value)
        # Velocities/efforts are optional; avoid partial-length arrays after passive joints.
        msg.joint_state.velocity = []
        msg.joint_state.effort = []
        return msg

    def constraints(self, state):
        values = dict(zip(state.joint_state.name, state.joint_state.position))
        return Constraints(
            joint_constraints=[
                JointConstraint(
                    joint_name=n,
                    position=values[n],
                    tolerance_above=1e-5,
                    tolerance_below=1e-5,
                    weight=1.0,
                )
                for n in self.names
            ]
        )

    def valid(self, state):
        result = self.call(
            "valid", GetStateValidity.Request(robot_state=state, group_name="both_arms")
        )
        if not result.valid:
            contacts = [(c.contact_body_1, c.contact_body_2) for c in result.contacts]
            raise RuntimeError("Planning state collision/constraint failure: " + str(contacts))

    def ik(self, state, targets):
        state = copy.deepcopy(state)
        for side, target in targets.items():
            req = GetPositionIK.Request()
            req.ik_request.group_name = side + "_arm"
            req.ik_request.robot_state = state
            # Let KDL search another redundancy branch instead of accepting
            # the first colliding solution and rejecting the entire target.
            req.ik_request.avoid_collisions = True
            req.ik_request.ik_link_name = f"openarm_{side}_link7"
            req.ik_request.pose_stamped.header.frame_id = f"openarm_{side}_base_link"
            matrix = np.asarray(target, dtype=float)
            req.ik_request.pose_stamped.pose = to_pose(matrix)
            req.ik_request.timeout.sec = 1
            result = self.call("ik", req)
            if result.error_code.val != 1:
                raise RuntimeError(f"MoveIt IK failed: {side} {result.error_code.val}")
            state = result.solution
        self.valid(state)
        return state

    def validate_insertion(self, endpoint, data):
        """Read-only IK/collision sweep from the exact alignment endpoint branch."""
        check = data.get("insertion_check")
        if check is None:
            return {}
        beginnings = data["targets"]
        endings = check["targets"]
        if set(beginnings) != {"left", "right"} or set(endings) != set(beginnings):
            raise ValueError("Bilateral insertion targets required")
        state = copy.deepcopy(endpoint)
        indices = {name: i for i, name in enumerate(state.joint_state.name)}
        opened = check["open_fingers"]
        if set(opened) != {f"openarm_{side}_finger_joint1" for side in ("left", "right")}:
            raise ValueError("Both open-finger positions required")
        original = {name: state.joint_state.position[indices[name]] for name in opened}
        # Opening happens before descent; include its intermediate collision states.
        for alpha in np.linspace(0, 1, 9):
            for name, value in opened.items():
                if not np.isfinite(value):
                    raise ValueError("Invalid open-finger position")
                state.joint_state.position[indices[name]] = original[name] + alpha * (
                    value - original[name]
                )
            self.valid(state)
        distances = []
        for side in endings:
            begin, end = np.asarray(beginnings[side]), np.asarray(endings[side])
            if not np.isfinite([begin, end]).all() or not np.allclose(
                begin[:3, :3], end[:3, :3], atol=1e-8
            ):
                raise ValueError("Insertion check must preserve orientation")
            distances.append(float(np.linalg.norm(end[:3, 3] - begin[:3, 3])))
        if max(distances) > 0.2 + 1e-8:
            raise ValueError("Insertion check exceeds 0.2 m")
        steps = max(1, int(np.ceil(max(distances) / 0.002)))
        samples = []
        for index in range(1, steps + 1):
            targets = {}
            for side in endings:
                begin, end = np.asarray(beginnings[side]), np.asarray(endings[side])
                matrix = begin.copy()
                matrix[:3, 3] += (end[:3, 3] - begin[:3, 3]) * index / steps
                targets[side] = matrix
            previous = dict(zip(state.joint_state.name, state.joint_state.position))
            try:
                state = self.ik(state, targets)
                if (
                    max(
                        abs(value - previous[name])
                        for name, value in zip(state.joint_state.name, state.joint_state.position)
                        if name in self.names
                    )
                    > 0.05
                ):
                    raise ValueError("Insertion IK branch jump")
            except Exception as error:
                raise RuntimeError(
                    f"Insertion precheck failed at sample {index}/{steps}: {error}"
                ) from error
            samples.append(dict(zip(state.joint_state.name, state.joint_state.position)))
        report = dict(
            accepted=True,
            steps=steps,
            max_step_m=0.002,
            open_fingers=opened,
            joint_samples=samples,
            scope="sampled geometric IK and collision check; not physical tracking guarantee",
        )
        (self.current_plan_run / "insertion_validation.json").write_text(
            json.dumps(report, indent=2)
        )
        return dict(accepted=True, steps=steps, max_step_m=0.002)

    def plan(self, data):
        started = time.monotonic()
        self.service_timing = {}
        identity = uuid.uuid4().hex
        run = self.log_dir / identity
        run.mkdir()
        self.current_plan_run = run
        (run / "request.json").write_text(json.dumps(data, indent=2))
        # The image clock belongs to the robot, never the laptop.
        if not self.mock:
            with urlopen("http://127.0.0.1:18030/api/state", timeout=0.5) as response:
                sdk = json.load(response)
            age = (int(sdk["server_mono_ns"]) - int(data["image_receive_mono_ns"])) / 1e9
            if not 0 <= age <= 2:
                raise ValueError("Image expired at fixed target admission")
            self.check_lift(data["height_um"])
        self.passive.update(data.get("passive_joint_positions", {}))
        start = self.state()
        scene = PlanningScene()
        scene.is_diff = True
        for obj in data.get("collision_objects", []):
            co = CollisionObject()
            co.id = obj["id"]
            co.header.frame_id = obj["frame_id"]
            co.operation = CollisionObject.ADD
            mesh = Mesh()
            mesh.vertices = [
                Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in obj["vertices"]
            ]
            mesh.triangles = [
                MeshTriangle(vertex_indices=[int(i) for i in face]) for face in obj["triangles"]
            ]
            co.meshes = [mesh]
            co.mesh_poses = [to_pose(np.asarray(obj["pose"]))]
            if data.get("phase") == "test_lift":
                # Model the carried box after the existing bilateral closure check.
                co.header.frame_id = "openarm_right_link7"
                co.mesh_poses = [to_pose(data["attached_object_pose"])]
                attached = AttachedCollisionObject()
                attached.link_name = "openarm_right_link7"
                attached.object = co
                attached.touch_links = [
                    f"openarm_{side}_ee_link{i}" for side in ("left", "right") for i in (1, 2)
                ]
                scene.robot_state.is_diff = True
                scene.robot_state.attached_collision_objects.append(attached)
                removal = CollisionObject(id=co.id, operation=CollisionObject.REMOVE)
                scene.world.collision_objects.append(removal)
            else:
                scene.world.collision_objects.append(co)
        if not self.mock and not scene.world.collision_objects:
            raise ValueError("Selected crate collision mesh required")
        if (
            scene.world.collision_objects
            and not self.call("scene", ApplyPlanningScene.Request(scene=scene)).success
        ):
            raise RuntimeError("Planning scene update rejected")
        if scene.robot_state.attached_collision_objects:
            start.attached_collision_objects = copy.deepcopy(
                scene.robot_state.attached_collision_objects
            )
        self.valid(start)
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = "both_arms"
        motion.start_state = start
        motion.max_velocity_scaling_factor = 1.0
        motion.max_acceleration_scaling_factor = 1.0
        motion.allowed_planning_time = 10.0
        motion.num_planning_attempts = 3
        if data.get("cartesian"):
            beginnings = data["start_targets"]
            endings = data["targets"]
            steps = max(
                2,
                max(
                    int(
                        np.ceil(
                            np.linalg.norm(
                                np.asarray(endings[s])[:3, 3] - np.asarray(beginnings[s])[:3, 3]
                            )
                            / 0.002
                        )
                    )
                    for s in endings
                ),
            )
            if steps > 250:
                raise ValueError("Cartesian phase too long")
            state = start
            waypoints = [self.constraints(start)]
            for index in range(1, steps + 1):
                targets = {}
                for side, end in endings.items():
                    begin = np.asarray(beginnings[side])
                    end = np.asarray(end)
                    alpha = index / steps
                    matrix = np.eye(4)
                    matrix[:3, 3] = (1 - alpha) * begin[:3, 3] + alpha * end[:3, 3]
                    matrix[:3, :3] = Slerp(
                        [0, 1], Rotation.from_matrix([begin[:3, :3], end[:3, :3]])
                    )([alpha]).as_matrix()[0]
                    targets[side] = matrix
                previous = dict(zip(state.joint_state.name, state.joint_state.position))
                state = self.ik(state, targets)
                if (
                    max(
                        abs(v - previous[n])
                        for n, v in zip(state.joint_state.name, state.joint_state.position)
                        if n in self.names
                    )
                    > 0.05
                ):
                    raise ValueError("Cartesian IK branch jump")
                waypoints.append(self.constraints(state))
            motion.trajectory_constraints.constraints = waypoints
            result = self.call("retime", request).motion_plan_response
        else:
            goal = self.ik(start, data["targets"])
            motion.goal_constraints = [self.constraints(goal)]
            motion.pipeline_id = "ompl"
            motion.planner_id = "RRTConnectkConfigDefault"
            result = self.call("plan", request).motion_plan_response
        if result.error_code.val != 1:
            raise RuntimeError("MoveIt planning failed: " + str(result.error_code.val))
        trajectory = result.trajectory.joint_trajectory
        if not trajectory.points:
            raise RuntimeError("Empty planned trajectory")
        # Retime can alter interpolation. Validate the returned samples as well
        # as the geometric planner path, before any controller takes ownership.
        if set(trajectory.joint_names) != set(self.names) or len(trajectory.joint_names) != 14:
            raise ValueError("Trajectory must contain each arm joint exactly once")
        previous_time = -1.0
        checked = copy.deepcopy(start)
        indices = {n: checked.joint_state.name.index(n) for n in self.names}
        for point in trajectory.points:
            stamp = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            if stamp <= previous_time or len(point.positions) != 14:
                raise ValueError("Invalid trajectory time/position dimensions")
            previous_time = stamp
            for name, value in zip(trajectory.joint_names, point.positions):
                lower, upper = self.limits[name]
                if not np.isfinite(value) or not lower <= value <= upper:
                    raise ValueError("Retimed trajectory violates joint limits: " + name)
                checked.joint_state.position[indices[name]] = value
            for values, bound in ((point.velocities, 0.2), (point.accelerations, 0.5)):
                if (
                    len(values) != 14
                    or not np.isfinite(values).all()
                    or max(map(abs, values)) > bound + 1e-6
                ):
                    raise ValueError("Retimed trajectory violates velocity/acceleration limits")
            self.valid(checked)
        insertion_validation = self.validate_insertion(checked, data)
        encoded = json.dumps(message_to_ordereddict(trajectory))
        (run / "trajectory.json").write_text(encoded)
        self.plans[identity] = (trajectory, start, run, data)
        (run / "planning_timing.json").write_text(json.dumps(self.service_timing, indent=2))
        return dict(
            plan_id=identity,
            insertion_validation=insertion_validation,
            points=len(trajectory.points),
            planning_s=time.monotonic() - started,
            service_timing=self.service_timing,
            trajectory_sha256=hashlib.sha256(encoded.encode()).hexdigest(),
            log_directory=str(run),
            mock=self.mock,
        )

    def check_lift(self, height_um):
        if self.mock:
            return
        with urlopen("http://127.0.0.1:8000/api/state", timeout=0.5) as response:
            state = json.load(response)
        if (
            abs(state["lift"]["current_um"] - height_um) > 200
            or not state["lift_status"]["stationary"]
            or state.get("control", {}).get("control_source") != "console"
        ):
            raise RuntimeError("Fixed lift height or console authority changed")

    def hardware(self, active):
        if self.mock:
            return
        req = SetHardwareComponentState.Request()
        req.name = "WheelSDK"
        req.target_state.id = State.PRIMARY_STATE_ACTIVE if active else State.PRIMARY_STATE_INACTIVE
        if active and not self.call("hardware", req).ok:
            raise RuntimeError("SDK hardware activation rejected; enabled stationary hold required")
        switch = SwitchController.Request()
        switch.strictness = SwitchController.Request.STRICT
        switch.activate_controllers = ["arm_controller"] if active else []
        switch.deactivate_controllers = [] if active else ["arm_controller"]
        if not self.call("switch", switch).ok:
            if active:
                req.target_state.id = State.PRIMARY_STATE_INACTIVE
                self.call("hardware", req)
            raise RuntimeError("Trajectory controller switch failed")
        if not active and not self.call("hardware", req).ok:
            raise RuntimeError("SDK hardware deactivation failed")

    def execute(self, data):
        trajectory, start, run, source = self.plans.pop(data["plan_id"])
        now = self.state()
        current = dict(zip(now.joint_state.name, now.joint_state.position))
        if (
            max(
                abs(current[n] - v)
                for n, v in zip(trajectory.joint_names, trajectory.points[0].positions)
            )
            > 0.02
        ):
            raise ValueError("Robot moved since planning")
        if not self.action.wait_for_server(timeout_sec=5):
            raise RuntimeError("JTC action unavailable")
        began = time.monotonic()
        goal = None
        hardware_attempted = False
        self.executing_plan = data["plan_id"]
        self.cancel_requested.clear()
        try:
            hardware_attempted = True
            self.check_lift(source["height_um"])
            self.hardware(True)
            if not self.mock:
                with urlopen("http://127.0.0.1:18030/api/state", timeout=0.5) as response:
                    sdk = json.load(response)
                self.owned_request = dict(
                    request_id=sdk["arm_startup"]["joint_control"]["request_id"],
                    generation=sdk["arm_startup"]["control_generation"],
                    plan_id=data["plan_id"],
                )
            if self.mock:
                self.owned_request = dict(plan_id=data["plan_id"])
            if self.cancel_requested.is_set():
                raise RuntimeError("Plan cancelled before trajectory submission")
            goal = self.wait(
                self.action.send_goal_async(
                    FollowJointTrajectory.Goal(trajectory=trajectory),
                    feedback_callback=lambda f: self.record_feedback(run, f),
                )
            )
            if not goal.accepted:
                raise RuntimeError("JTC rejected trajectory")
            self.active = goal
            seconds = (
                trajectory.points[-1].time_from_start.sec
                + trajectory.points[-1].time_from_start.nanosec * 1e-9
            )
            result_future = goal.get_result_async()
            deadline = time.monotonic() + seconds * 1.2 + 10
            while not result_future.done():
                if self.cancel_requested.is_set():
                    raise RuntimeError("Owned plan cancellation requested")
                if time.monotonic() > deadline:
                    raise TimeoutError("JTC execution deadline exceeded")
                self.check_lift(source["height_um"])
                time.sleep(0.05)
            result = result_future.result()
            report = dict(
                owned_request=self.owned_request,
                status=result.status,
                error_code=result.result.error_code,
                error_string=result.result.error_string,
                execution_s=time.monotonic() - began,
            )
            (run / "result.json").write_text(json.dumps(report, indent=2))
            if result.status != 4 or result.result.error_code != 0:
                raise RuntimeError("JTC execution failed: " + json.dumps(report))
            return report
        except Exception as error:
            (run / "failure.json").write_text(
                json.dumps(
                    dict(
                        error=str(error),
                        execution_s=time.monotonic() - began,
                    ),
                    indent=2,
                )
            )
            if goal is not None and goal.accepted:
                try:
                    self.wait(goal.cancel_goal_async(), 3)
                except Exception as cancel_error:
                    (run / "cancel_failure.txt").write_text(str(cancel_error))
            raise
        finally:
            self.active = None
            self.executing_plan = None
            if hardware_attempted:
                try:
                    self.hardware(False)
                except Exception as cleanup_error:
                    (run / "cleanup_failure.txt").write_text(str(cleanup_error))
                    raise
                finally:
                    self.last_owned_request = self.owned_request
                    self.owned_request = None

    def record_feedback(self, run, feedback):
        with (run / "feedback.jsonl").open("a") as stream:
            stream.write(
                json.dumps(
                    dict(
                        wall_time_ns=time.time_ns(),
                        feedback=message_to_ordereddict(feedback.feedback),
                    )
                )
                + "\n"
            )


def to_pose(matrix):
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("Finite 4x4 pose required")
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, matrix[:3, 3])
    q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)
    return pose


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config-dir", type=Path, required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18036)
    args, ros = parser.parse_known_args()
    rclpy.init(args=ros)
    node = Gateway(args.config_dir)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except Exception as error:
            if rclpy.ok():
                node.get_logger().error("ROS executor stopped: " + repr(error))
                (node.log_dir / "executor_failure.txt").write_text(repr(error))
                # Fail the critical node; launch shuts down the trajectory
                # controller, and the SDK command watchdog remains independent.
                os._exit(1)

    worker = threading.Thread(target=spin, daemon=True)
    worker.start()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path.startswith("/artifacts/"):
                identity = self.path.rsplit("/", 1)[-1]
                if len(identity) != 32 or any(c not in "0123456789abcdef" for c in identity):
                    self.send_error(400)
                    return
                folder = node.log_dir / identity
                if not folder.is_dir():
                    self.send_error(404)
                    return
                files = {
                    p.name: p.read_text()
                    for p in folder.iterdir()
                    if p.is_file() and p.stat().st_size <= 16 * 1024 * 1024
                }
                body = json.dumps(files).encode()
                self.send_response(200)
                self.end_headers()
                self.wfile.write(body)
                return
            body = json.dumps(
                dict(
                    mock=node.mock,
                    ready=(
                        node.latest is not None
                        and time.monotonic() - node.latest_at <= 0.15
                        and all(
                            node.rpc_clients[key].service_is_ready()
                            for key in ("ik", "plan", "valid", "scene", "retime")
                        )
                    ),
                    feedback_age_s=(
                        None if node.latest is None else time.monotonic() - node.latest_at
                    ),
                    services={
                        key: node.rpc_clients[key].service_is_ready()
                        for key in ("ik", "plan", "valid", "scene", "retime")
                    },
                    active=node.active is not None,
                    protocol=1,
                    insertion_precheck_protocol=1,
                    owned_request=node.owned_request,
                    last_owned_request=node.last_owned_request,
                )
            ).encode()
            self.send_response(200)
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 16 * 1024 * 1024:
                    raise ValueError("Invalid request size")
                data = json.loads(self.rfile.read(size))
                if self.path == "/cancel":
                    if node.executing_plan != data.get("plan_id") or node.executing_plan is None:
                        raise ValueError("Cancel requires the executing plan ID")
                    node.cancel_requested.set()
                    result = dict(cancelled=True, plan_id=node.executing_plan)
                    if node.active is not None:
                        node.wait(node.active.cancel_goal_async(), 3)
                else:
                    if not node.operation.acquire(blocking=False):
                        raise RuntimeError("Another motion operation owns gateway")
                    try:
                        if self.path == "/plan":
                            node.current_plan_run = None
                            result = node.plan(data)
                        elif self.path == "/execute":
                            result = node.execute(data)
                        else:
                            raise ValueError("Unknown operation")
                    except Exception as error:
                        # Record this operation while still owning the lock; a
                        # concurrent rejected request must not overwrite its log.
                        if self.path == "/plan" and node.current_plan_run is not None:
                            error.plan_id = node.current_plan_run.name
                            failure = dict(
                                error=str(error),
                                plan_id=error.plan_id,
                                service_timing=node.service_timing,
                            )
                            (node.current_plan_run / "failure.json").write_text(
                                json.dumps(failure, indent=2)
                            )
                        raise
                    finally:
                        node.operation.release()
                self.send_response(200)
                body = json.dumps(result).encode()
            except Exception as error:
                self.send_response(409)
                failure = dict(error=str(error))
                if getattr(error, "plan_id", None):
                    failure["plan_id"] = error.plan_id
                body = json.dumps(failure).encode()
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    try:
        ThreadingHTTPServer((args.host, args.port), Handler).serve_forever()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
