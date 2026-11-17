"""Independent laptop reference loop; perception and HTTP never run in this thread."""

import copy
import threading
import time
import traceback
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context

from wheel_motion_core.joint_servo import (
    JointPostureResolver,
    JointReferenceGenerator,
    initialize_posture_worker,
    posture_worker_ready,
    solve_posture_in_worker,
)
from wheel_sdk_bridge.joint_client import JointClient
from wheel_sdk_bridge.joint_stream_client import JointStreamClient

from wheel_execution.visual_servo import SDKVisualServoDriver


class LaptopReferenceLoop:
    def __init__(self, driver, generator, *, period_s=0.01, clock=time.monotonic):
        self.driver, self.generator = driver, generator
        self.period_s, self.clock = period_s, clock
        self.lock = threading.Lock()
        self.stopped = threading.Event()
        self.thread = None
        self.packet = None
        self.error = None
        self.tick_context = None
        self.metrics = dict(
            generated=0, max_period_s=0.0, max_compute_send_s=0.0, skipped_deadlines=0
        )

    def update(self, packet):
        with self.lock:
            if self.packet is not None:
                old_stamp = self.packet["image_receive_mono_ns"]
                new_stamp = packet["image_receive_mono_ns"]
                if new_stamp < old_stamp:
                    raise ValueError("Visual image timestamp moved backwards")
                if new_stamp == old_stamp and packet["pair_id"] != self.packet["pair_id"]:
                    raise ValueError("Visual image identity changed at the same timestamp")
            self.packet = copy.deepcopy(packet)

    def tick(self, dt):
        # 100 Hz 线程只读缓存并发送 UDP；视觉推理、HTTP 和磁盘写入均在其他线程。
        tick_started_ns = time.monotonic_ns()
        # 失败可能早于reference_send_attempt；保留当次输入，不用上一成功周期冒充。
        self.tick_context = dict(
            dt_s=dt,
            request_id=self.driver.request_id,
            generation=self.driver.generation,
            stage="logging_check",
        )
        trace = getattr(self.driver, "trace", None)
        if trace is not None:
            trace.check()
        self.tick_context["stage"] = "target_cache"
        with self.lock:
            packet = copy.deepcopy(self.packet)
        if packet is None:
            raise ValueError("No accepted visual target for laptop reference loop")
        self.tick_context.update(stage="feedback_cache", packet=packet)
        state, lift, received_at = self.driver.cached_feedback()
        self.tick_context.update(
            stage="feedback_validation",
            state=state,
            lift=lift,
            feedback_cached_at_local_s=received_at,
        )
        joint = state["arm_startup"]["joint_control"]
        if (
            joint["request_id"] != self.driver.request_id
            or joint["phase"] != "streaming"
            or state["arm_startup"]["control_generation"] != self.driver.generation
        ):
            raise RuntimeError(
                "Laptop joint stream stopped or superseded: "
                + str(joint.get("phase"))
                + "/"
                + str(joint.get("reason"))
            )
        if state["arms"].get("enabled") is not True or state["arms"].get("fresh") is not True:
            raise RuntimeError("Fresh enabled feedback required in reference loop")
        if (
            lift.get("stationary") is not True
            or abs(lift["current_um"] - self.driver.lift_um) > 200
        ):
            raise RuntimeError("Fixed lift changed during joint stream")
        # Both source timestamps are from the robot, elapsed cache time is local.
        elapsed_ns = int((self.clock() - received_at) * 1e9)
        now_robot_ns = int(state["server_mono_ns"]) + elapsed_ns
        feedback_stamp = int(state["arms"]["receive_mono_ns"])
        image_stamp = packet["image_receive_mono_ns"]
        self.tick_context.update(
            estimated_robot_now_ns=now_robot_ns,
            feedback_age_s=(now_robot_ns - feedback_stamp) / 1e9,
            image_age_s=(now_robot_ns - image_stamp) / 1e9,
        )
        if not 0 <= now_robot_ns - feedback_stamp <= 150_000_000:
            raise RuntimeError("Measured joint source expired")
        # 用户要求图像有效期 2 秒；关节反馈与断流仍采用各自更短的保护时限。
        if not packet.get("fixed_target") and not 0 <= now_robot_ns - image_stamp <= 2_000_000_000:
            raise RuntimeError("Visual image source expired")
        self.tick_context["stage"] = "joint_reference_generation"
        positions, velocities = self.generator.step(
            packet["targets"],
            state["arms"]["position"],
            lift["current_um"] * 1e-6,
            dt,
            joint_goal=packet.get("joint_goal"),
            measured_velocity=state["arms"].get("velocity"),
            command_bias=state["arm_startup"].get("static_compensation_rad"),
        )
        # 发送前记下实际输入，发送成功再记录序号，便于区分“算出、发出、板端接收”。
        evidence = dict(
            request_id=self.driver.request_id,
            generation=self.driver.generation,
            sequence=getattr(self.driver.servo, "sequence", 0) + 1,
            pair_id=packet["pair_id"],
            image_receive_mono_ns=image_stamp,
            feedback_receive_mono_ns=feedback_stamp,
            estimated_robot_now_ns=now_robot_ns,
            image_age_s=(now_robot_ns - image_stamp) / 1e9,
            dt_s=dt,
            reference_positions=positions,
            reference_velocities=velocities,
            measured=state["arms"],
            measured_endpoint_errors=self.generator.measured_errors,
            sdk_control=joint,
            lift=lift,
            reference_mode="endpoint_ik_joint_tracking"
            if "joint_goal" in packet
            else "differential_ik",
            compute_before_log_s=(time.monotonic_ns() - tick_started_ns) / 1e9,
        )
        if trace is not None:
            self.tick_context["stage"] = "reference_attempt_logging"
            trace.record("reference_send_attempt", **evidence)
        self.tick_context["stage"] = "udp_send"
        udp_started_ns = time.monotonic_ns()
        self.driver.servo.send(
            self.driver.request_id,
            positions,
            velocities,
            feedback_stamp_ns=feedback_stamp,
            image_stamp_ns=image_stamp,
            pair_id=packet["pair_id"],
        )
        udp_finished_ns = time.monotonic_ns()
        self.tick_context["stage"] = "sent"
        if trace is not None:
            # UDP 发送成功仅表示交给网络栈；板端接收序号另见 sdk_feedback。
            trace.record(
                "reference_sent",
                request_id=self.driver.request_id,
                sequence=self.driver.servo.sequence,
                udp_send_s=(udp_finished_ns - udp_started_ns) / 1e9,
                compute_to_send_s=(udp_finished_ns - tick_started_ns) / 1e9,
            )
        self.metrics["generated"] += 1
        self.metrics["max_period_s"] = max(self.metrics["max_period_s"], dt)

    def start(self):
        def run():
            previous = self.clock()
            deadline = previous + self.period_s
            try:
                while not self.stopped.wait(max(0.0, deadline - self.clock())):
                    started = self.clock()
                    self.tick(started - previous)
                    previous = started
                    self.metrics["max_compute_send_s"] = max(
                        self.metrics["max_compute_send_s"], self.clock() - started
                    )
                    deadline += self.period_s
                    if deadline <= self.clock():
                        self.metrics["skipped_deadlines"] += 1
                        deadline = self.clock() + self.period_s
            except BaseException as exc:
                self.error = exc
                # Stop sending immediately; the independent board watchdog remains active.
                self.driver.servo.close()
                trace = getattr(self.driver, "trace", None)
                if trace is not None:
                    try:
                        trace.record(
                            "reference_loop_failed",
                            error=str(exc),
                            traceback=traceback.format_exc(),
                            metrics=self.metrics,
                            failed_tick=self.tick_context,
                            last_sent_sequence=getattr(self.driver.servo, "sequence", None),
                        )
                    except Exception:
                        pass  # Closing the socket above must not depend on logging.

        self.thread = threading.Thread(target=run, daemon=True, name="laptop-joint-100hz")
        self.thread.start()

    def check(self):
        if self.error is not None:
            raise RuntimeError("Laptop joint reference failed: " + str(self.error)) from self.error

    def close(self):
        self.stopped.set()
        if self.thread is not None:
            self.thread.join(timeout=0.2)
            if self.thread.is_alive():
                raise RuntimeError("Laptop reference thread did not stop")


class LaptopVisualServoDriver(SDKVisualServoDriver):
    execution_mode = "laptop_visual_servo"

    def __init__(
        self,
        joints,
        *args,
        servo_client=None,
        trace=None,
        owns_trace=False,
        isolate_posture=None,
        **kwargs,
    ):
        self.trace = trace
        self.owns_trace = owns_trace
        super().__init__(
            joints,
            *args,
            servo_client=servo_client or JointStreamClient(joints, trace=trace),
            **kwargs,
        )
        self.reference_loop = None
        self.posture_resolver = JointPostureResolver(self.planner.model)
        self.posture_cache = None
        self.posture_pending = None
        self.posture_process = None
        isolate_posture = (
            isinstance(joints, JointClient) if isolate_posture is None else isolate_posture
        )
        if isolate_posture:
            # spawn不继承CUDA运行时；完整IK在子进程运行，不占高频回路的解释器锁。
            self.posture_process = ProcessPoolExecutor(
                max_workers=1,
                mp_context=get_context("spawn"),
                initializer=initialize_posture_worker,
                initargs=(self.planner.model,),
            )
            self.posture_process.submit(posture_worker_ready).result(timeout=15)

    def resolve_packet(self, packet, state, lift):
        if "alignment_phase" not in packet:
            return packet
        if self.posture_pending is not None:
            future, prepared, began = self.posture_pending
            if future.done():
                joint_goal, report = future.result()
                self.posture_cache = dict(prepared, joint_goal=joint_goal)
                self.posture_pending = None
                self.record(
                    "alignment_endpoint_ik",
                    phase=prepared["phase"],
                    joint_goal=joint_goal,
                    solver=report,
                )
            elif self.clock() - began > 1.5:
                raise TimeoutError("Endpoint IK worker exceeded its live-target deadline")
        cache = self.posture_cache
        changed = cache is None or cache["phase"] != packet["alignment_phase"]
        if not changed:
            changed = any(
                abs(packet["targets"][side] - cache["targets"][side]).max() > 0.002
                for side in ("left", "right")
            )
        if changed and self.posture_pending is None:
            prepared = dict(
                phase=packet["alignment_phase"],
                targets=copy.deepcopy(packet["targets"]),
                pair_id=packet["pair_id"],
                image_receive_mono_ns=packet["image_receive_mono_ns"],
            )
            arguments = (packet["targets"], state["arms"]["position"], lift["current_um"] * 1e-6)
            if self.posture_process is not None:
                future = self.posture_process.submit(solve_posture_in_worker, *arguments)
                if cache is not None:
                    self.posture_pending = (future, prepared, self.clock())
                else:
                    joint_goal, report = future.result(timeout=10)
                    cache = self.posture_cache = dict(prepared, joint_goal=joint_goal)
                    self.record(
                        "alignment_endpoint_ik",
                        phase=prepared["phase"],
                        joint_goal=joint_goal,
                        solver=report,
                    )
            else:
                joint_goal, report = self.posture_resolver.solve(*arguments)
                cache = self.posture_cache = dict(prepared, joint_goal=joint_goal)
                self.record(
                    "alignment_endpoint_ik",
                    phase=prepared["phase"],
                    joint_goal=joint_goal,
                    solver=report,
                )
        if self.posture_pending is not None and changed:
            # 等待新IK时沿用上一有效目标的原始时间戳，不给旧图续期。
            return dict(
                packet,
                joint_goal=cache["joint_goal"],
                targets=cache["targets"],
                alignment_phase=cache["phase"],
                pair_id=cache["pair_id"],
                image_receive_mono_ns=cache["image_receive_mono_ns"],
            )
        cache["pair_id"] = packet["pair_id"]
        cache["image_receive_mono_ns"] = packet["image_receive_mono_ns"]
        return dict(packet, joint_goal=cache["joint_goal"])

    def record(self, kind, **detail):
        if self.trace is not None:
            self.trace.record(kind, **detail)

    def feedback_reader_policy(self):
        return dict(
            self.joints.policy,
            feedback_diagnostics=None,
            feedback_path="/api/stream_state",
            feedback_max_age_s=0.15,
        )

    def measured(self):
        if self.history_worker is None:
            return super().measured()
        if self.cancelled.is_set():
            raise RuntimeError("Grasp cancelled")
        state, lift, _ = self.cached_feedback()
        if state["arm_startup"]["control_generation"] != self.generation:
            raise RuntimeError("Control generation changed")
        if lift.get("stationary") is not True or abs(lift["current_um"] - self.lift_um) > 200:
            raise RuntimeError("Fixed lift height changed")
        if state["arms"].get("enabled") is not True:
            raise RuntimeError("Arm hold lost")
        if self.feedback_guard is not None:
            self.feedback_guard(state)
        return state, lift

    def _adopt_request(self, response, previous_id, *, continuous=False):
        super()._adopt_request(response, previous_id, continuous=continuous)
        if self.history_worker is not None:
            deadline = self.clock() + 0.5
            while self.clock() < deadline:
                state, _, _ = self.cached_feedback()
                startup = state["arm_startup"]
                if (
                    startup["control_generation"] == self.generation
                    and startup["joint_control"]["request_id"] == self.request_id
                ):
                    return
                self.sleep(0.005)
            raise TimeoutError("Owned request did not reach the streaming feedback cache")

    def follow(self, observations, goal, *, name, complete=None, timeout_s=90, stable_s=0.5):
        if self.collision_policy != "onsite_supervised_skip":
            raise ValueError(
                "Laptop servo requires an accepted online checker or explicit onsite policy"
            )
        target = observations.latest()
        if target is None:
            raise ValueError("Fresh visual observation required before stream start")
        state, lift = self.measured()
        packet = self.resolve_packet(goal(target, state, lift), state, lift)
        self.record("phase_start", phase=name, target=target, packet=packet, state=state, lift=lift)
        previous_id = state["arm_startup"]["joint_control"]["request_id"]
        # 对准接受一次目标后冻结；下插等阶段维持原有视觉反馈逻辑。
        fixed_target = name == "align"
        if fixed_target:
            fresh_state, _, received_at = self.cached_feedback()
            robot_now = int(fresh_state["server_mono_ns"]) + int((self.clock() - received_at) * 1e9)
            if not 0 <= robot_now - packet["image_receive_mono_ns"] <= 2_000_000_000:
                raise RuntimeError("Visual image source expired before target acceptance")
            packet = copy.deepcopy(packet)
            packet["fixed_target"] = True
            target = copy.deepcopy(target)
            self.record("fixed_target_accepted", target=target, packet=packet)
        start_options = dict(
            speed_rad_s=self.speed_rad_s, acceleration_rad_s2=self.acceleration_rad_s2
        )
        if fixed_target:
            start_options["fixed_target"] = True
        response = self.servo.start(**start_options)
        self._adopt_request(response, previous_id)
        self.record(
            "phase_adopted",
            phase=name,
            request_id=self.request_id,
            generation=self.generation,
            response=response,
        )
        state, _ = self.measured()
        generator = JointReferenceGenerator(
            self.planner.model,
            state["arm_startup"]["nominal_target"],
            speed_rad_s=self.speed_rad_s,
            acceleration_rad_s2=self.acceleration_rad_s2,
            speed_limits=state["arm_startup"]["joint_control"]["joint_stream_speed_limits"],
        )
        self.record(
            "joint_reference_margin",
            margin_rad=generator.reference_margin_rad,
            arm_lower=generator.reference_lower[[*range(7), *range(8, 15)]],
            arm_upper=generator.reference_upper[[*range(7), *range(8, 15)]],
        )
        loop = self.reference_loop = LaptopReferenceLoop(self, generator, clock=self.clock)
        loop.update(packet)
        loop.start()
        deadline = self.clock() + timeout_s
        stable_since, stable_pairs = None, set()
        previous_pair = None
        try:
            while self.clock() < deadline:
                loop.check()
                state, lift = self.measured()
                if not fixed_target:
                    target = observations.latest()
                    if target is None:
                        raise RuntimeError("Visual observations stale")
                    packet = self.resolve_packet(goal(target, state, lift), state, lift)
                    loop.update(packet)
                errors = self.endpoint_errors(packet["targets"], state, lift)
                if packet["pair_id"] != previous_pair:
                    self.record(
                        "visual_goal",
                        phase=name,
                        request_id=self.request_id,
                        target=target,
                        packet=packet,
                        measured_errors=errors,
                    )
                    previous_pair = packet["pair_id"]
                arrived = (
                    complete(target, state, lift, packet)
                    if complete is not None
                    else all(
                        error["position_m"] <= self.position_tolerance_m
                        and error["angle_rad"] <= self.angle_tolerance_rad
                        for error in errors.values()
                    )
                )
                if arrived and max(abs(v) for v in state["arms"]["velocity"]) < 0.08:
                    if stable_since is None:
                        stable_since = self.clock()
                    stable_pairs.add(str(packet.get("confirmation_id", packet["pair_id"])))
                    if (
                        fixed_target or len(stable_pairs) >= 3
                    ) and self.clock() - stable_since >= stable_s:
                        loop.close()
                        loop.check()
                        stopped = self._stop_and_settle()
                        event = dict(
                            phase=name,
                            completed=True,
                            reference_timing=dict(loop.metrics),
                            measured_errors=errors,
                            collision_checked=False,
                            collision_policy=self.collision_policy,
                            execution_mode=self.execution_mode,
                        )
                        self.visual_events.append(event)
                        self.record("phase_completed", event=event, state=stopped)
                        return stopped, event
                else:
                    stable_since, stable_pairs = None, set()
                self.sleep(0.01)
            raise TimeoutError("Laptop visual servo phase deadline exceeded")
        except BaseException as error:
            self.visual_events.append(
                dict(
                    phase=name,
                    completed=False,
                    error=str(error),
                    reference_timing=dict(loop.metrics),
                )
            )
            try:
                self.record(
                    "phase_failed",
                    phase=name,
                    error=str(error),
                    traceback=traceback.format_exc(),
                    request_id=self.request_id,
                )
            except Exception:
                pass
            raise
        finally:
            loop.close()
            self.servo.close()

    def hold(self):
        if self.reference_loop is not None:
            self.reference_loop.close()
        self.servo.close()
        self.cancelled.set()
        pending = self.joints.pending_id
        if pending is not None:
            return self.servo.stop(pending)
        return dict(commands_sent=False, reason="No owned request; retain current hold")

    def finish(self):
        if self.reference_loop is not None:
            self.reference_loop.close()
        self.servo.close()
        super().finish()
        if self.posture_process is not None:
            self.posture_process.shutdown(wait=False, cancel_futures=True)
            self.posture_process = None
        if self.owns_trace and self.trace is not None:
            self.trace.close()
