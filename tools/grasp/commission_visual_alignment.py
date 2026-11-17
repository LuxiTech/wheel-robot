#!/usr/bin/env python3
"""Live vision grasp using selected resources, with explicit policy and full tracing.

The user-confirmed reuse policy treats historical acceptance annotations as
informational. Images, measured feedback, limits and request ownership remain checked.
"""

import argparse
import datetime
import fcntl
import hashlib
import json
import os
import sys
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from _bootstrap import configure
from _robot_model import selected_model

configure(ROOT)


def wait_hold_acknowledgement(
    joints, request_id, previous_id, *, clock=time.monotonic, sleep=time.sleep
):
    """An accepted POST can precede its ROS status; never accept a third owner."""
    deadline = clock() + 0.5
    while clock() < deadline:
        state = joints.snapshot()
        reported = state["arm_startup"]["joint_control"]["request_id"]
        if reported == request_id:
            return
        if reported != previous_id:
            raise RuntimeError("Initial hold superseded before acknowledgement")
        sleep(0.02)
    raise TimeoutError("Accepted initial hold was not acknowledged in SDK feedback")


def main(*, default_mode="alignment"):
    startup_started = time.monotonic()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--cad-preview", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--mode", choices=("alignment", "grasp"), default=default_mode)
    parser.add_argument("--contact-policy", type=Path)
    parser.add_argument("--acceptance", type=Path, help="Existing wrist/depth acceptance record")
    parser.add_argument("--vision-url", default="http://192.168.127.10:18035")
    parser.add_argument(
        "--execution-config", type=Path, default=ROOT / "config/robots/grasp_execution.json"
    )
    args = parser.parse_args()
    # 视觉、反馈和日志线程共存时缩短解释器时间片，减少10ms发送回路被排队的时间。
    sys.setswitchinterval(0.001)
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    for key, value in json.loads(
        (ROOT / ".runtime/vision/native-build-env.json").read_text()
    ).items():
        os.environ.setdefault(key, value)
    os.environ["PATH"] = str(Path(sys.executable).parent) + ":" + os.environ.get("PATH", "")
    os.environ["HF_HOME"] = str(ROOT / "assets/models/huggingface")
    os.environ["HF_HUB_OFFLINE"] = "1"
    os.environ["TRANSFORMERS_OFFLINE"] = "1"
    for key, name in (
        ("TORCH_EXTENSIONS_DIR", "torch-extensions"),
        ("XDG_CACHE_HOME", "cache"),
        ("MPLCONFIGDIR", "matplotlib"),
        ("WARP_CACHE_PATH", "warp-cache"),
    ):
        os.environ[key] = str(ROOT / ".runtime/vision" / name)
    import cv2
    import numpy as np
    import torch
    from scipy.spatial.transform import Rotation
    from wheel_execution.execution_trace import ExecutionTrace
    from wheel_execution.laptop_servo import LaptopVisualServoDriver
    from wheel_execution.moveit_driver import MoveItClient, MoveItTrajectoryDriver
    from wheel_execution.reuse_existing import REUSE_EXISTING
    from wheel_execution.stationary_pick import create_stationary_pick
    from wheel_execution.visual_servo import LatestObservation
    from wheel_execution.visual_servo_grasp import VisualServoGraspExecutor
    from wheel_motion_core.visual_grasp import VisualGraspPlanner
    from wheel_perception_core.backends import FoundationPoseBackend, NanoDetectorSegmenter
    from wheel_perception_core.extrinsics import CalibratedHeadTransform, HeadExtrinsics
    from wheel_perception_core.geometry import connected_depth_mask, prepare_input
    from wheel_perception_core.targets import visual_target
    from wheel_sdk_bridge.calibrated_capture import CaptureNotReady, CapturePairRejected
    from wheel_sdk_bridge.http_client import JSONClient
    from wheel_sdk_bridge.joint_client import JointClient
    from wheel_sdk_bridge.joint_stream_client import JointStreamClient
    from wheel_sdk_bridge.lift_client import LiftClient
    from wheel_sdk_bridge.streaming_capture import StreamingHeadCapture

    calibration = HeadExtrinsics(args.calibration)
    execution_config = json.loads(args.execution_config.read_text())
    if execution_config["execution_mode"] == "moveit_trajectory":
        execution_config["moveit_crate_mesh"] = str(args.cad_preview.resolve() / "opening_up.obj")
    resource_policy = execution_config.get("resource_policy", "strict")
    reuse_existing = resource_policy == REUSE_EXISTING
    max_p95_depth_error_m = execution_config.get("visual_max_p95_depth_error_m", 0.020)
    resources = selected_model(ROOT)
    policy = json.loads((ROOT / "config/robots/joint_control.wired.json").read_text())
    policy["feedback_diagnostics"] = str(out / "joint_feedback.jsonl")
    joints = JointClient(policy["base_url"], policy)
    http = JSONClient(policy["base_url"], timeout=0.4)
    preview = args.cad_preview.resolve()
    cad = json.loads((preview / "cad_and_rules.json").read_text())
    acceptance = json.loads(args.acceptance.read_text()) if args.acceptance else {}
    reference_detection = json.loads((preview / "detection.json").read_text())
    reference_boxes = reference_detection["candidates"]
    if len(reference_boxes) != 1:
        raise ValueError("Reference capture must explicitly select one box")
    tracked_roi = np.asarray(reference_boxes[0]["box"], dtype=float)
    engines = ROOT / "assets/models/engines"
    models_started = time.monotonic()
    detector = NanoDetectorSegmenter(
        engines / "owl_image_encoder.engine",
        engines / "resnet18_image_encoder.engine",
        engines / "mobile_sam_mask_decoder.engine",
        prompt="a blue plastic box",
    )
    backend = FoundationPoseBackend(
        ROOT / "third_party/FoundationPose",
        preview / "opening_up.obj",
        debug_dir=out / "foundationpose",
    )
    models_ready = time.monotonic()
    from Utils import nvdiffrast_render

    capture = StreamingHeadCapture(
        args.vision_url,
        calibration,
        max_age_ms=2000,
        strict_lift_max_age_ms=250,
        lift_max_sample_gap_ms=210,
    )
    trace = ExecutionTrace(out / "execution_trace")
    trace.record(
        "runtime_initialization",
        imports_configuration_s=models_started - startup_started,
        model_load_s=models_ready - models_started,
    )
    capture.client.on_response = trace.rgbd_response
    evidence_files = [
        Path(__file__).resolve(),
        ROOT / "tools/grasp/execution_log.py",
        ROOT / "tools/grasp/run_current_visual_grasp.sh",
        args.calibration,
        preview / "cad_and_rules.json",
        ROOT / "config/robots/grasp_execution.json",
        resources["urdf"],
        resources["sdk_chains"],
        resources["contacts"],
        resources["gripper"],
        ROOT / "ros2_ws/src/wheel_execution/wheel_execution/laptop_servo.py",
        ROOT / "ros2_ws/src/wheel_execution/wheel_execution/execution_trace.py",
        ROOT / "ros2_ws/src/wheel_execution/wheel_execution/grasp_executor.py",
        ROOT / "ros2_ws/src/wheel_execution/wheel_execution/reuse_existing.py",
        ROOT / "ros2_ws/src/wheel_execution/wheel_execution/visual_servo_grasp.py",
        ROOT / "packages/wheel_motion_core/wheel_motion_core/joint_servo.py",
    ]
    trace.record(
        "run_configuration",
        execute=args.execute,
        vision_url=args.vision_url,
        image_max_age_s=2.0,
        resources=[
            dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
            for path in evidence_files
        ],
        calibration=json.loads(args.calibration.read_text()),
        execution=json.loads((ROOT / "config/robots/grasp_execution.json").read_text()),
    )
    capture.start()
    transform = CalibratedHeadTransform(calibration)
    identity = ("head_rgb", "central_crate")
    registered = False
    previous_pose = None
    events = []
    summary = dict(
        mode="onsite_alignment_trial" if args.mode == "alignment" else "visual_grasp",
        resource_policy=resource_policy,
        physical_geometry_accepted=False,
        complete_grasp=False,
        commands_sent=False,
        completed=False,
    )

    def save(name, value):
        (out / name).write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")

    def observe():
        nonlocal registered, previous_pose, tracked_roi
        observe_started = time.monotonic()
        while True:
            try:
                if capture.stop_poll.is_set():
                    raise RuntimeError("Capture stopped")
                if capture.poll_error is not None:
                    raise RuntimeError("Lift polling failed: " + str(capture.poll_error))
                with capture.history_lock:
                    warm = capture.serial is not None and len(capture.history.samples) >= 3
                if not warm:
                    time.sleep(0.01)
                    continue
                # Infer while the lift poller collects the after-image bracket.
                # Transformation still waits for the original, strict context.
                capture.client.timeout = 0.3
                receive_started = time.monotonic()
                observation = capture.client.receive("head_rgb")
                received = time.monotonic()
                break
            except CaptureNotReady as error:
                trace.record("capture_rejected", error=str(error))
                with (out / "discarded_frames.jsonl").open("a") as stream:
                    stream.write(json.dumps(dict(error=str(error), at=time.monotonic())) + "\n")
            except ValueError as error:
                trace.record("capture_rejected", error=str(error))
                if not (
                    str(error).startswith("RGBD observation is stale:")
                    or str(error) == "Duplicate RGBD pair"
                ):
                    raise
                with (out / "discarded_frames.jsonl").open("a") as stream:
                    stream.write(json.dumps(dict(error=str(error), at=time.monotonic())) + "\n")
                time.sleep(0.005)
        # 分阶段记录墙钟耗时；相机原时间戳保持不变，不把耗时当成图像年龄。
        timing_s = dict(
            capture_wait=received - observe_started,
            receive_decode=received - receive_started,
        )
        image = prepare_input(observation)
        prepared = time.monotonic()
        timing_s["prepare_input"] = prepared - received
        # Keep the explicitly selected crate identity. Never switch to another
        # blue detection (such as the robot cover) during the motion.
        mask = detector.segment(image.rgb, tracked_roi)
        mask, _ = connected_depth_mask(mask, image.depth, image.k)
        yy, xx = np.where(mask)
        if not len(xx):
            raise ValueError("Selected live crate mask lost")
        tracked_roi = np.array(
            [
                max(0, int(xx.min()) - 16),
                max(0, int(yy.min()) - 16),
                min(image.rgb.shape[1], int(xx.max()) + 17),
                min(image.rgb.shape[0], int(yy.max()) + 17),
            ],
            dtype=float,
        )
        segmented = time.monotonic()
        timing_s["segment_depth_roi"] = segmented - prepared
        pose_mode = "track" if registered else "register"
        camera_pose = (
            backend.track(image, identity)
            if registered
            else backend.register(image, mask, identity)
        )
        posed = time.monotonic()
        timing_s["pose"] = posed - segmented
        registered = True
        if previous_pose is not None:
            translation_jump = np.linalg.norm(camera_pose[:3, 3] - previous_pose[:3, 3])
            angle_jump = Rotation.from_matrix(
                camera_pose[:3, :3] @ previous_pose[:3, :3].T
            ).magnitude()
            if translation_jump > 0.05 or angle_jump > 0.25:
                raise ValueError("Tracked visual pose jumped; trial stopped")
        previous_pose = camera_pose.copy()
        _, rendered, _ = nvdiffrast_render(
            K=image.k,
            H=image.rgb.shape[0],
            W=image.rgb.shape[1],
            ob_in_cams=torch.as_tensor(camera_pose[None], device="cuda", dtype=torch.float32),
            mesh=backend.mesh,
            glctx=backend.est.glctx,
            use_light=False,
        )
        depth = rendered[0].detach().cpu().numpy()
        silhouette = depth > 0
        overlap = silhouette & mask & (image.depth > 0)
        residual = np.abs(depth[overlap] - image.depth[overlap])
        if not len(residual):
            raise ValueError("No live CAD/depth overlap")
        quality = dict(
            silhouette_iou=float((silhouette & mask).sum() / max(1, (silhouette | mask).sum())),
            median_depth_error_m=float(np.median(residual)),
            p95_depth_error_m=float(np.quantile(residual, 0.95)),
            metric_depth_semantics_validated=(
                args.mode == "grasp" and acceptance.get("metric_depth_validated") is True
            ),
        )
        quality_finished = time.monotonic()
        # 这些阶段已返回CPU数组，包含GPU结果回传等待；不是仅统计CUDA提交时间。
        timing_s["render_quality"] = quality_finished - posed
        timing_s["processing"] = quality_finished - received
        age = (
            time.monotonic()
            - observation.received_at
            + observation.metadata["transport_age_ms"] / 1000
        )
        row = dict(
            pair_id=observation.key[1],
            age_s=age,
            quality=quality,
            image_receive_mono_ns=int(observation.metadata["receive_mono_ns"]),
            pose_mode=pose_mode,
            timing_s=timing_s,
        )
        with (out / "vision.jsonl").open("a") as stream:
            stream.write(json.dumps(row) + "\n")
        trace.record("vision_quality", **row, camera_pose=camera_pose)
        if age > 2.0:
            trace.record("image_expired_after_inference", **row)
            return None  # Warm registration never becomes a live command.
        if (
            quality["silhouette_iou"] < 0.5
            or quality["median_depth_error_m"] > 0.006
            or quality["p95_depth_error_m"] > max_p95_depth_error_m
        ):
            # 单帧深度/轮廓质量不合格就丢帧并继续新图；不把一次测量噪声锁存为整轮失败。
            # 上一有效图像的原时间戳不变，持续无有效帧仍由2s时效保护停止。
            trace.record("vision_quality_rejected", pair_id=observation.key[1], quality=quality)
            return None
        pairing_error = None
        while True:
            if capture.poll_error is not None:
                raise RuntimeError("Lift polling failed: " + str(capture.poll_error))
            try:
                with capture.history_lock:
                    context = capture.history.context(
                        observation.metadata, capture.serial, fixed_lift_stream=True
                    )
                break
            except CapturePairRejected as error:
                # 过去缺失的样本无法靠等待补齐；立即丢弃该帧，继续处理下一张新图。
                trace.record("image_pair_rejected", pair_id=observation.key[1], error=str(error))
                return None
            except CaptureNotReady as error:
                if str(error) != pairing_error:
                    pairing_error = str(error)
                    trace.record(
                        "lift_pairing_wait", pair_id=observation.key[1], error=pairing_error
                    )
                age = (
                    time.monotonic()
                    - observation.received_at
                    + observation.metadata["transport_age_ms"] / 1000
                )
                if age > 2.0:
                    with capture.history_lock:
                        samples = list(capture.history.samples)[-8:]
                    trace.record(
                        "image_expired_while_pairing",
                        pair_id=observation.key[1],
                        age_s=age,
                        error=pairing_error,
                        lift_samples=samples,
                    )
                    return None
                time.sleep(0.005)
        origin_camera, reasons = transform.resolve(observation, context)
        target = visual_target(
            target_id=identity[1],
            pair_id=observation.key[1],
            object_pose_origin=origin_camera @ camera_pose,
            rules=cad["rules"],
            height_um=context["height_um"],
            quality=quality,
            source_blockers=reasons,
            observed_at_monotonic=observation.received_at
            - observation.metadata["transport_age_ms"] / 1000,
            max_p95_depth_error_m=max_p95_depth_error_m,
        )
        target["image_receive_mono_ns"] = row["image_receive_mono_ns"]
        trace.record("vision_target", target=target, context=context)
        trace.grasp_points(image.rgb, image.k, origin_camera, target, observation.metadata, context)
        save("latest_target.json", target)
        cv2.imwrite(str(out / "latest_rgb.jpg"), cv2.cvtColor(image.rgb, cv2.COLOR_RGB2BGR))
        return target

    source = LatestObservation(observe)
    driver = None
    lock = open("/tmp/wheel-robot-live-motion.lock", "a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if args.mode == "grasp":
            # 用户确认沿用已有方案时只记录这些历史状态，不再要求重复验收。
            blockers = []
            contact_reference = json.loads(resources["contacts"].read_text())
            model_source = json.loads(resources["source"].read_text())
            if contact_reference.get("hardware_validated") is not True:
                blockers.append("hook_reference_not_hardware_accepted")
            if model_source.get("physical_gripper_alignment_accepted") is not True:
                blockers.append("physical_gripper_alignment_not_accepted")
            if calibration.config.get("deployable") is not True:
                blockers.append("selected_extrinsics_not_hardware_accepted")
            if cad.get("grasp_rules_validated") is not True:
                blockers.append("rim_rules_not_hardware_accepted")
            for key in ("wrist_geometry_validated", "metric_depth_validated"):
                if acceptance.get(key) is not True:
                    blockers.append(key + "_not_accepted")
            if args.contact_policy is None:
                blockers.append("accepted_bilateral_contact_policy_not_configured")
            else:
                policy_value = json.loads(args.contact_policy.read_text())
                if policy_value.get("hardware_validated") is not True:
                    blockers.append("bilateral_contact_policy_not_hardware_accepted")
            resource_warnings = list(blockers) if reuse_existing else []
            blockers = [] if reuse_existing else blockers
            readiness = dict(
                ready=not blockers,
                blockers=blockers,
                resource_warnings=resource_warnings,
                resource_policy=resource_policy,
            )
            save("grasp_readiness.json", readiness)
            trace.record("grasp_readiness", **readiness)
            if blockers:
                raise ValueError("Complete grasp blocked before enabling: " + ", ".join(blockers))
        source.start()
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline:
            target = source.latest()
            if target is not None:
                break
            time.sleep(0.02)
        else:
            raise TimeoutError("No fresh post-inference target; no motion sent")
        coverage = []
        previous = target
        while len(coverage) < 8 and time.monotonic() < deadline:
            current = source.latest()
            if current is not None and current["pair_id"] != previous["pair_id"]:
                age_at_update = time.monotonic() - previous["observed_at_monotonic"]
                if age_at_update >= 2.0:
                    trace.record(
                        "preflight_wait_for_continuity",
                        age_s=age_at_update,
                        previous_pair=previous["pair_id"],
                        pair_id=current["pair_id"],
                    )
                    coverage.clear()
                else:
                    coverage.append(age_at_update)
                previous = current
            time.sleep(0.01)
        summary["preflight_previous_image_age_at_next_update_s"] = coverage
        trace.record("vision_preflight", previous_image_ages_s=coverage)
        if len(coverage) < 8 or max(coverage) >= 2.0:
            raise RuntimeError("Live observation cadence cannot sustain the 2000ms image watchdog")
        executor = None
        if args.mode == "grasp":
            lift_policy = json.loads((ROOT / "config/robots/lift_control.wired.json").read_text())

            def live_target(target_id):
                value = source.latest()
                if value is None:
                    return None
                if value["target_id"] != target_id:
                    raise ValueError("No fresh selected crate observation")
                return value

            trace.record("operation_started", name="构建视觉抓取执行器")
            executor = create_stationary_pick(
                target_source=live_target,
                joints=joints,
                lift_client=LiftClient(lift_policy["base_url"], timeout=0.4),
                urdf=resources["urdf"],
                sdk_chains=resources["sdk_chains"],
                contacts=contact_reference,
                collision_check=None,
                contact_policy=args.contact_policy,
                wrist_geometry_validated=acceptance.get("wrist_geometry_validated", False),
                metric_depth_validated=acceptance.get("metric_depth_validated", False),
                clearance_m=acceptance.get(
                    "clearance_m", execution_config.get("rail_stop_clearance_m", 0.01)
                ),
                uncertainty_m=acceptance.get(
                    "uncertainty_m", execution_config.get("rail_uncertainty_m", 0.005)
                ),
                output=out / "grasp",
                gripper_config=json.loads(resources["gripper"].read_text())["sides"],
                execution_config=execution_config,
                max_insertion_m=execution_config.get("max_insertion_m", 0.04),
                trace=trace,
            )
            trace.record("operation_completed", name="构建视觉抓取执行器")
            driver = executor.driver
        if not args.execute:
            summary.update(completed=True, scope="live vision and executor preflight only")
            return
        state = joints.snapshot()
        if execution_config["execution_mode"] == "moveit_trajectory":
            MoveItClient(execution_config["moveit_gateway_url"]).require_supported()
        else:
            JointStreamClient(joints).require_supported(state)
        save("before_enable.json", state)
        initial_enabled = state["arms"]["enabled"]
        initial_generation = state["arm_startup"]["control_generation"]
        if initial_enabled and state["arm_startup"]["joint_control"]["phase"] not in (
            "idle",
            "ready",
            "stopped",
            "needs_correction",
        ):
            raise ValueError("Another active request or SDK fault must not be superseded")
        trace.record("operation_started", name="双臂停稳及原位保持确认")
        stationary_since = time.monotonic()
        stationary_deadline = stationary_since + 10
        anchor = state["arms"]["position"]
        speed_samples = []
        while time.monotonic() - stationary_since < 2:
            state = joints.snapshot()
            if (
                state["arms"]["enabled"] != initial_enabled
                or state["arm_startup"]["control_generation"] != initial_generation
            ):
                raise ValueError("Control ownership changed before current-pose hold")
            raw_speed = max(abs(v) for v in state["arms"]["velocity"])
            speed_samples = (speed_samples + [raw_speed])[-5:]
            displacement = max(abs(a - b) for a, b in zip(anchor, state["arms"]["position"]))
            trace.record(
                "before_hold_observation",
                state=state,
                raw_speed_rad_s=raw_speed,
                position_span_rad=displacement,
            )
            # CAN速度反馈有量化尖峰；按连续样本中值并同时检查实测位置变化，不拿单点尖峰
            # 当成机械臂真实移动。真正持续移动则等待新停稳窗口，超时退出。
            if displacement > 0.006 or (
                len(speed_samples) == 5 and np.median(speed_samples) >= 0.08
            ):
                stationary_since = time.monotonic()
                anchor = state["arms"]["position"]
            if time.monotonic() >= stationary_deadline:
                raise ValueError("Arms did not settle before current-pose hold")
            time.sleep(0.05)
        summary["commands_sent"] = True
        if initial_enabled:
            trace.record("existing_hold_reused", state=state)
        else:
            trace.record("initial_hold_request", state=state, speed_rad_s=0.03)
            hold_response = joints.move(state["arms"]["position"], 0.03, enable_if_needed=True)
            held_request = hold_response["request_id"]
            trace.record("initial_hold_receipt", response=hold_response)
            wait_hold_acknowledgement(
                joints, held_request, state["arm_startup"]["joint_control"]["request_id"]
            )
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                result = joints.result(held_request)
                trace.record("initial_hold_feedback", result=result)
                if result["completed"]:
                    break
                if result["phase"] in ("fault", "stopped", "unknown_or_superseded"):
                    raise RuntimeError("Initial hold failed: " + str(result))
                time.sleep(0.05)
            else:
                raise TimeoutError("Initial hold did not settle")
        trace.record("operation_completed", name="双臂停稳及原位保持确认")
        if executor is not None:
            driver = executor.driver
            result = executor.run(identity[1])
            summary.update(
                completed=result["completed"],
                complete_grasp=result["completed"],
                physical_geometry_accepted=not reuse_existing,
                grasp_result=result,
            )
            return
        state = joints.snapshot()
        limits = state["arm_startup"]["joint_control"]
        planner = VisualGraspPlanner(
            resources["urdf"],
            resources["sdk_chains"],
            json.loads(resources["contacts"].read_text()),
            limits["lower"],
            limits["upper"],
            gripper_config=json.loads(resources["gripper"].read_text())["sides"],
            position_tolerance_m=0.005,
        )

        def read_lift():
            live = http.request("GET", "/api/state")
            if live["control"]["control_source"] != "console":
                raise RuntimeError("Console authority lost")
            return dict(
                current_um=live["lift"]["current_um"], stationary=live["lift_status"]["stationary"]
            )

        if execution_config["execution_mode"] == "moveit_trajectory":
            driver = MoveItTrajectoryDriver(
                joints,
                read_lift,
                planner,
                collision_policy="required",
                gateway_url=execution_config["moveit_gateway_url"],
                crate_mesh=execution_config["moveit_crate_mesh"],
            )
            driver.trace = trace
        else:
            driver = LaptopVisualServoDriver(
                joints, read_lift, planner, collision_policy="onsite_supervised_skip", trace=trace
            )
        driver.begin()

        class AlignmentTrial(VisualServoGraspExecutor):
            def _validate(self, value, target_id):
                if (
                    value["target_id"] != target_id
                    or value["frame_id"] != "arm_origin"
                    or value["tool_frame"] != "cad_contact_midpoint_pinch_y"
                ):
                    raise ValueError("Trial target identity or convention changed")
                if abs(value["height_um"] - driver.lift_um) > 200:
                    raise ValueError("Lift changed during trial")
                allowed = {
                    "sdk_to_urdf_and_physical_tcp_not_registered",
                    "metric_depth_semantics_unvalidated",
                    "stationary_lift_host_receive_alignment",
                }
                for reason in value["execution_blockers"]:
                    if reason not in allowed and not reason.startswith("selected_calibration:"):
                        raise ValueError("Unaccepted trial source: " + reason)
                if type(value.get("image_receive_mono_ns")) is not int:
                    raise ValueError("Actual image timestamp required")

            def packet(self, value, goals):
                self._validate(value, identity[1])
                return dict(
                    targets=goals,
                    pair_id=value["pair_id"],
                    image_receive_mono_ns=value["image_receive_mono_ns"],
                )

            def run(self, target_id):
                raise RuntimeError("Commissioning trial cannot perform a complete grasp")

        runner = AlignmentTrial(
            planner,
            driver,
            None,
            None,
            None,
            None,
            open_verify=None,
            resource_policy=(
                resource_policy
                if execution_config["execution_mode"] == "moveit_trajectory"
                else "strict"
            ),
        )
        runner._alignment(source, identity[1])
        events = driver.visual_events
        summary.update(
            completed=True, scope="alignment only; no opening, insertion, closing or test lift"
        )
    except BaseException as error:
        summary["error"] = str(error)
        summary["traceback"] = traceback.format_exc()
        try:
            trace.record("execution_failed", error=str(error), traceback=summary["traceback"])
        except Exception as log_error:
            summary["logging_error"] = str(log_error)
        if driver is not None:
            try:
                stop_response = driver.hold()
                summary["stop_receipt"] = stop_response
            except Exception as stop_error:
                summary["stop_error"] = str(stop_error)
        elif joints.pending_id:
            try:
                summary["stop_receipt"] = JointStreamClient(joints).stop(joints.pending_id)
            except Exception as stop_error:
                summary["stop_error"] = str(stop_error)
        raise
    finally:
        if driver is not None:
            events = driver.visual_events
            try:
                driver.finish()
            except Exception as finish_error:
                summary["finish_error"] = str(finish_error)
        source.close(timeout=2)
        capture.close()
        summary.update(
            events=events, ended_utc=datetime.datetime.now(datetime.timezone.utc).isoformat()
        )
        with capture.history_lock:
            summary["lift_capture_samples"] = list(capture.history.samples)[-8:]
        summary["lift_capture_error"] = str(capture.poll_error) if capture.poll_error else None
        try:
            summary["final_state"] = joints.snapshot()
            trace.record("execution_finished", summary=summary)
        except Exception as final_error:
            summary["final_record_error"] = str(final_error)
        try:
            trace.close()
        except Exception as log_error:
            summary["logging_error"] = str(log_error)
            summary["completed"] = False
        save("summary.json", summary)
        try:
            from execution_log import write_execution_log

            write_execution_log(out)
        except Exception as log_error:
            print("Execution log generation failed: " + str(log_error), file=sys.stderr)
        print(json.dumps(summary, ensure_ascii=False))
        lock.close()


if __name__ == "__main__":
    main()
