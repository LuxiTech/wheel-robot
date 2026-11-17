"""Composition root for stationary vision -> traditional motion -> verified pick.

The live PipelineVision and collision checker are explicit dependencies.
Missing physical commissioning evidence rejects construction, not a motor POST.
The separate mobile transfer executor remains unready until transport/place work
is commissioned; this factory does not advertise those unrelated capabilities.
"""

import json
import uuid
from pathlib import Path

from wheel_motion_core.visual_grasp import VisualGraspPlanner

from wheel_execution.contact_closer import SDKContinuousContactCloser
from wheel_execution.execution_trace import ExecutionTrace
from wheel_execution.grasp_executor import (
    SDKContactCloser,
    SDKGripperOpener,
    SDKWaypointDriver,
    StationaryGraspExecutor,
)
from wheel_execution.grasp_feedback import (
    HeadModelClearanceSource,
    MovingWristClearanceSource,
    VisualTestLiftVerifier,
    WristClearanceSource,
)
from wheel_execution.laptop_servo import LaptopVisualServoDriver
from wheel_execution.moveit_driver import MoveItTrajectoryDriver
from wheel_execution.phase_driver import SDKTrajectoryDriver
from wheel_execution.reuse_existing import (
    REUSE_EXISTING,
    ObservedBoxFollowVerifier,
    SDKConfiguredGrippers,
)
from wheel_execution.two_stage_grasp import TwoStageGraspExecutor
from wheel_execution.visual_servo import SDKVisualServoDriver
from wheel_execution.visual_servo_grasp import VisualServoGraspExecutor


def create_stationary_pick(
    *,
    target_source,
    joints,
    lift_client,
    urdf,
    sdk_chains,
    contacts,
    collision_check,
    contact_policy,
    wrist_geometry_validated,
    metric_depth_validated,
    clearance_m,
    uncertainty_m,
    output=None,
    gripper_config=None,
    max_insertion_m=0.04,
    execution_config=None,
    trace=None,
):
    if isinstance(execution_config, (str, Path)):
        execution_config = json.loads(Path(execution_config).read_text())
    if execution_config is None:
        root = Path(__file__).resolve().parents[4]
        configuration = root / "config/robots/grasp_execution.json"
        if configuration.is_file():
            execution_config = json.loads(configuration.read_text())
        else:
            # Installed packages do not inherit this user's onsite collision
            # waiver. Deployments can explicitly supply their own configuration.
            execution_config = dict(
                schema="wheel_grasp_execution_v1",
                execution_mode="laptop_visual_servo",
                collision_policy="required",
                speed_rad_s=0.2,
                max_endpoint_corrections=2,
                endpoint_position_tolerance_m=0.005,
                endpoint_angle_tolerance_rad=0.02,
            )
    if execution_config.get("schema") != "wheel_grasp_execution_v1":
        raise ValueError("Unknown grasp execution configuration schema")
    if execution_config.get("image_max_age_s", 2.0) != 2.0:
        raise ValueError("This stream deployment requires the configured 2s image window")
    resource_policy = execution_config.get("resource_policy", "strict")
    if resource_policy not in ("strict", REUSE_EXISTING):
        raise ValueError("Unknown resource reuse policy")
    reuse_existing = resource_policy == REUSE_EXISTING
    mode = execution_config["execution_mode"]
    collision_policy = execution_config["collision_policy"]
    if mode not in (
        "moveit_trajectory",
        "laptop_visual_servo",
        "pure_visual_servo",
        "two_stage",
        "legacy_waypoints",
    ) or collision_policy not in (
        "required",
        "onsite_supervised_skip",
    ):
        raise ValueError("Unknown grasp execution or collision policy")
    if collision_check is None and collision_policy == "required" and mode != "moveit_trajectory":
        raise ValueError("Live scene collision checker required")
    if collision_policy == "onsite_supervised_skip":
        collision_check = None
    if not reuse_existing and (
        wrist_geometry_validated is not True or metric_depth_validated is not True
    ):
        raise ValueError("Current wrist geometry/depth checks have not been accepted")
    if not reuse_existing and contacts.get("hardware_validated") is not True:
        raise ValueError("Measured hook contact reference required for live execution")
    if gripper_config is None:
        raise ValueError("Current SDK gripper opening configuration required")
    if reuse_existing and mode not in ("laptop_visual_servo", "moveit_trajectory"):
        raise ValueError("Existing-resource demonstration uses the laptop joint stream")
    state = joints.snapshot()
    limits = state["arm_startup"]["joint_control"]
    planner = VisualGraspPlanner(
        urdf,
        sdk_chains,
        contacts,
        limits["lower"],
        limits["upper"],
        collision_check=collision_check,
        gripper_config=gripper_config,
    )

    def read_lift():
        state = lift_client.snapshot()  # GET only. Never open/move/stop the lift.
        source = state.get("control", {}).get("control_source")
        if source != "console":
            raise ValueError("SDK console authority required for stationary pick")
        return dict(
            current_um=state["lift"]["current_um"], stationary=state["lift_status"]["stationary"]
        )

    continuous_mode = mode in (
        "moveit_trajectory",
        "laptop_visual_servo",
        "pure_visual_servo",
        "two_stage",
    )
    if continuous_mode:
        driver_type = {
            "moveit_trajectory": MoveItTrajectoryDriver,
            "laptop_visual_servo": LaptopVisualServoDriver,
            "pure_visual_servo": SDKVisualServoDriver,
            "two_stage": SDKTrajectoryDriver,
        }[mode]
        driver = driver_type(
            joints,
            read_lift,
            planner,
            speed_rad_s=execution_config["speed_rad_s"],
            max_corrections=execution_config["max_endpoint_corrections"],
            collision_policy=collision_policy,
            position_tolerance_m=execution_config["endpoint_position_tolerance_m"],
            angle_tolerance_rad=execution_config["endpoint_angle_tolerance_rad"],
            acceleration_rad_s2=execution_config.get("acceleration_rad_s2", 0.5),
            endpoint_settle_timeout_s=execution_config.get("endpoint_settle_timeout_s", 2.0),
            **(
                dict(
                    gateway_url=execution_config["moveit_gateway_url"],
                    crate_mesh=execution_config["moveit_crate_mesh"],
                )
                if mode == "moveit_trajectory"
                else {}
            ),
        )
    else:
        if collision_policy != "required":
            raise ValueError(
                "Legacy waypoint executor does not implement a supervised collision skip"
            )
        driver = SDKWaypointDriver(joints, read_lift, planner)
    policy = json.loads(contact_policy.read_text()) if contact_policy is not None else None
    if reuse_existing:
        configured_grippers = SDKConfiguredGrippers(driver, gripper_config)
        opener = configured_grippers.open
        closer = configured_grippers.close
        verifier = ObservedBoxFollowVerifier(target_source, driver)
    else:
        if policy is None:
            raise ValueError("Accepted bilateral contact policy required for strict mode")
        opener = SDKGripperOpener(
            driver,
            gripper_config,
            effort_limits={side: policy[side]["hard_effort_max_nm"] for side in ("left", "right")},
            step_rad=None if continuous_mode else 0.02,
        )
        closer = (
            SDKContinuousContactCloser(driver, policy, gripper_config)
            if continuous_mode
            else SDKContactCloser(driver, policy)
        )
        verifier = VisualTestLiftVerifier(target_source, driver, policy)
    if reuse_existing:
        wrist = HeadModelClearanceSource(
            driver, clearance_m=clearance_m, uncertainty_m=uncertainty_m
        )
    else:
        wrist_type = MovingWristClearanceSource if continuous_mode else WristClearanceSource
        wrist = wrist_type(
            driver,
            urdf,
            contacts["urdf_sha256"],
            joints.http.base_url,
            clearance_m=clearance_m,
            uncertainty_m=uncertainty_m,
            geometry_validated=wrist_geometry_validated,
            metric_depth_validated=metric_depth_validated,
            resource_policy=resource_policy,
        )
    if mode in ("laptop_visual_servo", "moveit_trajectory"):
        # 每次抓取使用独立目录；异步写盘，禁止覆盖上次故障记录。
        owns_trace = trace is None
        if trace is None:
            base = Path(output) if output else Path.cwd() / "outputs/grasp/executions"
            base.mkdir(parents=True, exist_ok=True)
            trace = ExecutionTrace(base / ("trace-" + uuid.uuid4().hex))
        driver.trace = trace
        if hasattr(driver, "servo"):
            driver.servo.trace = trace
        driver.owns_trace = owns_trace
        if getattr(wrist, "client", None) is not None:
            wrist.client.on_response = trace.rgbd_response
        vision = getattr(target_source, "vision", None)
        source = getattr(vision, "source", None)
        if getattr(source, "client", None) is not None:
            source.client.on_response = trace.rgbd_response
            source.client.max_age_ms = 2000
            source.max_age_ms = 2000
        trace.record(
            "grasp_configuration",
            execution=execution_config,
            contacts=contacts,
            gripper_config=gripper_config,
            contact_policy=policy,
        )
    executor_type = {
        "moveit_trajectory": VisualServoGraspExecutor,
        "laptop_visual_servo": VisualServoGraspExecutor,
        "pure_visual_servo": VisualServoGraspExecutor,
        "two_stage": TwoStageGraspExecutor,
        "legacy_waypoints": StationaryGraspExecutor,
    }[mode]
    executor_options = (
        {"resource_policy": resource_policy}
        if mode in ("moveit_trajectory", "laptop_visual_servo", "pure_visual_servo")
        else {}
    )
    return executor_type(
        planner,
        driver,
        target_source,
        wrist,
        closer,
        verifier,
        open_verify=opener,
        output=output,
        max_insertion_m=max_insertion_m,
        **executor_options,
    )
