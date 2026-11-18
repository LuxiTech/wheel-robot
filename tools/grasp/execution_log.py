#!/usr/bin/env python3
"""Produce one readable per-run log after exit; never contacts robot hardware."""

import argparse
import datetime
import hashlib
import json
import os
import statistics
import tempfile
import time
from collections import Counter, defaultdict
from pathlib import Path

PHASES = {
    "ALIGN_VISUAL_SERVO": "视觉对准",
    "OPEN": "张开夹爪",
    "INSERT_VISUAL_SERVO": "下插",
    "CLOSE": "闭合夹爪",
    "TEST_LIFT": "试抬及箱体随动确认",
    "COMPLETE": "抓取完成",
    "BLOCKED": "停止执行",
    "HOLD": "保持",
    "align": "视觉对准",
    "insert": "下插",
    "test_lift": "试抬",
}

CAUSES = {
    "Control deadline missed": "主机生成参考时未通过0<周期≤30ms检查，主动停发；随后出现的板端超时可能是后果。",
    "Measured nominal following error": "实测关节与名义参考的偏差超过0.25rad，主机主动停止继续推进目标。",
    "joint_stream_following_error": "板端检测到发送目标与实测关节偏差超过0.25rad，触发跟随保护。",
    "joint_stream_feedback_timeout": "板端最后接受参考携带的反馈时间超过150ms；需结合接受序号判断，不能直接等同于网络断流。",
    "joint_stream_command_timeout": "板端超过150ms没有接受到新参考；原因可能是主机停发、传输或参考被拒绝。",
    "joint_stream_board_reference_rejected": "板端最终插值输出保护触发；旧SDK统一使用此原因，无法仅凭该字段区分周期超时与输出越限。",
    "joint_feedback_outside_limits": "实测关节超出SDK原始限位，触发故障保护。",
    "Visual image source expired": "当前使用图像超过2s有效期，高频线程停止发送。",
}


def load_object(path):
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return {}


def read_events(path):
    rows, malformed = [], []
    if path.exists():
        for number, line in enumerate(path.open(errors="replace"), 1):
            try:
                row = json.loads(line)
                if (
                    not isinstance(row, dict)
                    or not {"kind", "detail", "local_mono_ns"} <= row.keys()
                ):
                    raise ValueError("Incomplete event")
                rows.append(row)
            except ValueError:
                malformed.append(number)
    return rows, malformed


def stats(values):
    if not values:
        return "未记录"
    return f"样本{len(values)}，平均{statistics.mean(values):.2f} ms，最大{max(values):.2f} ms"


def timestamp(wall_ns):
    if not wall_ns:
        return "未记录"
    zone = datetime.timezone(datetime.timedelta(hours=8))
    return datetime.datetime.fromtimestamp(wall_ns / 1e9, zone).isoformat(timespec="milliseconds")


def write_execution_log(run, output=None, exit_code=None):
    run = Path(run).resolve()
    output = Path(output) if output else run / "execution.log"
    launch = load_object(run.parent / "launch.json")
    exit_metadata = run.parent / "process_exit.json"
    if exit_code is None:
        exit_code = load_object(exit_metadata).get("exit_code")
    elif launch and not exit_metadata.exists():
        exit_metadata.write_text(json.dumps(dict(exit_code=exit_code)) + "\n")
    summary = load_object(run / "summary.json")
    rows, malformed = read_events(run / "execution_trace/events.jsonl")
    kinds = defaultdict(list)
    for row in rows:
        kinds[row["kind"]].append(row)
    started = launch.get("local_mono_ns", rows[0]["local_mono_ns"] if rows else None)
    finished = rows[-1]["local_mono_ns"] if rows else started

    def offset(row):
        return (row["local_mono_ns"] - started) / 1e9 if started is not None else 0

    failures = kinds["reference_loop_failed"] or kinds["phase_failed"] or kinds["execution_failed"]
    failure = failures[0] if failures else None
    error = failure["detail"].get("error") if failure else summary.get("error")
    console_file = run.parent / "console.log"
    console_tail = (
        console_file.read_text(errors="replace")[-16000:].splitlines()[-25:]
        if console_file.exists()
        else []
    )
    if not error and exit_code not in (None, 0) and console_tail:
        error = next(
            (
                line
                for line in reversed(console_tail)
                if "Error" in line or "Exception" in line or "KeyboardInterrupt" in line
            ),
            console_tail[-1],
        )
    task_states = kinds["task_state"]
    if not task_states:
        # Historical state-machine records used the same host monotonic clock.
        task_states = [
            dict(kind="task_state", detail=event, local_mono_ns=int(event["monotonic"] * 1e9))
            for event in load_object(run / "grasp/execution.json").get("events", [])
            if "monotonic" in event
        ]
    active_tasks = [
        row for row in task_states if row["detail"]["phase"] not in ("BLOCKED", "HOLD", "COMPLETE")
    ]
    phases = kinds["phase_start"]
    if active_tasks:
        stopped_at = PHASES.get(
            active_tasks[-1]["detail"]["phase"], active_tasks[-1]["detail"]["phase"]
        )
    elif phases:
        stopped_at = PHASES.get(phases[-1]["detail"]["phase"], phases[-1]["detail"]["phase"])
    elif not rows:
        stopped_at = "程序启动/导入/模型加载之前或期间（未进入执行日志）"
    elif not kinds["vision_preflight"]:
        stopped_at = "视觉采集、跟踪与数据配对准备"
    else:
        stopped_at = "执行器准备或原位保持确认"
    pending_operations = {}
    for row in rows:
        if row["kind"] == "operation_started":
            pending_operations[row["detail"]["name"]] = row
        elif row["kind"] == "operation_completed":
            pending_operations.pop(row["detail"]["name"], None)
    if pending_operations and not phases and not active_tasks:
        stopped_at = next(reversed(pending_operations))
    if summary.get("completed") is True:
        result = "完成" if summary.get("complete_grasp") else "限定流程完成（非完整抓取）"
        stopped_at = "无，已完成本次限定流程"
    elif error:
        result = "失败 / 已停止"
    else:
        result = "异常退出或日志不完整，原因未记录"
    if exit_code not in (None, 0):
        result = f"进程异常退出（退出码{exit_code}）" if not error else result

    owned_ids = {row["detail"].get("request_id") for row in kinds["reference_sent"]}
    sdk_stop = next(
        (
            row
            for row in kinds["sdk_feedback"]
            if row["detail"]["state"]["arm_startup"]["joint_control"].get("request_id") in owned_ids
            and row["detail"]["state"]["arm_startup"]["joint_control"].get("phase")
            in ("stopped", "fault")
        ),
        None,
    )
    sdk_reason = (
        sdk_stop["detail"]["state"]["arm_startup"]["joint_control"].get("reason")
        if sdk_stop
        else None
    )
    explanation = next((note for token, note in CAUSES.items() if token in (error or "")), None)
    if explanation is None and sdk_reason:
        explanation = CAUSES.get(sdk_reason)

    lines = [
        f"视觉抓取执行日志：{run.parent.name}",
        f"运行ID：{rows[0].get('run_id', '未记录') if rows else '未记录'}",
        f"汇总生成时间：{timestamp(time.time_ns())}；汇总器SHA256：{hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}",
        f"启动时间：{timestamp(launch.get('wall_time_ns', rows[0].get('wall_time_ns') if rows else None))}",
        f"最后事件时间：{timestamp(rows[-1].get('wall_time_ns') if rows else None)}",
        f"模式：{launch.get('mode', summary.get('mode', '未记录'))}；进程退出码：{exit_code if exit_code is not None else '未记录'}",
        f"结果：{result}",
        f"停止位置：{stopped_at}",
        f"{'初次触发原因（后续已恢复）' if summary.get('recovered') else '触发原因'}：{error or ('无' if summary.get('completed') else '没有记录到异常，不能据此推断原因')}",
        f"原因说明：{explanation or '请结合下方阶段与原始异常记录判断；不自动推断未记录的根因。'}",
        f"SDK首个观测停止原因：{sdk_reason or '未记录到本次请求的停止状态'}",
        f"日志覆盖时长：{(finished - started) / 1e9:.3f} s"
        if started is not None and rows
        else "日志覆盖时长：未记录",
        "",
        "=== 执行逻辑与阶段耗时（+时间为本次启动后的秒数） ===",
    ]
    if summary.get("recovered"):
        lines.insert(-2, "后续已恢复：" + str(summary.get("recovery_evidence", "见恢复记录")))
    recording = load_object(run.parent / "rosbag/session.json")
    if recording:
        bag_report = load_object(run.parent / "rosbag/download/report.json")
        lines.insert(
            -2,
            "ROS bag："
            + json.dumps(
                {
                    "remote": f"{recording.get('host')}:{recording.get('remote_path')}",
                    "local": str(run.parent / "rosbag/download/bag"),
                    "downloaded": recording.get("downloaded", False),
                    "complete": bag_report.get("complete", False),
                    "message_counts": bag_report.get("message_counts", {}),
                    "error": recording.get("error", bag_report.get("error")),
                },
                ensure_ascii=False,
            ),
        )
    timeline = []
    initialization = kinds["runtime_initialization"]
    if initialization:
        detail = initialization[0]["detail"]
        timeline.append(
            (
                initialization[0],
                f"初始化完成：导入/配置{detail['imports_configuration_s']:.3f}s，模型加载{detail['model_load_s']:.3f}s",
            )
        )
    elif rows and launch:
        timeline.append(
            (rows[0], f"日志系统就绪：启动至此{offset(rows[0]):.3f}s（旧日志未细分模型加载）")
        )
    if kinds["vision_target"]:
        timeline.append((kinds["vision_target"][0], "首个有效箱体目标就绪"))
    if kinds["vision_preflight"]:
        row = kinds["vision_preflight"][0]
        anchor = kinds["vision_target"][0] if kinds["vision_target"] else rows[0]
        timeline.append(
            (
                row,
                f"视觉连续性准备检查结束：耗时{(row['local_mono_ns'] - anchor['local_mono_ns']) / 1e9:.3f}s",
            )
        )
    operations = {}
    for row in rows:
        detail = row["detail"]
        if row["kind"] == "operation_started":
            operations[detail["name"]] = row
            timeline.append((row, f"开始：{detail['name']}"))
        elif row["kind"] == "operation_completed":
            before = operations.pop(detail["name"], None)
            elapsed = (row["local_mono_ns"] - before["local_mono_ns"]) / 1e9 if before else None
            timeline.append(
                (
                    row,
                    f"完成：{detail['name']}，耗时{elapsed:.3f}s"
                    if elapsed is not None
                    else f"完成：{detail['name']}",
                )
            )
        elif row["kind"] == "task_state":
            phase = detail["phase"]
            timeline.append((row, f"抓取状态机进入：{PHASES.get(phase, phase)}"))
        elif row["kind"] in ("existing_hold_reused", "initial_hold_receipt"):
            timeline.append(
                (
                    row,
                    "复用现有使能保持"
                    if row["kind"] == "existing_hold_reused"
                    else "原位保持请求已返回",
                )
            )
        elif row["kind"] == "moveit_plan":
            plan = detail["plan"]
            timeline.append(
                (
                    row,
                    f"MoveIt轨迹规划完成：{plan['planning_s']:.3f}s，"
                    f"{plan['points']}个轨迹点，plan_id={plan['plan_id']}",
                )
            )
        elif row["kind"] == "moveit_result":
            result = detail.get("result") or {}
            timeline.append(
                (
                    row,
                    "板端轨迹执行回执："
                    + json.dumps(result, ensure_ascii=False)
                    + f"，需要重新规划={detail.get('replan', False)}",
                )
            )
        elif row["kind"] == "phase_start":
            phase = detail["phase"]
            timeline.append((row, f"运动阶段开始：{PHASES.get(phase, phase)}"))
        elif row["kind"] in ("phase_failed", "phase_completed"):
            phase = detail.get("phase", detail.get("event", {}).get("phase"))
            before = next(
                (
                    item
                    for item in reversed(phases)
                    if item["local_mono_ns"] <= row["local_mono_ns"]
                    and item["detail"].get("phase") == phase
                ),
                None,
            )
            duration = (
                f"，耗时{(row['local_mono_ns'] - before['local_mono_ns']) / 1e9:.3f}s"
                if before
                else ""
            )
            timeline.append(
                (
                    row,
                    f"运动阶段{'失败' if row['kind'] == 'phase_failed' else '完成'}：{PHASES.get(phase, phase)}{duration}",
                )
            )
        elif row["kind"] == "reference_loop_failed":
            context = detail.get("failed_tick") or {}
            timeline.append(
                (
                    row,
                    f"100Hz回路停止：{detail.get('error')}；失败点={context.get('stage', '旧日志未记录')}，当次dt={context.get('dt_s', '未记录')}s",
                )
            )
        elif row["kind"] == "execution_failed":
            timeline.append((row, f"执行异常：{detail.get('error')}"))
    for row, message in sorted(timeline, key=lambda item: item[0]["local_mono_ns"]):
        lines.append(f"[+{offset(row):9.3f}s] {message} [事件{row.get('event_sequence', '?')}]")
    for name, row in operations.items():
        lines.append(
            f"{'初次未完成，后续已恢复' if summary.get('recovered') else '尚未完成'}：{name}，从开始至最后记录{(finished - row['local_mono_ns']) / 1e9:.3f}s"
        )
    for before, after in zip(task_states, task_states[1:]):
        phase = before["detail"]["phase"]
        if phase not in ("BLOCKED", "HOLD", "COMPLETE"):
            lines.append(
                f"状态机阶段耗时：{PHASES.get(phase, phase)} → {PHASES.get(after['detail']['phase'], after['detail']['phase'])}：{(after['local_mono_ns'] - before['local_mono_ns']) / 1e9:.3f}s（包含等待和检查）"
            )

    lines.extend(["", "=== 分段性能统计 ==="])
    vision = kinds["vision_quality"]
    timing_names = {
        "receive_decode": "取图与解码",
        "prepare_input": "图像预处理",
        "segment_depth_roi": "分割/深度/ROI",
        "pose": "位姿估计",
        "render_quality": "渲染与质量检查",
        "processing": "视觉处理合计（不含取图）",
    }
    for mode in ("register", "track"):
        frames = [row for row in vision if row["detail"].get("pose_mode") == mode]
        if frames:
            lines.append(f"{'首帧注册' if mode == 'register' else '连续跟踪'}：{len(frames)}帧")
            for name, label in timing_names.items():
                values = [
                    row["detail"]["timing_s"][name] * 1000
                    for row in frames
                    if name in row["detail"].get("timing_s", {})
                ]
                lines.append(f"  {label}：{stats(values)}")
    if vision and not any(row["detail"].get("timing_s") for row in vision):
        lines.append("旧日志未细分视觉阶段；图像年龄不是推理耗时。")
    lines.append(f"视觉帧总数：{len(vision)}；有效目标发布：{len(kinds['vision_target'])}")
    targets = kinds["vision_target"]
    lines.append(
        f"有效目标更新间隔：{stats([(b['local_mono_ns'] - a['local_mono_ns']) / 1e6 for a, b in zip(targets, targets[1:])])}"
    )
    ik = [
        row["detail"]["solver"]["solve_wall_s"] * 1000
        for row in kinds["alignment_endpoint_ik"]
        if "solve_wall_s" in row["detail"]["solver"]
    ]
    lines.append(f"完整IK求解：{stats(ik)}")
    sent = kinds["reference_sent"]
    sessions = defaultdict(list)
    for row in sent:
        sessions[row["detail"].get("request_id")].append(row)
    gaps = [
        (b["local_mono_ns"] - a["local_mono_ns"]) / 1e6
        for group in sessions.values()
        for a, b in zip(group, group[1:])
    ]
    lines.append(f"100Hz参考发送：{len(sent)}条；同会话相邻发送间隔：{stats(gaps)}")
    lines.append(
        f"参考计算至UDP发送：{stats([row['detail']['compute_to_send_s'] * 1000 for row in sent if 'compute_to_send_s' in row['detail']])}"
    )
    for row in kinds["reference_loop_failed"]:
        lines.append(
            "回路计数及耗时上界：" + json.dumps(row["detail"].get("metrics"), ensure_ascii=False)
        )
    requests = defaultdict(list)
    round_trips = defaultdict(list)
    for row in rows:
        detail = row["detail"]
        if row["kind"] == "sdk_request":
            requests[detail["command"]].append(row["local_mono_ns"])
        elif row["kind"] == "sdk_receipt" and requests[detail["command"]]:
            began = requests[detail["command"]].pop(0)
            round_trips[detail["command"]].append((row["local_mono_ns"] - began) / 1e6)
    for command, durations in round_trips.items():
        lines.append(f"SDK请求往返 {command}：{stats(durations)}")
    if vision:
        lines.extend(["", "=== 每帧视觉处理与目标生成 ==="])
        published = {row["detail"]["target"]["pair_id"]: row for row in targets}
        for row in vision:
            detail = row["detail"]
            stages = detail.get("timing_s", {})
            parts = [
                f"{label}={stages[name] * 1000:.2f}ms"
                for name, label in timing_names.items()
                if name in stages
            ]
            target = published.get(detail["pair_id"])
            parts.append(
                f"质量检查后配对/目标生成={(target['local_mono_ns'] - row['local_mono_ns']) / 1e6:.2f}ms"
                if target
                else "此帧未发布目标（拒绝或结束前未完成）"
            )
            lines.append(
                f"[+{offset(row):.3f}s] pair={detail['pair_id']}，{detail.get('pose_mode', '旧日志未区分注册/跟踪')}，图像年龄={detail['age_s'] * 1000:.2f}ms；"
                + "；".join(parts)
            )
    lines.extend(["", "=== SDK状态与停止处理 ==="])
    previous = None
    for row in kinds["sdk_feedback"]:
        state = row["detail"]["state"]
        control = state["arm_startup"]["joint_control"]
        if control.get("request_id") not in sessions:
            continue
        key = (control.get("request_id"), control.get("phase"), control.get("reason"))
        if key != previous:
            lines.append(
                f"[+{offset(row):.3f}s] request={key[0]}，phase={key[1]}，reason={key[2]}，板端观测序号={control.get('joint_stream_sequence')}"
            )
        previous = key
    lines.append(
        "停止回执：" + json.dumps(summary.get("stop_receipt", "未记录"), ensure_ascii=False)
    )
    for name in ("stop_error", "finish_error", "logging_error", "final_record_error"):
        if name in summary:
            lines.append(f"{name}：{summary[name]}")
    final = summary.get("final_state") or {}
    if final:
        control = final.get("arm_startup", {}).get("joint_control", {})
        lines.append(
            f"最后状态：使能={final.get('arms', {}).get('enabled')}，SDK阶段={control.get('phase')}，原因={control.get('reason')}，板端序号={control.get('joint_stream_sequence')}"
        )
    rejects = Counter(
        row["detail"].get("error", row["kind"])
        for row in rows
        if row["kind"]
        in (
            "capture_rejected",
            "image_pair_rejected",
            "lift_pairing_wait",
            "vision_quality_rejected",
        )
    )
    lines.append("等待/拒绝记录计数：" + json.dumps(dict(rejects), ensure_ascii=False))
    if failure and failure["detail"].get("failed_tick"):
        lines.extend(
            [
                "",
                "=== 失败当次输入快照 ===",
                json.dumps(failure["detail"]["failed_tick"], ensure_ascii=False, indent=2),
            ]
        )
    for event in kinds["fixed_target_accepted"]:
        packet = event["detail"]["packet"]
        lines.extend(
            [
                "",
                "=== 固定对准目标 ===",
                f"接受时间：{timestamp(event['wall_time_ns'])}；源帧：{packet['pair_id']}；"
                f"原始图像时间戳：{packet['image_receive_mono_ns']}ns（机器人时钟）",
                "图像2秒门限用于接受目标/SDK首包；接受后固定目标，以实测反馈判定到位。",
            ]
        )
    if kinds["grasp_points_visualization"]:
        lines.extend(["", "=== 抓取点生成可视化（每帧独立保存） ==="])
        for event in kinds["grasp_points_visualization"]:
            detail = event["detail"]
            artifact = event.get("artifact", {})
            lines.append(
                f"生成时间：{timestamp(detail['generated_wall_time_ns'])}；"
                f"帧：{detail['target']['pair_id']}；"
                f"原始采集时间信息：{json.dumps(detail['source_metadata'], ensure_ascii=False)}；"
                f"图像：{run / 'execution_trace' / artifact.get('file', '未完成')}；"
                f"坐标/配对数据：{artifact.get('metadata', '未完成')}"
            )
        lines.append("采集时间保留源时钟；机器人单调时间、相机设备时间不能直接当作本机日历时间。")
    if summary.get("traceback") or (failure and failure["detail"].get("traceback")):
        lines.extend(
            ["", "=== 原始异常堆栈 ===", summary.get("traceback") or failure["detail"]["traceback"]]
        )
    if summary.get("diagnosis"):
        lines.extend(["", "=== 本次故障分析（执行后核对） ===", *summary["diagnosis"]])
    lines.extend(
        [
            "",
            "=== 原始依据 ===",
            f"结构化完整事件：{run / 'execution_trace/events.jsonl'}",
            f"原始图像：{run / 'execution_trace/frames'}",
            f"运行结果：{run / 'summary.json'}",
            f"原始控制台输出：{console_file if console_file.exists() else '未单独保存，见结构化事件及异常堆栈'}",
            "主机UDP发送不等于板端收到；旧SDK无逐包拒收原因，不能仅凭序号差断言网络丢包。",
        ]
    )
    for row in kinds["run_configuration"]:
        lines.append("执行配置：" + json.dumps(row["detail"].get("execution"), ensure_ascii=False))
        for resource in row["detail"].get("resources", []):
            lines.append(f"资源/代码：{resource['path']}，SHA256={resource['sha256']}")
    if not summary and console_tail:
        lines.extend(["", "=== 未产生完整结果时的控制台末尾 ===", *console_tail])
    if malformed:
        lines.append(f"事件文件有未完整写入/损坏行：{malformed}；已保留其余记录，不伪造缺失内容。")
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", dir=output.parent, delete=False
    ) as stream:
        temporary = Path(stream.name)
        stream.write("\n".join(lines) + "\n")
    os.replace(temporary, output)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--exit-code", type=int)
    parser.add_argument("--start", action="store_true")
    parser.add_argument("--mode")
    args = parser.parse_args()
    if args.start:
        args.run.parent.mkdir(parents=True, exist_ok=True)
        with (args.run.parent / "launch.json").open("x") as stream:
            json.dump(
                dict(
                    mode=args.mode, wall_time_ns=time.time_ns(), local_mono_ns=time.monotonic_ns()
                ),
                stream,
            )
        return
    print(write_execution_log(args.run, args.output, args.exit_code))


if __name__ == "__main__":
    main()
