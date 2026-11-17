#!/usr/bin/env python3
"""Read execution traces and report timing without accessing robot hardware."""

import argparse
import hashlib
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path


def distribution(values):
    values = sorted(float(value) for value in values if math.isfinite(value))
    if not values:
        return None

    def percentile(fraction):
        index = (len(values) - 1) * fraction
        low = math.floor(index)
        high = math.ceil(index)
        return values[low] + (values[high] - values[low]) * (index - low)

    return dict(
        count=len(values),
        mean=statistics.mean(values),
        p50=statistics.median(values),
        p95=percentile(0.95),
        p99=percentile(0.99),
        maximum=max(values),
    )


def intervals(events):
    # Never count the pause between two different motion sessions as a send gap.
    groups = defaultdict(list)
    for event in events:
        groups[event["detail"].get("request_id")].append(event["local_mono_ns"])
    return [
        (after - before) / 1e6
        for stamps in groups.values()
        for before, after in zip(stamps, stamps[1:])
    ]


def analyze(run):
    run = Path(run)
    trace = run / "execution_trace/events.jsonl"
    rows = [json.loads(line) for line in trace.open()]
    kinds = defaultdict(list)
    for row in rows:
        kinds[row["kind"]].append(row)
    summary_file = run / "summary.json"
    summary = json.loads(summary_file.read_text()) if summary_file.exists() else {}
    frames = {}
    vision = []
    for row in rows:
        # A later duplicate response must not overwrite an earlier frame's start.
        if row["kind"] == "rgbd_response":
            headers = {key.lower(): value for key, value in row["detail"]["headers"].items()}
            frames[headers.get("x-pair-id")] = row["local_mono_ns"]
            continue
        if row["kind"] != "vision_quality":
            continue
        detail = row["detail"]
        frame_stamp = frames.get(detail["pair_id"])
        timing = detail.get("timing_s")
        vision.append(
            dict(
                pair_id=detail["pair_id"],
                pose_mode=detail.get("pose_mode"),
                image_age_ms=detail["age_s"] * 1000,
                processing_ms=timing["processing"] * 1000 if timing else None,
                stages_ms={key: value * 1000 for key, value in timing.items()} if timing else {},
                body_to_quality_event_ms=(row["local_mono_ns"] - frame_stamp) / 1e6
                if frame_stamp is not None
                else None,
            )
        )
    stage_names = sorted({name for frame in vision for name in frame["stages_ms"]})
    visual_summary = dict(
        quality_frames=len(vision),
        accepted_targets=len(kinds["vision_target"]),
        first_frame=vision[0] if vision else None,
        processing_ms=distribution(
            [frame["processing_ms"] for frame in vision if frame["processing_ms"] is not None]
        ),
        body_to_quality_event_ms=distribution(
            [
                frame["body_to_quality_event_ms"]
                for frame in vision
                if frame["body_to_quality_event_ms"] is not None
            ]
        ),
        subsequent_body_to_quality_event_ms=distribution(
            [
                frame["body_to_quality_event_ms"]
                for frame in vision[1:]
                if frame["body_to_quality_event_ms"] is not None
            ]
        ),
        image_age_ms=distribution([frame["image_age_ms"] for frame in vision]),
        target_update_interval_ms=distribution(intervals(kinds["vision_target"])),
        stages_ms={
            name: distribution(
                [frame["stages_ms"][name] for frame in vision if name in frame["stages_ms"]]
            )
            for name in stage_names
        },
        stages_by_pose_mode_ms={
            mode: {
                name: distribution(
                    [
                        frame["stages_ms"][name]
                        for frame in vision
                        if frame["pose_mode"] == mode and name in frame["stages_ms"]
                    ]
                )
                for name in stage_names
            }
            for mode in ("register", "track")
        },
    )
    sent = kinds["reference_sent"]
    attempts = kinds["reference_send_attempt"]
    request_ids = {row["detail"].get("request_id") for row in sent}
    send_intervals = intervals(sent)
    attempts_by_key = {
        (row["detail"].get("request_id"), row["detail"]["sequence"]): row for row in attempts
    }
    log_to_sent = []
    for row in sent:
        detail = row["detail"]
        before = attempts_by_key.get((detail.get("request_id"), detail["sequence"]))
        if before:
            log_to_sent.append((row["local_mono_ns"] - before["local_mono_ns"]) / 1e6)
    metrics = [row["detail"]["metrics"] for row in kinds["reference_loop_failed"]]
    if not metrics:
        metrics = [
            event["reference_timing"]
            for event in summary.get("events", [])
            if "reference_timing" in event
        ]
    references = dict(
        sent=len(sent),
        send_interval_ms=distribution(send_intervals),
        observed_hz=1000 / statistics.mean(send_intervals) if send_intervals else None,
        intervals_over_15ms=sum(value > 15 for value in send_intervals),
        intervals_over_30ms=sum(value > 30 for value in send_intervals),
        intervals_over_150ms=sum(value > 150 for value in send_intervals),
        tick_dt_ms=distribution([row["detail"]["dt_s"] * 1000 for row in attempts]),
        logged_attempt_to_sent_ms=distribution(log_to_sent),
        compute_to_send_ms=distribution(
            [
                row["detail"]["compute_to_send_s"] * 1000
                for row in sent
                if "compute_to_send_s" in row["detail"]
            ]
        ),
        udp_send_ms=distribution(
            [row["detail"]["udp_send_s"] * 1000 for row in sent if "udp_send_s" in row["detail"]]
        ),
        feedback_age_at_send_ms=distribution(
            [
                (
                    row["detail"]["estimated_robot_now_ns"]
                    - row["detail"]["feedback_receive_mono_ns"]
                )
                / 1e6
                for row in attempts
            ]
        ),
        loop_metrics=metrics,
    )
    transitions = []
    for row in kinds["sdk_feedback"]:
        state = row["detail"]["state"]
        control = state["arm_startup"]["joint_control"]
        if control.get("request_id") not in request_ids:
            continue
        status = (control.get("request_id"), control.get("phase"), control.get("reason"))
        if transitions and status == transitions[-1]["_status"]:
            continue
        transitions.append(
            dict(
                _status=status,
                local_mono_ns=row["local_mono_ns"],
                request_id=status[0],
                phase=status[1],
                reason=status[2],
                accepted_sequence=control.get("joint_stream_sequence"),
            )
        )
    for row in transitions:
        del row["_status"]
    observed_sequences = [
        row["detail"]["state"]["arm_startup"]["joint_control"].get("joint_stream_sequence", 0)
        for row in kinds["sdk_feedback"]
        if row["detail"]["state"]["arm_startup"]["joint_control"].get("request_id") in request_ids
    ]
    command_requests = defaultdict(list)
    command_durations = defaultdict(list)
    for row in rows:
        detail = row["detail"]
        if row["kind"] == "sdk_request":
            command_requests[detail["command"]].append(row["local_mono_ns"])
        elif row["kind"] == "sdk_receipt" and command_requests[detail["command"]]:
            began = command_requests[detail["command"]].pop(0)
            command_durations[detail["command"]].append((row["local_mono_ns"] - began) / 1e6)
    feedback_file = run / "joint_feedback.jsonl"
    feedback_requests = []
    if feedback_file.exists():
        for line in feedback_file.open():
            detail = json.loads(line)
            duration = (detail.get("timing") or {}).get("request_ms")
            if duration is not None:
                feedback_requests.append(duration)
    ik = [row["detail"]["solver"].get("solve_wall_s") for row in kinds["alignment_endpoint_ik"]]
    return dict(
        run=str(run.resolve()),
        trace=str(trace.resolve()),
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        commands_sent=summary.get("commands_sent", bool(sent)),
        completed=summary.get("completed"),
        error=summary.get("error"),
        vision=visual_summary,
        references=references,
        sdk_last_observed_accepted_sequence=max(observed_sequences, default=None),
        sdk_status_transitions=transitions,
        sdk_http_round_trip_ms={
            name: distribution(values) for name, values in command_durations.items()
        },
        feedback_http_round_trip_ms=distribution(feedback_requests),
        endpoint_ik_solve_ms=distribution([value * 1000 for value in ik if value is not None]),
        limitations=[
            "Historical body-to-quality timing includes decode, preprocessing, segmentation, pose and quality; it is not pure model inference.",
            "First frame is reported separately; subsequent frames are not assumed to be equally warmed up.",
            "reference_sent means UDP handed to the host network stack, not delivery to the robot or motor.",
            "No per-packet robot arrival timestamp exists; SDK status samples cannot establish network latency or packet loss.",
            "All elapsed intervals use one host monotonic clock; robot timestamp differences use robot timestamps only.",
            "Failed ticks may not emit reference_send_attempt; use the exception and loop metrics as well.",
        ],
    )


def cell(stats):
    return (
        "未记录"
        if not stats
        else f"{stats['mean']:.2f} / {stats['p95']:.2f} / {stats['maximum']:.2f}"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "runs", nargs="+", type=Path, help="Run directories containing execution_trace/"
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    reports = [analyze(run) for run in args.runs]
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "timing.json").write_text(
        json.dumps(reports, ensure_ascii=False, indent=2) + "\n"
    )
    lines = [
        "# 视觉抓取执行耗时统计",
        "",
        "当前架构：完整IK子进程、100Hz参考线程都在笔记本；板端10ms校验、限速并输出。视觉通过缓存更新目标，高频线程不等待新图或HTTP。",
        "",
        "数值单位ms，统计列为平均 / P95 / 最大。只分析已归档日志，不发送运动指令。",
        "",
        "| 运行 | 视觉处理（首帧后，接收正文到质量日志） | 实际发送间隔 | 发送数 / 板端最后观测序号 | 中断原因 |",
        "| --- | --- | --- | --- | --- |",
    ]
    for report in reports:
        name = Path(report["run"]).parent.name
        lines.append(
            f"| {name} | {cell(report['vision']['subsequent_body_to_quality_event_ms'])} | "
            f"{cell(report['references']['send_interval_ms'])} | {report['references']['sent']} / "
            f"{report['sdk_last_observed_accepted_sequence']} | {report['error'] or '无'} |"
        )
    for report in reports:
        lines.extend(["", f"## {Path(report['run']).parent.name}", ""])
        first = report["vision"]["first_frame"]
        if first:
            lines.append(
                f"首帧正文到质量日志：{first['body_to_quality_event_ms']}ms；图像年龄包含上游等待/传输，不能直接当成识别时间。"
            )
        lines.append(
            f"完整IK耗时：{cell(report['endpoint_ik_solve_ms'])}；旧日志无开始时间时不倒推耗时。"
        )
        lines.append(f"反馈HTTP往返：{cell(report['feedback_http_round_trip_ms'])}。")
        lines.append(f"有效目标发布间隔：{cell(report['vision']['target_update_interval_ms'])}。")
        lines.extend(["", "| 当前直接计时阶段 | 平均 / P95 / 最大 ms |", "| --- | --- |"])
        for name, stats in report["vision"]["stages_ms"].items():
            lines.append(f"| {name} | {cell(stats)} |")
        lines.extend(
            [
                "",
                "SDK状态变化：",
                "",
                "```json",
                json.dumps(report["sdk_status_transitions"], ensure_ascii=False, indent=2),
                "```",
            ]
        )
    lines.extend(
        [
            "",
            "统计边界：",
            "",
            "- 历史视觉处理是正文归档事件到质量事件，包含解码、预处理、分割、位姿与质量检查，不能当成纯模型推理。新日志的timing_s细分这些阶段。",
            "- 指令间隔是主机交给UDP网络栈后的日志间隔；板端接收序号单独记录，不能把二者差值直接算成丢包。",
            "- 尚无逐包板端到达时间，不能给出实测单向网络延迟或驱动器1kHz周期。",
            "- 中断原因按原SDK状态与主机异常分别判断；出现反馈超时不自动等同于网络卡顿。",
        ]
    )
    (args.output / "report.md").write_text("\n".join(lines) + "\n")
    print(args.output / "report.md")


if __name__ == "__main__":
    main()
