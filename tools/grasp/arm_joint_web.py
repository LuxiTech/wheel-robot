#!/usr/bin/env python3
"""Local web panel for the deployed SDK's sixteen joint_move axes (radians)."""

import argparse
import datetime
import fcntl
import ipaddress
import json
import math
import secrets
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from _bootstrap import configure

configure(ROOT)
from wheel_sdk_bridge.http_client import SDKError
from wheel_sdk_bridge.joint_client import JointClient, vector


class JointPanel:
    def __init__(self, client, log_path):
        self.client = client
        self.log_path = Path(log_path)
        self.token = secrets.token_urlsafe(32)
        self.lock = threading.Lock()
        self.actions = {}
        self.last_motion = None

    def record(self, event, **fields):
        record = dict(
            utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
            event=event,
            **fields,
        )
        with self.log_path.open("a") as stream:
            stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")

    def _status(self):
        state = self.client.snapshot()
        joint = state["arm_startup"]["joint_control"]
        pending = self.client.pending_id
        if pending and joint.get("request_id") == pending:
            error = max(
                abs(actual - target)
                for actual, target in zip(state["arms"]["position"], joint["target"])
            )
            completed = (
                joint["phase"] == "ready"
                and state["arms"]["enabled"] is True
                and error <= 0.02
                and all(abs(value) < 0.08 for value in state["arms"]["velocity"])
            )
            self.last_motion = dict(
                request_id=pending,
                phase=joint["phase"],
                completed=completed,
                max_error_rad=error,
                reason=joint.get("reason", ""),
            )
            if completed or joint["phase"] in ("stopped", "fault"):
                self.record("motion_result", **self.last_motion)
                self.client.pending_id = None
        elif pending:
            # A lost POST acknowledgement must never result in resending motion.
            self.last_motion = dict(
                request_id=pending, phase="unconfirmed_or_superseded", completed=False
            )
        return dict(
            connected=True,
            positions_rad=state["arms"]["position"],
            velocities_rad_s=state["arms"]["velocity"],
            enabled=state["arms"]["enabled"],
            arm_phase=state["arms"]["phase"],
            joint_control=joint,
            fault_latched=state["arm_startup"].get("manual_pose", {}).get("fault_latched"),
            pending_id=self.client.pending_id,
            can_stop=bool(pending and joint.get("request_id") == pending),
            last_motion=self.last_motion,
        )

    def status(self):
        with self.lock:
            return self._status()

    def move(self, payload):
        changes = payload.get("changes")
        if not isinstance(changes, dict) or not changes:
            raise ValueError("至少选择一个关节")
        selected = {}
        for key, value in changes.items():
            if key not in {str(index) for index in range(16)}:
                raise ValueError("关节索引必须为 0～15")
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError("目标弧度必须为有限数值")
            if not math.isfinite(value):
                raise ValueError("目标弧度必须为有限数值")
            selected[int(key)] = float(value)
        enable = payload.get("enable_if_needed", False)
        if type(enable) is not bool:
            raise ValueError("enable_if_needed 必须为布尔值")
        state = self.client.snapshot()
        startup = state["arm_startup"]
        joint = startup["joint_control"]
        if startup.get("manual_pose", {}).get("fault_latched") is not False:
            raise SDKError("SDK 故障锁存状态未就绪", 409)
        if joint["phase"] in ("preparing", "moving", "fault"):
            raise SDKError("SDK 正在运动或处于故障状态", 409)
        # Keep existing nominal hold targets when enabled. Measured droop must
        # not become a new command for the other joints during compensated hold.
        holding = state["arms"]["enabled"] is True
        target = vector(
            startup.get("nominal_target") if holding else state["arms"]["position"],
            "Holding target",
        )
        for index, value in selected.items():
            target[index] = value
        self.record(
            "move_intent",
            changes=changes,
            positions_rad=target,
            speed_rad_s=payload.get("speed_rad_s"),
            enable_if_needed=enable,
            measured_positions_rad=state["arms"]["position"],
            unselected_source="nominal_target" if holding else "measured_position",
        )
        result = self.client.move(target, payload.get("speed_rad_s"), enable_if_needed=enable)
        self.last_motion = dict(result, phase="submitted")
        return dict(result, positions_rad=target)

    def action(self, path, payload):
        if not isinstance(payload, dict):
            raise ValueError("JSON body 必须为对象")
        action_id = payload.get("action_id")
        if not isinstance(action_id, str) or not 16 <= len(action_id) <= 80:
            raise ValueError("需要有效的 action_id")
        fingerprint = json.dumps([path, payload], sort_keys=True, allow_nan=False)
        with self.lock:
            previous = self.actions.get(action_id)
            if previous:
                if previous[0] != fingerprint:
                    raise SDKError("action_id 已用于不同请求", 409)
                return previous[1], previous[2]
            if len(self.actions) >= 10000:
                raise SDKError("本次会话操作数已达上限，请重启页面服务", 409)
            try:
                self._status()
                if path == "/api/move":
                    if self.client.pending_id:
                        raise SDKError("前一个请求仍未完成或回执未确认", 409)
                    result = self.move(payload)
                elif path == "/api/stop":
                    if not self.client.pending_id:
                        raise SDKError("没有本页面尚未完成的运动请求", 409)
                    result = self.client.stop(self.client.pending_id)
                    # joint_stop is global in this SDK, even after ownership
                    # checks; a different controller can race this command.
                    result = dict(result, completed=False, message="已请求停止，等待 SDK 反馈")
                else:
                    raise ValueError("未知接口")
                code = 200
            except (ValueError, SDKError) as error:
                code = error.status if isinstance(error, SDKError) else 400
                result = dict(
                    error=str(error),
                    uncertain=getattr(error, "uncertain", False),
                    pending_id=self.client.pending_id,
                )
            # Cache even failed/uncertain submissions: duplicate browser POSTs
            # with this action_id never send a second SDK command.
            self.actions[action_id] = (fingerprint, code, result)
            self.record("action_result", path=path, action_id=action_id, status=code, result=result)
            return code, result


def make_server(panel, host="127.0.0.1", port=8091):
    html = Path(__file__).with_suffix(".html").read_text().replace("__TOKEN__", panel.token)

    class Handler(BaseHTTPRequestHandler):
        def send(self, status, value, content_type="application/json; charset=utf-8"):
            body = value.encode() if isinstance(value, str) else json.dumps(value).encode()
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("X-Frame-Options", "DENY")
            self.end_headers()
            self.wfile.write(body)

        def valid_host(self):
            try:
                hostname = urlsplit("http://" + self.headers.get("Host", "")).hostname
                if hostname != "localhost":
                    ipaddress.ip_address(hostname)
                return True
            except ValueError:
                return False

        def do_GET(self):
            if not self.valid_host():
                return self.send(403, dict(error="Invalid Host"))
            if self.path == "/":
                return self.send(200, html, "text/html; charset=utf-8")
            if self.path != "/api/state":
                return self.send(404, dict(error="Not found"))
            try:
                return self.send(200, panel.status())
            except (SDKError, ValueError, OSError) as error:
                return self.send(503, dict(connected=False, error=str(error)))

        def do_POST(self):
            if self.path not in ("/api/move", "/api/stop"):
                return self.send(404, dict(error="Not found"))
            origin = self.headers.get("Origin")
            if (
                not self.valid_host()
                or self.headers.get("X-Panel-Token") != panel.token
                or (origin and origin != "http://" + self.headers.get("Host", ""))
            ):
                return self.send(403, dict(error="请从本页面发起控制请求"))
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 8192:
                    raise ValueError("Invalid body length")
                if self.headers.get_content_type() != "application/json":
                    raise ValueError("需要 application/json")
                payload = json.loads(self.rfile.read(length))
                code, result = panel.action(self.path, payload)
                return self.send(code, result)
            except (ValueError, SDKError) as error:
                return self.send(400, dict(error=str(error)))

    return ThreadingHTTPServer((host, port), Handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/robots/joint_control.json")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8091)
    args = parser.parse_args()
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    output = ROOT / "outputs/arm-joint-web" / stamp
    output.mkdir(parents=True)
    lock_path = ROOT / ".runtime/arm-joint-web.lock"
    lock_path.parent.mkdir(exist_ok=True)
    with lock_path.open("a") as lock_file:
        try:
            fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error("另一个关节页面服务已在运行")
        policy = json.loads(args.config.read_text())
        policy["feedback_diagnostics"] = str(output / "feedback.jsonl")
        panel = JointPanel(JointClient(policy["base_url"], policy), output / "commands.jsonl")
        server = make_server(panel, args.host, args.port)
        print(f"页面：http://{args.host}:{server.server_port}", flush=True)
        print(f"日志：{output}", flush=True)
        print(
            "启动及刷新只读取反馈；点击发送才会提交运动。关闭页面或服务不会停止真机。", flush=True
        )
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            server.server_close()


if __name__ == "__main__":
    main()
