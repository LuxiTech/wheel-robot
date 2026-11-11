#!/usr/bin/env python3
"""Read-only ROS -> HTTP feedback service. No motor publishers or motion clients.

Run on the robot with its ROS overlay sourced. Only GET /api/state exists.
Monotonic timestamps stay in the robot clock domain; clients add request RTT.
"""

import argparse
import copy
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class StateCache:
    def __init__(self):
        self.lock = threading.Lock()
        self.updated = threading.Condition(self.lock)
        self.arms = self.startup = None
        self.mock = None
        self.checked_ns = 0

    def snapshot(self):
        with self.updated:
            # Topics arrive independently. Wait briefly for a current pair instead
            # of returning mismatched samples or weakening the client's checks.
            deadline = time.monotonic() + 0.08
            while self.arms is not None and self.startup is not None:
                now = time.monotonic_ns()
                arm_time = int(self.arms["receive_mono_ns"])
                startup_time = int(self.startup["receive_mono_ns"])
                if (
                    abs(arm_time - startup_time) <= 30_000_000
                    or not 0 <= now - arm_time <= 400_000_000
                    or not 0 <= now - startup_time <= 400_000_000
                ):
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self.updated.wait(remaining)
            arms, startup = copy.deepcopy(self.arms), copy.deepcopy(self.startup)
            mock, checked = self.mock, self.checked_ns
        now = time.monotonic_ns()
        for obj in (arms, startup):
            if obj is not None:
                obj["fresh"] = 0 <= now - int(obj["receive_mono_ns"]) <= 400_000_000
        verified = 0 <= now - checked <= 2_000_000_000
        return dict(
            server_mono_ns=str(now),
            arms=arms,
            arm_startup=startup,
            system=dict(mock=mock if verified else None),
            feedback_source="ros_joint_feedback_v1",
        )


def make_server(cache, host, port):
    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(2)

        def do_GET(self):
            if self.path != "/api/state":
                self.send_error(404)
                return
            body = json.dumps(cache.snapshot(), allow_nan=False).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *args):
            pass

    return ThreadingHTTPServer((host, port), Handler)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=18032)
    p.add_argument("--driver-node", default="/openarm_adapter")
    a = p.parse_args()
    import rclpy
    from luxi_interfaces.msg import ArmState
    from rcl_interfaces.msg import ParameterType
    from rcl_interfaces.srv import GetParameters
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    cache = StateCache()
    rclpy.init()
    node = Node("wheel_joint_feedback", enable_rosout=False)
    qos = QoSProfile(
        depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT
    )

    def arm_callback(msg):
        value = {
            key: [
                (
                    bool(v)
                    if key == "valid"
                    else int(v)
                    if key in ("age_ns", "mos_temperature", "rotor_temperature")
                    else float(v)
                )
                for v in getattr(msg, key)
            ]
            for key in (
                "position",
                "velocity",
                "effort",
                "valid",
                "age_ns",
                "mos_temperature",
                "rotor_temperature",
            )
        }
        value.update(
            receive_mono_ns=str(msg.receive_mono_ns),
            enabled=msg.enabled,
            phase=msg.phase,
            joint_order="right_then_left",
            schema="luxi_arms_v1",
        )
        with cache.updated:
            cache.arms = value
            cache.updated.notify_all()

    def startup_callback(msg):
        try:
            value = json.loads(msg.data)
            int(value["receive_mono_ns"])
        except (ValueError, KeyError, TypeError):
            return
        with cache.updated:
            cache.startup = value
            cache.updated.notify_all()

    _subscriptions = [
        node.create_subscription(ArmState, "/luxi/arms/state", arm_callback, qos),
        node.create_subscription(String, "/luxi/arms/startup_status", startup_callback, qos),
    ]
    # Read the driver's actual mock parameter. Unknown/disconnected is never real.
    param_client = node.create_client(GetParameters, a.driver_node + "/get_parameters")
    pending = [None, 0]

    def check_driver():
        now = time.monotonic_ns()
        if pending[0] is not None and not pending[0].done():
            if now - pending[1] < 1_000_000_000:
                return
            pending[0].cancel()
        if not param_client.service_is_ready():
            return
        # Refuse ambiguous publishers (e.g. a second mock driver).
        infos = node.get_publishers_info_by_topic("/luxi/arms/state")
        if (
            len(infos) != 1
            or (infos[0].node_namespace.rstrip("/") + "/" + infos[0].node_name) != a.driver_node
        ):
            with cache.lock:
                cache.mock, cache.checked_ns = None, now
            return
        req = GetParameters.Request(names=["mock"])
        future = param_client.call_async(req)
        pending[:] = [future, now]

        def received(f):
            if f.cancelled():
                return
            try:
                values = f.result().values
                value = (
                    values[0].bool_value
                    if len(values) == 1 and values[0].type == ParameterType.PARAMETER_BOOL
                    else None
                )
            except Exception:
                value = None
            with cache.lock:
                cache.mock, cache.checked_ns = value, time.monotonic_ns()

        future.add_done_callback(received)

    _timer = node.create_timer(0.5, check_driver)
    server = make_server(cache, a.host, a.port)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
