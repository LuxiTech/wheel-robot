import copy
import importlib.util
import json
import threading
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener

import pytest
from wheel_sdk_bridge.http_client import SDKError
from wheel_sdk_bridge.joint_client import JointClient

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    "arm_joint_web", ROOT / "tools/grasp/arm_joint_web.py"
)
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)


class Transport:
    def __init__(self):
        self.posts = []
        self.failure = None
        self.state = dict(
            system=dict(mock=False),
            server_mono_ns="1000000000",
            arms=dict(
                fresh=True,
                receive_mono_ns="999000000",
                joint_order="right_then_left",
                phase="hold",
                enabled=True,
                position=[0.03] * 16,
                velocity=[0.0] * 16,
                valid=[True] * 16,
                age_ns=[10000] * 16,
            ),
            arm_startup=dict(
                fresh=True,
                receive_mono_ns="999000000",
                nominal_target=[0.01] * 16,
                static_compensation_rad=[0.0] * 16,
                manual_pose=dict(fault_latched=False),
                joint_control=dict(
                    phase="idle",
                    reason="test",
                    request_id="",
                    unit="rad",
                    joint_order="right_then_left",
                    lower=[-2.0] * 16,
                    upper=[2.0] * 16,
                    target=[0.0] * 16,
                    min_speed_rad_s=0.02,
                    max_speed_rad_s=0.2,
                ),
            ),
        )

    def request(self, method, path, payload=None):
        if method == "GET":
            return copy.deepcopy(self.state)
        self.posts.append((path, payload))
        if self.failure:
            raise self.failure
        return dict(success=True, message="test accepted")


@pytest.fixture
def panel(tmp_path):
    transport = Transport()
    client = JointClient("http://unused", {}, transport=transport)
    return tool.JointPanel(client, tmp_path / "commands.jsonl"), transport


def intent(changes=None, **kwargs):
    return dict(
        action_id="a" * 32,
        changes=changes or {"8": -0.1},
        speed_rad_s=0.02,
        enable_if_needed=False,
        **kwargs,
    )


def test_get_only_and_single_left_axis_preserves_nominal_hold(panel):
    app, transport = panel
    assert app.status()["positions_rad"] == [0.03] * 16
    assert not transport.posts
    code, result = app.action("/api/move", intent())
    assert code == 200 and not result["completed"]
    expected = [0.01] * 16
    expected[8] = -0.1
    payload = transport.posts[0][1]
    assert transport.posts[0][0] == "/api/arm_startup"
    assert payload["command"] == "joint_move"
    assert json.loads(payload["detail"])["positions"] == expected
    assert json.loads(payload["detail"])["unit"] == "rad"


def test_disabled_enable_is_explicit_and_unselected_axes_use_feedback(panel):
    app, transport = panel
    transport.state["arms"]["enabled"] = False
    assert app.action("/api/move", intent())[0] == 409
    assert not transport.posts
    payload = intent()
    payload.update(action_id="b" * 32, enable_if_needed=True)
    assert app.action("/api/move", payload)[0] == 200
    target = json.loads(transport.posts[0][1]["detail"])["positions"]
    assert target[8] == -0.1 and target[:8] == [0.03] * 8


@pytest.mark.parametrize("indices", [range(8), range(8, 16), range(16), [7], [15]])
def test_right_left_both_and_gripper_mapping(panel, indices):
    app, transport = panel
    changes = {str(index): -0.2 for index in indices}
    assert app.action("/api/move", intent(changes))[0] == 200
    target = json.loads(transport.posts[0][1]["detail"])["positions"]
    assert target == [-0.2 if str(index) in changes else 0.01 for index in range(16)]


@pytest.mark.parametrize(
    "changes,speed",
    [
        ({"16": 0.1}, 0.02),
        ({"8": True}, 0.02),
        ({"8": "0.1"}, 0.02),
        ({"8": 2.1}, 0.02),
        ({"8": 0.1}, 0.3),
    ],
)
def test_rejects_bad_input_without_write(panel, changes, speed):
    app, transport = panel
    payload = intent(changes)
    payload["speed_rad_s"] = speed
    assert app.action("/api/move", payload)[0] == 400
    assert not transport.posts


@pytest.mark.parametrize("failure", ["stale", "fault", "moving"])
def test_bad_feedback_fault_or_active_motion_never_writes(panel, failure):
    app, transport = panel
    if failure == "stale":
        transport.state["arms"]["age_ns"][8] = 500000000
    elif failure == "fault":
        transport.state["arm_startup"]["manual_pose"]["fault_latched"] = True
    else:
        transport.state["arm_startup"]["joint_control"]["phase"] = "moving"
    assert app.action("/api/move", intent())[0] >= 400
    assert not transport.posts


def test_duplicate_and_uncertain_submission_are_never_retried(panel):
    app, transport = panel
    transport.failure = SDKError("lost acknowledgement", 503, uncertain=True)
    first = app.action("/api/move", intent())
    assert first[0] == 503 and first[1]["uncertain"]
    assert app.action("/api/move", intent()) == first
    payload = intent()
    payload["changes"] = {"8": 0.2}
    with pytest.raises(SDKError, match="action_id"):
        app.action("/api/move", payload)
    payload["action_id"] = "b" * 32
    assert app.action("/api/move", payload)[0] == 409
    assert len(transport.posts) == 1 and app.client.pending_id


def test_arrival_requires_matching_request_and_measured_position(panel):
    app, transport = panel
    _, result = app.action("/api/move", intent())
    joint = transport.state["arm_startup"]["joint_control"]
    joint.update(phase="ready", target=result["positions_rad"])
    assert app.status()["pending_id"]
    joint["request_id"] = result["request_id"]
    assert not app.status()["last_motion"]["completed"]
    transport.state["arms"]["position"] = result["positions_rad"]
    assert app.status()["last_motion"]["completed"]
    assert app.client.pending_id is None
    assert len(transport.posts) == 1


def test_stop_only_owned_request_and_waits_for_feedback(panel):
    app, transport = panel
    _, result = app.action("/api/move", intent())
    stop = dict(action_id="b" * 32)
    assert app.action("/api/stop", stop)[0] == 409
    joint = transport.state["arm_startup"]["joint_control"]
    joint.update(request_id=result["request_id"], phase="moving")
    stop["action_id"] = "c" * 32
    code, response = app.action("/api/stop", stop)
    assert code == 200 and response["completed"] is False
    assert transport.posts[-1][1]["command"] == "joint_stop"
    assert app.client.pending_id
    joint["phase"] = "stopped"
    assert app.status()["pending_id"] is None


def test_http_panel_reads_only_and_rejects_cross_origin_writes(panel):
    app, transport = panel
    server = tool.make_server(app, port=0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    opener = build_opener(ProxyHandler({}))
    try:
        with opener.open(base) as response:
            html = response.read().decode()
            assert app.token in html and "左臂" in html and "右臂" in html
        with opener.open(base + "/api/state") as response:
            assert json.load(response)["connected"]
        assert not transport.posts
        for headers in ({}, {"X-Panel-Token": app.token, "Origin": "http://other-site"}):
            request = Request(
                base + "/api/move",
                data=json.dumps(intent()).encode(),
                headers={"Content-Type": "application/json", **headers},
            )
            with pytest.raises(HTTPError) as error:
                opener.open(request)
            assert error.value.code == 403
        request = Request(
            base + "/api/move",
            data=json.dumps(intent()).encode(),
            headers={"Content-Type": "application/json", "X-Panel-Token": app.token},
        )
        with opener.open(request) as response:
            assert json.load(response)["accepted"]
        assert len(transport.posts) == 1
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
