import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    "alignment_ack", ROOT / "tools/grasp/commission_visual_alignment.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.mark.parametrize(
    "ids,exception",
    [
        (["old", "old", "new"], None),
        (["old", "other"], RuntimeError),
        (["old"] * 100, TimeoutError),
    ],
)
def test_ack_waits_only_for_previous_status(ids, exception):
    values = iter(ids)
    now = [0.0]
    joints = SimpleNamespace(
        snapshot=lambda: {"arm_startup": {"joint_control": {"request_id": next(values)}}}
    )

    def sleep(dt):
        now[0] += dt

    def call():
        module.wait_hold_acknowledgement(joints, "new", "old", clock=lambda: now[0], sleep=sleep)

    if exception:
        with pytest.raises(exception):
            call()
    else:
        call()
