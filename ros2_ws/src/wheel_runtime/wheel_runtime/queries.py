"""Read-only runtime descriptions. No node startup or hardware access."""
from dataclasses import asdict
from wheel_task_core.contracts import TransferJob
from wheel_task_core.coordinator import WORKFLOW


def readiness(app):
    return dict(navigation_ready=app.navigation.ready is True,
                vision_source_ready=app.vision.ready is True,
                motion_executor_ready=app.motion.ready is True,
                hardware_execution_implemented=False)


def workflow(config):
    return dict(job=asdict(TransferJob(**config['job'])), phases=WORKFLOW,
                hardware_commands_sent=False)
