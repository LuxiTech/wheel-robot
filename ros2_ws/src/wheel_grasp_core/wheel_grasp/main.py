"""Hardware application entry point: configure adapters, then start a task.

No simulation fallback and no implicit robot enable. The default adapters are
explicit integration boundaries until hardware feedback/control is implemented.
"""
import argparse
import asyncio
from dataclasses import asdict
import json
from pathlib import Path
import uuid

import yaml

from wheel_grasp.contracts import TransferJob
from wheel_grasp.motion.executor import SDKMotionExecutor
from wheel_grasp.navigation.adapter import Nav2AprilTagNavigation
from wheel_grasp.task.coordinator import TaskCoordinator, WORKFLOW
from wheel_grasp.vision.service import UnconfiguredVision


def create_application(config):
    """Composition root: replace adapters here, not inside the task state machine."""
    return TaskCoordinator(Nav2AprilTagNavigation(), UnconfiguredVision(),
                           SDKMotionExecutor(), **config.get('coordinator', {}))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='config/task.yaml')
    parser.add_argument('command', choices=('status', 'workflow', 'run'))
    args = parser.parse_args(argv)
    config = yaml.safe_load(Path(args.config).read_text())
    job = TransferJob(**config['job'])
    app = create_application(config)
    if args.command == 'workflow':
        print(json.dumps(dict(job=asdict(job), phases=WORKFLOW,
                              hardware_commands_sent=False), ensure_ascii=False, indent=2))
        return 0
    if args.command == 'status':
        print(json.dumps(dict(navigation_ready=app.navigation.ready,
                              vision_source_ready=app.vision.ready,
                              motion_executor_ready=app.motion.ready,
                              hardware_execution_implemented=False), indent=2))
        return 0
    directory = Path(config.get('output_directory', 'outputs/robot')) / uuid.uuid4().hex
    directory.mkdir(parents=True, exist_ok=False)
    try:
        result = asyncio.run(app.run(job))
    except KeyboardInterrupt:
        report = dict(success=False, phase=app.phase, reason='operator_interrupt',
                      may_be_holding=app.may_be_holding, stop_errors=app.stop_errors)
        code = 130
    else:
        report = asdict(result)
        code = 0 if result.success else 2
    report.update(job=asdict(job), events=app.events)
    (directory/'result.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps(dict(report=report, directory=str(directory)), ensure_ascii=False, indent=2))
    return code


if __name__ == '__main__':
    raise SystemExit(main())
