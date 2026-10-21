"""Durable per-run events and atomic artifacts; standard library only."""
import contextlib
import datetime
import json
import os
from pathlib import Path
import time
import traceback


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    with temporary.open('w') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


class RunLog:
    def __init__(self, directory, component):
        self.directory = Path(directory)
        self.component = component

    def event(self, stage, status, **details):
        event = dict(run_id=self.directory.name, component=self.component,
                     stage=stage, status=status, utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                     monotonic_ns=time.monotonic_ns(), **details)
        with (self.directory / (self.component + '.events.jsonl')).open('a') as stream:
            stream.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + '\n')
            stream.flush()
            os.fsync(stream.fileno())
        print(json.dumps(event, ensure_ascii=False), flush=True)

    @contextlib.contextmanager
    def stage(self, name, **details):
        start = time.monotonic()
        self.event(name, 'started', **details)
        try:
            yield
        except BaseException as exc:
            error = dict(stage=name, component=self.component, error_type=type(exc).__name__,
                         error=str(exc), traceback=traceback.format_exc(), **details)
            error_path = self.directory / (self.component + '.error.json')
            if not error_path.exists():
                write_json(error_path, error)
            self.event(name, 'failed', duration_s=time.monotonic()-start, **error_without_stage(error))
            raise
        else:
            self.event(name, 'completed', duration_s=time.monotonic()-start, **details)


def error_without_stage(error):
    return {k: v for k, v in error.items() if k not in ('stage', 'component')}
