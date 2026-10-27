"""Atomic command journal and interprocess lock for one navigation owner."""
from contextlib import contextmanager
import json
import os
from pathlib import Path
import threading


class Journal:
    def __init__(self, path):
        self.path = Path(path)
        self.thread_lock = threading.RLock()

    @contextmanager
    def transaction(self):
        import fcntl  # Robot/backend deployment is Linux; no SDK file is opened.
        with self.thread_lock:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.with_suffix('.lock').open('a') as guard:
                fcntl.flock(guard, fcntl.LOCK_EX)
                try:
                    data = json.loads(self.path.read_text(encoding='utf-8')) if self.path.exists() else {'active':None, 'commands':{}}
                    if not isinstance(data.get('commands'), dict):
                        raise ValueError('Invalid navigation journal')
                    before = json.dumps(data, sort_keys=True)
                    yield data
                    if before != json.dumps(data, sort_keys=True):
                        self.save(data)
                finally:
                    fcntl.flock(guard, fcntl.LOCK_UN)

    def save(self, data):
        # Called before a motion POST as well as on transaction completion.
        temporary = self.path.with_suffix('.tmp')
        with temporary.open('w', encoding='utf-8') as out:
            json.dump(data, out, ensure_ascii=False, allow_nan=False)
            out.flush()
            os.fsync(out.fileno())
        os.replace(temporary, self.path)
