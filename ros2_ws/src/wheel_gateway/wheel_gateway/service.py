"""Northbound read-only projection. Task execution remains owned by runtime."""
import json
from pathlib import Path
import yaml
from wheel_runtime.main import create_application
from wheel_runtime.queries import readiness, workflow


class RuntimeGateway:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.task_path = self.root / 'config/deployments/task.yaml'
        if not self.task_path.is_file():
            raise ValueError('Project root must contain config/deployments/task.yaml')

    def _configuration(self):
        return yaml.safe_load(self.task_path.read_text(encoding='utf-8'))

    def status(self):
        config = self._configuration()
        return dict(schema_version=1, read_only=True,
                    source='default_runtime_composition',
                    **readiness(create_application(config)))

    def workflow(self):
        return workflow(self._configuration())

    def configurations(self):
        # Catalog filenames only; no credentials, .env or arbitrary file reads.
        result = {}
        for group in ('robots', 'stations', 'recipes', 'deployments'):
            folder = self.root / 'config' / group
            result[group] = sorted(p.name for p in folder.iterdir()
                                   if p.is_file() and not p.is_symlink()
                                   and p.suffix in ('.yaml', '.json'))
        return result

    def runs(self, limit=30):
        # Fixed runtime output location. Never follow symlinks outside the project.
        folder = self.root / 'outputs/robot'
        if not folder.is_dir() or folder.is_symlink():
            return []
        rows = []
        for directory in folder.iterdir():
            if directory.is_symlink() or not directory.is_dir():
                continue
            if len(directory.name) != 32 or any(c not in '0123456789abcdef' for c in directory.name):
                continue
            report = directory / 'result.json'
            if not report.is_file() or report.is_symlink() or report.stat().st_size > 1024 * 1024:
                continue
            rows.append((report.stat().st_mtime, directory.name, report))
        result = []
        for _, run_id, report in sorted(rows, reverse=True)[:limit]:
            try:
                data = json.loads(report.read_text(encoding='utf-8'))
                if not isinstance(data, dict):
                    continue
                item = {key:data.get(key) for key in ('success','phase','reason','may_be_holding')}
                result.append(dict(run_id=run_id, **item))
            except (ValueError, OSError):
                continue
        return result
