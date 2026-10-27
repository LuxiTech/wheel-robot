"""Discover first-party source packages for uninstalled developer tools."""
from pathlib import Path
import sys


def configure(root=None):
    root = Path(root) if root else Path(__file__).resolve().parents[1]
    paths = []
    for parent in (root / 'packages', root / 'ros2_ws/src', root / 'apps'):
        for folder in sorted(parent.iterdir()):
            if folder.is_dir() and (folder / folder.name / '__init__.py').is_file():
                paths.append(str(folder))
    for path in reversed(paths):
        if path not in sys.path:
            sys.path.insert(0, path)
    return paths
