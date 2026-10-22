#!/usr/bin/env python3
"""Restore locked sources without replacing locally modified checkouts."""
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]


def git(*args, cwd=ROOT):
    return subprocess.check_output(['git', *args], cwd=cwd, text=True).strip()


def main():
    lock = json.loads((ROOT/'config/grasp/dependencies.lock.json').read_text())
    for name, item in lock.items():
        if not isinstance(item, dict) or 'commit' not in item:
            continue
        directory = ROOT/'third_party'/name
        if not directory.exists():
            git('clone', item['url'], str(directory))
        if git('status', '--porcelain', '--untracked-files=no', cwd=directory):
            raise RuntimeError(f'Modified tracked source cannot be reproduced: {directory}')
        if git('rev-parse', 'HEAD', cwd=directory) != item['commit']:
            if git('status', '--porcelain', cwd=directory):
                raise RuntimeError(f'Refusing to replace modified source: {directory}')
            git('fetch', 'origin', item['commit'], cwd=directory)
            git('checkout', '--detach', item['commit'], cwd=directory)
        print(name, git('rev-parse', 'HEAD', cwd=directory))


if __name__ == '__main__':
    main()
