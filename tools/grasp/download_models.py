#!/usr/bin/env python3
"""Download explicitly locked model files and verify their SHA-256 before use."""
import argparse
import hashlib
import http.client
import json
from pathlib import Path
import shutil
import time
import urllib.error
import urllib.request

ROOT = Path(__file__).resolve().parents[2]


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


def download_url(url, temporary):
    """Resume interrupted HTTP transfers; final identity is checked by SHA-256."""
    for attempt in range(5):
        offset = temporary.stat().st_size if temporary.exists() else 0
        request = urllib.request.Request(url, headers={'Range': f'bytes={offset}-'} if offset else {})
        try:
            with urllib.request.urlopen(request, timeout=120) as response:
                append = offset > 0 and response.status == 206
                if response.status == 206 and not response.headers.get('Content-Range','').startswith(f'bytes {offset}-'):
                    raise ValueError('Server returned a mismatched byte range')
                length = response.headers.get('Content-Length')
                written = 0
                with temporary.open('ab' if append else 'wb') as out:
                    while True:
                        chunk = response.read(1024*1024)
                        if not chunk:
                            break
                        out.write(chunk)
                        written += len(chunk)
                if length is not None and written != int(length):
                    raise OSError('Incomplete model transfer')
            return
        except (OSError, urllib.error.URLError, http.client.IncompleteRead):
            if attempt == 4:
                raise
            time.sleep(min(8, 2**attempt))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--verify-only', action='store_true')
    args = parser.parse_args()
    manifest = json.loads((ROOT/'config/grasp/models.lock.json').read_text())
    for item in manifest['files']:
        path = ROOT/item['path']
        if path.exists():
            if digest(path) != item['sha256']:
                raise RuntimeError(f'Checksum mismatch; preserving file for inspection: {path}')
        elif args.verify_only:
            raise FileNotFoundError(path)
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary = path.with_suffix(path.suffix+'.partial')
            if temporary.exists() and digest(temporary) == item['sha256']:
                pass
            elif item['url'].startswith('gdrive:'):
                import gdown
                result = gdown.download(id=item['url'].split(':',1)[1], output=str(temporary), resume=True)
                if result is None:
                    raise RuntimeError(f'Download failed: {item["url"]}')
            else:
                download_url(item['url'], temporary)
            if digest(temporary) != item['sha256']:
                raise RuntimeError(f'Download checksum mismatch: {temporary}')
            temporary.replace(path)
        print('verified', item['path'], flush=True)
    # Build a portable, offline Hugging Face snapshot without resolving main.
    owl = manifest['owlvit']
    cache = ROOT/'assets/models/huggingface/hub/models--google--owlvit-base-patch32'
    snapshot = cache/'snapshots'/owl['revision']
    owl_files = [x for x in manifest['files'] if x['path'].startswith('assets/models/owlvit-base-patch32/')]
    if args.verify_only:
        if (cache/'refs/main').read_text().strip() != owl['revision']:
            raise RuntimeError('Offline OWL revision mismatch')
        for item in owl_files:
            if digest(snapshot/Path(item['path']).name) != item['sha256']:
                raise RuntimeError('Offline OWL cache checksum mismatch')
    else:
        snapshot.mkdir(parents=True, exist_ok=True)
        for item in owl_files:
            src = ROOT/item['path']
            dest = snapshot/src.name
            if dest.exists() and digest(dest) == item['sha256']:
                continue
            temporary = dest.with_suffix(dest.suffix+'.partial')
            shutil.copy2(src, temporary)
            temporary.replace(dest)
        (cache/'refs').mkdir(exist_ok=True)
        (cache/'refs/main').write_text(owl['revision'])
        (cache.parent/'version.txt').write_text('1')
    print('Model files verified; offline cache:', snapshot)


if __name__ == '__main__':
    main()
