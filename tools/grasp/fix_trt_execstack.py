#!/usr/bin/env python3
"""Clear TRT 10.8's executable-stack flag in this isolated venv only (glibc >=2.41)."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import sysconfig


def main():
    prefix=Path(sys.prefix).resolve()
    expected=Path(__file__).resolve().parents[2]/'.runtime/vision/venv'
    if prefix != expected.resolve(): raise RuntimeError('Run with .runtime/vision/venv/bin/python only')
    lib=Path(sysconfig.get_paths()['purelib'])/'tensorrt_libs/libnvinfer_builder_resource.so.10.8.0'
    if not lib.is_file(): raise FileNotFoundError(lib)
    patcher=prefix/'bin/patchelf'
    before=subprocess.check_output([str(patcher),'--print-execstack',str(lib)],text=True).strip()
    if before.endswith('-'):
        print('Already non-executable stack:',lib);return
    backup=prefix.parent/'libnvinfer_builder_resource.so.10.8.0.original'
    if backup.exists(): raise FileExistsError('Inspect existing backup before retrying: '+str(backup))
    shutil.copy2(lib,backup)
    temporary=lib.with_suffix(lib.suffix+'.patched')
    shutil.copy2(lib,temporary)  # detach uv cache hardlinks before modifying
    subprocess.run([str(patcher),'--clear-execstack',str(temporary)],check=True)
    after=subprocess.check_output([str(patcher),'--print-execstack',str(temporary)],text=True).strip()
    temporary.replace(lib)
    def sha(path):
        digest=hashlib.sha256()
        with path.open('rb') as stream:
            for chunk in iter(lambda:stream.read(1024*1024),b''): digest.update(chunk)
        return digest.hexdigest()
    record=dict(library=str(lib),backup=str(backup),before=before,after=after,
                original_sha256=sha(backup),patched_sha256=sha(lib))
    (prefix.parent/'trt-execstack-patch.json').write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps(record,indent=2))


if __name__=='__main__':main()
