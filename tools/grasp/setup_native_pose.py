#!/usr/bin/env python3
"""Build pinned pose extensions inside the native vision venv, without sudo."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import sysconfig

ROOT=Path(__file__).resolve().parents[2]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cuda-home',required=True,help='Toolkit with bin/nvcc, include, lib64 (symlink layout is allowed)')
    p.add_argument('--cc',required=True,help='CUDA 12.8 compatible C compiler')
    p.add_argument('--cxx',required=True,help='CUDA 12.8 compatible C++ compiler')
    p.add_argument('--headers',required=True,help='Extracted sysroot/usr/include, containing eigen3/Eigen and boost')
    p.add_argument('--uv',default='uv')
    args=p.parse_args()
    if Path(sys.prefix).resolve() != ROOT/'.runtime/vision/venv':
        raise RuntimeError('Run with .runtime/vision/venv/bin/python')
    os.chdir(ROOT)
    headers=Path(args.headers).resolve();cuda=Path(args.cuda_home).resolve()
    for path in (headers/'eigen3/Eigen/Core',headers/'boost/assign.hpp',cuda/'bin/nvcc'):
        if not path.exists():raise FileNotFoundError(path)
    def run(command,**kwargs):subprocess.run(command,check=True,**kwargs)
    run([sys.executable,'tools/grasp/bootstrap_sources.py','--lock','config/deployments/native-pose.sources.lock.json'])
    run([args.uv,'pip','install','--python',sys.executable,'-r','tools/grasp/requirements-native-pose.txt'])
    env=os.environ.copy()
    env.update(CUDA_HOME=str(cuda),CC=args.cc,CXX=args.cxx,MAX_JOBS='4',CUB_HOME=str(cuda/'include'))
    import torch
    major,minor=torch.cuda.get_device_capability(0)
    env['TORCH_CUDA_ARCH_LIST']=f'{major}.{minor}'
    env['PATH']=str(Path(sys.prefix)/'bin')+':'+env['PATH']
    env['CPATH']=':'.join(str(x) for x in (Path(sysconfig.get_paths()['purelib'])/'nvidia').glob('*/include'))
    run([args.uv,'pip','install','--python',sys.executable,'--no-build-isolation','./third_party/pytorch3d','./third_party/nvdiffrast'],env=env)
    import pybind11
    target=Path(sysconfig.get_paths()['purelib'])/('mycpp'+sysconfig.get_config_var('EXT_SUFFIX'))
    run([args.cxx,'-O3','-shared','-std=c++17','-fPIC','-fopenmp','-I'+pybind11.get_include(),
         '-I'+sysconfig.get_paths()['include'],'-I'+str(headers),'-I'+str(headers/'eigen3'),
         '-Ithird_party/FoundationPose/mycpp/include','third_party/FoundationPose/mycpp/src/Utils.cpp',
         'third_party/FoundationPose/mycpp/src/app/pybind_api.cpp','-o',str(target)])
    link=ROOT/'third_party/FoundationPose/weights';weights=ROOT/'assets/models/FoundationPose'
    if not link.exists() and not link.is_symlink():link.symlink_to(weights,target_is_directory=True)
    if link.resolve()!=weights.resolve():raise RuntimeError('Unexpected FoundationPose weights path: '+str(link))
    # Persist compiler/cache settings for nvdiffrast's first-use CUDA JIT.
    runtime={k:env[k] for k in ('CUDA_HOME','CC','CXX','CPATH','TORCH_CUDA_ARCH_LIST','MAX_JOBS')}
    (ROOT/'.runtime/vision/native-build-env.json').write_text(json.dumps(runtime,indent=2)+'\n')
    with (ROOT/'.runtime/vision/installed.txt').open('w') as out:
        run([args.uv,'pip','freeze','--python',sys.executable],stdout=out)


if __name__=='__main__':main()
