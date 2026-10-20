#!/usr/bin/env python3
"""Capture portable package/version facts, not credentials or environment values."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import platform
import subprocess


def run(args, json_output=False):
    try:
        p = subprocess.run(args, capture_output=True, text=True, timeout=120)
    except (subprocess.TimeoutExpired, FileNotFoundError) as error:
        return {'available': False, 'reason': type(error).__name__}
    if p.returncode:
        return {'available': False, 'exit_code': p.returncode}
    return json.loads(p.stdout) if json_output else p.stdout.strip()


def capture(output):
    output.mkdir(parents=True, exist_ok=False)
    facts = {'created_utc': datetime.now(timezone.utc).isoformat(), 'os': platform.platform(),
             'system_python': run(['/usr/bin/python3','--version']),
             'gpu': run(['nvidia-smi','--query-gpu=name,driver_version,memory.total','--format=csv,noheader']),
             'ros_distribution': 'humble', 'system_packages': run(['dpkg-query','-W','-f=${Package}\t${Version}\n']),
             'system_python_packages': run(['/usr/bin/python3','-m','pip','list','--format=json'], True),
             'notes': ['Package names/versions only; no shell environment or account settings exported.',
                       'Conda environment binaries outside this repository are NOT included.',
                       'final-dev handoff excludes outputs/surface_runtime; dependencies must be installed again.']}
    conda_prefix = Path('/home/subin/anaconda3/envs/teamkai-yolo')
    if conda_prefix.exists():
        packages = [json.loads(p.read_text()) for p in sorted((conda_prefix/'conda-meta').glob('*.json'))]
        facts['teamkai_yolo_conda_packages'] = [{k:p.get(k) for k in ('name','version','build')} for p in packages]
        python = conda_prefix / 'bin/python'
        if python.exists():
            facts['teamkai_yolo_python_packages'] = run([str(python),'-s','-m','pip','list','--format=json'], True)
            facts['torch_cuda'] = run([str(python),'-s','-c',
                'import torch,json;print(json.dumps(dict(torch=torch.__version__,cuda=torch.version.cuda,available=torch.cuda.is_available(),device=torch.cuda.get_device_name(0) if torch.cuda.is_available() else None)))'], True)
    (output/'environment_versions.json').write_text(json.dumps(facts, ensure_ascii=False, indent=2)+'\n')
    print('ENVIRONMENT_VERSION_FACTS_CAPTURED', flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    capture(p.parse_args().output)
