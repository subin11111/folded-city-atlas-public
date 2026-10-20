#!/usr/bin/env python3
"""Reproduce audited matched-support dense exports for both BA-PGO candidates."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src'))
from export_floor2_local_clouds import export_variant


if __name__=='__main__':
    run=Path(__file__).resolve().parent
    with ThreadPoolExecutor(max_workers=2) as executor:
        jobs=[executor.submit(export_variant,run,run/'clouds',variant)
              for variant in ('integration_nominal','integration_weak')]
        for job in jobs:
            job.result()
