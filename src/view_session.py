#!/usr/bin/env python3
"""Launch one independent session in RViz with all three trajectory sources."""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys

import yaml


def session_identifier(value):
    """One filesystem component that is also valid in ROS topic/frame names."""
    if re.fullmatch(r'[a-z][a-z0-9_]*', value) is None:
        raise argparse.ArgumentTypeError(
            'session must start with a lowercase letter and contain only lowercase letters, digits, or underscores'
        )
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=session_identifier,
                        help='Independent session identifier, e.g. outside, floor2, or bridge_12_sub')
    parser.add_argument('--run', type=Path, default=Path('outputs/student_union_20261017_reviewed'))
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--publisher-only', action='store_true')
    mode.add_argument('--config-only', action='store_true',
                      help='Generate comparison.rviz without starting a publisher or RViz')
    args = parser.parse_args(argv)
    workspace = Path(__file__).resolve().parents[1]
    run = args.run.resolve()
    root = run / args.session
    prefix = '/folded_city/' + args.session
    frame = args.session + '_map'
    required = [root / 'cloud_raw_filtered.ply', root / 'cloud_optimized_filtered.ply', root / 'cloud_app_filtered.ply',
                root / 'raw_main_trajectory.csv', root / 'optimized_main_trajectory.csv',
                run / 'audit' / args.session / 'app_optimized_trajectory.csv']
    if not all(path.is_file() for path in required):
        raise FileNotFoundError('Run processing first: ' + ', '.join(str(p) for p in required if not p.exists()))
    command = [sys.executable, str(workspace / 'src/rviz_cloud_publisher.py'),
               '--raw-cloud', str(required[0]), '--optimized-cloud', str(required[1]),
               '--reference-cloud', str(required[2]), '--raw-trajectory', str(required[3]),
               '--optimized-trajectory', str(required[4]), '--reference-trajectory', str(required[5]),
               '--frame', frame, '--topic-prefix', prefix]
    if args.publisher_only:
        os.execv(sys.executable, command)
    config = yaml.safe_load((workspace / 'config/folded_city.rviz').read_text())
    manager = config['Visualization Manager']
    manager['Global Options']['Fixed Frame'] = frame
    displays = manager['Displays']
    # Exclude obsolete flat-ground displays, which are not generated for multi-level scenes.
    displays[:] = [d for d in displays if d.get('Name') not in ('Optimized No-Ground', 'Optimized Ground')]
    for display in displays:
        if display.get('Class') == 'rviz_default_plugins/Path':
            # A visual Z offset would masquerade as a registration height error.
            display['Offset'] = {'X': 0, 'Y': 0, 'Z': 0}
        topic = display.get('Topic')
        if isinstance(topic, dict):
            topic['Value'] = topic['Value'].replace('/folded_city/', prefix + '/')
    cloud = copy.deepcopy(next(d for d in displays if d.get('Name') == 'Optimized Cloud'))
    cloud.update({'Name': 'App Saved Cloud', 'Enabled': False, 'Value': False})
    cloud['Topic']['Value'] = prefix + '/cloud_app'
    path = copy.deepcopy(next(d for d in displays if d.get('Name') == 'Optimized Trajectory'))
    path.update({'Name': 'App Saved Trajectory', 'Color': '40; 210; 120'})
    path['Topic']['Value'] = prefix + '/trajectory_app'
    displays.extend([cloud, path])
    summary = json.loads((root / 'export_optimized.json').read_text())
    low, high = summary['cloud_bounds_min_m'], summary['cloud_bounds_max_m']
    view = manager['Views']['Current']
    view['Focal Point'] = dict(zip(('X', 'Y', 'Z'), [(a + b) / 2 for a, b in zip(low, high)]))
    view['Distance'] = max(10.0, 1.6 * math.sqrt(sum((b - a)**2 for a, b in zip(low, high))))
    config_path = root / 'comparison.rviz'
    config_path.write_text(yaml.safe_dump(config, sort_keys=False))
    if args.config_only:
        print(config_path)
        return
    with (root / 'rviz_publisher.log').open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        try:
            subprocess.run(['rviz2', '-d', str(config_path)], check=True)
        finally:
            process.terminate()
            process.wait(timeout=10)


if __name__ == '__main__':
    main()
