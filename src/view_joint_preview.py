#!/usr/bin/env python3
"""Show the explicitly unvalidated lobby + floor2 candidate in RViz, not a final map.

No transforms are estimated here: all clouds and paths already use the candidate
joint poses. Unconnected sessions are deliberately excluded. ROS imports are lazy
so config generation and safety checks can be tested without sourcing ROS.
"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import yaml


FRAME = 'map_preview'
PREFIX = '/folded_city/unvalidated_preview'
SESSIONS = ('lobby', 'floor2')
COLORS = {'lobby': '255; 150; 40', 'floor2': '45; 210; 255'}
DEFAULT_COMPONENT = Path(
    'outputs/student_union_20261017_revalidated/joint_disjoint_hypothesis/component_0'
)


def load_preview(component: Path) -> dict:
    """Refuse accidental final-map use or inclusion of unconnected sessions."""
    component = component.resolve()
    target = component / 'preview_clouds'
    manifest_path = target / 'export_manifest.json'
    manifest = json.loads(manifest_path.read_text())
    if manifest.get('unvalidated_preview') is not True:
        raise ValueError('Manifest must explicitly contain unvalidated_preview=true')
    if manifest.get('accepted_by_validation') is not False or manifest.get('navigation_ready') is not False:
        raise ValueError('Preview manifest must retain accepted_by_validation=false and navigation_ready=false')
    if set(manifest.get('component_sessions', [])) != set(SESSIONS):
        raise ValueError('This preview only supports lobby + floor2, no disconnected maps')
    report = json.loads((component / 'joint_report.json').read_text())
    if report.get('accepted') is not False:
        raise ValueError('Expected an unchanged rejected component report (accepted=false)')
    if set(report.get('sessions', [])) != set(SESSIONS):
        raise ValueError('Component report must contain exactly lobby + floor2')
    if not manifest.get('finished_utc'):
        raise ValueError('Cloud export has not completed')
    required = [target / 'combined_candidate.ply']
    for name in SESSIONS:
        required.extend([target / f'{name}_candidate.ply',
                         component / name / 'candidate_trajectory.csv'])
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError('Missing preview outputs: ' + ', '.join(missing))
    return {'component': component, 'target': target, 'manifest': manifest,
            'report': report, 'required': required}


def cloud_bounds(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Read the exact binary XYZRGB export layout, reject truncated/nonfinite data."""
    with path.open('rb') as stream:
        if stream.readline().strip() != b'ply':
            raise ValueError('Not a PLY file')
        count = None
        properties = []
        format_ok = False
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('Incomplete PLY header')
            line = line.decode('ascii').strip()
            if line.startswith('format '):
                format_ok = line == 'format binary_little_endian 1.0'
            elif line.startswith('element vertex '):
                count = int(line.split()[-1])
            elif line.startswith('element '):
                raise ValueError('Expected a vertex-only XYZRGB PLY')
            elif line.startswith('property '):
                properties.append(line)
            elif line == 'end_header':
                break
        expected = ['property float x', 'property float y', 'property float z',
                    'property uchar red', 'property uchar green', 'property uchar blue']
        if not format_ok or properties != expected or count is None or count <= 0:
            raise ValueError('Expected a nonempty binary little-endian XYZRGB PLY')
        dtype = [('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                 ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')]
        cloud = np.fromfile(stream, dtype=dtype, count=count)
    if len(cloud) != count:
        raise ValueError('Truncated PLY data')
    low, high = [], []
    for axis in ('x', 'y', 'z'):
        if not np.isfinite(cloud[axis]).all():
            raise ValueError('Nonfinite PLY coordinates')
        low.append(float(cloud[axis].min()))
        high.append(float(cloud[axis].max()))
    return np.asarray(low), np.asarray(high)


def build_config(template: dict, low: np.ndarray, high: np.ndarray) -> dict:
    config = copy.deepcopy(template)
    manager = config['Visualization Manager']
    manager['Global Options']['Fixed Frame'] = FRAME
    templates = manager['Displays']
    cloud_template = next(d for d in templates if d.get('Class') == 'rviz_default_plugins/PointCloud2')
    path_template = next(d for d in templates if d.get('Class') == 'rviz_default_plugins/Path')
    displays = []
    merged = copy.deepcopy(cloud_template)
    merged.update(Name='UNVALIDATED PREVIEW | lobby + floor2 RGB',
                  Enabled=True, Value=True, Alpha=1.0)
    merged['Color Transformer'] = 'RGB8'
    merged['Topic']['Value'] = PREFIX + '/cloud_merged'
    displays.append(merged)
    for name in SESSIONS:
        cloud = copy.deepcopy(cloud_template)
        cloud.update(Name=f'{name} candidate (tint)', Enabled=False, Value=False, Alpha=0.9)
        cloud['Color Transformer'] = 'FlatColor'
        cloud['Color'] = COLORS[name]
        cloud['Topic']['Value'] = PREFIX + '/cloud_' + name
        displays.append(cloud)
        trajectory = copy.deepcopy(path_template)
        trajectory.update(Name=f'{name} candidate trajectory', Enabled=True, Value=True,
                          Color=COLORS[name], Offset={'X': 0, 'Y': 0, 'Z': 0})
        trajectory['Line Width'] = 0.04
        trajectory['Topic']['Value'] = PREFIX + '/trajectory_' + name
        displays.append(trajectory)
    displays.append({
        'Class': 'rviz_default_plugins/Marker',
        'Name': 'UNVALIDATED PREVIEW - NOT FOR NAVIGATION',
        'Enabled': True, 'Value': True,
        'Namespaces': {'unvalidated_preview': True},
        'Topic': {'Value': PREFIX + '/warning', 'Depth': 1,
                  'Durability Policy': 'Transient Local', 'History Policy': 'Keep Last',
                  'Reliability Policy': 'Reliable', 'Filter size': 10},
    })
    manager['Displays'] = displays
    focus = (low + high) / 2
    view = manager['Views']['Current']
    view['Focal Point'] = dict(zip(('X', 'Y', 'Z'), map(float, focus)))
    view['Distance'] = max(10.0, float(np.linalg.norm(high - low)) * 1.5)
    view['Pitch'] = 0.66
    view['Yaw'] = 4.1
    view['Target Frame'] = '<Fixed Frame>'
    for panel in config.get('Panels', []):
        if panel.get('Class') == 'rviz_common/Displays':
            panel['Property Tree Widget']['Expanded'] = ['/Global Options1']
    return config


def run_publisher(preview: dict, low: np.ndarray, high: np.ndarray) -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Path as PathMessage
    from visualization_msgs.msg import Marker
    from rviz_cloud_publisher import read_colored_ply, point_cloud_message, trajectory_message

    class PreviewPublisher(Node):
        def __init__(self):
            super().__init__('folded_city_unvalidated_joint_preview')
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.messages = []
            stamp = self.get_clock().now().to_msg()
            for name in ('merged',) + SESSIONS:
                filename = 'combined_candidate.ply' if name == 'merged' else f'{name}_candidate.ply'
                cloud = read_colored_ply(preview['target'] / filename)
                self.messages.append((self.create_publisher(PointCloud2, PREFIX + '/cloud_' + name, qos),
                                      point_cloud_message(cloud, FRAME, stamp)))
                self.get_logger().info(f'{name}: {len(cloud):,} points in {FRAME}')
            for name in SESSIONS:
                self.messages.append((self.create_publisher(PathMessage, PREFIX + '/trajectory_' + name, qos),
                                      trajectory_message(preview['component'] / name / 'candidate_trajectory.csv', FRAME, stamp)))
            warning = Marker()
            warning.header.frame_id = FRAME
            warning.header.stamp = stamp
            warning.ns = 'unvalidated_preview'
            warning.id = 0
            warning.type = Marker.TEXT_VIEW_FACING
            warning.action = Marker.ADD
            warning.pose.orientation.w = 1.0
            warning.pose.position.x = float((low[0] + high[0]) / 2)
            warning.pose.position.y = float((low[1] + high[1]) / 2)
            warning.pose.position.z = float(high[2] + 1)
            warning.scale.z = max(.35, min(.9, float(np.linalg.norm(high - low)) * .018))
            warning.color.r, warning.color.g, warning.color.b, warning.color.a = 1.0, .45, .1, 1.0
            warning.text = 'UNVALIDATED PREVIEW: lobby + floor2\nNot for navigation; other sessions excluded'
            self.messages.append((self.create_publisher(Marker, PREFIX + '/warning', qos), warning))
            self.timer = self.create_timer(10.0, self.publish_all)
            self.publish_all()
            self.get_logger().warning('READY: UNVALIDATED PREVIEW; joint_report accepted=false is unchanged')

        def publish_all(self):
            stamp = self.get_clock().now().to_msg()
            for publisher, message in self.messages:
                message.header.stamp = stamp
                publisher.publish(message)

    rclpy.init()
    node = PreviewPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('component', nargs='?', type=Path, default=DEFAULT_COMPONENT)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--config-only', action='store_true')
    mode.add_argument('--publisher-only', action='store_true')
    args = parser.parse_args(argv)
    preview = load_preview(args.component)
    low, high = cloud_bounds(preview['target'] / 'combined_candidate.ply')
    if args.publisher_only:
        run_publisher(preview, low, high)
        return
    workspace = Path(__file__).resolve().parents[1]
    template = yaml.safe_load((workspace / 'config/folded_city.rviz').read_text())
    config = build_config(template, low, high)
    config_path = preview['target'] / 'UNVALIDATED_PREVIEW_lobby_floor2.rviz'
    config_path.write_text(yaml.safe_dump(config, sort_keys=False))
    if args.config_only:
        print(config_path)
        return
    command = [sys.executable, str(Path(__file__).resolve()), str(preview['component']), '--publisher-only']
    log_path = preview['target'] / 'rviz_preview_publisher.log'
    with log_path.open('w') as log:
        publisher = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        try:
            # Surface startup failures instead of opening a misleading empty map window.
            deadline = time.monotonic() + 45
            while 'READY: UNVALIDATED PREVIEW' not in log_path.read_text():
                if publisher.poll() is not None:
                    raise RuntimeError('Preview publisher failed; see ' + str(log_path))
                if time.monotonic() >= deadline:
                    raise RuntimeError('Preview publisher timed out; see ' + str(log_path))
                time.sleep(.2)
            subprocess.run(['rviz2', '-d', str(config_path)], check=True)
        finally:
            if publisher.poll() is None:
                publisher.terminate()
                try:
                    publisher.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    publisher.kill()
                    publisher.wait(timeout=5)


if __name__ == '__main__':
    main()
