#!/usr/bin/env python3
"""Display explicitly exploratory session placements without approving a map.

All transforms must already be baked into the bundle's clouds and trajectories.
This viewer estimates no transforms, changes no graph reports, and publishes no
TF connection to the operational map frame. ROS imports are deliberately lazy.
"""
from __future__ import annotations

import argparse
import copy
import csv
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import yaml

from view_joint_preview import cloud_bounds


FRAME = 'map_all_preview'
PREFIX = '/folded_city/exploratory_all_preview'
STATUS = 'EXPLORATORY_ALL_SESSION_PREVIEW_NOT_FOR_NAVIGATION'
SESSION_ORDER = ('lobby', 'floor2', 'bridge_out1f', 'outside', 'bridge_12_sub')
COLORS = {
    'lobby': '255; 150; 40',
    'floor2': '45; 210; 255',
    'bridge_out1f': '240; 80; 180',
    'outside': '95; 235; 100',
    'bridge_12_sub': '170; 130; 255',
}
DEFAULT_MANIFEST = Path('outputs/student_union_20261017_all_preview/bundle/manifest.json')


def bundle_file(bundle: Path, name: object) -> Path:
    """Resolve only existing bundle-relative files, including symlink checking."""
    if not isinstance(name, str) or not name or Path(name).is_absolute():
        raise ValueError('Bundle files must use nonempty relative paths')
    path = (bundle / name).resolve()
    if not path.is_relative_to(bundle.resolve()):
        raise ValueError('Bundle file escapes its bundle directory')
    if not path.is_file():
        raise FileNotFoundError(path)
    return path


def validate_trajectory(path: Path) -> int:
    """Reject empty/invalid paths before RViz can show misleading partial data."""
    fields = ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')
    with path.open(newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        if not set(fields).issubset(reader.fieldnames or []):
            raise ValueError(f'Trajectory is missing pose columns: {path}')
        count = 0
        for row in reader:
            pose = np.array([float(row[field]) for field in fields])
            if not np.isfinite(pose).all():
                raise ValueError(f'Nonfinite trajectory pose: {path}')
            if abs(float(np.linalg.norm(pose[3:])) - 1.0) > .01:
                raise ValueError(f'Trajectory quaternion is not unit length: {path}')
            count += 1
    if not count:
        raise ValueError(f'Trajectory has no poses: {path}')
    return count


def load_preview(manifest_path: Path) -> dict:
    manifest_path = manifest_path.resolve()
    manifest = json.loads(manifest_path.read_text())
    if manifest.get('status') != STATUS:
        raise ValueError('Expected explicitly exploratory, not-for-navigation status')
    if manifest.get('unvalidated_preview') is not True:
        raise ValueError('Manifest must contain unvalidated_preview=true')
    if manifest.get('accepted_by_validation') is not False or manifest.get('navigation_ready') is not False:
        raise ValueError('Preview must retain accepted_by_validation=false and navigation_ready=false')
    if manifest.get('frame') != FRAME:
        raise ValueError(f'Bundle frame must be {FRAME}; operational map TF is not supported')
    if not manifest.get('finished_utc'):
        raise ValueError('Bundle export has not completed')
    records = manifest.get('sessions')
    if not isinstance(records, dict) or not 3 <= len(records) <= 5:
        raise ValueError('Expected 3 to 5 explicitly placed sessions')
    if not set(records).issubset(SESSION_ORDER) or not {'lobby', 'floor2'}.issubset(records):
        raise ValueError('Expected lobby + floor2 and supported additional sessions')
    bundle = manifest_path.parent
    sessions = {}
    for name in SESSION_ORDER:
        if name not in records:
            continue
        entry = records[name]
        if not isinstance(entry, dict) or not isinstance(entry.get('placement'), str) or not entry['placement'].strip():
            raise ValueError(f'Session {name} must identify its exploratory placement')
        cloud = bundle_file(bundle, entry.get('cloud'))
        trajectory = bundle_file(bundle, entry.get('trajectory'))
        validate_trajectory(trajectory)
        sessions[name] = {'cloud': cloud, 'trajectory': trajectory, 'placement': entry['placement']}
    return {'manifest_path': manifest_path, 'bundle': bundle, 'manifest': manifest,
            'sessions': sessions, 'combined_cloud': bundle_file(bundle, manifest.get('combined_cloud'))}


def identity_transform_spec() -> dict:
    """Only a local display TF; never join this preview to map/odom/base_link."""
    return {'parent': FRAME, 'child': 'preview_axes',
            'translation': (0.0, 0.0, 0.0), 'rotation': (0.0, 0.0, 0.0, 1.0)}


def build_config(template: dict, low: np.ndarray, high: np.ndarray, sessions) -> dict:
    names = tuple(sessions)
    if not names or not set(names).issubset(SESSION_ORDER):
        raise ValueError('Unsupported preview session list')
    low, high = np.asarray(low), np.asarray(high)
    if low.shape != (3,) or high.shape != (3,) or not np.isfinite([low, high]).all() or np.any(low > high):
        raise ValueError('Invalid cloud bounds')
    config = copy.deepcopy(template)
    manager = config['Visualization Manager']
    manager['Global Options']['Fixed Frame'] = FRAME
    cloud_template = next(d for d in manager['Displays'] if d.get('Class') == 'rviz_default_plugins/PointCloud2')
    path_template = next(d for d in manager['Displays'] if d.get('Class') == 'rviz_default_plugins/Path')
    merged = copy.deepcopy(cloud_template)
    merged.update(Name='UNVALIDATED ALL-SESSION PREVIEW | merged RGB', Enabled=True, Value=True, Alpha=1.0)
    merged['Color Transformer'] = 'RGB8'
    merged['Topic']['Value'] = PREFIX + '/cloud_merged'
    displays = [merged]
    for name in names:
        cloud = copy.deepcopy(cloud_template)
        cloud.update(Name=f'{name} exploratory placement (tint)', Enabled=False, Value=False,
                     Alpha=.9, Color=COLORS[name])
        cloud['Color Transformer'] = 'FlatColor'
        cloud['Topic']['Value'] = PREFIX + '/cloud_' + name
        displays.append(cloud)
        path = copy.deepcopy(path_template)
        path.update(Name=f'{name} exploratory trajectory', Enabled=True, Value=True,
                    Color=COLORS[name], Offset={'X': 0, 'Y': 0, 'Z': 0})
        path['Line Width'] = .04
        path['Topic']['Value'] = PREFIX + '/trajectory_' + name
        displays.append(path)
    displays.append({
        'Class': 'rviz_default_plugins/Marker',
        'Name': 'Optional warning label - NOT FOR NAVIGATION',
        'Enabled': False, 'Value': False,
        'Namespaces': {'exploratory_all_preview': True},
        'Topic': {'Value': PREFIX + '/warning', 'Depth': 1,
                  'Durability Policy': 'Transient Local', 'History Policy': 'Keep Last',
                  'Reliability Policy': 'Reliable', 'Filter size': 10},
    })
    manager['Displays'] = displays
    view = manager['Views']['Current']
    view['Focal Point'] = dict(zip(('X', 'Y', 'Z'), map(float, (low + high) / 2)))
    view['Distance'] = max(10.0, float(np.linalg.norm(high - low)) * 1.35)
    view['Pitch'] = .72
    view['Yaw'] = 4.1
    view['Target Frame'] = '<Fixed Frame>'
    for panel in config.get('Panels', []):
        if panel.get('Class') == 'rviz_common/Displays':
            panel['Property Tree Widget']['Expanded'] = ['/Global Options1']
    return config


def run_publisher(preview: dict, low: np.ndarray, high: np.ndarray) -> None:
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Path as PathMessage
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from tf2_ros import StaticTransformBroadcaster
    from visualization_msgs.msg import Marker
    from rviz_cloud_publisher import read_colored_ply, point_cloud_message, trajectory_message

    class PreviewPublisher(Node):
        def __init__(self):
            super().__init__('folded_city_exploratory_all_preview')
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.messages = []
            stamp = self.get_clock().now().to_msg()
            # Keep the preview isolated. This TF only gives RViz a known root.
            self.tf_broadcaster = StaticTransformBroadcaster(self)
            spec = identity_transform_spec()
            transform = TransformStamped()
            transform.header.frame_id = spec['parent']
            transform.header.stamp = stamp
            transform.child_frame_id = spec['child']
            transform.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(transform)
            sources = {'merged': preview['combined_cloud']}
            sources.update({name: entry['cloud'] for name, entry in preview['sessions'].items()})
            for name, source in sources.items():
                cloud_bounds(source)  # Validate layout, completeness and finite XYZ.
                cloud = read_colored_ply(source)
                self.messages.append((self.create_publisher(PointCloud2, PREFIX + '/cloud_' + name, qos),
                                      point_cloud_message(cloud, FRAME, stamp)))
                self.get_logger().info(f'{name}: {len(cloud):,} points in isolated frame {FRAME}')
            for name, entry in preview['sessions'].items():
                self.messages.append((self.create_publisher(PathMessage, PREFIX + '/trajectory_' + name, qos),
                                      trajectory_message(entry['trajectory'], FRAME, stamp)))
            warning = Marker()
            warning.header.frame_id = FRAME
            warning.header.stamp = stamp
            warning.ns = 'exploratory_all_preview'
            warning.id = 0
            warning.type = Marker.TEXT_VIEW_FACING
            warning.action = Marker.ADD
            warning.pose.orientation.w = 1.0
            warning.pose.position.x = float((low[0] + high[0]) / 2)
            warning.pose.position.y = float((low[1] + high[1]) / 2)
            warning.pose.position.z = float(high[2] + .5)
            warning.scale.z = .3
            warning.color.r, warning.color.g, warning.color.b, warning.color.a = 1.0, .45, .1, 1.0
            warning.text = 'EXPLORATORY ALIGNMENT - NOT FOR NAVIGATION'
            self.messages.append((self.create_publisher(Marker, PREFIX + '/warning', qos), warning))
            self.timer = self.create_timer(10.0, self.publish_all)
            self.publish_all()
            self.get_logger().warning('READY: EXPLORATORY ALL-SESSION PREVIEW; all validation flags remain false')

        def publish_all(self):
            stamp = self.get_clock().now().to_msg()
            for publisher, message in self.messages:
                message.header.stamp = stamp
                publisher.publish(message)

    rclpy.init()
    node = None
    try:
        node = PreviewPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', nargs='?', type=Path, default=DEFAULT_MANIFEST)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--config-only', action='store_true')
    mode.add_argument('--publisher-only', action='store_true')
    args = parser.parse_args(argv)
    preview = load_preview(args.manifest)
    low, high = cloud_bounds(preview['combined_cloud'])
    if args.publisher_only:
        run_publisher(preview, low, high)
        return
    workspace = Path(__file__).resolve().parents[1]
    template = yaml.safe_load((workspace / 'config/folded_city.rviz').read_text())
    config = build_config(template, low, high, preview['sessions'])
    config_path = preview['bundle'] / 'EXPLORATORY_ALL_SESSIONS.rviz'
    config_path.write_text(yaml.safe_dump(config, sort_keys=False))
    if args.config_only:
        print(config_path)
        return
    command = [sys.executable, str(Path(__file__).resolve()), str(preview['manifest_path']), '--publisher-only']
    log_path = preview['bundle'] / 'rviz_all_preview_publisher.log'
    with log_path.open('w') as log:
        publisher = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 45
            while 'READY: EXPLORATORY ALL-SESSION PREVIEW' not in log_path.read_text():
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
