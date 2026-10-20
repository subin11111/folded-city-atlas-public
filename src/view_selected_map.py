#!/usr/bin/env python3
"""View the user-designated working map on isolated ROS topics, not map/odom."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

from optimize_session import sha256_file
from view_joint_preview import cloud_bounds
import view_all_sessions_preview as legacy

ROOT = Path(__file__).resolve().parents[1]
FRAME = 'map_selected_working'
PREFIX = '/folded_city/selected_working'


def load_selected(manifest_path):
    manifest = json.loads(manifest_path.read_text())
    if manifest.get('user_designated_working_map') is not True or manifest.get('selection_fixed') is not True:
        raise ValueError('Not the explicitly selected working map')
    if manifest.get('topic_prefix') != PREFIX or manifest.get('frame') != FRAME:
        raise ValueError('Working-map namespace/frame differs')
    old = legacy.FRAME
    try:
        legacy.FRAME = FRAME
        preview = legacy.load_preview(manifest_path)
    finally:
        legacy.FRAME = old
    if sha256_file(preview['combined_cloud']) != manifest['combined_sha256']:
        raise ValueError('Combined map hash differs from the frozen release')
    for name, entry in manifest['sessions'].items():
        if entry['candidate'] != manifest['selection']['sessions'][name]['candidate']:
            raise ValueError('Candidate selection differs')
    return preview


def publish(preview):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Path as PathMessage
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from tf2_ros import StaticTransformBroadcaster
    from rviz_cloud_publisher import read_colored_ply, point_cloud_message, trajectory_message

    class SelectedMapPublisher(Node):
        def __init__(self):
            super().__init__('folded_city_selected_working_map')
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.messages = []
            stamp = self.get_clock().now().to_msg()
            self.broadcaster = StaticTransformBroadcaster(self)
            transform = TransformStamped(); transform.header.frame_id = FRAME
            transform.header.stamp = stamp; transform.child_frame_id = 'selected_working_axes'
            transform.transform.rotation.w = 1.; self.broadcaster.sendTransform(transform)
            paths = {'merged': preview['combined_cloud']} | {n: e['cloud'] for n, e in preview['sessions'].items()}
            for name, path in paths.items():
                cloud_bounds(path)
                self.messages.append((self.create_publisher(PointCloud2, PREFIX+'/cloud_'+name, qos),
                    point_cloud_message(read_colored_ply(path), FRAME, stamp)))
            for name, entry in preview['sessions'].items():
                self.messages.append((self.create_publisher(PathMessage, PREFIX+'/trajectory_'+name, qos),
                    trajectory_message(entry['trajectory'], FRAME, stamp)))
            self.timer = self.create_timer(10., self.publish_all)
            self.publish_all()
            self.get_logger().info('READY: SELECTED WORKING MAP; J04 preserved; navigation unvalidated')

        def publish_all(self):
            stamp = self.get_clock().now().to_msg()
            for publisher, message in self.messages:
                message.header.stamp = stamp; publisher.publish(message)

    rclpy.init(); node = None
    try:
        node = SelectedMapPublisher(); rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', nargs='?', type=Path)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--check-only', action='store_true')
    modes.add_argument('--publisher-only', action='store_true')
    args = parser.parse_args()
    manifest_path = args.manifest
    if manifest_path is None:
        pointer = json.loads((ROOT/'config/current_map.json').read_text())
        manifest_path = ROOT/pointer['manifest']
        if sha256_file(manifest_path) != pointer['manifest_sha256']:
            raise ValueError('Current working-map pointer hash mismatch')
    manifest_path = manifest_path.resolve(); preview = load_selected(manifest_path)
    config = legacy.bundle_file(preview['bundle'], preview['manifest']['rviz_config'])
    if args.check_only:
        print(json.dumps({'checked': True, 'sessions': len(preview['sessions']),
                          'manifest': str(manifest_path), 'rviz_config': str(config)})); return
    if args.publisher_only:
        publish(preview); return
    log_path = preview['bundle']/'selected_map_publisher.log'
    with log_path.open('a') as log:
        # Seek from this invocation, never trust a READY message from an old run.
        start_position = log_path.stat().st_size
        process = subprocess.Popen([sys.executable, str(Path(__file__)), str(manifest_path), '--publisher-only'],
                                   stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic()+45.
            while True:
                with log_path.open() as stream:
                    stream.seek(start_position); text = stream.read()
                if 'READY: SELECTED WORKING MAP' in text:
                    break
                if process.poll() is not None or time.monotonic()>deadline:
                    raise RuntimeError('Selected map publisher failed; inspect '+str(log_path))
                time.sleep(.2)
            subprocess.run(['rviz2', '-d', str(config)], check=True)
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait(timeout=5)


if __name__ == '__main__':
    main()
