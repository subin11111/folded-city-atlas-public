#!/usr/bin/env python3
"""Real ROS nodes executing DB replay, depth filtering, loop verification and PGO.

Not RTAB-Map live capture. No final-pose interpolation or artificial loop snap.
Backpressure preserves every batch; wall time and recorded time are separate.
"""
import argparse
import json
from pathlib import Path
import time
import uuid

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from optimize_session import sha256_file, utc_now, apply
from slam_replay_core import ORDER, ROOT, PrefixOptimizer, decode_rgbd, load_sessions
from rviz_cloud_publisher import point_cloud_message

PREFIX = '/folded_city_demo'
FRAME = 'recorded_session_local'
RUN_ID = None
QOS = QoSProfile(depth=4, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL)


def send(pub, value):
    pub.publish(String(data=json.dumps(dict(value, run_id=RUN_ID))))


class StageNode(Node):
    def __init__(self, name, input_topic=None, callback=None, output_topic=None):
        super().__init__(name, namespace='folded_city_demo')
        if input_topic:
            def receive(msg):
                packet=json.loads(msg.data)
                if packet.get('run_id') == RUN_ID:
                    callback(packet)
            self.sub = self.create_subscription(String, PREFIX+input_topic, receive, QOS)
        if output_topic:
            self.pub = self.create_publisher(String, PREFIX+output_topic, QOS)


def main():
    global RUN_ID
    RUN_ID = str(uuid.uuid4())
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--batch', type=int, default=40)
    p.add_argument('--solve-interval', type=int, default=160)
    p.add_argument('--max-evaluations', type=int, default=60)
    p.add_argument('--start-delay', type=float, default=12.)
    p.add_argument('--hold', type=float, default=120.)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    (a.output/'snapshots').mkdir(); (a.output/'photos').mkdir()
    (a.output/'source_snapshot').mkdir()
    for source_file in (Path(__file__), ROOT/'src/slam_replay_core.py'):
        (a.output/'source_snapshot'/source_file.name).write_bytes(source_file.read_bytes())
    sessions = load_sessions()
    backends = {n: PrefixOptimizer(s, a.solve_interval, a.max_evaluations) for n, s in sessions.items()}
    local = {n: {} for n in ORDER}; colours = {n: {} for n in ORDER}
    batches = [{'session': name, 'ids': s.ids[i:i+a.batch], 'session_last': i+a.batch >= len(s.ids)}
        for name, s in sessions.items() for i in range(0, len(s.ids), a.batch)]
    for i, b in enumerate(batches):
        b['sequence'] = i
    state = {'next': 0, 'inflight': False, 'done': False, 'snapshots': [], 'started': time.monotonic(), 'finish': None}
    rclpy.init()

    def ack(packet):
        if packet['sequence'] != state['next']-1:
            raise ValueError('Unexpected ACK; refusing silent frame loss')
        state['inflight'] = False

    source = StageNode('capture_db_replay', '/ack', ack, '/recorded_batch')
    rgb_pub = source.create_publisher(CompressedImage, PREFIX+'/source_rgb/compressed', QOS)

    def filter_depth(packet):
        s = sessions[packet['session']]; start = time.perf_counter()
        errors = []; count = 0
        for node in packet['ids']:
            try:
                xyz, rgb, bgr = decode_rgbd(s.row(node))
            except ValueError as error:
                errors.append({'node': node, 'reason': str(error)})
                xyz, rgb = np.empty((0, 3), np.float32), np.empty((0, 3), np.uint8)
            local[packet['session']][node], colours[packet['session']][node] = xyz, rgb
            count += len(xyz)
        image = s.connection.execute('SELECT image FROM Data WHERE id=?', (packet['ids'][-1],)).fetchone()[0]
        photo = cv2.imdecode(np.frombuffer(image, np.uint8), cv2.IMREAD_COLOR)
        photo_path = a.output/'photos'/f'{packet["sequence"]:04d}.jpg'
        cv2.imwrite(str(photo_path), cv2.resize(photo, (360, 480)))
        msg = CompressedImage(); msg.format = 'jpeg'; msg.data = photo_path.read_bytes()
        msg.header.stamp = source.get_clock().now().to_msg(); rgb_pub.publish(msg)
        packet.update(depth_points_added=count, depth_filter_seconds=time.perf_counter()-start,
                      depth_errors=errors, photo=str(photo_path.resolve()))
        send(depth.pub, packet)

    depth = StageNode('rgbd_confidence_filter', '/recorded_batch', filter_depth, '/filtered_batch')

    def validate(packet):
        b = backends[packet['session']]
        packet['loop_checks'], packet['loop_verification_seconds'] = b.verify_arrived(packet['ids'])
        send(verifier.pub, packet)

    verifier = StageNode('stored_loop_reverification', '/filtered_batch', validate, '/verified_constraints')

    def solve(packet):
        b = backends[packet['session']]
        b.extend(packet['ids'])
        before, packet['solver'] = b.solve(force=packet['session_last'])
        packet['before_poses'] = {str(n): v.tolist() for n, v in before.items()}
        packet['poses'] = {str(n): v.tolist() for n, v in b.poses.items()}
        send(optimizer.pub, packet)

    optimizer = StageNode('incremental_prefix_optimizer', '/verified_constraints', solve, '/optimized_prefix')
    cloud_pub = None; markers_pub = None

    def assemble(packet):
        name = packet['session']; b = backends[name]
        points = np.concatenate([apply(local[name][n], b.poses[n]) for n in b.poses])
        rgb = np.concatenate([colours[name][n] for n in b.poses])
        # A deterministic display budget only, never optimizer observations.
        stride = max(1, int(np.ceil(len(points)/180000)))
        shown, crgb = points[::stride].astype(np.float32), rgb[::stride]
        dtype = np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('red','u1'),('green','u1'),('blue','u1')])
        cloud = np.empty(len(shown), dtype=dtype)
        for i, key in enumerate(('x','y','z')): cloud[key] = shown[:, i]
        for i, key in enumerate(('red','green','blue')): cloud[key] = crgb[:, i]
        cloud_pub.publish(point_cloud_message(cloud, FRAME, assembler.get_clock().now().to_msg()))
        marker = Marker(); marker.header.frame_id = FRAME; marker.header.stamp = assembler.get_clock().now().to_msg()
        marker.ns = 'replay_scope'; marker.id = 0; marker.type = Marker.TEXT_VIEW_FACING; marker.action = Marker.ADD
        marker.pose.orientation.w = 1.; marker.pose.position.z = 7.; marker.scale.z = .7
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.
        marker.text = f'{name}: {len(b.seen)} frames | RECORDED REPLAY / not final v9'
        markers_pub.publish(MarkerArray(markers=[marker]))
        ids = sorted(b.poses)
        path = a.output/'snapshots'/f'{packet["sequence"]:04d}.npz'
        np.savez_compressed(path, points=shown, rgb=crgb, ids=np.array(ids),
            poses=np.array([b.poses[n] for n in ids]),
            before_poses=np.array([packet['before_poses'][str(n)] for n in ids]),
            raw_poses=np.array([sessions[name].raw[n] for n in ids]),
            loops=np.array([[e.a, e.b] for e in b.accepted], int).reshape(-1, 2))
        packet.pop('before_poses'); packet.pop('poses')
        packet.update(snapshot=str(path.resolve()), snapshot_sha256=sha256_file(path),
            seen_count=len(b.seen), displayed_points=len(shown),
            recorded_elapsed_s=sessions[name].stamps[packet['ids'][-1]]-sessions[name].stamps[sessions[name].ids[0]],
            wall_elapsed_s=time.monotonic()-state['started'])
        send(assembler.pub, packet)

    assembler = StageNode('prefix_cloud_assembler', '/optimized_prefix', assemble, '/map_snapshot')
    cloud_pub = assembler.create_publisher(PointCloud2, PREFIX+'/map_cloud', QOS)
    markers_pub = assembler.create_publisher(MarkerArray, PREFIX+'/scope', QOS)

    def record(packet):
        state['snapshots'].append(packet)
        (a.output/'events.json').write_text(json.dumps(state['snapshots'], indent=2)+'\n')
        print(json.dumps({k: packet[k] for k in ('sequence','session','seen_count','solver','recorded_elapsed_s')}, ensure_ascii=False), flush=True)
        send(recorder.pub, {'sequence': packet['sequence']})

    recorder = StageNode('evidence_recorder', '/map_snapshot', record, '/ack')

    def load_artifacts(packet):
        # Explicit loader of previously computed OFFLINE outputs, not a fake
        # real-time TSDF/manual-review node. Its name and message say so.
        selection = json.loads((ROOT/'config/final_map_selection.json').read_text())
        send(offline.pub, {'stage': 'PREVIOUSLY_COMPUTED_OFFLINE_FINAL',
            'manifest': selection['structural_manifest'], 'sha256': selection['structural_manifest_sha256'],
            'not_generated_by_this_replay': True})

    offline = StageNode('offline_final_artifact_loader', '/replay_complete', load_artifacts, '/offline_final_reference')

    def final_reference(packet):
        (a.output/'offline_final_reference.json').write_text(json.dumps(packet, indent=2)+'\n')

    reference = StageNode('final_reference_recorder', '/offline_final_reference', final_reference)
    done_pub = source.create_publisher(String, PREFIX+'/replay_complete', QOS)

    def tick():
        now = time.monotonic()
        if now-state['started'] < a.start_delay or state['inflight'] or state['done']:
            return
        if state['next'] == len(batches):
            state['done'] = True; state['finish'] = now
            send(done_pub, {'recorded_replay_complete': True})
            manifest = {'created_utc': utc_now(), 'run_id': RUN_ID, 'sessions': {n: {'frames': len(s.ids),
                'invalid_poses': s.invalid, 'invalid_edges': s.invalid_edges,
                'verified_loops': len(backends[n].accepted)} for n,s in sessions.items()},
                'snapshots': len(state['snapshots']), 'wall_seconds': now-state['started'],
                'new_live_odometry_estimation': False, 'loop_proposals_from_stored_database': True,
                'loop_geometry_reverified_after_arrival': True, 'future_poses_used_in_prefix_solver': False,
                'final_v9_generated_online': False, 'source_map_replaced': False,
                'nodes': [n.get_fully_qualified_name() for n in nodes],
                'arguments': vars(a)|{'output': str(a.output)},
                'code_sha256': {str(Path(__file__).relative_to(ROOT)): sha256_file(Path(__file__)),
                    'src/slam_replay_core.py': sha256_file(ROOT/'src/slam_replay_core.py')}}
            (a.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
            print('REPLAY_COMPLETE: offline final remains a separate artifact', flush=True)
            return
        packet = dict(batches[state['next']]); state['next'] += 1; state['inflight'] = True
        send(source.pub, packet)

    source.create_timer(.1, tick)
    nodes = [source, depth, verifier, optimizer, assembler, recorder, offline, reference]
    executor = SingleThreadedExecutor()
    for node in nodes: executor.add_node(node)
    print('ROS_REPLAY_READY: eight functional nodes; waiting for graph capture', flush=True)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=.1)
            if state['finish'] and time.monotonic()-state['finish'] > a.hold: break
    except KeyboardInterrupt:
        pass
    finally:
        for node in nodes: node.destroy_node()
        for session in sessions.values(): session.connection.close()
        rclpy.shutdown()


if __name__ == '__main__': main()
