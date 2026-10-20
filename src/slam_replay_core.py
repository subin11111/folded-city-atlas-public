"""Causal recorded-data replay utilities; NOT a live RGB-D odometry frontend.

Stored loop proposals are hindsight candidates from the phone database. They
are re-verified only after both endpoints arrive. Final selected poses and the
hand-finished display mesh are never inputs to this incremental optimizer.
"""
import contextlib
import io
import json
from pathlib import Path
import sqlite3
import time
import zlib

import cv2
import numpy as np

from export_rtabmap_db import calibration_from_blob
from optimize_session import (Clouds, apply, checked_transform, connected_components,
                              inverse, optimize, read_edges, verify_loop)

ROOT = Path(__file__).resolve().parents[1]
ORDER = ('outside', 'lobby', 'bridge_out1f', 'floor2', 'bridge_12_sub')


def eligible_edges(edges, seen):
    return [e for e in edges if e.a in seen and e.b in seen]


def decode_rgbd(row, pixel_stride=6):
    """Strict confidence decoding; no synthetic confidence or hole filling."""
    image_blob, depth_blob, confidence_blob, calibration = row
    packed = cv2.imdecode(np.frombuffer(depth_blob, np.uint8), cv2.IMREAD_UNCHANGED)
    if packed is None or packed.ndim != 3 or packed.shape[2] != 4:
        raise ValueError('Unsupported depth encoding')
    depth = np.ascontiguousarray(packed).view('<f4').reshape(packed.shape[:2])
    raw = zlib.decompress(confidence_blob) if confidence_blob else b''
    if len(raw) not in (depth.size, depth.size + 12):
        raise ValueError('Missing/malformed confidence; do not assume valid depth')
    confidence = np.frombuffer(raw[:depth.size], np.uint8).reshape(depth.shape)
    bgr = cv2.imdecode(np.frombuffer(image_blob, np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError('Invalid RGB')
    cw, ch, k, local = calibration_from_blob(calibration)
    h, w = depth.shape
    yy, xx = np.mgrid[0:h:pixel_stride, 0:w:pixel_stride]
    z = depth[::pixel_stride, ::pixel_stride]
    valid = np.isfinite(z) & (z >= .25) & (z <= 4.) & (confidence[::pixel_stride, ::pixel_stride] >= 50)
    z = z[valid]
    optical = np.c_[((xx[valid]*cw/w-k[0, 2])*z/k[0, 0]),
                    ((yy[valid]*ch/h-k[1, 2])*z/k[1, 1]), z]
    small = cv2.resize(bgr, (w, h), interpolation=cv2.INTER_AREA)
    rgb = small[::pixel_stride, ::pixel_stride][valid, ::-1].copy()
    return apply(optical, local).astype(np.float32), rgb, bgr


class RecordedSession:
    def __init__(self, database):
        self.database = Path(database)
        self.connection = sqlite3.connect(f'file:{self.database.resolve()}?mode=ro', uri=True)
        self.raw, self.stamps, self.invalid = {}, {}, []
        for node, stamp, blob in self.connection.execute('SELECT id,stamp,pose FROM Node ORDER BY stamp,id'):
            try:
                self.raw[node] = checked_transform(blob)
                self.stamps[node] = float(stamp)
            except ValueError as error:
                self.invalid.append({'node': node, 'reason': str(error)})
        self.ids = list(self.raw)
        self.edges, self.invalid_edges = read_edges(self.connection, set(self.ids), .04, 1.)
        self.odom = [e for e in self.edges if e.type == 0]
        self.proposals = [e for e in self.edges if e.type in (1, 2)]
        self.clouds = Clouds(self.connection, None)

    def row(self, node):
        return self.connection.execute('SELECT image,depth,depth_confidence,calibration FROM Data WHERE id=?', (int(node),)).fetchone()


class PrefixOptimizer:
    def __init__(self, session, interval=160, max_evaluations=60):
        self.session, self.interval, self.max_evaluations = session, interval, max_evaluations
        self.seen, self.proposed, self.accepted = set(), set(), []
        self.poses, self.last_solve_count = {}, 0

    def verify_arrived(self, ids):
        self.seen.update(ids)
        available_odom = eligible_edges(self.session.odom, self.seen)
        components = connected_components(sorted(self.seen), available_odom)
        lookup = {n: i for i, group in enumerate(components) for n in group}
        rows = []
        start = time.perf_counter()
        for e in eligible_edges(self.session.proposals, self.seen):
            key = e.type, e.a, e.b
            if key in self.proposed:
                continue
            self.proposed.add(key)
            row = verify_loop(e, self.session.clouds, lookup)
            row['available_at_seen_count'] = len(self.seen)
            row['proposal_source'] = 'stored_DB_hindsight_candidate_not_new_place_discovery'
            rows.append(row)
            if row['accepted']:
                self.accepted.append(e)
        return rows, time.perf_counter()-start

    def extend(self, ids):
        # Transport the last available correction only along actual odometry
        # edges. No correction jumps across tracking components or future poses.
        pending = set(ids)
        while pending:
            progressed = False
            for e in eligible_edges(self.session.odom, self.seen):
                for n, neighbour in ((e.a, e.b), (e.b, e.a)):
                    if n in pending and neighbour in self.poses:
                        self.poses[n] = self.poses[neighbour] @ inverse(self.session.raw[neighbour]) @ self.session.raw[n]
                        pending.remove(n)
                        progressed = True
            if not progressed:
                n = min(pending)
                self.poses[n] = self.session.raw[n].copy()
                pending.remove(n)

    def solve(self, force=False):
        due = self.accepted and (len(self.seen)-self.last_solve_count >= self.interval or force)
        before = {n: p.copy() for n, p in self.poses.items()}
        report = {'ran': False, 'applied': False, 'seconds': 0., 'reason': 'no_due_verified_loop'}
        if due:
            odom = eligible_edges(self.session.odom, self.seen)
            # A genuinely incremental initial estimate: previous accepted
            # solution plus causally transported new nodes, not a restart from
            # the raw trajectory at every loop. No selected final poses enter.
            raw = {n: self.poses[n] for n in sorted(self.seen)}
            start = time.perf_counter()
            with contextlib.redirect_stdout(io.StringIO()):
                solved, details = optimize(raw, odom, self.accepted,
                    connected_components(sorted(self.seen), odom), max_evaluations=self.max_evaluations)
            good = details['success'] and not details['active_bounds']
            report = {'ran': True, 'applied': bool(good), 'seconds': time.perf_counter()-start,
                      'warm_start_from_previous_prefix': True, **details}
            if good:
                self.poses = solved
            self.last_solve_count = len(self.seen)
        shift = np.array([np.linalg.norm(self.poses[n][:3, 3]-before[n][:3, 3]) for n in self.poses])
        report['maximum_pose_update_m'] = float(shift.max(initial=0))
        report['verified_loops_available'] = len(self.accepted)
        return before, report


def load_sessions():
    inventory = json.loads((ROOT/'config/student_union_all_sessions.json').read_text())
    return {name: RecordedSession(ROOT/inventory['sessions'][name]['database']) for name in ORDER}
