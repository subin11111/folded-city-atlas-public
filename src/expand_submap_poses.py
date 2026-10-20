#!/usr/bin/env python3
"""Expand optimized submap-anchor poses into quality-filtered frame poses.

The robust backend optimizes only submap anchors.  Rebuilding a dense cloud by
applying one rigid correction per submap would introduce block seams, so this
stage interpolates the *left correction* between neighboring anchors in the
same uninterrupted quality run.  It never interpolates across a hard run
boundary.  Frames without an A/B-quality primary submap are deliberately
omitted; ``export_rtabmap_db.py --missing-pose-policy skip`` can then rebuild a
quality-filtered cloud without editing the source database.

Pose convention is ``T_world_frame``.  This deliberately matches the backend
parameterization: world-frame translation delta is additive, while orientation
uses a left rotation ``R_candidate = R_delta R_initial``.  Translation deltas
are linear in frozen-trajectory path distance and correction rotations use
SLERP.
"""
from __future__ import annotations

import argparse
import csv
import json
import hashlib
from pathlib import Path
import shutil
import sys

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from export_rtabmap_db import load_pose_csv
from optimize_session import sha256_file, utc_now


POSE_FIELDS = ["id", "stamp", "x", "y", "z", "qx", "qy", "qz", "qw"]


def as_transform(value):
    value = np.asarray(value, dtype=float)
    if value.shape == (4, 4):
        result = value.copy()
    elif value.shape == (7,):
        result = np.eye(4)
        result[:3, :3] = Rotation.from_quat(value[3:]).as_matrix()
        result[:3, 3] = value[:3]
    else:
        raise ValueError(f"pose must be 4x4 or length 7, got {value.shape}")
    if not np.isfinite(result).all() or not np.allclose(result[3], [0, 0, 0, 1]):
        raise ValueError("invalid transform")
    rotation = result[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(rotation), 1., atol=1e-6)):
        raise ValueError("rotation is not in SO(3)")
    return result


def interpolate_correction(first, second, fraction):
    """Interpolate two SE(3) corrections without crossing extrapolation."""
    fraction = float(np.clip(fraction, 0., 1.))
    result = np.eye(4)
    result[:3, 3] = (1. - fraction) * first[:3, 3] + fraction * second[:3, 3]
    rotations = Rotation.from_matrix(np.stack((first[:3, :3], second[:3, :3])))
    result[:3, :3] = Slerp([0., 1.], rotations)([fraction]).as_matrix()[0]
    return result


def anchor_correction(initial, candidate):
    """Encode the backend's additive-position/left-rotation correction."""
    result = np.eye(4)
    result[:3, :3] = candidate[:3, :3] @ initial[:3, :3].T
    result[:3, 3] = candidate[:3, 3] - initial[:3, 3]
    return result


def apply_correction(correction, initial):
    """Apply a backend-compatible correction to a frame pose."""
    result = np.eye(4)
    result[:3, :3] = correction[:3, :3] @ initial[:3, :3]
    result[:3, 3] = initial[:3, 3] + correction[:3, 3]
    return result


def correction_at(anchors, position):
    """Return the interpolated correction for sorted ``(position, C, id)``."""
    if not anchors:
        raise ValueError("run has no anchors")
    if len(anchors) == 1 or position <= anchors[0][0]:
        return anchors[0][1]
    if position >= anchors[-1][0]:
        return anchors[-1][1]
    for first, second in zip(anchors, anchors[1:]):
        if first[0] <= position <= second[0]:
            span = second[0] - first[0]
            fraction = 0. if span <= 1e-12 else (position - first[0]) / span
            return interpolate_correction(first[1], second[1], fraction)
    raise AssertionError("position was not bracketed")


def read_frames(path):
    rows = []
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            primary = row.get("primary_submap_id") or row.get("primary_submap")
            usable = str(row.get("usable_for_submap", "true")).lower() in {
                "true", "1", "yes"
            }
            usable = usable and row.get("quality_grade", "B").upper() != "C"
            rows.append({
                **row,
                "node_id": int(row["node_id"]),
                "stamp": float(row.get("stamp") or 0.),
                "session_path_m": float(row.get("session_path_m") or 0.),
                "primary_submap_id": primary,
                "usable_for_submap": usable,
            })
    if len({row['node_id'] for row in rows}) != len(rows):
        raise ValueError('duplicate frame node id')
    if any(not np.isfinite(row['session_path_m']) for row in rows):
        raise ValueError('non-finite frame path distance')
    return rows


def resolve_path(value, root):
    path = Path(value)
    for candidate in (path, root / path, root / path.name):
        if candidate.is_file():
            return candidate.resolve()
    raise FileNotFoundError(value)


def write_pose_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=POSE_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def expand_session(session_name, quality_session, quality_root, initial_graph,
                   backend_submaps, candidate_poses, source_hashes=None):
    artifacts = quality_session.get("artifacts", {})
    frames_path = resolve_path(
        artifacts.get("frames_csv", quality_session.get("frames_csv")), quality_root)
    submaps_path = resolve_path(
        artifacts.get("submaps_json", quality_session.get("submaps_json")), quality_root)
    frames = read_frames(frames_path)
    submaps_payload = json.loads(submaps_path.read_text(encoding="utf-8"))
    submaps = submaps_payload.get("submaps", submaps_payload)
    initial_path = (initial_graph / session_name / "initial.csv").resolve()
    if source_hashes is not None:
        for path in (frames_path, submaps_path, initial_path):
            expected = source_hashes.get(str(path.resolve()))
            if not expected or sha256_file(path) != expected:
                raise ValueError(f'{session_name}: source artifact differs from graph provenance: {path}')
    initial = load_pose_csv(initial_path)
    frame_by_node = {row["node_id"]: row for row in frames}

    primary_to_run = {}
    primary_members = {}
    runs = {}
    for submap in submaps:
        submap_id = str(submap.get("submap_id", submap.get("id")))
        if submap_id not in backend_submaps or submap_id not in candidate_poses:
            raise ValueError(f"{session_name}: missing backend pose for {submap_id}")
        run_id = str(submap.get("run_id", "0"))
        anchor_id = int(submap.get("anchor_node_id", submap.get("root_node_id")))
        if anchor_id not in initial or anchor_id not in frame_by_node:
            raise ValueError(f"{session_name}: missing initial anchor {anchor_id}")
        backend_row = backend_submaps[submap_id]
        if backend_row.get('session_id') != session_name:
            raise ValueError(f'{submap_id}: backend session differs from quality session')
        if (str(backend_row.get('run_id', run_id)) != run_id
                or int(backend_row.get('anchor_node_id', anchor_id)) != anchor_id):
            raise ValueError(f'{submap_id}: backend run/anchor differs from quality definition')
        if not np.allclose(as_transform(backend_row['pose']), initial[anchor_id],
                           atol=1e-9, rtol=1e-9):
            raise ValueError(f'{submap_id}: backend initial anchor differs from frozen common pose')
        correction = anchor_correction(initial[anchor_id], candidate_poses[submap_id])
        anchor_path = frame_by_node[anchor_id]["session_path_m"]
        runs.setdefault(run_id, []).append((anchor_path, correction, submap_id))
        primary_to_run[submap_id] = run_id
        if 'node_ids' in submap:
            primary_members[submap_id] = set(map(int, submap['node_ids']))
    for run_id in runs:
        runs[run_id].sort(key=lambda item: (item[0], item[2]))
        positions = [item[0] for item in runs[run_id]]
        if any(b <= a for a, b in zip(positions, positions[1:])):
            raise ValueError(f"{session_name}: non-increasing anchors in run {run_id}")

    output_rows, skipped = [], []
    for frame in frames:
        node_id, primary = frame["node_id"], frame["primary_submap_id"]
        if not frame["usable_for_submap"] or not primary:
            skipped.append({"node_id": node_id, "reason": "quality_C_or_no_primary_submap"})
            continue
        if node_id not in initial:
            skipped.append({"node_id": node_id, "reason": "absent_from_common_initial"})
            continue
        if primary not in primary_to_run:
            raise ValueError(f"{session_name}: unknown primary submap {primary}")
        if primary in primary_members and node_id not in primary_members[primary]:
            raise ValueError(f'{session_name}: frame {node_id} is outside its primary submap')
        correction = correction_at(runs[primary_to_run[primary]], frame["session_path_m"])
        pose = apply_correction(correction, initial[node_id])
        quaternion = Rotation.from_matrix(pose[:3, :3]).as_quat()
        output_rows.append(dict(zip(
            POSE_FIELDS,
            [node_id, frame["stamp"], *pose[:3, 3].tolist(), *quaternion.tolist()],
        )))

    return output_rows, skipped, {
        "frames_csv": str(frames_path),
        "frames_sha256": sha256_file(frames_path),
        "submaps_json": str(submaps_path),
        "submaps_sha256": sha256_file(submaps_path),
        "initial_pose_csv": str(initial_path),
        "initial_pose_sha256": sha256_file(initial_path),
        "input_frame_count": len(frames),
        "output_frame_count": len(output_rows),
        "skipped_frame_count": len(skipped),
        "run_count": len(runs),
        "submap_count": sum(len(values) for values in runs.values()),
    }


def expand(quality, quality_root, graph_manifest, backend_report, initial_graph,
           variants):
    source_hashes = graph_manifest.get('metadata', {}).get('source_sha256', {})
    quality_path = (quality_root / 'manifest.json').resolve()
    if (not source_hashes.get(str(quality_path))
            or sha256_file(quality_path) != source_hashes[str(quality_path)]):
        raise ValueError('quality manifest differs from graph source provenance')
    backend_submaps = {
        str(row["id"]): row for row in graph_manifest["submaps"]
    }
    available = backend_report.get("variants", {})
    if not variants:
        variants = sorted(available)
    unknown = sorted(set(variants) - set(available))
    if unknown:
        raise ValueError(f"unknown variants: {unknown}")
    result = {}
    for variant in variants:
        candidate = {
            submap_id: as_transform(pose)
            for submap_id, pose in available[variant]["poses"].items()
        }
        if set(candidate) != set(backend_submaps):
            raise ValueError(f'{variant}: candidate submap ids differ from graph')
        sessions = {}
        for session_name, session in quality["sessions"].items():
            rows, skipped, provenance = expand_session(
                session_name, session, quality_root, initial_graph,
                backend_submaps, candidate, source_hashes)
            sessions[session_name] = {
                "rows": rows, "skipped": skipped, "provenance": provenance
            }
        result[variant] = sessions
    return result


def merge_backend_reports(graph, reports):
    """Bind every report to this graph, not merely to the other reports."""
    canonical = json.dumps(graph, sort_keys=True, separators=(',', ':'),
                           ensure_ascii=False).encode('utf-8')
    expected = hashlib.sha256(canonical).hexdigest()
    merged = {'variants': {}}
    for report in reports:
        actual = report.get('provenance', {}).get('canonical_manifest_sha256')
        if actual != expected:
            raise ValueError('backend canonical manifest hash differs from supplied graph')
        for name, variant in report.get('variants', {}).items():
            if name in merged['variants']:
                raise ValueError(f'duplicate backend variant {name}')
            merged['variants'][name] = variant
    if not merged['variants']:
        raise ValueError('backend reports contain no variants')
    return merged


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quality-root", type=Path, required=True)
    parser.add_argument("--graph-manifest", type=Path, required=True)
    parser.add_argument("--backend-report", type=Path, nargs="+", required=True,
                        help="One or more reports with disjoint variant names")
    parser.add_argument("--initial-graph", type=Path,
                        default=Path("outputs/student_union_20261017_geometry_review/graph_final"))
    parser.add_argument("--variants", nargs="*")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite {args.output}")
    quality_path = args.quality_root / "manifest.json"
    quality = json.loads(quality_path.read_text(encoding="utf-8"))
    graph = json.loads(args.graph_manifest.read_text(encoding="utf-8"))
    backend_reports = [json.loads(path.read_text(encoding="utf-8"))
                       for path in args.backend_report]
    backend = merge_backend_reports(graph, backend_reports)
    expanded = expand(quality, args.quality_root, graph, backend,
                      args.initial_graph, args.variants)
    args.output.mkdir(parents=True)
    summary = {
        "created_utc": utc_now(),
        "status": "EXPERIMENTAL_QUALITY_FILTERED_EXPANSION_NOT_NAVIGATION_APPROVED",
        "accepted": False,
        "navigation": False,
        "algorithm": [
            "backend-compatible correction: additive world translation delta and left orientation rotation",
            "linear translation-delta and correction-rotation SLERP by frozen-trajectory path distance",
            "interpolation only within one hard-boundary-delimited quality run",
            "omit C-quality/no-primary frames from candidate dense reconstruction",
        ],
        "inputs": {
            "quality_manifest": str(quality_path.resolve()),
            "quality_manifest_sha256": sha256_file(quality_path),
            "graph_manifest": str(args.graph_manifest.resolve()),
            "graph_manifest_sha256": sha256_file(args.graph_manifest),
            "backend_reports": [
                {"path": str(path.resolve()), "sha256": sha256_file(path)}
                for path in args.backend_report],
        },
        "variants": {},
    }
    for variant, sessions in expanded.items():
        variant_dir = args.output / variant
        variant_record = {"sessions": {}}
        for name, record in sessions.items():
            pose_path = variant_dir / name / "trajectory.csv"
            write_pose_csv(pose_path, record["rows"])
            skipped_path = variant_dir / name / "skipped_frames.json"
            skipped_path.write_text(json.dumps(record["skipped"], indent=2) + "\n")
            variant_record["sessions"][name] = {
                **record["provenance"],
                "trajectory_csv": str(pose_path.resolve()),
                "trajectory_sha256": sha256_file(pose_path),
                "skipped_frames_json": str(skipped_path.resolve()),
                "skipped_frames_sha256": sha256_file(skipped_path),
            }
        summary["variants"][variant] = variant_record
    shutil.copyfile(Path(__file__), args.output / "source_snapshot.py")
    summary["source_sha256"] = sha256_file(Path(__file__))
    summary["argv"] = sys.argv if argv is None else [str(Path(__file__)), *argv]
    (args.output / "manifest.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({
        "output": str(args.output), "variants": list(expanded),
        "accepted": False, "navigation": False,
    }))


if __name__ == "__main__":
    main()
