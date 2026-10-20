#!/usr/bin/env python3
"""Rebuild and render experimental robust-submap candidates without touching RViz.

Every RGB-D frame is decoded from the original RTAB-Map database opened by the
existing exporter in read-only mode.  Only A/B-quality poses present in the
expanded trajectory are used; omitted frames are recorded, not interpolated or
silently restored.  Outputs are experimental comparison artifacts and can
never replace an accepted map or the repository's default RViz configuration.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from export_rtabmap_db import voxel_reduce, write_binary_ply
from optimize_session import sha256_file, utc_now
from render_session_report import read_ply, trajectory


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_VARIANTS = ("baseline_huber", "dcs", "group_switch")
SESSION_COLORS = {
    "outside": "#309c54",
    "lobby": "#e99731",
    "bridge_out1f": "#d358a4",
    "floor2": "#1594c7",
    "bridge_12_sub": "#8c59c6",
}


def ensure_fresh_output(path: Path) -> None:
    if path.exists():
        raise FileExistsError(f"refusing to overwrite existing output: {path}")


def count_csv_rows(path: Path) -> int:
    with path.open(newline="", encoding="utf-8") as stream:
        return sum(1 for _ in csv.DictReader(stream))


def load_pose_xyz(path: Path) -> dict[int, np.ndarray]:
    with path.open(newline="", encoding="utf-8") as stream:
        return {
            int(row["id"]): np.asarray([float(row[key]) for key in ("x", "y", "z")])
            for row in csv.DictReader(stream)
        }


def sample_cloud(cloud: np.ndarray, maximum: int) -> np.ndarray:
    if len(cloud) <= maximum:
        return cloud
    return cloud[np.linspace(0, len(cloud) - 1, maximum).astype(np.int64)]


def cloud_bounds(clouds: list[np.ndarray], lower=.5, upper=99.5) -> dict[str, list[float]]:
    if not clouds or any(not len(cloud) for cloud in clouds):
        raise ValueError("cannot compute display bounds from an empty cloud")
    return {
        axis: [float(min(np.percentile(cloud[axis], lower) for cloud in clouds)),
               float(max(np.percentile(cloud[axis], upper) for cloud in clouds))]
        for axis in ("x", "y", "z")
    }


def crop_structured_cloud(cloud: np.ndarray, lower: np.ndarray,
                          upper: np.ndarray) -> np.ndarray:
    mask = np.ones(len(cloud), dtype=bool)
    for index, axis in enumerate(("x", "y", "z")):
        mask &= (cloud[axis] >= lower[index]) & (cloud[axis] <= upper[index])
    return cloud[mask]


def rgb_colors(cloud: np.ndarray) -> np.ndarray:
    return np.column_stack((cloud["r"], cloud["g"], cloud["b"])) / 255.


def render_projection(axis, cloud: np.ndarray, first: str, second: str,
                      bounds: dict[str, list[float]], *, color_mode="height",
                      title="", max_points=180000) -> int:
    sampled = sample_cloud(cloud, max_points)
    if color_mode == "rgb":
        color = rgb_colors(sampled)
        axis.scatter(sampled[first], sampled[second], c=color, s=.16,
                     rasterized=True, linewidths=0)
    else:
        axis.scatter(sampled[first], sampled[second], c=sampled["z"], s=.16,
                     cmap="viridis", vmin=bounds["z"][0], vmax=bounds["z"][1],
                     rasterized=True, linewidths=0)
    axis.set(xlim=bounds[first], ylim=bounds[second],
             xlabel=first.upper() + " (m)", ylabel=second.upper() + " (m)",
             title=title)
    axis.set_aspect("equal", adjustable="box")
    axis.grid(alpha=.08)
    return len(sampled)


def render_global_figures(work: Path, variants: list[str], max_points: int) -> dict:
    figures = work / "figures"
    figures.mkdir(exist_ok=True)
    samples = []
    for variant in variants:
        samples.append(sample_cloud(read_ply(work / variant / "combined.ply"), max_points))
    bounds = cloud_bounds(samples)
    figure, axes = plt.subplots(2, len(variants),
                                figsize=(6.2 * len(variants), 10.2), squeeze=False)
    sampled_counts = {}
    for column, (variant, cloud) in enumerate(zip(variants, samples)):
        sampled_counts[variant] = len(cloud)
        render_projection(axes[0, column], cloud, "x", "y", bounds,
                          title=f"{variant} — XY", max_points=max_points)
        render_projection(axes[1, column], cloud, "x", "z", bounds,
                          title=f"{variant} — XZ", max_points=max_points)
    figure.suptitle(
        "Quality-filtered robust-submap candidates\n"
        "EXPERIMENTAL COMPARISON — NOT FOR NAVIGATION; common axes",
        fontsize=14,
    )
    figure.tight_layout(rect=(0., 0., 1., .94))
    comparison = figures / "all_candidates_xy_xz.png"
    figure.savefig(comparison, dpi=220)
    plt.close(figure)

    individual = {}
    for variant, cloud in zip(variants, samples):
        figure, axes = plt.subplots(1, 2, figsize=(13., 5.4))
        render_projection(axes[0], cloud, "x", "y", bounds,
                          title=f"{variant} — XY", max_points=max_points)
        render_projection(axes[1], cloud, "x", "z", bounds,
                          title=f"{variant} — XZ", max_points=max_points)
        figure.suptitle("EXPERIMENTAL / accepted=false / navigation=false — common comparison limits")
        figure.tight_layout(rect=(0., 0., 1., .92))
        path = figures / f"{variant}_xy_xz.png"
        figure.savefig(path, dpi=220)
        plt.close(figure)
        individual[variant] = str(path.relative_to(work))
    return {
        "comparison": str(comparison.relative_to(work)),
        "individual": individual,
        "bounds_m": bounds,
        "display_percentiles": [.5, 99.5],
        "sampled_points": sampled_counts,
        "maximum_display_points_per_candidate": max_points,
        "sampling": "deterministic evenly spaced PLY record indices; visualization only",
    }


def region_bounds_from_poses(pose_sets: list[dict[int, np.ndarray]], node_ids,
                             margin: tuple[float, float, float]) -> tuple[np.ndarray, np.ndarray]:
    points = [poses[node] for poses in pose_sets for node in node_ids if node in poses]
    if not points:
        raise ValueError("none of the requested representative nodes has a pose")
    points = np.asarray(points)
    margin_array = np.asarray(margin, dtype=float)
    return points.min(axis=0) - margin_array, points.max(axis=0) + margin_array


def render_session_figures(work: Path, variants: list[str], sessions: list[str],
                          max_points: int) -> dict:
    """Keep each session visible without visually asserting cross-session links."""
    records = {}
    for session in sessions:
        clouds = [sample_cloud(read_ply(work / variant / f"{session}.ply"), max_points)
                  for variant in variants]
        bounds = cloud_bounds(clouds)
        figure, axes = plt.subplots(2, len(variants),
                                    figsize=(6.2 * len(variants), 10.2), squeeze=False)
        for column, (variant, cloud) in enumerate(zip(variants, clouds)):
            render_projection(axes[0, column], cloud, "x", "y", bounds,
                              color_mode="rgb", title=f"{variant} — XY RGB",
                              max_points=max_points)
            render_projection(axes[1, column], cloud, "x", "z", bounds,
                              title=f"{variant} — XZ height",
                              max_points=max_points)
        figure.suptitle(f"{session}: reconstructed original RGB-D observations\n"
                       "EXPERIMENTAL; accepted=false / navigation=false; common axes",
                       fontsize=13)
        figure.tight_layout(rect=(0., 0., 1., .94))
        path = work / "figures" / f"session_{session}_xy_xz.png"
        figure.savefig(path, dpi=220)
        plt.close(figure)
        records[session] = {
            "path": str(path.relative_to(work)),
            "bounds_m": bounds,
            "display_percentiles": [.5, 99.5],
            "actual_rgb_data": True,
            "alignment_refit": False,
            "sampled_point_counts": {variant: len(cloud)
                                      for variant, cloud in zip(variants, clouds)},
        }
    return records


def render_region_comparison(path: Path, variants: list[str], clouds: list[np.ndarray],
                             trajectories: list[dict[int, np.ndarray]],
                             lower: np.ndarray, upper: np.ndarray, node_ids,
                             title: str, max_points: int) -> dict:
    bounds = {axis: [float(lower[index]), float(upper[index])]
              for index, axis in enumerate(("x", "y", "z"))}
    cropped = [crop_structured_cloud(cloud, lower, upper) for cloud in clouds]
    figure, axes = plt.subplots(2, len(variants),
                                figsize=(6.2 * len(variants), 10.2), squeeze=False)
    records = {}
    for column, (variant, cloud, poses) in enumerate(zip(variants, cropped, trajectories)):
        records[variant] = {"crop_point_count": int(len(cloud))}
        for row, (first, second) in enumerate((("x", "y"), ("x", "z"))):
            render_projection(axes[row, column], cloud, first, second, bounds,
                              color_mode="rgb", title=f"{variant} — {first.upper()}{second.upper()}",
                              max_points=max_points)
            track = np.asarray([poses[node] for node in node_ids if node in poses])
            if len(track):
                indices = {"x": 0, "y": 1, "z": 2}
                # Selected visits and quality-filtered poses need not be
                # consecutive. A line would invent a continuous path through
                # missing observations (or connect two distant revisits).
                axes[row, column].scatter(track[:, indices[first]], track[:, indices[second]],
                                          color="#ff1f1f", s=9, alpha=.9,
                                          label="observed camera positions")
                axes[row, column].legend(fontsize=7, loc="best")
        records[variant]["displayed_point_count"] = int(min(len(cloud), max_points))
        records[variant]["trajectory_node_count"] = int(sum(node in poses for node in node_ids))
    figure.suptitle(title + "\nactual RGB-D colors; fixed world crop; no alignment fit", fontsize=13)
    figure.tight_layout(rect=(0., 0., 1., .94))
    figure.savefig(path, dpi=220)
    plt.close(figure)
    return {"path": str(path), "crop_bounds_m": bounds, "variants": records,
            "actual_rgb_data": True, "alignment_refit": False}


def render_representative_regions(work: Path, variants: list[str],
                                  max_points: int) -> dict:
    figures = work / "figures"
    floor_clouds = [read_ply(work / variant / "floor2.ply") for variant in variants]
    floor_poses = [load_pose_xyz(work / variant / "floor2_trajectory.csv")
                   for variant in variants]

    wall_nodes = (656, 1055)
    lower, upper = region_bounds_from_poses(floor_poses, wall_nodes, (4., 4., 2.5))
    wall_path = figures / "representative_wall_floor2_656_1055.png"
    wall = render_region_comparison(
        wall_path, variants, floor_clouds, floor_poses, lower, upper, wall_nodes,
        "Representative repeated wall region: floor2 frames 656 / 1055",
        max_points,
    )
    wall["path"] = str(wall_path.relative_to(work))
    wall["selection"] = "pre-existing critical finite-wall pair; crop centered on candidate camera poses"

    stair_nodes = tuple(range(80, 241))
    lower, upper = region_bounds_from_poses(floor_poses, stair_nodes, (2.5, 2.5, 1.5))
    stair_path = figures / "representative_main_stairs_floor2_nodes_80_240.png"
    stairs = render_region_comparison(
        stair_path, variants, floor_clouds, floor_poses, lower, upper, stair_nodes,
        "Representative main-stair ascent: floor2 capture nodes 80–240",
        max_points,
    )
    stairs["path"] = str(stair_path.relative_to(work))
    stairs["selection"] = "capture-order interval spanning the observed 1F-to-2F height increase; not semantic segmentation"
    return {"wall": wall, "main_stairs": stairs}


def normalize_command(command: list[str], work: Path) -> list[str]:
    prefix = str(work.resolve())
    return ["<OUTPUT>" + value[len(prefix):] if value.startswith(prefix) else value
            for value in command]


def declared_file(record: dict, key: str, hash_key: str) -> Path:
    path = Path(record[key]).resolve()
    if not path.is_file():
        raise FileNotFoundError(path)
    expected = record.get(hash_key)
    if not expected:
        raise ValueError(f"missing declared hash for {path}")
    if sha256_file(path) != expected:
        raise ValueError(f"declared hash mismatch for {path}")
    return path


def validate_upstream(expanded: dict, database_hashes: dict[str, str]) -> dict[str, str]:
    """Bind reconstruction to the graph, backend, quality audit, and original DBs."""
    inputs = expanded["inputs"]
    verified = {}
    for key in ("quality_manifest", "graph_manifest"):
        path = declared_file(inputs, key, key + "_sha256")
        verified[str(path)] = inputs[key + "_sha256"]
    reports = inputs.get("backend_reports", [])
    if not reports:
        raise ValueError("expanded trajectory has no declared backend reports")
    for report in reports:
        path = declared_file(report, "path", "sha256")
        verified[str(path)] = report["sha256"]
    quality = json.loads(Path(inputs["quality_manifest"]).read_text(encoding="utf-8"))
    for session, actual_hash in database_hashes.items():
        record = quality["sessions"].get(session, {})
        expected = record.get("database_sha256_after")
        if not expected or expected != actual_hash:
            raise ValueError(f"original database differs from quality audit: {session}")
    return verified


def export(args) -> Path:
    output = args.output.resolve()
    ensure_fresh_output(output)
    expanded_root = args.expanded_root.resolve()
    expanded_path = expanded_root / "manifest.json"
    expanded = json.loads(expanded_path.read_text(encoding="utf-8"))
    config_path = args.config.resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))["sessions"]
    variants = list(args.variants or DEFAULT_VARIANTS)
    if len(variants) != len(set(variants)) or any(name not in expanded.get("variants", {}) for name in variants):
        raise ValueError("variants must be unique names present in the expanded manifest")

    rviz_paths = sorted(ROOT.glob("config/*.rviz"))
    rviz_before = {str(path.resolve()): sha256_file(path) for path in rviz_paths}
    database_before = {name: sha256_file((ROOT / row["database"]).resolve())
                       for name, row in config.items()}
    upstream_hashes = validate_upstream(expanded, database_before)
    source_paths = [Path(__file__).resolve(), ROOT / "src/export_rtabmap_db.py",
                    ROOT / "src/render_session_report.py", expanded_path, config_path]
    manifest = {
        "schema_version": 1,
        "started_utc": utc_now(),
        "status": "EXPERIMENTAL_POINT_CLOUD_COMPARISON_NOT_FOR_NAVIGATION",
        "experimental": True,
        "rejected": bool(args.rejected),
        "accepted": False,
        "navigation": False,
        "accepted_by_validation": False,
        "default_rviz_replaced": False,
        "output_root": str(output),
        "expanded_root": str(expanded_root),
        "variants": variants,
        "coordinate_frame": "map_submap_candidate_display_gauge",
        "display_placement_warning": (
            "Disconnected or weakly connected components retain the graph candidate display gauge; "
            "visual proximity is not proof of cross-session registration."),
        "method": (
            "Decode original RGB/depth/confidence/calibration read-only and reproject each available "
            "A/B-quality frame through its expanded candidate pose; combine session voxel clouds."),
        "parameters": {
            "frame_stride": args.frame_stride,
            "pixel_stride": args.pixel_stride,
            "voxel_size_m": args.voxel,
            "minimum_depth_m": args.min_depth,
            "maximum_depth_m": args.max_depth,
            "minimum_confidence": args.min_confidence,
            "batch_frames": args.batch_frames,
            "missing_pose_policy": "skip",
            "semantic_masks": None,
            "maximum_display_points": args.max_display_points,
        },
        "input_hashes": {**{str(path): sha256_file(path) for path in source_paths},
                         **upstream_hashes},
        "upstream_artifact_hashes_verified": True,
        "databases_match_original_quality_audit": True,
        "database_sha256_before": database_before,
        "rviz_sha256_before": rviz_before,
        "exports": {},
        "commands": [],
        "limitations": [
            "C-quality/no-primary frames are absent by design, so coverage differs from prior full-frame previews.",
            "Depth confidence and calibration remain sensor-derived and are not surveyed geometry.",
            "Robust backend alternatives share inputs and are correlated, not independent experiments.",
            "Representative wall/stair crops were chosen from known capture regions and are diagnostic, not exhaustive.",
        ],
    }

    output.parent.mkdir(parents=True, exist_ok=True)
    work = Path(tempfile.mkdtemp(prefix=f".{output.name}.tmp-", dir=output.parent))
    environment = os.environ.copy()
    environment.update(OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="2", MKL_NUM_THREADS="1")
    try:
        for variant in variants:
            variant_dir = work / variant
            variant_dir.mkdir()
            clouds = []
            variant_record = {"sessions": {}}
            for session, spec in config.items():
                expanded_record = expanded["variants"][variant]["sessions"].get(session)
                if not expanded_record:
                    raise ValueError(f"expanded manifest lacks {variant}/{session}")
                pose_path = declared_file(expanded_record, "trajectory_csv", "trajectory_sha256")
                skipped_path = declared_file(expanded_record, "skipped_frames_json", "skipped_frames_sha256")
                database = (ROOT / spec["database"]).resolve()
                manifest["input_hashes"][str(pose_path)] = sha256_file(pose_path)
                manifest["input_hashes"][str(skipped_path)] = sha256_file(skipped_path)
                manifest["input_hashes"][str(database)] = database_before[session]

                cloud_path = variant_dir / f"{session}.ply"
                trajectory_path = variant_dir / f"{session}_trajectory.csv"
                summary_path = variant_dir / f"{session}_export_summary.json"
                log_path = variant_dir / f"{session}_export.log"
                command = [
                    sys.executable, str(ROOT / "src/export_rtabmap_db.py"), str(database),
                    "--pose-csv", str(pose_path), "--missing-pose-policy", "skip",
                    "--frame-stride", str(args.frame_stride),
                    "--pixel-stride", str(args.pixel_stride),
                    "--voxel", str(args.voxel),
                    "--min-depth", str(args.min_depth),
                    "--max-depth", str(args.max_depth),
                    "--min-confidence", str(args.min_confidence),
                    "--batch-frames", str(args.batch_frames),
                    "--cloud", str(cloud_path), "--trajectory", str(trajectory_path),
                    "--summary", str(summary_path),
                ]
                started = time.monotonic()
                with log_path.open("w", encoding="utf-8") as stream:
                    process = subprocess.run(command, cwd=ROOT, env=environment,
                                             stdout=stream, stderr=subprocess.STDOUT)
                command_record = {"variant": variant, "session": session,
                                  "argv_template": normalize_command(command, work),
                                  "exit_code": int(process.returncode),
                                  "elapsed_seconds": float(time.monotonic() - started),
                                  "log": str(log_path.relative_to(work))}
                manifest["commands"].append(command_record)
                if process.returncode:
                    raise RuntimeError(f"cloud export failed: {variant}/{session}; see {log_path}")
                summary = json.loads(summary_path.read_text(encoding="utf-8"))
                cloud = read_ply(cloud_path)
                if len(cloud) != summary["output_points"]:
                    raise ValueError(f"PLY/summary point mismatch: {variant}/{session}")
                clouds.append(cloud)
                declared_skipped = json.loads(skipped_path.read_text(encoding="utf-8"))
                input_pose_count = count_csv_rows(pose_path)
                exported_pose_count = count_csv_rows(trajectory_path)
                pose_ids = set(load_pose_xyz(pose_path))
                exported_pose_ids = set(load_pose_xyz(trajectory_path))
                variant_record["sessions"][session] = {
                    "cloud": str(cloud_path.relative_to(work)),
                    "trajectory": str(trajectory_path.relative_to(work)),
                    "export_summary": str(summary_path.relative_to(work)),
                    "log": str(log_path.relative_to(work)),
                    "point_count": int(len(cloud)),
                    "input_pose_count": input_pose_count,
                    "exported_pose_count": exported_pose_count,
                    "expanded_manifest_output_frame_count": expanded_record.get("output_frame_count"),
                    "expanded_manifest_skipped_frame_count": expanded_record.get("skipped_frame_count"),
                    "declared_skipped_frames": len(declared_skipped),
                    "declared_skipped_reasons": dict(sorted({
                        reason: sum(row.get("reason") == reason for row in declared_skipped)
                        for reason in {row.get("reason") for row in declared_skipped}
                    }.items())),
                    "database_rows_missing_from_pose_csv": summary["skipped_missing_pose_ids"],
                    "pose_csv_ids_absent_from_exportable_database_rows": sorted(pose_ids - exported_pose_ids),
                    "export_parameters_and_counts": summary,
                    "input_hashes": {
                        "pose_csv": sha256_file(pose_path),
                        "skipped_frames_json": sha256_file(skipped_path),
                        "database": database_before[session],
                    },
                    "output_hashes": {
                        "cloud": sha256_file(cloud_path),
                        "trajectory": sha256_file(trajectory_path),
                        "export_summary": sha256_file(summary_path),
                        "log": sha256_file(log_path),
                    },
                }
                print(f"{variant}/{session}: {len(cloud):,} points; "
                      f"poses {exported_pose_count}/{input_pose_count}", flush=True)

            combined = np.concatenate(clouds)
            xyz = np.column_stack([combined[key] for key in ("x", "y", "z")])
            rgb = np.column_stack([combined[key] for key in ("r", "g", "b")])
            xyz, rgb, _ = voxel_reduce(xyz, rgb, args.voxel)
            combined_path = variant_dir / "combined.ply"
            write_binary_ply(combined_path, xyz, rgb)
            variant_record.update({
                "session_point_sum_before_cross_session_voxel_merge": int(sum(len(c) for c in clouds)),
                "combined_point_count": int(len(xyz)),
                "combined_cloud": str(combined_path.relative_to(work)),
                "combined_cloud_sha256": sha256_file(combined_path),
            })
            manifest["exports"][variant] = variant_record
            print(f"{variant}/combined: {len(xyz):,} points", flush=True)
            del combined, xyz, rgb, clouds

        manifest["visualizations"] = {
            "global": render_global_figures(work, variants, args.max_display_points),
            "sessions": render_session_figures(
                work, variants, list(config), args.max_display_points),
            "representative_regions": render_representative_regions(
                work, variants, min(args.max_display_points, 160000)),
        }
        database_after = {name: sha256_file((ROOT / row["database"]).resolve())
                          for name, row in config.items()}
        rviz_after = {str(path.resolve()): sha256_file(path) for path in rviz_paths}
        manifest["database_sha256_after"] = database_after
        manifest["original_databases_unchanged"] = database_before == database_after
        manifest["rviz_sha256_after"] = rviz_after
        manifest["rviz_configuration_unchanged"] = rviz_before == rviz_after
        if not manifest["original_databases_unchanged"]:
            raise RuntimeError("an original database hash changed")
        if not manifest["rviz_configuration_unchanged"]:
            raise RuntimeError("an RViz configuration changed")
        shutil.copyfile(Path(__file__), work / "source_snapshot.py")
        output_hashes = {}
        for path in sorted(work.rglob("*")):
            if path.is_file() and path.name not in {"manifest.json", "manifest.sha256"}:
                output_hashes[str(path.relative_to(work))] = sha256_file(path)
        manifest["output_hashes"] = output_hashes
        manifest["finished_utc"] = utc_now()
        manifest_path = work / "manifest.json"
        manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
                                 encoding="utf-8")
        (work / "manifest.sha256").write_text(sha256_file(manifest_path) + "  manifest.json\n",
                                               encoding="ascii")
        ensure_fresh_output(output)
        work.rename(output)
        return output
    except Exception:
        manifest["failed_utc"] = utc_now()
        manifest["accepted"] = False
        manifest["navigation"] = False
        (work / "incomplete_manifest.json").write_text(
            json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        raise


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--expanded-root", type=Path, required=True)
    parser.add_argument("--config", type=Path,
                        default=Path("config/student_union_all_sessions.json"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--variants", nargs="*", choices=DEFAULT_VARIANTS)
    parser.add_argument("--rejected", action="store_true",
                        help="Record an already established rejection; this tool never grants acceptance")
    parser.add_argument("--frame-stride", type=int, default=1)
    parser.add_argument("--pixel-stride", type=int, default=1)
    parser.add_argument("--voxel", type=float, default=.05)
    parser.add_argument("--min-depth", type=float, default=.25)
    parser.add_argument("--max-depth", type=float, default=4.)
    parser.add_argument("--min-confidence", type=int, default=50)
    parser.add_argument("--batch-frames", type=int, default=20)
    parser.add_argument("--max-display-points", type=int, default=180000)
    args = parser.parse_args(argv)
    if min(args.frame_stride, args.pixel_stride, args.batch_frames,
           args.max_display_points) <= 0:
        parser.error("stride, batch size, and display count must be positive")
    if args.voxel <= 0 or args.min_depth <= 0 or args.max_depth <= args.min_depth:
        parser.error("invalid voxel/depth parameters")
    if not 0 <= args.min_confidence <= 100:
        parser.error("min confidence must be in [0,100]")
    return args


def main(argv=None):
    args = parse_args(argv)
    result = export(args)
    manifest = json.loads((result / "manifest.json").read_text(encoding="utf-8"))
    print(json.dumps({
        "output": str(result),
        "variant_points": {name: row["combined_point_count"]
                           for name, row in manifest["exports"].items()},
        "accepted": False,
        "navigation": False,
        "default_rviz_replaced": False,
    }, ensure_ascii=False))


if __name__ == "__main__":
    main()
