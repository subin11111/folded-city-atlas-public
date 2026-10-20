#!/usr/bin/env python3
"""Post-hoc read-only audit of frozen local floor2 results; never refits/gates.

Full-depth metrics deliberately include the diagnostic sample partition and
cannot be reused to select measurements. This audits metric sampling effects,
calibration arithmetic, source imagery, and finite accepted test diagnostics.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shlex
import sys

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from check_session_overlap import Session, mutual_ratio_matches
from optimize_session import apply, geometric_score, inverse, sha256_file, utc_now
from refine_geometry_surfaces import correspondences, normals
from remeasure_floor2_local import depth_cloud, feature_train_mask, partition_masks


def finite_tree(value):
    if isinstance(value, dict):
        return all(finite_tree(item) for item in value.values())
    if isinstance(value, list):
        return all(finite_tree(item) for item in value)
    return not isinstance(value, (float, np.floating)) or bool(np.isfinite(value))


def features_audit(session, a, b):
    ka, da, _ = session.feature(a, full=True)
    kb, db, _ = session.feature(b, full=True)
    matches = mutual_ratio_matches(da, db, .75)
    if not matches:
        return {"mutual_matches": 0}
    pixels = [np.asarray([ka[m.queryIdx].pt for m in matches]),
              np.asarray([kb[m.trainIdx].pt for m in matches])]
    points = [session.unproject(n, p) for n, p in zip((a, b), pixels)]
    valid = np.isfinite(points[0]).all(axis=1) & np.isfinite(points[1]).all(axis=1)
    keep = [feature_train_mask(session, n, p, partition_masks(session.depth(n)[0].shape)[0])
            for n, p in zip((a, b), pixels)]
    errors = []
    for n, p, x in zip((a, b), pixels, points):
        finite = np.isfinite(x).all(axis=1)
        _, _, cw, ch, k, local = session.depth(n)
        ih, iw = session.image(n).shape[:2]
        optical = apply(x[finite], np.linalg.inv(local))
        projected = optical @ k.T
        reconstructed_rgb = projected[:, :2]/projected[:, 2:3] * [iw/cw, ih/ch]
        errors.extend(np.linalg.norm(reconstructed_rgb-p[finite], axis=1).tolist())
    return {"mutual_matches": len(matches), "valid_depth_matches_without_partition": int(valid.sum()),
            "strict_training_depth_matches": int((valid & keep[0] & keep[1]).sum()),
            "reprojection_roundtrip_max_rgb_px": max(errors) if errors else None,
            "roundtrip_scope": "Numeric consistency only; not external camera calibration verification"}


def montage(session, a, b, path):
    fig, axes = plt.subplots(2, 4, figsize=(13, 10))
    for row, node in enumerate((a, b)):
        rgb = cv2.cvtColor(session.image(node), cv2.COLOR_BGR2RGB)
        depth, confidence, *_ = session.depth(node)
        valid = np.isfinite(depth) & (depth >= .25) & (depth <= 4.) & (confidence >= 50)
        train, test = partition_masks(depth.shape)
        image = np.ma.masked_where(~valid, depth)
        axes[row, 0].imshow(rgb); axes[row, 0].set_title(f"Frame {node}: actual RGB")
        axes[row, 1].imshow(cv2.resize(rgb, (depth.shape[1], depth.shape[0])))
        axes[row, 1].imshow(image, cmap="turbo", vmin=.25, vmax=4., alpha=.45)
        axes[row, 1].set_title("RGB + valid depth overlay")
        view = axes[row, 2].imshow(image, cmap="turbo", vmin=.25, vmax=4.)
        axes[row, 2].set_title("Actual depth, 0.25–4m")
        partitions = np.zeros((*depth.shape, 3), dtype=float)
        partitions[train & valid] = [0., .45, .75]
        partitions[test & valid] = [.93, .45, .12]
        partitions[~train & ~test & valid] = [.6, .6, .6]
        axes[row, 3].imshow(partitions)
        axes[row, 3].set_title("Blue: fit / orange: test / gray: guard")
        for ax in axes[row]:
            ax.axis("off")
            ax.title.set_fontsize(10)
    fig.suptitle(f"floor2 {a} → {b}: actual RGB/depth and predeclared split\n"
                 "Read-only diagnostic; no refit, no changed acceptance; matching tiles move on surfaces between frames")
    fig.colorbar(view, cax=fig.add_axes([.96, .2, .012, .6]), label="Depth (m)")
    fig.subplots_adjust(left=.02, right=.93, top=.9, bottom=.025, wspace=.04, hspace=.1)
    fig.savefig(path, dpi=150); plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists(): parser.error("Choose a new directory")
    report_hash = sha256_file(args.report)
    report = json.loads(args.report.read_text())
    database = next(Path(p) for p in report["input_hashes"] if p.endswith(".db"))
    db_hash = sha256_file(database)
    args.output.mkdir(parents=True)
    (args.output/"source_snapshot.py").write_bytes(Path(__file__).read_bytes())
    session = Session(database, "floor2")
    rows = []
    calibration = []
    case_studies = []
    cv2.setNumThreads(1)
    try:
        for i, record in enumerate(report["records"]):
            a, b = record["a"], record["b"]
            full = [depth_cloud(session, n, np.ones(session.depth(n)[0].shape, bool)) for n in (a, b)]
            before = geometric_score(*full, np.asarray(record["initial_T_a_b"]), .2)
            after = geometric_score(*full, np.asarray(record["T_a_b"]), .2)
            rows.append({"a": a, "b": b, "original_accepted": record["accepted"],
                         "train_inlier_rmse_m": record.get("fit", {}).get("after", {}).get("rmse_m"),
                         "full_before": before, "full_after": after})
            if (a, b) in ((351, 352), (550, 551)):
                for n in (a, b):
                    depth, confidence, cw, ch, k, local = session.depth(n)
                    calibration.append({"node": n, "rgb_shape": list(session.image(n).shape),
                        "depth_shape": list(depth.shape), "calibration_width_height": [cw, ch],
                        "intrinsic": k.tolist(), "local_T_node_optical": local.tolist(),
                        "local_rotation_determinant": float(np.linalg.det(local[:3, :3])),
                        "local_rotation_orthogonality_max_error": float(np.max(np.abs(local[:3, :3].T@local[:3, :3]-np.eye(3))))})
                train = [depth_cloud(session, n, partition_masks(session.depth(n)[0].shape)[0]) for n in (a, b)]
                tn, tg = normals(train[0]); sn, sg = normals(train[1])
                p, q, normal = correspondences(*train, np.asarray(record["T_a_b"]), tn, sn, tg, sg, gate=.35)
                residual = np.abs(np.einsum("ij,ij->i", p-q, normal))
                case = {"a": a, "b": b, "features": features_audit(session, a, b),
                        "point_plane_median_abs_m": float(np.median(residual)),
                        "point_plane_p90_abs_m": float(np.percentile(residual, 90)),
                        "point_plane_matches": len(residual), "original_cycle": record["fit"]["cycle"],
                        "metric_sampling": rows[-1]}
                links = session.connection.execute("SELECT from_id,to_id,type,transform FROM Link WHERE (from_id=? AND to_id=?) OR (from_id=? AND to_id=?)", (a,b,b,a)).fetchall()
                from export_rtabmap_db import transform_from_blob
                case["native_links"] = [{"from": fa, "to": fb, "type": ty,
                    "measurement": transform_from_blob(blob).tolist()} for fa,fb,ty,blob in links]
                path = args.output/f"rgb_depth_{a}_{b}.png"
                montage(session, a, b, path)
                case["figure"] = str(path.resolve())
                case_studies.append(case)
            session.geometry.clear()
            if (i+1) % 40 == 0: print(f"audited {i+1}/{len(report['records'])}", flush=True)
    finally:
        session.connection.close()
    accepted = [r for r in report["records"] if r["accepted"]]
    applicable = [r for r in rows if r["train_inlier_rmse_m"] is not None and r["full_after"]["rmse_m"] is not None]
    both = [r for r in applicable if r["train_inlier_rmse_m"] > .07 and r["full_after"]["rmse_m"] <= .07]
    result = {"created_utc": utc_now(), "command": shlex.join(sys.argv), "source_sha256": sha256_file(Path(__file__)),
        "frozen_report": str(args.report.resolve()), "frozen_report_sha256": report_hash,
        "database": str(database), "database_sha256": db_hash,
        "calibration_arithmetic": calibration, "case_studies": case_studies, "all_fixed_transform_metrics": rows,
        "accepted_test_metrics_all_finite": all(finite_tree(r["test"]) for r in accepted),
        "summary": {"pairs": len(rows), "accepted_unchanged": len(accepted),
            "training_rmse_above_7cm": sum(r["train_inlier_rmse_m"] > .07 for r in applicable),
            "full_depth_rmse_above_7cm": sum(r["full_after"]["rmse_m"] > .07 for r in applicable),
            "above_train_but_below_full_7cm_count": len(both),
            "median_train_inlier_rmse_m": float(np.median([r["train_inlier_rmse_m"] for r in applicable])),
            "median_full_inlier_rmse_m": float(np.median([r["full_after"]["rmse_m"] for r in applicable]))},
        "interpretation": ["No refit and no gate changed. Full-depth metrics include reserved samples and are post-hoc diagnostics only.",
            "Image-space disjoint tiles produce holes at different physical surfaces as the camera moves; point-to-point NN RMSE mixes pose error with sampling spacing and visibility.",
            "A 7cm threshold chosen before this split is conservative but not calibrated to the tile sampling; failing it is not proof of a bad sensor measurement.",
            "Cauchy point-to-plane minimizes normal displacement, not the point-to-point nearest-neighbor metric used by the safety gate.",
            "T_a_b maps source b into target a; reverse ICP maps a into b; T_a_b @ T_b_a is the identity cycle in frame a.",
            "The reverse ICP starts at inverse(forward candidate), so its agreement is a local consistency diagnostic, not independent initialization.",
            "Numeric reprojection consistency does not externally verify hardware RGB-depth synchronization or calibration."]}
    result["inputs_unchanged"] = sha256_file(database) == db_hash and sha256_file(args.report) == report_hash
    if not result["inputs_unchanged"]: raise RuntimeError("Frozen input changed")
    (args.output/"report.json").write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(result["summary"], indent=2))


if __name__ == "__main__": main()
