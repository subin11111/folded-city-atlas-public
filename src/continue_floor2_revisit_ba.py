#!/usr/bin/env python3
"""Continue frozen cross-visit BA using batch residual and dense exact TR solve.

No features, frames, landmarks, depths, calibration, priors or acceptance gates
are selected here. The 324 recorded observations are read directly from the
previous immutable experiment. Only residual evaluation and linear solver vary.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shlex
import sys

import numpy as np
from scipy.optimize import least_squares

from export_rtabmap_db import load_pose_csv
from optimize_session import apply, inverse, sha256_file, utc_now
from recover_geometry_connections import intrinsics_at_rgb
from remeasure_floor2_revisits import Features, ROOT, SETTINGS, rigid_observability
from rgbd_bundle_vectorized import make_bundle_problem


def robust_cost(residual):
    squared = residual**2
    return .5*float(np.sum(np.where(squared <= 1., squared, 2*np.sqrt(squared)-1)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--previous", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-evaluations", type=int, default=1000)
    parser.add_argument("--solver-result", type=Path,
        help="Package an independently hash-verified exact-solver result instead of running a duplicate solve")
    args = parser.parse_args()
    prior_report = json.loads(args.previous.read_text())
    if not prior_report["complete"] or not prior_report["inputs_unchanged"]:
        raise ValueError("Previous experiment incomplete/mutated")
    for path, digest in prior_report["input_hashes"].items():
        if sha256_file(Path(path)) != digest:
            raise ValueError(f"Previous source/input changed: {path}")
    args.output.mkdir(parents=True, exist_ok=False)
    selection_path = args.previous.parent/"selection.json"
    (args.output/"selection.json").write_bytes(selection_path.read_bytes())
    sources = [Path(__file__), ROOT/"src/rgbd_bundle_vectorized.py", ROOT/"src/floor2_vectorized_solver.py"]
    hashes = dict(prior_report["input_hashes"])
    for p in [args.previous.resolve(), selection_path.resolve()] + sources:
        hashes[str(p)] = sha256_file(p)
    supplied_result = None
    if args.solver_result:
        supplied_result = json.loads(args.solver_result.read_text())
        if not supplied_result["complete"] or not supplied_result["input_hashes_unchanged"]:
            raise ValueError("Supplied numerical solve incomplete/mutated")
        for path, digest in supplied_result["equivalence"]["input_hashes"].items():
            if sha256_file(Path(path)) != digest:
                raise ValueError(f"Supplied numerical input changed: {path}")
            hashes[path] = digest
        hashes[str(args.solver_result.resolve())] = sha256_file(args.solver_result)
    for p in sources: (args.output/("source_"+p.name)).write_bytes(p.read_bytes())
    report = dict(prior_report, complete=False, created_utc=utc_now(),
        command=shlex.join(sys.argv), input_hashes=hashes, records=[],
        selection_sha256=sha256_file(args.output/"selection.json"),
        solver_revision="Equivalent vectorized RGB-D residual; dense finite-difference Jacobian and exact trust-region linear algebra replace sparse LSMR; objective and gates preserved")
    if supplied_result:
        report["numerical_solver_report"] = str(args.solver_result.resolve())
        report["numerical_equivalence"] = supplied_result["equivalence"]
        report["solver_revision"] = supplied_result["equivalence"]["solver"]
    session = Features(ROOT/"data/raw/rtabmap/261017-013501-2f.db", "floor2")
    selected = load_pose_csv(ROOT/"outputs/student_union_20261017_geometry_review/graph_final/floor2/initial.csv")
    for old_record in prior_report["records"]:
        record = json.loads(json.dumps(old_record))
        record["surface_hypotheses"] = {"reference_report": str(args.previous.resolve()),
            "reference_sha256": sha256_file(args.previous), "a": record["a"], "b": record["b"],
            "meaning": "Earlier rigid surface hypotheses are not refitted or relabelled as this BA result"}
        if not old_record["bundle"]["ba_executed"]:
            report["records"].append(record); continue
        directory = args.output/f"{record['a']}_{record['b']}"; directory.mkdir()
        observed_path = args.previous.parent/f"{record['a']}_{record['b']}"/"frozen_observations.json"
        raw_observations = json.loads(observed_path.read_text())
        if sha256_file(observed_path) != old_record["bundle"]["frozen_observations_sha256"]:
            raise ValueError("Frozen observations changed")
        hashes[str(observed_path.resolve())] = sha256_file(observed_path)
        (directory/"frozen_observations.json").write_bytes(observed_path.read_bytes())
        observations = [(o["node"], o["landmark"], np.array(o["rgb_pixel"]),
            np.nan if o["optical_depth_m"] is None else o["optical_depth_m"]) for o in raw_observations]
        nodes = sum(record["train_frames"], [])
        calibrations = {n: intrinsics_at_rgb(session, n) for n in nodes}
        candidates = []
        for old in sorted(old_record["bundle"]["candidates"], key=lambda c: c["cost"]):
            if supplied_result and old["name"] != supplied_result["candidate"]["name"]:
                continue
            initial = {int(n): np.array(p) for n, p in old["poses"].items()}
            landmarks = np.array(old["landmarks"])
            priors = [(r["a"], r["b"], inverse(selected[r["a"]]) @ selected[r["b"]],
                r["sigma_m"], np.radians(r["sigma_deg"])) for r in old["priors"]]
            x0, residual, _, unpack = make_bundle_problem(nodes, record["a"], initial,
                observations, landmarks, priors, calibrations)
            initial_cost = robust_cost(residual(x0))
            if not np.isclose(initial_cost, old["cost"], atol=1e-7, rtol=1e-10):
                raise ValueError(f"Objective changed at restart: {initial_cost} != {old['cost']}")
            if supplied_result:
                candidate = supplied_result["candidate"]
                final_poses = {int(n): np.array(p) for n, p in candidate["poses"].items()}
                xf, final_residual, _, _ = make_bundle_problem(nodes, record["a"], final_poses,
                    observations, np.array(candidate["landmarks"]), priors, calibrations)
                if not np.isclose(robust_cost(final_residual(xf)), candidate["cost"], atol=1e-7, rtol=1e-10):
                    raise ValueError("Supplied final candidate does not match frozen observation objective")
                candidates.append(candidate)
                (directory/(old["name"]+"_candidate.json")).write_text(json.dumps(candidate, indent=2)+"\n")
                print(f"PACKAGED EXACT BA: success={candidate['success']} nfev={candidate['nfev']} cost={candidate['cost']:.6f}", flush=True)
                continue
            fit = least_squares(residual, x0, jac="2-point", tr_solver="exact", loss="huber",
                f_scale=1., max_nfev=args.max_evaluations, x_scale="jac",
                ftol=1e-7, xtol=1e-7, gtol=1e-7)
            poses, points = unpack(fit.x)
            residuals = residual(fit.x)[:3*len(observations)].reshape(-1, 3)
            reproj = np.linalg.norm(residuals[:, :2], axis=1)*SETTINGS["reprojection_sigma_px"]
            valid_depth = np.array([np.isfinite(o[3]) for o in observations])
            depth = np.abs(residuals[valid_depth, 2])*SETTINGS["depth_sigma_m"]
            positive = all(apply(points[j:j+1], inverse(poses[n] @ calibrations[n][1]))[0, 2] > .1
                for n, j, *_ in observations)
            candidate = dict(old, success=bool(fit.success), message=fit.message, status=fit.status,
                nfev=fit.nfev, njev=fit.njev, previous_nfev=old["nfev"], previous_cost=old["cost"],
                objective_restart_cost=initial_cost, cost=float(fit.cost), optimality=float(fit.optimality),
                reprojection_median_px=float(np.median(reproj)), reprojection_p90_px=float(np.percentile(reproj, 90)),
                depth_median_abs_m=float(np.median(depth)), depth_p90_abs_m=float(np.percentile(depth, 90)),
                positive_depth=positive, fixed_landmark_rigid_observability=rigid_observability(points),
                T_a_b=(inverse(poses[record["a"]]) @ poses[record["b"]]).tolist(),
                poses={str(n): p.tolist() for n, p in poses.items()}, landmarks=points.tolist(),
                continuation_settings={"max_evaluations": args.max_evaluations, "linear_solver": "dense exact",
                    "jacobian": "2-point finite difference", "ftol": 1e-7, "xtol": 1e-7, "gtol": 1e-7})
            candidates.append(candidate)
            (directory/(old["name"]+"_candidate.json")).write_text(json.dumps(candidate, indent=2)+"\n")
            print(f"EXACT BA {record['a']}/{record['b']} {old['name']}: success={fit.success} nfev={fit.nfev} cost={fit.cost:.6f} optimality={fit.optimality:.3g}", flush=True)
        chosen = min(candidates, key=lambda c: c["cost"])
        reasons = []
        if not chosen["success"]: reasons.append("bundle_optimizer_not_converged")
        if not chosen["positive_depth"]: reasons.append("nonpositive_landmark_depth")
        if chosen["reprojection_p90_px"] > SETTINGS["maximum_reprojection_p90_px"]: reasons.append("reprojection_p90_above_8px")
        if chosen["depth_p90_abs_m"] > SETTINGS["maximum_depth_p90_m"]: reasons.append("depth_p90_above_10cm")
        if chosen["fixed_landmark_rigid_observability"]["rank"] < 6: reasons.append("rigid_correspondence_geometry_degenerate")
        bundle = dict(record["bundle"], candidates=candidates, chosen=chosen["name"],
            accepted=not reasons, reasons=reasons, T_a_b=chosen["T_a_b"])
        record.update(bundle=bundle, accepted=not reasons, T_a_b=chosen["T_a_b"])
        record["camera_visual_support_counts"] = {str(n): sum(o[0] == n for o in observations) for n in nodes}
        record["free_camera_parameters"] = 6*(len(nodes)-1)
        record["landmark_parameters"] = 3*len(chosen["landmarks"])
        (directory/"bundle.json").write_text(json.dumps(bundle, indent=2)+"\n")
        report["records"].append(record)
        (args.output/"report.json").write_text(json.dumps(report, indent=2)+"\n")
    session.connection.close()
    report["inputs_unchanged"] = all(sha256_file(Path(p)) == h for p, h in hashes.items())
    if not report["inputs_unchanged"]: raise RuntimeError("Source/input changed during exact solve")
    report.update(complete=True, finished_utc=utc_now())
    (args.output/"report.json").write_text(json.dumps(report, indent=2)+"\n")


if __name__ == "__main__":
    main()
