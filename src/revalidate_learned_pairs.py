#!/usr/bin/env python3
"""LoFTR fallback retrieval verification, not permission to join unsupported maps.

LoFTR only supplies candidate pixels. Confidence-filtered RGB-D unprojection,
nondegenerate 3-D support, disjoint held-out depth neighborhoods, bidirectional
ICP, feature/reprojection holdouts and three independent session supports remain
mandatory. No stored/map transform initializes local correspondence estimation.
Official pretrained models run locally; no user image or DB leaves this machine.
"""
from __future__ import annotations

import argparse
from collections import Counter
import copy
import hashlib
import json
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import cv2
import numpy as np

import check_session_overlap as checker
import revalidate_cross_sessions as cross
import revalidate_loop_constraints as intra
from optimize_session import inverse, sha256_file


SETTINGS = {"matcher": "kornia.feature.LoFTR", "kornia_version": "0.7.1",
            "max_image_side": 480, "resize_multiple": 8, "minimum_confidence": .5,
            "depth_thinning_cell_pixels": 6, "cpu_threads": 2,
            "candidate_budget_per_session_pair": 40,
            "reprojection_median_max_px": 6., "reprojection_p90_max_px": 15.,
            "seed": 20261017}
OFFICIAL_URLS = {
    "outdoor": "https://cmp.felk.cvut.cz/~mishkdmy/models/loftr_outdoor.ckpt",
    "indoor": "https://cmp.felk.cvut.cz/~mishkdmy/models/loftr_indoor.ckpt"}
DOC_URL = "https://kornia.readthedocs.io/en/v0.7.1/_modules/kornia/feature/loftr/loftr.html"
INTRA_PRIORITIES = [("bridge_out1f", 41, 63), ("floor2", 474, 940),
                    ("floor2", 548, 981), ("floor2", 503, 961),
                    ("bridge_12_sub", 20, 74), ("bridge_12_sub", 75, 209)]


def resized_shape(shape, max_side=480, multiple=8):
    height, width = shape[:2]
    scale = min(1., max_side / max(height, width))
    return max(multiple, int(width * scale) // multiple * multiple), max(multiple, int(height * scale) // multiple * multiple)


def restore_pixels(pixels, original_shape, resized_wh):
    height, width = original_shape[:2]
    return np.asarray(pixels, dtype=float) * [width / resized_wh[0], height / resized_wh[1]]


def spatial_thin(depth_a, depth_b, confidence, cell=6):
    """Keep the strongest match per cell in BOTH depth images, before holdout."""
    selected, used_a, used_b = [], set(), set()
    for i in np.argsort(-np.asarray(confidence), kind="stable"):
        a, b = tuple(np.floor(depth_a[i] / cell).astype(int)), tuple(np.floor(depth_b[i] / cell).astype(int))
        if a in used_a or b in used_b:
            continue
        selected.append(int(i)); used_a.add(a); used_b.add(b)
    return np.asarray(selected, dtype=int)


def select_candidates(checkpoint, first, second, limit):
    """Mix depth/SIFT ranking with temporal spread; scores are NOT geometric proof."""
    candidates = {(int(r["outside_id"]), int(r["lobby_id"])): dict(r)
                  for r in checkpoint["selected_candidates"]}
    prior = {(int(r["outside_id"]), int(r["lobby_id"])): r for r in checkpoint.get("pair_results", [])}
    for key, row in candidates.items():
        previous = prior.get(key, {})
        row["prior_sift_valid_depth_matches"] = previous.get("valid_depth_matches", 0)
        row["prior_sift_mutual_matches"] = previous.get("mutual_matches", 0)
        row["prior_sift_reason"] = previous.get("reason", "not_completed")
    selected, used = [], set()

    def add(row, reason):
        key = row["outside_id"], row["lobby_id"]
        if key in used: return False
        if any(abs(first.stamps[key[0]] - first.stamps[r["outside_id"]]) < 3.
               and abs(second.stamps[key[1]] - second.stamps[r["lobby_id"]]) < 3.
               for r in selected): return False
        selected.append({**row, "selection_reason": reason}); used.add(key)
        return True

    ranked_depth = sorted(candidates.values(), key=lambda r: (-r["prior_sift_valid_depth_matches"],
                           -r["prior_sift_mutual_matches"], -r["retrieval_score"], r["outside_id"], r["lobby_id"]))
    for row in ranked_depth:
        if len(selected) >= max(1, limit // 2): break
        add(row, "prior_depth_and_sift_rank")
    for row in sorted(candidates.values(), key=lambda r: (-r["prior_sift_mutual_matches"], -r["retrieval_score"])):
        if len(selected) >= max(1, 3 * limit // 4): break
        add(row, "prior_sift_rank")
    scale_a = max(first.stamps.values()) - min(first.stamps.values()) or 1.
    scale_b = max(second.stamps.values()) - min(second.stamps.values()) or 1.
    while len(selected) < limit:
        remaining = [r for k, r in candidates.items() if k not in used]
        if not remaining: break
        def spread(row):
            return min(((first.stamps[row["outside_id"]] - first.stamps[r["outside_id"]]) / scale_a)**2
                       + ((second.stamps[row["lobby_id"]] - second.stamps[r["lobby_id"]]) / scale_b)**2
                       for r in selected) if selected else row["retrieval_score"]
        row = max(remaining, key=spread)
        key = row["outside_id"], row["lobby_id"]
        if not add(row, "temporal_spread_of_retrieval_candidates"):
            used.add(key)
    return selected


class Matcher:
    def __init__(self, model_name, weights):
        import torch
        import kornia
        from kornia.feature.loftr.loftr import default_cfg
        if kornia.__version__ != SETTINGS["kornia_version"]:
            raise ValueError("This reproducible run requires Kornia 0.7.1")
        torch.set_num_threads(2)
        torch.manual_seed(SETTINGS["seed"])
        self.torch = torch
        self.model = kornia.feature.LoFTR(pretrained=None, config=copy.deepcopy(default_cfg))
        # These checkpoints originate from the official model URL recorded below.
        # Do not point this loader at an untrusted user-supplied pickle checkpoint.
        self.model.load_state_dict(torch.load(str(weights), map_location="cpu")["state_dict"])
        self.model.eval()
        self.metadata = {"model_name": model_name, "official_source_url": OFFICIAL_URLS[model_name],
                         "weights_path": str(weights.resolve()), "weights_sha256": sha256_file(weights),
                         "torch_version": torch.__version__, "kornia_version": kornia.__version__,
                         "official_documentation": DOC_URL,
                         "pretraining_dataset": "MegaDepth" if model_name == "outdoor" else "ScanNet"}

    def match(self, first, second):
        shapes = [resized_shape(image.shape) for image in (first, second)]
        tensors = [self.torch.from_numpy(cv2.resize(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), shape,
                    interpolation=cv2.INTER_AREA).astype(np.float32) / 255)[None, None]
                   for image, shape in zip((first, second), shapes)]
        with self.torch.inference_mode():
            result = self.model({"image0": tensors[0], "image1": tensors[1]})
        confidence = result["confidence"].cpu().numpy()
        valid = confidence >= SETTINGS["minimum_confidence"]
        pixa = restore_pixels(result["keypoints0"].cpu().numpy()[valid], first.shape, shapes[0])
        pixb = restore_pixels(result["keypoints1"].cpu().numpy()[valid], second.shape, shapes[1])
        return pixa, pixb, confidence[valid], {"unfiltered_matches": len(confidence),
                    "confidence_filtered_matches": int(valid.sum()), "resized_shapes_wh": shapes}


def reprojection_metrics(first, second, a, b, transform, source, target, pixa, pixb):
    forward = cross.project_node_points(first, a, checker.apply_transform(transform, source))
    reverse = cross.project_node_points(second, b, checker.apply_transform(inverse(transform), target))
    errors = np.r_[np.linalg.norm(forward - pixa, axis=1), np.linalg.norm(reverse - pixb, axis=1)]
    finite = bool(np.isfinite(errors).all())
    result = {"count_both_directions": len(errors), "finite": finite,
              "median_px": float(np.median(errors)) if finite else None,
              "p90_px": float(np.percentile(errors, 90)) if finite else None}
    accepted = finite and result["median_px"] <= 6. and result["p90_px"] <= 15.
    return accepted, result


def verify_pixels(first, second, candidate, pixa, pixb, confidence, output):
    result = {**candidate, "accepted": False, "reason": "not_evaluated"}
    a, b = result["outside_id"], result["lobby_id"]
    result["learned_confident_matches"] = len(pixa)
    if len(pixa) < checker.DEFAULTS["minimum_mutual_matches"]:
        result["reason"] = "too_few_confident_learned_matches"; return result
    target, source = first.unproject(a, pixa), second.unproject(b, pixb)
    valid = np.isfinite(target).all(axis=1) & np.isfinite(source).all(axis=1)
    target, source, pixa, pixb, confidence = target[valid], source[valid], pixa[valid], pixb[valid], confidence[valid]
    result["valid_depth_matches_before_thinning"] = len(source)
    pixel_a, pixel_b = intra.depth_pixels(first, a, pixa), intra.depth_pixels(second, b, pixb)
    selected = spatial_thin(pixel_a, pixel_b, confidence, SETTINGS["depth_thinning_cell_pixels"])
    target, source, pixa, pixb, pixel_a, pixel_b, confidence = [array[selected] for array in
        (target, source, pixa, pixb, pixel_a, pixel_b, confidence)]
    result["valid_depth_matches"] = len(source)
    if len(source) < checker.DEFAULTS["minimum_depth_matches"]:
        result["reason"] = "too_few_spatially_distinct_valid_depth_matches"; return result
    token = f"{first.label}:{a}:{second.label}:{b}:{SETTINGS['seed']}".encode()
    seed = int.from_bytes(hashlib.sha256(token).digest()[:4], "little")
    train, held, masks = intra.split_correspondences(pixel_a, pixel_b, first.depth(a)[0].shape,
                                                    second.depth(b)[0].shape, seed)
    evidence = output / "correspondences" / f"{a}_{b}.npz"
    np.savez_compressed(evidence, target=target, source=source, rgb_pixels_a=pixa, rgb_pixels_b=pixb,
                        depth_pixels_a=pixel_a, depth_pixels_b=pixel_b, confidence=confidence,
                        train_indices=train, heldout_indices=held)
    result.update({"ransac_seed": seed, "train_count": len(train), "heldout_count": len(held),
                   "correspondence_evidence": str(evidence.resolve()),
                   "excluded_depth_pixels": [int(mask.sum()) for mask in masks],
                   "holdout_depth_exclusion_radius_pixels": 3})
    if len(train) < 12 or len(held) < 5:
        result["reason"] = "too_few_disjoint_train_or_holdout"; return result
    transform, inliers, trials = checker.rigid_ransac(source[train], target[train], seed)
    result.update({"train_inliers": int(inliers.sum()), "train_inlier_ratio": float(inliers.mean()),
                   "ransac_valid_trials": trials, "visual_T_a_b": transform.tolist()})
    if inliers.sum() < 12 or inliers.mean() < .4:
        result["reason"] = "insufficient_3d_ransac_consensus"; return result
    supported = []
    for points, pixels, session, node, label in ((target[train][inliers], pixa[train][inliers], first, a, "a"),
                                                (source[train][inliers], pixb[train][inliers], second, b, "b")):
        # Fixed 3D-to-3D correspondences can identify SE(3) on a noncollinear
        # textured plane. This is not the rank-deficient point-to-plane case.
        okay, diagnostic = intra.planar_aware_support(points, pixels, session.image(node).shape)
        result["spatial_support_" + label] = diagnostic; supported.append(okay)
    result["visual_heldout"] = intra.holdout_score(transform, source[held], target[held])
    if not all(supported):
        result["reason"] = "degenerate_or_narrow_3d_support"; return result
    cloud_a, cloud_b = intra.cloud_without_holdout(first, a, masks[0]), intra.cloud_without_holdout(second, b, masks[1])
    result["holdout_excluded_cloud_points"] = [len(cloud_a), len(cloud_b)]
    if min(len(cloud_a), len(cloud_b)) < 100:
        result["reason"] = "insufficient_holdout_excluded_icp_cloud"; return result
    forward, fm = checker.refine_icp(cloud_a, cloud_b, transform)
    reverse, rm = checker.refine_icp(cloud_b, cloud_a, inverse(transform))
    cycle = intra.delta_summary(np.eye(4), forward @ reverse)
    held_forward = intra.holdout_score(forward, source[held], target[held])
    held_reverse = intra.holdout_score(reverse, target[held], source[held])
    reprojection_ok, reprojection = reprojection_metrics(first, second, a, b, forward,
        source[held], target[held], pixa[held], pixb[held])
    world_rotation = first.raw_poses[a][:3, :3] @ forward[:3, :3] @ second.raw_poses[b][:3, :3].T
    tilt = float(np.degrees(np.arccos(np.clip(world_rotation[2, 2], -1., 1.))))
    result.update({"icp": fm, "reverse_icp": rm, "icp_cycle": cycle,
                   "heldout": held_forward, "reverse_heldout": held_reverse,
                   "heldout_reprojection": reprojection, "world_gravity_disagreement_degrees": tilt,
                   "lobby_node_to_outside_node": forward.tolist(), "T_b_a_independent_icp": reverse.tolist()})
    failures = []
    for label, score in (("forward", held_forward), ("reverse", held_reverse)):
        if score["inliers"] < 5 or score["inlier_ratio"] < .5 or score["rmse_m"] > .07:
            failures.append(label + "_heldout_failed")
    for label, score in (("forward", fm), ("reverse", rm)):
        if score["symmetric_overlap"] < .4 or score["symmetric_rmse_m"] > .07:
            failures.append(label + "_icp_geometry_failed")
        if score["refinement_m"] > .15 or score["refinement_deg"] > 5.:
            failures.append(label + "_icp_large_change")
    if cycle["translation_m"] > .10 or cycle["rotation_deg"] > 2.:
        failures.append("icp_cycle_failed")
    if tilt > 10.: failures.append("gravity_disagreement")
    if not reprojection_ok: failures.append("heldout_reprojection_failed")
    result["accepted"] = not failures
    result["reason"] = ";".join(failures) if failures else "learned_rgbd_geometry_verified"
    result["covariance_model"] = intra.covariance_model()
    return result


def match_image(first, second, pixa, pixb, destination):
    shape_a, shape_b = resized_shape(first.shape), resized_shape(second.shape)
    ia, ib = cv2.resize(first, shape_a), cv2.resize(second, shape_b)
    canvas = np.zeros((max(ia.shape[0], ib.shape[0]), ia.shape[1] + ib.shape[1], 3), np.uint8)
    canvas[:ia.shape[0], :ia.shape[1]] = ia
    canvas[:ib.shape[0], ia.shape[1]:] = ib
    aa = pixa * np.array(shape_a) / [first.shape[1], first.shape[0]]
    bb = pixb * np.array(shape_b) / [second.shape[1], second.shape[0]] + [ia.shape[1], 0]
    for i in np.linspace(0, max(0, len(aa)-1), min(40, len(aa))).astype(int):
        cv2.line(canvas, tuple(np.rint(aa[i]).astype(int)), tuple(np.rint(bb[i]).astype(int)), (30, 220, 80), 1)
    cv2.imwrite(str(destination), canvas)


def run_intra_priorities(args, sessions, pose_sets, models, source_hashes, db_hashes):
    root = args.output / "intra_priority"
    records, edges = [], []
    for name, a, b in INTRA_PRIORITIES:
        output = root / name
        (output / "matches").mkdir(parents=True, exist_ok=True)
        (output / "correspondences").mkdir(exist_ok=True)
        session = sessions[name]
        model_name = "indoor" if args.model == "auto" else args.model
        if model_name not in models:
            models[model_name] = Matcher(model_name, args.weights_dir / f"loftr_{model_name}.ckpt")
        matcher = models[model_name]
        ia, ib = session.image(a), session.image(b)
        pixa, pixb, confidence, metadata = matcher.match(ia, ib)
        match_image(ia, ib, pixa, pixb, output / "matches" / f"{a}_{b}.jpg")
        result = verify_pixels(session, session, {"outside_id": a, "lobby_id": b,
                              "selection_reason": "predeclared_problematic_stored_intra_loop"},
                               pixa, pixb, confidence, output)
        previous = Path("outputs") / intra.SPECS[name][1] / name
        old_holdout, old_trained, _ = intra.old_graph_roles(previous)
        held, role = intra.graph_holdout(name, SimpleNamespace(a=a, b=b), old_holdout, old_trained)
        result.update({"session_a": name, "session_b": name, "a": a, "b": b,
                       "matcher_output": metadata, "matcher": matcher.metadata,
                       "held_out": held, "graph_role_reason": role,
                       "database_sha256": db_hashes[name], "source_hashes": source_hashes,
                       "parameters": SETTINGS, "geometry_parameters": checker.DEFAULTS,
                       "holdout_parameters": intra.SETTINGS})
        records.append(result)
        if result["accepted"]:
            in_main = a in pose_sets[name]["raw"] and b in pose_sets[name]["raw"]
            edges.append({"edge_id": f"{name}:{a}__{name}:{b}:learned", "session_a": name,
                "node_a": a, "session_b": name, "node_b": b,
                "T_a_b": result["lobby_node_to_outside_node"],
                "transform_convention": "T_a_b maps node B coordinates into node A coordinates",
                "local_geometry_verified": True, "nodes_in_main_components": in_main,
                "eligible_for_joint": in_main, "held_out": held, "joint_holdout": held,
                "joint_role": "heldout" if held else "train", "initialization_allowed": False,
                "sigma_translation_m": .10, "sigma_rotation_deg": 3., "metrics": result,
                "measurement_source": "LoFTR with disjoint-heldout RGB-D verification",
                "limitation": "Same-sensor validation; original whole-pair holdouts preserved; not ground truth."})
        cross.atomic_json(output / f"pair_{a}_{b}.json", result)
        cross.atomic_json(args.output / "intra_accepted_frame_edges.json", edges)
        cross.atomic_json(root / "report.json", {"command": sys.argv, "records": records,
            "accepted": len(edges), "complete": len(records) == len(INTRA_PRIORITIES)})
        print(f"LoFTR intra {name} {a}/{b}: {result['reason']} depth={result.get('valid_depth_matches',0)}", flush=True)
        session.geometry.clear()
    return edges


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate-source", type=Path, default=Path("outputs/student_union_20261017_revalidated/cross_session"))
    parser.add_argument("--output", type=Path, default=Path("outputs/student_union_20261017_revalidated/learned_cross_session"))
    parser.add_argument("--weights-dir", type=Path, default=Path("outputs/student_union_20261017_revalidated/model_cache/checkpoints"))
    parser.add_argument("--pairs", nargs="+", type=cross.parse_pair)
    parser.add_argument("--candidate-limit", type=int, default=40)
    parser.add_argument("--model", choices=("auto", "outdoor", "indoor"), default="auto")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--intra-only", action="store_true", help="Only the six predeclared problematic stored loops")
    parser.add_argument("--skip-intra-priorities", action="store_true")
    args = parser.parse_args()
    if args.candidate_limit < 1: parser.error("Candidate limit must be positive")
    workspace = Path(__file__).resolve().parents[1]
    os.chdir(workspace)
    cv2.setNumThreads(2)
    pairs = [] if args.intra_only else (args.pairs or cross.PRIORITY_PAIRS)
    args.output.mkdir(parents=True, exist_ok=True)
    models, sessions, pose_sets, pose_metadata = {}, {}, {}, {}
    names = {x for pair in pairs for x in pair}
    if not args.skip_intra_priorities:
        names |= {name for name, _, _ in INTRA_PRIORITIES}
    for name in sorted(names):
        sessions[name] = checker.Session(workspace / "data/raw/rtabmap" / cross.SESSION_DATABASES[name], name)
        pose_sets[name], pose_metadata[name] = cross.pose_sources(workspace, name, sessions[name])
    source_paths = [Path(__file__), Path(checker.__file__), Path(cross.__file__), Path(intra.__file__),
                    workspace / "src/export_rtabmap_db.py"]
    source_hashes = {path.name: sha256_file(path) for path in source_paths}
    db_hashes = {name: sha256_file(session.path) for name, session in sessions.items()}
    if not args.skip_intra_priorities:
        run_intra_priorities(args, sessions, pose_sets, models, source_hashes, db_hashes)
    summaries, all_edges, all_proposals = [], [], []
    for name_a, name_b in pairs:
        started = time.monotonic()
        first, second = sessions[name_a], sessions[name_b]
        model_name = args.model if args.model != "auto" else ("outdoor" if "outside" in (name_a, name_b) else "indoor")
        if model_name not in models:
            models[model_name] = Matcher(model_name, args.weights_dir / f"loftr_{model_name}.ckpt")
        matcher = models[model_name]
        source_path = args.candidate_source / f"{name_a}__{name_b}" / "verification_checkpoint.json"
        source = json.loads(source_path.read_text())
        selected = select_candidates(source, first, second, args.candidate_limit)
        output = args.output / f"{name_a}__{name_b}"
        (output / "correspondences").mkdir(parents=True, exist_ok=True)
        (output / "matches").mkdir(exist_ok=True)
        provenance = {"database_sha256": {name_a: db_hashes[name_a], name_b: db_hashes[name_b]},
                      "source_hashes": source_hashes, "matcher": matcher.metadata,
                      "parameters": {**SETTINGS, "candidate_budget_per_session_pair": args.candidate_limit},
                      "geometry_parameters": checker.DEFAULTS, "holdout_parameters": intra.SETTINGS,
                      "pose_sources": {name_a: pose_metadata[name_a], name_b: pose_metadata[name_b]},
                      "selected_candidates": selected, "candidate_source_path": str(source_path.resolve()),
                      "candidate_source_sha256": sha256_file(source_path), "semantic_mask_applied": False}
        checkpoint = output / "verification_checkpoint.json"
        results = []
        if checkpoint.exists():
            old = json.loads(checkpoint.read_text())
            if not args.resume or old["provenance"] != provenance:
                raise ValueError("Existing learned evidence needs --resume and exact provenance, or fresh output")
            results = old["pair_results"]
        for candidate in selected[len(results):]:
            a, b = candidate["outside_id"], candidate["lobby_id"]
            ia, ib = first.image(a), second.image(b)
            pixa, pixb, confidence, metadata = matcher.match(ia, ib)
            match_image(ia, ib, pixa, pixb, output / "matches" / f"{a}_{b}.jpg")
            result = verify_pixels(first, second, candidate, pixa, pixb, confidence, output)
            result.update({"session_a": name_a, "session_b": name_b, "matcher_output": metadata})
            results.append(result)
            cross.atomic_json(checkpoint, {"provenance": provenance, "pair_results": results})
            print(f"LoFTR {name_a}/{name_b} {len(results)}/{len(selected)} {a}/{b}: {result['reason']} depth={result.get('valid_depth_matches',0)}", flush=True)
            # Bound per-session geometry cache; learned extraction does not use SIFT.
            first.geometry.clear(); second.geometry.clear()
        proposals = cross.clusters_across_pose_sources(results, first, second, pose_sets[name_a], pose_sets[name_b])
        edges = cross.edge_records(results, proposals, name_a, name_b, pose_sets[name_a], pose_sets[name_b])
        for edge in edges:
            edge["held_out"] = edge["joint_holdout"]
            edge["sigma_translation_m"], edge["sigma_rotation_deg"] = .10, 3.
            edge["measurement_source"] = "LoFTR followed by strict disjoint-heldout RGB-D validation"
        for i, proposal in enumerate(proposals):
            proposal.update(session_a=name_a, session_b=name_b, cluster_id=i,
                            proposal_id=f"{name_a}__{name_b}:learned:{i}")
        summary = {"session_a": name_a, "session_b": name_b, "provenance": provenance,
                   "command": sys.argv, "checked_pairs": len(results), "local_verified": len(edges),
                   "eligible_for_joint": sum(e["eligible_for_joint"] for e in edges),
                   "failure_counts": dict(Counter(r["reason"] for r in results if not r["accepted"])),
                   "elapsed_seconds": time.monotonic() - started,
                   "limitations": ["Bounded previous appearance retrieval may miss real overlap; negative result is not proof of no overlap.",
                       "Learned match confidence is not geometric acceptance or calibrated probability.",
                       "Held-out depth neighborhoods are excluded from RANSAC and ICP, but the same sensor and nearby samples remain correlated.",
                       "Modelled constraint covariance is an explicit assumption, not calibrated sensor uncertainty.",
                       "Session consensus uses three temporally/spatially independent supports; no unsupported map merge occurs here."]}
        cross.atomic_json(output / "report.json", summary)
        cross.atomic_json(output / "accepted_frame_edges.json", edges)
        cross.atomic_json(output / "session_alignment_proposals.json", proposals)
        checker.write_csv(output / "candidate_results.csv", results)
        summaries.append(summary); all_edges.extend(edges); all_proposals.extend(proposals)
        cross.atomic_json(args.output / "accepted_frame_edges.json", all_edges)
        cross.atomic_json(args.output / "session_alignment_proposals.json", all_proposals)
        cross.atomic_json(args.output / "summary.json", {"status": "complete" if len(summaries) == len(pairs) else "in_progress",
            "pair_summaries": summaries, "accepted_frame_edges": len(all_edges),
            "eligible_for_joint_edges": sum(e["eligible_for_joint"] for e in all_edges),
            "cross_session_map_merge_performed": False})
    after = {name: sha256_file(session.path) for name, session in sessions.items()}
    if after != db_hashes: raise RuntimeError("Original database hash changed during evaluation")
    cross.atomic_json(args.output / "database_integrity.json", {"before": db_hashes, "after": after, "unchanged": True})
    for session in sessions.values(): session.connection.close()


if __name__ == "__main__":
    main()
