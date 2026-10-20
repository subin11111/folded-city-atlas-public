#!/usr/bin/env python3
"""Publish matched-support patch metrics and actual experiment images.

Each candidate is compared with both baselines on exactly the patches for
which that candidate has a finite value. Missing observations remain missing.
This is an internal consistency report, never an accuracy certification.
Image packaging reuses the pixel-equality verifier from the previous review.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from publish_geometry_evidence import encoded_png


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stats(values):
    a = np.asarray(values, dtype=float)
    return {"count": len(a), "median_m": float(np.median(a)) if len(a) else None,
            "p90_m": float(np.quantile(a, .9)) if len(a) else None,
            "maximum_m": float(a.max()) if len(a) else None}


def patch_key(row):
    return tuple(row[k] for k in ("session", "a", "b", "kind", "plane_a", "plane_b"))


def paired_summary(report, variant):
    if variant not in report["candidate_stages"]:
        raise ValueError(f"Unknown candidate {variant}")
    rows = report["comparisons"]
    keys = [patch_key(r) for r in rows]
    if len(keys) != len(set(keys)):
        raise ValueError("Duplicate fixed patch identity")
    available = []
    for r in rows:
        values = r["stage_plane_abs_median_m"]
        value = values.get(variant)
        if value is None:
            continue
        if not np.isfinite(value) or value < 0:
            raise ValueError("Non-finite or negative patch distance")
        if any(values.get(k) is None or not np.isfinite(values[k])
               for k in ("previous_preview", "selected_initial")):
            raise ValueError("Candidate lacks paired baseline")
        available.append(r)
    stages = {name: stats([r["stage_plane_abs_median_m"][name] for r in available])
              for name in ("previous_preview", "selected_initial", variant)}
    threshold = report["severe_regression_threshold_m"]
    regressions = {}
    for baseline in ("previous_preview", "selected_initial"):
        delta = [r["stage_plane_abs_median_m"][variant] -
                 r["stage_plane_abs_median_m"][baseline] for r in available]
        regressions[baseline] = sum(d > threshold for d in delta)
    return {"total_patches": len(rows), "available_patches": len(available),
            "missing_patches": len(rows)-len(available),
            "coverage_fraction": len(available)/len(rows) if rows else 0.,
            "same_support_metrics": stages,
            "severe_regression_threshold_m": threshold,
            "severe_regressions_vs_each_baseline": regressions,
            "severe_regressions_vs_per_patch_best_baseline": sum(
                r["stage_plane_abs_median_m"][variant] - min(
                    r["stage_plane_abs_median_m"]["previous_preview"],
                    r["stage_plane_abs_median_m"]["selected_initial"]) > threshold
                for r in available),
            "missing_patch_ids": [list(patch_key(r)) for r in rows
                                  if r["stage_plane_abs_median_m"].get(variant) is None]}


def render_matched(candidates, output):
    fig, axes = plt.subplots(1, 3, figsize=(15, 5), layout="constrained")
    x = np.arange(len(candidates))
    colors = ["#777777", "#d18a2f", "#2379b5"]
    for ax, metric, title in zip(axes, ["median_m", "p90_m", "maximum_m"],
                                 ["Median", "P90", "Maximum"]):
        for offset, name, color in zip([-.24, 0., .24],
                                       ["previous_preview", "selected_initial", "candidate"], colors):
            vals = []
            for c in candidates:
                key = c["variant"] if name == "candidate" else name
                value = c["metrics"]["same_support_metrics"][key][metric]
                vals.append(np.nan if value is None else 100*value)
            ax.bar(x+offset, vals, width=.23, label=name.replace("_", " "), color=color)
        ax.set_xticks(x, [c["label"] for c in candidates], rotation=30, ha="right")
        ax.set_title(title); ax.set_ylabel("Fixed patch separation (cm)")
        ax.grid(axis="y", alpha=.2)
    axes[0].legend(fontsize=8)
    counts = "; ".join(f"{c['label']}: {c['metrics']['available_patches']}/{c['metrics']['total_patches']}"
                       for c in candidates)
    fig.suptitle("Actual frozen patches | identical available support within each candidate comparison\n"
                 + counts + "\nHistorical sensor consistency; not surveyed accuracy", fontsize=11)
    fig.savefig(output, dpi=200); plt.close(fig)


def render_critical(candidates, reports, output):
    wanted = [("floor2", 656, 1055, "wall", 0, 1),
              ("lobby", 726, 738, "wall", 0, 0),
              ("floor2", 548, 981, "wall", 0, 0),
              ("floor2", 503, 961, "horizontal", 0, 0),
              ("outside", 69, 323, "wall", 0, 0)]
    reference = {patch_key(r):r for r in reports[0]["comparisons"]}
    matrix = []
    for key in wanted:
        values = [reference[key]["stage_plane_abs_median_m"][s]
                  for s in ("previous_preview", "selected_initial")]
        for candidate, report in zip(candidates, reports):
            row = next((r for r in report["comparisons"] if patch_key(r) == key), None)
            v = row["stage_plane_abs_median_m"].get(candidate["variant"]) if row else None
            values.append(np.nan if v is None else v)
        matrix.append(values)
    a = 100*np.asarray(matrix)
    fig, ax = plt.subplots(figsize=(max(10, len(candidates)*1.8), 5), layout="constrained")
    cmap=plt.get_cmap("magma_r").copy(); cmap.set_bad("#dddddd")
    im=ax.imshow(a, cmap=cmap, aspect="auto", vmin=0)
    ax.set_xticks(range(a.shape[1]), ["previous preview", "selected initial"]+
                  [c["label"] for c in candidates], rotation=25, ha="right")
    ax.set_yticks(range(len(wanted)), [f"{k[0]} {k[1]}/{k[2]} {k[3]}" for k in wanted])
    for (i,j), v in np.ndenumerate(a):
        ax.text(j,i,"missing" if not np.isfinite(v) else f"{v:.1f}",ha="center",va="center",
                color="white" if np.isfinite(v) and v > np.nanmax(a)*.45 else "black")
    fig.colorbar(im,ax=ax,label="Fixed patch separation (cm)")
    ax.set_title("Fixed observations across experiments | missing is not improvement\n"
                 "Some correspondences supplied training; no independent accuracy claim")
    fig.savefig(output,dpi=200);plt.close(fig)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--patch-report",nargs=3,action="append",required=True,
                   metavar=("LABEL","VARIANT","PATH"))
    p.add_argument("--image",nargs=3,action="append",default=[],
                   metavar=("SOURCE","DESTINATION","CAPTION"))
    p.add_argument("--output",type=Path,required=True)
    args=p.parse_args()
    if args.output.exists(): raise FileExistsError(args.output)
    reports=[];candidates=[]
    for label,variant,path in args.patch_report:
        report=json.loads(Path(path).read_text()); reports.append(report)
        candidates.append({"label":label,"variant":variant,"report":str(Path(path).resolve()),
                           "report_sha256":digest(path),"metrics":paired_summary(report,variant)})
    # Cross-report baselines must be the same historical observations.
    reference={patch_key(r):r["stage_plane_abs_median_m"] for r in reports[0]["comparisons"]}
    for report in reports[1:]:
        if {patch_key(r) for r in report["comparisons"]} != set(reference):
            raise ValueError("Historical patch inventory changed")
        for row in report["comparisons"]:
            key=patch_key(row)
            if key not in reference: raise ValueError("Historical patch inventory changed")
            for stage in ("previous_preview","selected_initial"):
                if not np.isclose(row["stage_plane_abs_median_m"][stage],reference[key][stage],atol=1e-10,rtol=0):
                    raise ValueError("Historical baseline values changed")
    prepared=[];names=set()
    for source,name,caption in args.image:
        if Path(name).name != name or not name.endswith(".png") or name in names:
            raise ValueError("Unsafe or duplicate image destination")
        names.add(name)
        if name in {"matched_support_metrics.png","critical_patch_progress.png"}:
            raise ValueError("Reserved generated figure name")
        payload,metadata=encoded_png(Path(source))
        prepared.append((name,payload,{"source":str(Path(source).resolve()),"destination":name,
                                      "caption":caption,**metadata}))
    args.output.mkdir(parents=True)
    render_matched(candidates,args.output/"matched_support_metrics.png")
    render_critical(candidates,reports,args.output/"critical_patch_progress.png")
    for name,payload,_ in prepared: (args.output/name).write_bytes(payload)
    figures=[{"destination":name,"sha256":digest(args.output/name),"actual_data_only":True}
             for name in ("matched_support_metrics.png","critical_patch_progress.png")]
    manifest={"created_utc":datetime.now(timezone.utc).isoformat(),"accepted":False,"navigation":False,
              "command":sys.argv,
              "source_sha256":digest(__file__),"candidates":candidates,
              "generated_figures":figures,"copied_images":[r for _,_,r in prepared],
              "limitations":["Candidate and baselines share identical available patches within each column group.",
                 "Different coverage across candidates must still be reported; missing values never score zero.",
                 "These historical sensor observations have been used for repeated model selection.",
                 "No surveyed ground truth, measured stair height, or navigation approval."]}
    (args.output/"manifest.json").write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+"\n")
    shutil.copyfile(__file__,args.output/"source_snapshot.py")
    print(json.dumps({"output":str(args.output),"candidates":len(candidates),"images":len(prepared)+2,
                      "accepted":False},ensure_ascii=False))


if __name__ == "__main__": main()
