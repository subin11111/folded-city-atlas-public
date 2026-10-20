#!/usr/bin/env python3
"""Publish actual floor2 experiment figures with lossless pixel verification.

This does not render a new map, synthesize images, change an experiment, or
approve a candidate. It packages an explicit inventory only after the final
evaluation and read-only remeasurement audit have completed successfully.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shlex
import sys

from PIL import __version__ as PILLOW_VERSION

from optimize_session import sha256_file, utc_now
from publish_geometry_evidence import encoded_png

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RUN = ROOT/"outputs/student_union_20261017_floor2_local_v1"


def image_item(source, destination, stage, caption, limitations):
    return {"source": str(source), "destination": destination, "stage": stage,
            "caption": caption, "limitations": limitations}


def inventory(run):
    final = run/"final_evaluation"/"figures"
    finite_note = ("과거 RGB-D에서 고정한 같은 관측의 조건부 일관성이다. 반복적인 모델 검토에 사용된 자료이며 "
                   "독립 측량 정확도·벽 두께·출입문 폭을 의미하지 않는다. 누락값은 0으로 채우지 않는다.")
    view_note = ("같은 원본 깊이 샘플과 표시 축을 사용한다. 한 개의 강체 기준만 표시 정렬하며 "
                 "높이·촬영 시점 구분은 탐색용이다. 실측 층 구분·층고 또는 완성 지도 점밀도가 아니다.")
    return [
        image_item(final/"common_support_all_stages.png", "common_support_all_stages.png", "04_common_support_evaluation",
                   "모든 단계에서 공통으로 평가 가능한 동일 평면 패치만 사용한 비교. 공통 표본 수와 전체 패치 수를 함께 표시한다.", finite_note),
        image_item(final/"matched_support.png", "matched_support.png", "04_candidate_matched_support_evaluation",
                   "각 후보가 평가 가능한 패치에 기준 포즈도 맞춘 비교. 후보마다 평가 가능한 표본 수가 다를 수 있다.", finite_note),
        image_item(final/"floor2_656_1055_wall_0_1.png", "reserved_wall_656_1055.png", "04_critical_reserved_wall",
                   "2층656–1055의 과거 예약 벽을 동일 관측·축으로 비교. 성공과 악화를 모두 보존한 대표 진단이다.", finite_note),
        image_item(final/"floor2_548_981_wall_0_0.png", "wall_548_981.png", "04_critical_wall",
                   "2층548–981 벽의 고정 표면 패치 비교. 이 장소의 기존 학습 사용 이력을 포함해 해석해야 한다.", finite_note),
        image_item(final/"camera_height_bands_xy.png", "camera_height_bands_xy.png", "03_height_separated_reprojection",
                   "촬영 카메라 높이별로 분리한 실제 원본 깊이의 XY 재투영. 층 간 투영 중첩과 같은 높이의 중복 구조를 구분하는 진단이다.", view_note),
        image_item(final/"upper_corridor_time_windows_xy.png", "upper_corridor_time_windows_xy.png", "03_time_separated_reprojection",
                   "상부 복도 촬영 시점별 원본 깊이 재투영. 같은 벽의 서로 다른 방문이 벌어지는지 확인한다.", view_note),
        image_item(final/"trajectory_height_by_node.png", "trajectory_height_by_node.png", "03_camera_height_continuity",
                   "프레임별 카메라 높이의 단계별 비교. 포즈가 없는 구간을 인위적인 선으로 잇지 않는다.",
                   "카메라 높이는 사용자의 휴대폰 움직임을 포함한다. 바닥 높이나 계단 층고 실측으로 사용할 수 없다."),
        image_item(final/"original_rgb_critical_pairs.png", "original_rgb_critical_pairs.png", "00_original_sensor_evidence",
                   "문제 벽 대응에 사용된 실제 DB RGB 프레임. 원래 보이는 문·간판·구조물로 장소 가설을 검토할 수 있다.",
                   "원본 이미지의 진단용 배열이다. 시각적 유사성만으로 정확한 3D 대응 또는 계측 정확도를 보증하지 않는다."),
        image_item(run/"remeasurement_audit_v2/rgb_depth_351_352.png", "rgb_depth_351_352.png", "02_local_acceptance_audit",
                   "351–352 실제 RGB·깊이·훈련/검증 타일. 시간상 인접한 표면 제약은 통과했지만 충분한 RGB-D 특징 합의는 없었다.",
                   "검증 깊이와 주변3픽셀을 훈련에서 제외했다. 같은 센서의 상관된 표본이며 독립 실측이 아니다. 어두운 영상과 깊이 공백을 보정하거나 생성하지 않았다."),
        image_item(run/"remeasurement_audit_v2/rgb_depth_550_551.png", "rgb_depth_550_551.png", "02_local_rejection_audit",
                   "550–551 실제 RGB·깊이·훈련/검증 타일. 유효 RGB-D 특징39개 중 엄격한 훈련 분리 후7개만 남은 국소 정합 사례이다.",
                   "타일이 서로 다른 물리적 표면으로 이동하면서 최근접 거리 지표가 커질 수 있다. 전체 깊이 점군 사후 진단으로 임계값이나 채택 결정을 바꾸지 않았다."),
        image_item(run/"remeasurements/figures/local_550_551_reserved_depth.png", "local_550_551_reserved_depth.png", "02_rejected_local_before_after",
                   "550–551 국소 표면 재측정 전후의 동일 예약 깊이 샘플 비교. 사전에 정한 기준을 통과하지 못한 결과도 보존한다.",
                   "전체 포즈 그래프 결과가 아닌 두 프레임의 상대 변환 실험이다. 새 지도에 채택된 국소 측정으로 해석하지 않는다."),
    ]


def prepare_images(items, maximum_total_mib):
    if maximum_total_mib <= 0:
        raise ValueError("Image budget must be positive")
    names, prepared = set(), []
    for row in items:
        name = row["destination"]
        if Path(name).name != name or not name.endswith(".png") or name in names:
            raise ValueError("Unsafe or duplicate image destination")
        names.add(name)
        path = Path(row["source"]).resolve()
        if path.suffix.lower() != ".png":
            raise ValueError("Only explicit actual PNG sources are allowed")
        encoded, metadata = encoded_png(path)
        prepared.append((name, encoded, {**row, "source": str(path), **metadata,
            "transform": "Lossless PNG recompression; exact dimensions and decoded RGBA pixel identity verified"}))
    total = sum(len(data) for _, data, _ in prepared)
    if total > maximum_total_mib*1024**2:
        raise ValueError(f"Prepared images require {total/1024**2:.3f} MiB; budget is {maximum_total_mib:.3f} MiB")
    return prepared, total


def read_completed_reports(run):
    paths = {"evaluation": run/"final_evaluation/report.json",
             "remeasurement": run/"remeasurements/report.json",
             "remeasurement_audit": run/"remeasurement_audit_v2/report.json"}
    payloads = {name: json.loads(path.read_text()) for name, path in paths.items()}
    evaluation = payloads["evaluation"]
    if evaluation.get("original_database_unchanged") is not True or evaluation.get("pose_trajectories_unchanged_during_evaluation") is not True:
        raise ValueError("Evaluation must finish with unchanged original database and trajectories")
    if payloads["remeasurement"].get("complete") is not True or payloads["remeasurement"].get("inputs_unchanged") is not True:
        raise ValueError("Remeasurement is incomplete or input changed")
    audit = payloads["remeasurement_audit"]
    if audit.get("inputs_unchanged") is not True or audit.get("accepted_test_metrics_all_finite") is not True:
        raise ValueError("Remeasurement audit is incomplete or invalid")
    if audit["frozen_report_sha256"] != sha256_file(paths["remeasurement"]):
        raise ValueError("Audit is bound to a different remeasurement report")
    expected = {row["source"] for row in inventory(run) if str(run/"final_evaluation") in row["source"]}
    advertised = {str(run/"final_evaluation"/name) for name in evaluation["figures"]}
    if not expected <= advertised:
        raise ValueError("Final evaluation does not advertise all required figures")
    refs = {name: {"path": str(path.resolve()), "sha256": sha256_file(path)} for name, path in paths.items()}
    return payloads, refs


def publish(run, output, maximum_total_mib=24.):
    run, output = Path(run).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError(f"Choose a fresh output; existing assets are never overwritten: {output}")
    payloads, refs = read_completed_reports(run)
    prepared, total = prepare_images(inventory(run), maximum_total_mib)
    # Validate every image and the full byte budget before creating any output.
    for ref in refs.values():
        if sha256_file(Path(ref["path"])) != ref["sha256"]:
            raise RuntimeError("Experiment report changed while images were packaged")
    output.mkdir(parents=True, exist_ok=False)
    for name, encoded, _ in prepared:
        (output/name).write_bytes(encoded)
    sources = [Path(__file__), ROOT/"src/publish_geometry_evidence.py"]
    source_dir = output/"source_snapshot"
    source_dir.mkdir()
    for path in sources:
        (source_dir/path.name).write_bytes(path.read_bytes())
    evaluation = payloads["evaluation"]
    manifest = {"schema_version": 1, "created_utc": utc_now(), "command": shlex.join(sys.argv),
        "run": str(run), "source_hashes": {str(p.resolve()): sha256_file(p) for p in sources},
        "pillow_version": PILLOW_VERSION, "experiment_reports": refs,
        "image_count": len(prepared), "total_image_bytes": total, "maximum_image_mib": maximum_total_mib,
        "all_decoded_pixels_verified_identical": True, "no_synthetic_images": True,
        "candidate_accepted": evaluation.get("accepted", False), "navigation": evaluation.get("navigation", False),
        "candidate_stages": evaluation["candidate_stages"], "fixed_patch_count": evaluation["fixed_patch_count"],
        "common_support_all_stages": evaluation["common_support_all_stages"],
        "matched_support_by_candidate": evaluation["matched_support_by_candidate"],
        "remeasurement_sampling_audit": payloads["remeasurement_audit"]["summary"],
        "images": [row for _, _, row in prepared],
        "limitations": ["Only actual stored experiment images; no fictional/reconstructed results or screenshots.",
            "Lossless encoding may strip embedded metadata but does not change dimensions or RGBA pixels.",
            "Historical internal consistency, not independent surveyed accuracy or navigation certification.",
            "Full-depth post-hoc sampling audit never changes the frozen training-only remeasurement gates.",
            "Actual original DB files and full-resolution local experiment outputs remain untouched."]}
    (output/"manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+"\n")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, default=DEFAULT_RUN)
    parser.add_argument("--output", type=Path, default=ROOT/"docs/assets/floor2-local-review")
    parser.add_argument("--maximum-total-mib", type=float, default=24.)
    args = parser.parse_args()
    report = publish(args.run, args.output, args.maximum_total_mib)
    print(json.dumps({"output": str(args.output), "images": report["image_count"],
        "image_mib": report["total_image_bytes"]/1024**2, "all_decoded_pixels_verified_identical": True}))


if __name__ == "__main__": main()
