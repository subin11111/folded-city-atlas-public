#!/usr/bin/env python3
"""Package an explicit, small scientific image inventory for repository docs.

No image content is generated or retouched. PNGs are losslessly recompressed,
with original dimensions and exact RGBA pixel equality checked after encoding.
Source/output SHA-256, captions and interpretation limits accompany each image.
The script only owns its manifest-listed destination files; unrelated files or
edited gallery assets are never silently overwritten. Source experiment files
remain untouched, and raw DB/PLY files cannot enter the image inventory.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
from io import BytesIO
import json
from pathlib import Path
import sys

from PIL import Image, __version__ as PILLOW_VERSION


ROOT = Path(__file__).resolve().parents[1]
RUN = 'outputs/student_union_20261017_geometry_review'


def item(source, destination, stage, caption, limitations):
    return dict(source=source, destination=destination, stage=stage,
                caption=caption, limitations=limitations)


INVENTORY = [
    item('outputs/student_union_20261017_all_preview/bundle/rviz_all_preview_screenshot.png',
         'historic_rviz_preview.png', '00_historical_exploratory_preview',
         '이전 작업에서 실제 RViz로 표시한 외부·로비·2층·두 브리지 통합 미리보기.',
         '과거 화면이며 이번 재최적화 결과가 아니다. 통합 배치는 미검증 가설이고 RViz 정상 표시는 지도 정확도를 보증하지 않는다.'),
    item('outputs/student_union_20261017_added_reviewed/bridge_12_sub/trajectory_comparison.png',
         'historic_substairs_trajectories.png', '01_prior_session_optimization',
         '이전 서브 계단 세션의 원본·앱·오프라인 포즈 궤적 및 카메라 높이 비교.',
         '카메라 높이는 휴대폰 움직임을 포함하므로 실측 층고가 아니다. 이전 실험의 기존 그림을 보존했다.'),
    item('outputs/student_union_20261017_added_reviewed/floor2/cloud_comparison.png',
         'historic_floor2_cloud_comparison.png', '01_prior_session_optimization',
         '이전 2층 처리의 원본·앱 저장·오프라인 최적화 점군을 같은 축 범위에서 비교.',
         '동일한 RGB-D와 필터를 사용한 과거 결과이며 이번 단계의 최종 승인 지도는 아니다. 표시 범위는 원본 그림의 0.5–99.5 백분위이다.'),
    *[item(f'{RUN}/baselines/{name}/worst_wall.png', f'baseline_{name}_worst_wall.png',
           '02_baseline_wall_diagnosis',
           f'{name}: 가장 큰 기준 벽 불일치의 동일 관측·동일 축 단면과 원본 RGB 프레임.',
           '벽 대응은 기존 루프 측정에서 선정한 국소 평면 가설이다. 실측 오차가 아니며 반사면 깊이 편향이나 잘못된 대응 가능성이 남아 있다.')
      for name in ('outside', 'lobby', 'floor2')],
    *[item(f'{RUN}/baselines/{name}/cloud_baselines.png', f'baseline_{name}_clouds.png',
           '02_baseline_selection',
           f'{name}: 같은 RGB-D 샘플을 원본·앱·기존 오프라인 포즈로 투영한 동일 축 비교. 색은 촬영 순서.',
           '전체 지도에서 일부 프레임·점만 결정적으로 샘플링한 진단용 그림이다. frozen은 추가 최적화의 임시 기준이며 검증 완료를 뜻하지 않는다.')
      for name in ('lobby', 'floor2')],
    *[item(f'{RUN}/surfaces/{name}/{pair}/holdout_before_after.png', f'local_{name}_{pair}_before_after.png',
           '03_local_surface_refinement',
           f'{name} {pair}: 국소 RGB-D 표면 재정합 전후의 예약 샘플 비교.',
           '국소 상대 변환 실험이다. 훈련과 같은 관측의 예약 샘플은 독립 실측이 아니며 국소 개선이 전체 지도 개선을 보증하지 않는다.')
      for name, pair in (('floor2', '548_981'), ('lobby', '595_650'))],
    *[item(f'{RUN}/surfaces/near_holdouts/{name}/{pair}/near_frame_before_after.png', f'near_{name}_{pair}.png',
           '04_near_frame_audit',
           f'{name} {pair}: 정합에 사용하지 않은 인접 프레임에서 국소 변환을 평가.',
           '인접 촬영 프레임은 시공간적으로 상관되어 있다. 개선되지 않은 구간도 그대로 표시하며 독립 외부 검증으로 부르지 않는다.')
      for name, pair in (('floor2', '548_981'), ('lobby', '595_650'))],
    item(f'{RUN}/surfaces/surface_refinement_progression.png', 'surface_refinement_progression.png',
         '04_local_ablation_summary',
         '문제 표면 재정합과 주변 프레임 검사 단계별 측정 결과.',
         '국소 측정과 전역 포즈 그래프는 다른 단계다. 이 비교만으로 새 지도를 승인하지 않는다.'),
    item(f'{RUN}/connections/connection_search_summary.png', 'connection_search_summary.png',
         '05_cross_session_search_before_union_audit',
         '과거 검증 픽셀 합집합 제외 전의 초기 세션 간 탐색 진단. 최종 채택 제약 수가 아니다.',
         '이 그림의 통과 수는 잠정 결과이다. 이후 검증 깊이 영역 합집합과 기존 7cm 기준을 적용하여 176/51 및 396/208을 철회했다. 최종 새 적격 대응은 495/85이며 112/14는 기존 대응의 중복이다. 최종 입력은 connections/union_audit/graph_candidates.json을 확인한다. 연속 프레임·매칭 수는 독립 구역 수가 아니다.'),
    *[item(f'{RUN}/surfaces/plane_reviewed/{pair}_{kind}_patch_projection.png',
           f'plane_{pair}_{kind}_projection.png', '06_finite_plane_support_review',
           f'2층 {pair}: {kind} 국소 평면의 학습·예약 깊이 점을 저장된 보정값으로 원본 RGB에 재투영.',
           '검토된 실제 유한 표면 지지점이다. 이 평면의 학습 점은 같은 표면 가설에 사용되므로 최종 독립 검증으로 계산하지 않는다. 관측하지 않은 곳까지 벽이나 지면을 연장하지 않는다.')
      for pair, kind in (('548_981', 'wall'), ('503_961', 'horizontal'))],
    item(f'{RUN}/connections/manual_feasibility/lobby_39__bridge_out1f_79_manual_corner_audit.jpg',
         'manual_lobby_bridge_corner_audit.png', '05_manual_correspondence_feasibility',
         '로비39–외부 브리지79의 실제 문틀 모서리를 수동 검토한 대응과 깊이 유효성.',
         '8개 모서리 중 양쪽 깊이가 유효한 것은 3개이다. 수동 클릭 오차는 약 ±5픽셀이고 사후 검토이며 실측 기준점·새 독립 검증이 아니다. 새 그래프 제약으로 승격하지 않았다.'),
]

FINAL_INVENTORY = [
    item(f'{RUN}/connections/progress/connection_recovery_progress.png',
         'connection_recovery_progress.png', '05_final_cross_session_union_audit',
         '교차 세션 후보 415쌍과 PnP 추정 67개를 조사하고 과거 모든 예약 깊이 영역을 제외한 최종 재검증 결과.',
         '초기 4개 통과 중 과거 예약 샘플 재검증을 통과한 것은 2개이며, 기존 대응 중복을 제외한 새 그래프 후보는 로비495–서브 계단85 한 쌍이다. 이는 독립된 여러 연결 구역 확보 또는 전역 지도 승인과 다르다.'),
    item(f'{RUN}/connections/progress/stair_pair_alignment_progress.png',
         'stair_pair_alignment_progress.png', '05_local_stair_connection_progress',
         '로비495–서브 계단85의 같은 국소 관측과 19개 예약 대응에서 이전 배치·비대칭 PnP·과거 예약 영역 합집합 제외 PnP를 비교.',
         '국소 예약 대응 RMSE는 14.76cm에서 5.23cm로 감소했다. 이 쌍만의 상대 변환 평가이며 독립 실측 또는 공동 최적화된 전체 지도의 정확도가 아니다. 동일 crop·축 범위를 유지했다.'),
    *[item(f'{RUN}/figures/{source}', destination, '07_global_graph_progression', caption,
           '동일 RGB-D 샘플과 공통 표시 축의 단계별 비교이다. 연결되지 않은 세션의 표시 위치는 탐색용이며 모든 후보는 검증 한계를 갖는다. 학습 표면의 개선을 독립 정확도 향상으로 해석하지 않는다.')
      for source, destination, caption in (
          ('01_global_progress.png','final_global_progress.png','이전 RViz 배치, 고정 기준, 강건 포즈 그래프 및 법선·유한 평면 제약 실험의 실제 점군 진행 과정.'),
          ('03_floor2548_floor2981.png','final_floor2_wall_progress.png','2층548–981의 동일 관측을 전역 포즈 그래프 단계별로 재투영한 벽 중복 비교.'),
          ('03_outside69_outside323.png','final_outside_wall_progress.png','외부69–323의 동일 관측을 전역 포즈 그래프 단계별로 재투영한 벽 중복 비교.'),
          ('04_height_profiles.png','final_height_profiles.png','주 계단·서브 계단 촬영의 카메라 Z 변화 비교. 실제 층고 실측값은 아니다.'),
          ('05_added_evidence_ablation.png','final_added_evidence_ablation.png','같은 제한 SE(3) 풀이에서 추가 측정 전후의 동일 관측 비교.'))],
    item(f'{RUN}/evaluation/final/finite_patch_progression.png','final_finite_patch_metrics.png',
         '08_final_fixed_patch_evaluation',
         '고정된 실제 벽·수평면 관측에 대해 원본 기준과 전역 최적화 단계의 잔차를 비교.',
         '학습에 포함된 유한 평면과 사용하지 않은 진단 구간을 구분해야 한다. 모든 값은 동일 센서의 조건부 일관성이며 측량 정확도가 아니다.'),
    item(f'{RUN}/evaluation/final/floor2/548_981_wall_0_0.png','final_wall_training_patch.png',
         '08_final_training_surface_fit',
         '2층548–981의 같은 벽: 이전 통합 미리보기의 평면 간격23.28cm에서 유한 평면 제약을 포함한 실험의1.54cm로 감소.',
         '이 벽은 같은 평면 제약의 학습 대상이다. 동일한 원본 샘플·평면 축·표시 범위를 사용한 학습 일관성 개선이며 독립 검증 성공 또는 전체 지도 정확도 향상을 뜻하지 않는다.'),
    item(f'{RUN}/evaluation/final/floor2/656_1055_wall_0_1.png','final_wall_reserved_regression.png',
         '08_final_reserved_surface_regression',
         '2층656–1055의 예약 벽: 이전 통합 미리보기의 평면 간격4.05cm에서 유한 평면 제약 실험의54.42cm로 악화.',
         '그래프 학습에서 제외된 대응의 실패를 동일 관측·평면 축·표시 범위로 보존했다. 과거 모델 선택에도 사용된 예약 자료로 새 외부 실측은 아니며, 학습 벽의 개선만으로 후보를 채택할 수 없음을 보여준다.'),
]


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def encoded_png(source):
    original_bytes = source.read_bytes()
    with Image.open(BytesIO(original_bytes)) as opened:
        if opened.format not in ('PNG', 'JPEG'):
            raise ValueError(f'Unsupported source image: {source}')
        source_rgba = opened.convert('RGBA')
        # Remove only a universally opaque alpha channel, retaining every RGBA
        # value after decode; this is lossless, not palette quantization.
        payload = (source_rgba.convert('RGB') if source_rgba.getchannel('A').getextrema() == (255,255)
                   else source_rgba)
        buffer = BytesIO()
        payload.save(buffer, format='PNG', optimize=True, compress_level=9)
        result = buffer.getvalue()
        with Image.open(BytesIO(result)) as verified:
            verified_rgba = verified.convert('RGBA')
            if verified_rgba.size != source_rgba.size or verified_rgba.tobytes() != source_rgba.tobytes():
                raise ValueError(f'Pixel equality check failed: {source}')
        dimensions = list(source_rgba.size)
        pixel_sha = sha256(source_rgba.tobytes())
    if source.read_bytes() != original_bytes:
        raise RuntimeError(f'Source changed during packaging; rerun when stable: {source}')
    return result, {'source_sha256': sha256(original_bytes), 'source_bytes': len(original_bytes),
                    'destination_sha256': sha256(result), 'destination_bytes': len(result),
                    'dimensions_pixels': dimensions, 'decoded_rgba_sha256': pixel_sha,
                    'decoded_rgba_identical': True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'docs/assets/geometry-review')
    parser.add_argument('--additional-inventory', type=Path, action='append', default=[],
                        help='Explicit JSON array with source,destination,stage,caption,limitations for final figures.')
    parser.add_argument('--include-final', action='store_true', help='Require and publish completed final graph figures and fixed-patch metric figure.')
    parser.add_argument('--maximum-total-mib', type=float, default=24.)
    args = parser.parse_args()
    inventory = list(INVENTORY)
    if args.include_final:
        inventory += FINAL_INVENTORY
    for path in args.additional_inventory:
        inventory += json.loads(path.read_text())
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    manifest_path = output/'manifest.json'
    previous = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    owned = {row['destination']: row for row in previous.get('images', [])}
    prepared, destinations = [], set()
    for row in inventory:
        if not {'source','destination','stage','caption','limitations'} <= set(row):
            raise ValueError('Incomplete image inventory row')
        destination = row['destination']
        if Path(destination).name != destination or not destination.endswith('.png') or destination in destinations:
            raise ValueError(f'Unsafe or duplicate image destination: {destination}')
        destinations.add(destination)
        source = (ROOT/row['source']).resolve()
        if not source.is_file():
            raise FileNotFoundError(f'Inventory source does not exist: {source}')
        if source.suffix.lower() not in ('.png','.jpg','.jpeg'):
            raise ValueError('Only explicit image files can be published')
        target = output/destination
        if target.exists() and (destination not in owned or sha256(target.read_bytes()) != owned[destination]['destination_sha256']):
            raise RuntimeError(f'Unowned or user-modified image will not be overwritten: {target}')
        encoded, metadata = encoded_png(source)
        record = dict(row, **metadata, transform='Lossless PNG recompression; dimensions and decoded RGBA values unchanged')
        prepared.append((target, encoded, record))
    total = sum(len(data) for _,data,_ in prepared)
    if total > args.maximum_total_mib*1024**2:
        raise ValueError(f'Gallery would be {total/1024**2:.2f} MiB; explicit budget is {args.maximum_total_mib:.2f} MiB')
    # All images must pass checks before publishing any destination.
    for target,data,_ in prepared:
        if not target.exists() or target.read_bytes()!=data:
            target.write_bytes(data)
    manifest = {'schema_version':1, 'created_utc':datetime.now(timezone.utc).isoformat(),
        'script':'src/publish_geometry_evidence.py','script_sha256':sha256(Path(__file__).read_bytes()),
        'pillow_version':PILLOW_VERSION,'argv':sys.argv,'image_count':len(prepared),'total_image_bytes':total,
        'all_decoded_pixels_verified_identical':True,'no_synthetic_or_reconstructed_screenshots':True,
        'image_metadata_note':'Source embedded metadata may be stripped; image dimensions and RGBA pixel values are unchanged.',
        'gallery_note':'Selected historical and current experiment evidence; captions identify stages and limits. Original full outputs remain local.',
        'images':[record for _,_,record in prepared]}
    manifest_path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'output':str(output),'images':len(prepared),'image_mib':round(total/1024**2,3),
                      'pixel_equality_verified':True},ensure_ascii=False))


if __name__=='__main__':main()
