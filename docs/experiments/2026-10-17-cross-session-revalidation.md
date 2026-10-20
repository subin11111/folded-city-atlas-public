# 기존 데이터의 루프 재검증과 다중 세션 연결 실험

## 완료 결과

루프 재검증과 가중치 민감도, 공동 최적화 및 표면 평가를 수행했다. **다섯 세션을 연결한 최종 지도는 아직 검증되지 않았다.** 실패한 공동 최적화 결과를 기본 지도나 길찾기 지도로 승격하지 않았다.

| 대상 | 실행 결과 | 현재 처리 |
|---|---|---|
| 기존 내부 루프 | 160개 재검증, 6개 통과: 학습 5개·기존 검증 전용 1개 | 검증된 측정만 새 후보에 반영 |
| 서브 계단 | 새 재방문 측정으로 4DoF 및 제한된 SE(3) 후보, 점군 3개 재생성 | 별도 비교 후보로 보존; 기존 지도 유지 |
| 로비↔2층 | 국소 공통 구간 확인, 6가지 공동 최적화 후보 계산 | 검증용 연결 잔차 기준 미달로 통합 보류 |
| 외부↔외부 브리지↔로비 | SIFT·CLAHE·평면 대응 검사·LoFTR 재검증 | 통과하는 연결 제약 미확보; 합치지 않음 |
| 로비/2층↔서브 계단 | 일부 국소 대응은 통과하나 독립적인 다중 연결 부족 | 합치지 않음 |

계량 기록: [재검증 지표 JSON](student_union_20261017_revalidation_metrics.json). 전체 실행 결과는 아래 로컬 산출물 루트에 있다. 총 SIFT 기반 후보 **3,200회**, LoFTR 세션 간 후보 **200회**를 검사했다. 전자는 설정별 반복 검사를 포함하며 3,200개의 서로 다른 장소나 독립 관측을 뜻하지 않는다.

## 범위와 보존 원칙

2026년 10월 17일 제1학생회관 외부, 1층 로비, 외부 연결 브리지, 메인 계단과 2층, 서브 계단의 다섯 원본 DB를 대상으로 수행한 후속 실험이다. 재촬영 없이 기존 관측에서 사용할 수 있는 제약을 찾고, 검증되는 연결만 공동 최적화하는 것이 목적이다.

원본은 SQLite 읽기 전용으로 열며, 기존 `student_union_20261017_reviewed`와 `student_union_20261017_added_reviewed` 결과를 대체하지 않는다. 새 산출물의 루트는 `outputs/student_union_20261017_revalidated/`이다. 아래 경로는 별도 표시가 없으면 이 루트 기준이다. `config/student_union_all_sessions.json`에 DB와 이전 결과의 대응을 기록했다. DB, PLY, 가중치 파일, 실행 산출물과 PDF는 기존 Git 제외 규칙을 유지한다.

## 입력과 비교 기준

| 이름 | 원본 파일 | 촬영 범위 |
|---|---|---|
| outside | 261016-234854-outside.db | 외부 |
| lobby | 261017-000622-1f.db | 1층 로비 |
| bridge_out1f | 261017-011331-bridgeout1f.db | 외부와 1층 로비 연결 |
| floor2 | 261017-013501-2f.db | 로비 일부, 메인 계단, 2층 |
| bridge_12_sub | 261017-013904-bridg12_2.db | 서브 계단과 양 층의 엘리베이터 앞 |

비교 기준은 `Node.pose` 원본 이동 궤적, 앱 저장 최적화 포즈, 이전 오프라인 후보이다. 앱과 원본은 전역 기준 좌표가 다를 수 있으므로 표면 평가는 첫 공통 노드로 기준 좌표만 맞추며, 전체 궤적에 대한 best-fit으로 오차를 감추지 않는다. 포즈 그래프와 점군의 상대 오차는 공통 좌표계의 강체 변환에 불변이다.

## 기존 루프의 독립 재검증

구현: `src/revalidate_loop_constraints.py`, 결과: `intra_session/constraints.json` 및 세션별 `constraints.json`.

1. 기존 type 0 odometry, type 1/2 loop를 읽고 비정상 변환과 단절 성분을 구분했다. 기존 그래프의 whole-pair held-out 역할을 유지했다.
2. 원본 RGB의 SIFT와 RootSIFT, 양방향 ratio matching으로 대응을 다시 찾았다. 기존 저장 변환을 시각 정합의 정답으로 사용하지 않았다.
3. confidence≥50, depth 0.25–5m, 3×3 깊이 패치의 안정성 검사를 적용해 보정값으로 3D 대응을 복원했다.
4. 대응점의 학습 및 검증 분할을 고정했다. 검증용 깊이 픽셀 주변은 ICP에서도 제외해 동일 픽셀 재사용을 줄였다. 분할·제외 마스크·대응점은 NPZ와 JSON에 저장했다.
5. 3D RANSAC, 공간적으로 퍼진 지지점, 검증 대응의 오차, 양방향 trimmed point-to-point ICP, 왕복 일관성을 검사했다.
6. 검증된 새 측정만 대체 후보로 만들었다. 재검증 실패는 기존 약한 루프를 자동 삭제할 근거로 사용하지 않았다.

### 평면 대응의 관측 가능성

초기 검사는 대응점의 세 번째 공간 표준편차가 2.5cm 이상이어야 했다. 이 기준은 질감 있는 평면의 고정 3D 대응도 제외했다. 그러나 고정 point-to-point 대응의 강체 변환은 비공선 평면 점들로도 식별할 수 있으며, 무늬 없는 point-to-plane ICP의 자유도 모호성과 구분해야 한다.

별도 실험에서 세 번째 표준편차는 진단값으로만 남기고, 다음 관측 가능성 검사를 사용했다. 기존 volume-gate 실험은 `intra_session_volume_gate_trial/`에 보존했다.

- 두 번째 공간 표준편차 ≥0.15m, 영상 convex hull 면적 비율 ≥0.04.
- 중심화한 점 `p`에 대해 회전 normal matrix `H = Σ (||p||² I − ppᵀ)`를 계산.
- `λmin(H/N) ≥ 0.020m²`, condition number ≤100.
- 깊이 신뢰도, 검증 대응, ICP 오차 및 왕복 기준은 별도로 유지.

이는 관측 가능성 검사이며, 깊이 노이즈의 상관성이나 잘못된 대응을 없애 주는 통계적 공분산 추정은 아니다.

### 측정과 가중치 민감도

새 측정의 기본 모델 표준편차는 이동 0.10m/회전 3°, 약한 대조 실험은 0.20m/5°이다. 이 값은 보수적인 모델 가정이며 센서 보정으로 얻은 정확도가 아니다. 기존 odometry의 전체 공분산에 이동 3cm/회전 0.5° 모델 공분산을 더하는 이전 설정은 유지했다.

기존 4DoF x/y/z/yaw, 6D 잔차의 전체 공분산 whitening, edge별 Huber δ=3, sparse TRF/LSMR를 사용했다. 기존 검증 프레임쌍은 학습에 넣지 않았다. 검증 쌍이 없는 결과는 수렴하더라도 실험 후보이지 지도 정확도 개선의 독립 증거가 아니다.

160개 기존 유효 루프를 검사해 6개가 새 검증을 통과했다. 그중 학습용은 5개이고 나머지 1개는 기존 검증 전용이다. 세션별 통과 수는 서브 계단 1개, 2층 1개, 로비 4개, 외부와 외부 브리지 각각 0개이다.

외부 브리지 41→63은 SIFT 시각 대응 29개 중 양쪽 신뢰 깊이를 갖는 대응이 1개뿐이었다. 신뢰도가 낮은 깊이를 허용해 이 루프를 강하게 만들지 않았다.

서브 계단 75→209는 같은 자판기와 주변 구조의 재방문을 확인했다. 새 측정은 저장 측정과 약 22cm/6.565° 다르다. SIFT 검증 대응 23개 중 22개가 일치하고 RMSE는 4.015cm, 양방향 ICP overlap은 약 0.665, RMSE는 약 5.6cm, 왕복 오차는 6.39mm/0.363°였다. 기존 저장 변환 주변의 ICP 실패 기준을 단순 완화한 결과가 아니다.

## 기울기 보정을 제한한 추가 실험

구현: `src/optimize_bounded_attitude.py`, `src/evaluate_intra_attitude.py`.

서브 계단 새 루프는 4DoF에서 이동 잔차가 약 1cm로 감소해도 회전 잔차 약 4.64°가 남았다. 원래 roll/pitch를 고정할 때 남는 최소 tilt 불일치가 약 4.63°이므로 별도 SE(3) 후보를 계산했다.

- 포즈별 translation 및 rotation-vector 보정을 사용하되 성분 기준점은 고정.
- 원래 카메라의 중력 방향에 대한 chordal up-vector 차이를 regularizer로 사용. 새 IMU 측정을 추가한 것이 아니다.
- regularizer σ=2°/5°를 각각 실행. 실제 tilt 보정은 최대 5° 이하인지 별도 검사.
- 이동 보정량 bound는 XY 각 ±3m, Z ±1m. 실제 계단 높이를 1m로 제한하는 뜻이 아니다.
- 지면 평탄화, 공통 바닥 높이, 외부 보정값 변경, GNSS prior는 사용하지 않음.

서브 계단 75→209의 새 측정 대비 잔차는 이전 후보 0.3035m/4.7749°에서 4DoF 0.0100m/4.6370°, σ=2° 후보 0.00832m/4.074°, σ=5° 후보 0.00649m/3.340°가 됐다. 실제 최대 tilt 변경은 각각 약 0.305°/0.795°다. 이 루프는 학습 제약이므로 해당 잔차 감소를 독립 정확도 평가로 해석하지 않는다.

## 세션 간 공통 구간 탐색

구현: `src/revalidate_cross_sessions.py`, `src/revalidate_learned_pairs.py`.

우선 검사한 연결은 outside↔bridge_out1f, lobby↔bridge_out1f, lobby↔floor2, lobby↔bridge_12_sub, floor2↔bridge_12_sub이다.

### 기본 시각 검색

양방향 RootSIFT/FLANN 검색, 시간적 중복 억제, 원본 RGB의 full-resolution 검증을 적용했다. 세션 쌍마다 160개, 총 800개 후보를 검사했다. RANSAC의 train/heldout, 3D 공간 지지, ICP overlap≥0.40/RMSE≤7cm, refinement≤15cm/5°, 검증 대응 재투영 중앙값≤6px/p90≤15px를 사용했다.

국소 통과는 lobby↔floor2 9개, lobby↔bridge_12_sub 1개이며 나머지 세 연결은 0개였다. 이 결과만으로 지도 전체를 병합하지 않았다. 기준 포즈의 세션 변환이 35cm/5° 이내에서 일치하고 양쪽 세션 모두 시간 8초/위치 0.75m 이상 떨어진 관측 3개 이상인 엄격한 독립 연결 집합은 없었다. 국소적으로 통과한 측정은 삭제하지 않고 `local_evidence_only`로 보존했다.

### 저조도와 학습 기반 추가 검사

기본 검사와 별도로 CLAHE 및 신뢰 깊이 영역 중심의 SIFT 검색을 수행했다. 평면 대응 관측 가능성 검사와 ICP의 검증 깊이 제외를 적용한 별도 변형은 원래 검사와 출력·소스 기록을 분리했다.

LoFTR는 특징 검출 없이 대응을 찾는 추가 후보 생성기이며, 모델 confidence만으로 연결을 채택하지 않는다. task-local Kornia 0.7.1, 기존 CPU PyTorch 1.13.1, task-local typing_extensions 4.12.2를 사용한다. 공식 HTTPS checkpoint만 내려받고 RGB/DB를 외부 서버에 전송하지 않았다. checkpoint URL/SHA, 모델 종류, 영상 크기, 대응점·검증 결과는 각 실행 JSON에 기록한다.

문제 루프 6쌍의 LoFTR 검사는 모두 최종 검증에 실패했다. 더 많은 대응을 찾았지만, 3D 검증 또는 재투영 오차가 기준을 넘었다. 이는 일부 정확한 SIFT 대응의 존재를 부정하지 않으며, 낮은 품질의 추가 대응으로 포즈를 끌어당기지 않기 위한 제외다.

## 공동 최적화의 채택 조건

구현: `src/joint_optimize_sessions.py`, 조건부 점군 생성: `src/export_joint_candidate.py`.

기존 지도 드리프트 때문에 엄격한 고정 세션 변환에 모이지 않는 국소 연결은 별도 가설로 시험한다. 국소 검증을 통과하고 여러 촬영 지점에서 지지되는 경우에만 넓은 초기 변환 범위의 후보를 만들며, 고정 지도 연결 성공으로 부르지 않는다.

로비–2층의 첫 가설은 독립 5쌍이다. 112↔14, 188↔1192, 313↔1158, 346↔93을 학습에 사용하고 324↔1을 검증 전용으로 남겼다. 변환 가설의 pairwise 차이는 중앙값 0.762m/4.21°, 최대 1.235m/6.35°였다. 가설 선택에는 1.5m/10° 범위를 명시적으로 사용하되, 최종 지도 채택 기준을 완화하지 않았다.

초기 배치는 학습 측정만으로 yaw와 translation을 구한다. 독립적으로 연결되지 않은 세션은 별도 성분으로 남긴다. 기존 전체 odometry 및 검증 루프, 새 검증된 내부 측정, 채택된 학습용 세션 간 측정을 함께 최적화한다. 검증 전용 측정은 초기화·잔차 최소화 모두에 사용하지 않는다. 다만 가설 후보 선택에 검증 대응이 참여한 selection leakage가 있으므로 완전한 외부 검증 집합은 아니다.

최종 채택 검사:

- solver 수렴, 보정 bound 미도달, 목적함수 비증가.
- 세션 간 위치 잔차 중앙값≤15cm/최대≤35cm, 회전 중앙값≤3°/최대≤8°.
- 위 위치·회전 기준을 검증 전용 연결 집합에도 별도로 적용. 학습 연결이 많아져 검증 실패가 전체 중앙값에 가려지는 것을 금지.
- 기존 held-out loop 및 인접 프레임의 clipped RMSE 중앙값 악화≤5mm, overlap 감소≤3%p.
- 최종 포즈가 각 검증 측정의 점군 정합을 실제로 구현하는지 검사: 직접 측정 배치 대비 clipped RMSE 악화≤2cm, overlap 감소≤10%p.
- 드리프트 가설은 세션 쌍별 학습 3쌍 이상과 whole-link holdout이 필요.

실패 후보도 궤적과 잔차를 보존하지만, 합쳐진 최종 지도로 내보내지 않는다. `accepted`는 위 내부 일관성 기준의 통과만 의미하며 측량 정확도 보증은 아니다.

## 표면과 높이 변화의 별도 평가

구현: `src/evaluate_map_surfaces.py`. 결과는 `surface_evaluation/`, `surface_evaluation_old/`와 후속 후보별 디렉터리에 저장한다.

원래의 높이-band least-squares가 수직면이나 불안정한 외삽을 바닥으로 포함하는 문제를 피하기 위해 국소 PCA 법선과 RANSAC의 유한 크기 plane patch를 사용했다. confidence≥50, 6cm voxel, 학습/검증 점 분리, 법선·면적·접선 방향 폭·검증 잔차를 검사하고 수평 patch와 벽 patch를 따로 유지했다. 프레임 간 표면 대응은 이전에 검증된 loop의 상대 측정 아래 유한 지지 영역의 중첩으로 정했다.

수평 patch는 책상이나 계단 디딤판일 수도 있어 실제 바닥으로 확정하지 않는다. 시간별 높이 증가·정체·감소 구간 역시 카메라 이동 proxy이지 층 라벨이나 실측 계단 높이가 아니다. 전역 z 표준편차를 바닥 오차로 쓰지 않으며 전체를 한 평면으로 만들지 않는다.

2층 이전 결과의 검증 전용 반복 관측에서 수평 patch 7쌍의 대칭 point-to-plane 절대 잔차 중앙값은 raw 29.3cm → 이전 오프라인 6.42cm, 앱 7.10cm였다. 벽은 8개 patch/7개 루프에서 raw 14.3cm → 오프라인 4.86cm, 앱 12.33cm였다. 반면 모든 수평 대응 33쌍의 중앙값은 오프라인 6.62cm보다 앱 5.00cm가 작았고 오프라인 최대 57.2cm 불일치가 남았다. 이 값은 같은 RGB-D 관측의 상대 일관성 진단이며 실측 바닥 정확도가 아니다.

## 세션 간 재검증의 최종 집계

각 셀은 국소 기하 통과 수 / 엄격한 독립 연결 집합에 포함된 수이다. 뒤 숫자가 3이라고 해도 별도 검증 전용 연결 없이 곧바로 최종 병합을 허용하지 않는다.

| 연결 | 기본 SIFT, 각 160회 | CLAHE+depth, 각 240회 | 평면 관측 가능성+ICP 검증 픽셀 제외, 각 240회 | LoFTR, 각 40회 |
|---|---:|---:|---:|---:|
| 외부↔외부 브리지 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| 로비↔외부 브리지 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| 로비↔2층 | 9 / 0 | 11 / 3 | 10 / 0 | 1 / 0 |
| 로비↔서브 계단 | 1 / 0 | 2 / 0 | 1 / 0 | 0 / 0 |
| 2층↔서브 계단 | 0 / 0 | 0 / 0 | 2 / 0 | 0 / 0 |

기본 및 최초 CLAHE 실험에서는 검증 특징점의 깊이 픽셀이 dense ICP에 들어갈 수 있다. 따라서 이 실험의 feature holdout을 완전히 독립이라고 부르지 않는다. 마지막 SIFT 변형과 LoFTR는 검증 깊이 주변까지 ICP에서 제외했다. 원래 후보 검색 목록을 재사용하되 기하 검증은 다시 계산했다.

LoFTR의 유일한 통과 쌍은 로비 314↔2층 1215이다. 113개 검증 대응 중 103개가 일치하고 RMSE 4.93cm, 재투영 중앙값/p90은 4.96/8.75px였다. 그러나 기존 CLAHE 314↔51과 로비 기준 프레임을 공유하므로 독립 지지점이 하나 늘었다고 계산하지 않았다.

LoFTR 첫 실행은 앞의 세 연결 각 40회 완료 후 네 번째 연결 8회에서 작업이 중단됐다. 처음 완료한 120회는 `learned_cross_session/`에서 사용하고, 나머지 두 연결 80회는 `learned_cross_session_continuation/`에 다시 실행해 완료했다. 중간 코드 변경 후 기존 provenance를 무시한 resume을 하지 않았다. 미완료 8회는 보존하되 위 200회 집계에서 제외했다. 내부 문제 루프 6쌍과 아래의 특정 검증 쌍 1개에 대한 추가 LoFTR 검사는 별도이다.

외부 연결은 한쪽 깊이만으로 PnP를 적용할 여지가 있는지도 `outside_one_sided_depth_audit.json`에서 조사했다. 신뢰 깊이와 공간 지지가 충분한 별도 집합을 얻지 못해 PnP 측정은 생성하지 않았다. 연결 미확보는 촬영 구간이 전혀 겹치지 않는다는 증명이 아니다.

## 공동 최적화: 성공한 수렴과 실패한 지도 검증 구분

`joint_strict/`에서는 적격 연결이 없어 공동 solver를 실행하지 않았다. 이후 로비–2층의 별도 드리프트 가설만 아래와 같이 시험했다. 여섯 후보 모두 수치적으로 수렴하고 bound에 닿지 않았지만 지도 채택 검증에는 실패했다. 표의 연결 수는 검증 전용 1쌍을 포함한다.

| 출력 폴더 | 세션 간 연결 수 | 검증 쌍 | 검증 위치 잔차 | 검증 회전 잔차 | 판정 |
|---|---:|---|---:|---:|---|
| joint_drift_hypothesis | 5 | 324↔1 | 57.28cm | 3.58° | 거부 |
| joint_drift_hypothesis_weak | 5 | 324↔1 | 44.48cm | 2.73° | 거부 |
| joint_expanded_hypothesis | 9 | 324↔1 | 62.77cm | 3.83° | 거부 |
| joint_combined_hypothesis | 12 | 324↔1 | 46.50cm | 3.96° | 거부 |
| joint_disjoint_hypothesis | 5 | 327↔51 | 17.19cm | 2.43° | 검증 위치 중앙값 15cm 기준 초과 |
| joint_disjoint_expanded_hypothesis | 8 | 327↔51 | 19.54cm | 2.61° | 검증 위치 중앙값 15cm 기준 초과 |

마지막 두 행은 검증 깊이를 정합 계산에서 제외하고 평면 관측 가능성 검사를 사용한 **다른 실험 정책**이다. 첫 네 행보다 수치가 작다는 사실만으로 같은 외부 검증 성능이 향상됐다고 비교할 수 없다. 마지막 두 행에서는 다른 채택 guard는 통과했으나 검증 전용 위치 잔차가 기준을 넘었다. 통과를 위해 15cm 기준을 올리지 않았다.

최종 5쌍 가설의 학습 연결은 112↔14, 172↔1158, 317↔1215, 337↔66이고 검증 연결은 327↔51이다. 전체 영상에서 같은 출입문, AED 안내, 키오스크와 계단 주변을 확인한 뒤 기록했다. 확장 가설에서는 나머지 국소 후보 5개를 검사해 그중 초기 가설과 일관된 3개만 추가했다. 상관된 추가 연결은 표준편차 0.20m/6°로 정보량을 기본의 1/4로 줄이고 초기화에 쓰지 않았다. 5쌍/8쌍 실험의 모든 새 학습 제약에 대한 위치 잔차 중앙값은 각각 5.23/5.20cm이다. 작은 학습 잔차가 검증 통과를 대신하지 않는다.

초기 실패 검증 쌍 324↔1을 삭제해 성공을 선언하지 않았다. `targeted_holdout_check/report.json`에서 LoFTR로 독립 재검사했지만, 검증 대응 48개 중 22개만 일치하고 전체 RMSE 27.87cm, 재투영 중앙값/p90 10.95/27.22px로 실패했다. 이것만으로 원래 측정이 틀렸다고 확정할 수는 없다. 디스플레이/반사면 주변 깊이, 관측 모호성 및 지도 드리프트를 구분할 증거가 부족하므로 **미해결 불일치**로 유지한다.

초기 실험 후 공동 optimizer의 로비 기준점 고정, `initialization_allowed=false` 준수, 검증 전용 guard 분리와 입력 SHA 기록을 보강했다. 앞선 실험의 source SHA와 실행 보고서는 보존했다. 초기 9쌍 실험은 상관된 학습 연결도 초기화에 사용했으며 검증 전용 연결은 사용하지 않았다. 12쌍 및 최종 disjoint 실험은 수정된 초기화를 사용했다. 과거 JSON의 source SHA가 현재 코드와 다른 경우 현재 코드로 과거 실행을 byte-identical하게 재현할 수 있다고 주장하지 않는다. 일부 오래된 archive의 NPZ 절대 경로는 이동 전 디렉터리를 가리키므로 `intra_session_volume_gate_trial/`로 상대 위치를 해석해야 한다.

## 개별 지도 후보의 평가와 점군

2층과 로비의 새 기본/약한 측정 후보 모두 계산했다. 기존 whole-pair holdout의 clipped RMSE 중앙값은 2층에서 이전 0.148640m → 새 기본 0.148650m, 로비에서 0.166016m → 0.166090m로 사실상 유지됐다. 큰 개선을 확인하지 못했으므로 기존 5cm 점군을 교체하지 않았다. 외부와 외부 브리지는 새 내부 측정을 확보하지 못해 이전 결과를 유지했다.

서브 계단은 211개 전체 RGB-D 프레임으로 아래 점군을 새로 만들었다. 기존 완성 점군을 변형한 것이 아니라 각 프레임에 새 pose를 적용했다. 프레임·픽셀 stride=1, 깊이 0.25–4m, confidence≥50, voxel=5cm, 동적 마스크 없음이다.

| 경로 (`intra_session/bridge_12_sub/` 아래) | 점 수 | 용도 |
|---|---:|---|
| candidate_nominal/cloud.ply | 471,654 | 새 측정의 4DoF 후보 |
| candidate_attitude_2deg/cloud.ply | 472,446 | 원래 기울기 regularizer σ=2° 후보 |
| candidate_attitude_5deg/cloud.ply | 474,255 | 원래 기울기 regularizer σ=5° 후보 |

새 루프 75→209의 정합은 좋아졌지만, 기존 20→74의 유일한 유효 수평 patch 대응은 이전 9.585cm → 새 4DoF 9.583cm → σ=2° 10.427cm → σ=5° 11.772cm로 기울기 보정 시 악화됐다. 이 세션에는 그래프 학습에서 제외된 whole-pair holdout이 없다. 또한 표면 검사의 대응은 이전 검증 루프에 기반하므로 새 75→209의 표면 개선을 직접 측정하지 않는다. 따라서 SE(3) 후보를 기본 지도로 승격하지 않았다.

서브 계단의 상승 구간 82–135에서 카메라 Z 범위는 이전 4.1906m, 새 4DoF 4.1506m, σ=2° 4.1587m, σ=5° 4.1742m이다. 하강 구간 170–202는 각각 4.1300/4.1519/4.1500/4.1423m이다. 높이 변화는 남아 있으며 전체를 평탄화하지 않았다. 이 값에는 휴대폰 높이 변화가 포함되므로 실측 층고나 계단 높이는 아니다.

같은 축 범위의 비교 이미지:

- `intra_session/bridge_12_sub/revalidation_cloud_comparison.png`
- `intra_session/bridge_12_sub/revalidation_trajectory_comparison.png`
- `cross_session_planar_interior/lobby__floor2/accepted_pair_review.jpg`

점군 비교에서는 계단 형상이 유지되는 것을 확인했으나 중복된 면이 모두 제거되지는 않았다. 이 재검증 단계에서는 RViz를 실행하지 않았다. 후속 통합 미리보기는 아래 별도 절에 기록한다.

## 실행 코드와 재현

작업 디렉터리는 `/home/subin/folded-city-atlas`이다. 모든 원본 입력 위치는 `config/student_union_all_sessions.json`에 고정했다. 새 출력 디렉터리를 사용한다. 많은 원본 프레임을 처리하므로 아래 전체 재실행은 시간이 걸린다.

```bash
# 내부 루프 재측정 및 기본/약한 가중치 후보
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 /usr/bin/python3 src/revalidate_loop_constraints.py \
  --evaluate-candidates --output outputs/reproduction_intra

# 서브 계단의 제한된 기울기 보정 실험
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 /usr/bin/python3 src/evaluate_intra_attitude.py \
  --session bridge_12_sub --sigmas 2 5 --input-root outputs/reproduction_intra

# 세션 간 검색: 기본, 저조도, disjoint 깊이 재검증
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 /usr/bin/python3 src/revalidate_cross_sessions.py \
  --candidate-limit 160 --output outputs/reproduction_cross
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 /usr/bin/python3 src/revalidate_cross_sessions.py \
  --strategy clahe_depth --candidate-limit 240 --output outputs/reproduction_clahe
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 /usr/bin/python3 src/revalidate_cross_sessions.py \
  --strategy clahe_depth --geometry planar_disjoint --candidate-limit 240 \
  --candidate-source outputs/reproduction_clahe --output outputs/reproduction_disjoint

# 완료된 최종 8연결 가설을 같은 입력으로 재평가
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 /usr/bin/python3 src/joint_optimize_sessions.py \
  --cross-edges outputs/student_union_20261017_revalidated/cross_session/disjoint_expanded_hypothesis/drift_hypothesis_edges.json \
  --intra-edges outputs/student_union_20261017_revalidated/intra_session/constraints.json \
  --allow-drift-hypotheses --max-evaluations 500 --output outputs/reproduction_joint

# 서브 계단의 새 4DoF pose로 전체 RGB-D 점군 재생성
/usr/bin/python3 src/export_rtabmap_db.py data/raw/rtabmap/261017-013904-bridg12_2.db \
  --pose-csv outputs/reproduction_intra/bridge_12_sub/candidate_nominal/main_trajectory.csv \
  --missing-pose-policy skip --frame-stride 1 --pixel-stride 1 \
  --min-depth .25 --max-depth 4 --min-confidence 50 --voxel .05 \
  --cloud outputs/reproduction_intra/bridge_12_sub/candidate_nominal/cloud.ply \
  --trajectory outputs/reproduction_intra/bridge_12_sub/candidate_nominal/export_trajectory.csv \
  --summary outputs/reproduction_intra/bridge_12_sub/candidate_nominal/export.json

# 전체 테스트와 현재 DB 해시를 다시 확인하여 별도 집계 생성
/usr/bin/python3 src/finalize_revalidation.py --output outputs/reproduction_metrics.json
```

LoFTR는 시스템 패키지를 교체하지 않고 `outputs/student_union_20261017_revalidated/dependencies`에 설치한 Kornia 0.7.1/typing_extensions 4.12.2를 `PYTHONPATH`로 사용했다. `src/revalidate_learned_pairs.py --help`의 `--candidate-source`/`--output` 옵션으로 새 실행 위치를 지정할 수 있다. 검색·공동 최적화·학습 매처 실행의 argv, 체크포인트 출처·SHA와 매칭 설정은 해당 report/provenance에 남았다. 개별 점군 export의 설정은 `export.json` 및 위 재현 명령에 기록했다. 모델 파일은 `model_cache/checkpoints/`에만 있으며 Git에 포함하지 않는다.

| 코드 | 역할 |
|---|---|
| revalidate_loop_constraints.py | 독립적인 내부 루프 측정, 검증 픽셀 분리, 측정 대체·가중치 실험 |
| revalidate_cross_sessions.py | 세션 간 검색, 여러 정책의 기하 검증, 독립 지지점 및 드리프트 가설 |
| revalidate_learned_pairs.py / verify_targeted_pairs.py | LoFTR 보완 검사와 특정 실패 검증 쌍 재검사 |
| joint_optimize_sessions.py | 성분 분리, 학습 전용 초기화, 강건 공동 PGO, 채택 guard |
| optimize_bounded_attitude.py / evaluate_intra_attitude.py | 제한된 SE(3) 기울기 민감도 실험 |
| evaluate_map_surfaces.py | 수평면·벽 patch 및 시간별 높이 proxy 평가 |
| export_joint_candidate.py | guard 통과 성분만 프레임별 재생성; 이번에는 통과 성분 없음 |
| render_revalidation_candidates.py | 개별 후보의 동일 축 점군·궤적 비교 |
| finalize_revalidation.py | 완료 보고서 집계, 원본 해시 재확인, 전체 테스트 기록 |

기존 `optimize_session.py`, `export_rtabmap_db.py`, `check_session_overlap.py`는 수정하지 않았다. 새 기능은 별도 파일과 테스트에 추가했다. 최종 전체 테스트는 **87 passed**, 원본 DB 다섯 개의 최종 SHA-256은 표면 평가 전·후 값과 모두 일치한다. 자세한 SHA와 출력은 지표 JSON에 저장했다. 코드·문서는 로컬 변경 상태이며 이번 후속 작업에서 Git commit/push는 하지 않았다.

## 사용하지 않은 처리와 남은 한계

- EKF로 과거 프레임을 다시 정합하지 않았다. 이번 변경은 오프라인 pose graph 최적화다.
- GNSS prior, 바닥 높이 강제 통일, 새 IMU/gravity 측정, 지면 삭제는 사용하지 않았다. GNSS만으로 실내 지도나 층을 연결하지 않았다.
- RANSAC은 3D 대응의 오대응 제거와 표면 관측 추출에 사용했다. ICP는 실제 실행에서 trimmed **point-to-point** 방식이며 GICP/point-to-plane ICP를 실행했다고 기록하지 않는다.
- YOLO, TSDF/메시, 보행 가능 영역과 통합 길찾기 그래프는 이번 검증의 산출물이 아니다. 미검증 통합 지도 위에 경로를 만들지 않았다.
- 수평면/벽은 관측 patch 수준으로 평가했다. 검증된 층 라벨·실측 층고가 없어 '1층 바닥 오차'와 '2층 바닥 오차'를 확정값으로 보고할 수 없다.
- 여러 정책을 같은 데이터에 시험했으므로 완전히 독립적인 일반화 성능 평가가 아니다. 최종 후보의 15cm 미달을 성공으로 바꾸기 위한 추가 임계값 탐색은 하지 않았다.
- 재촬영을 요구하지 않고 현재 데이터로 진행 가능한 절차를 실행했지만, 모든 연결이 성립했다는 보장은 얻지 못했다. 현재 안전한 기본 사용 단위는 여전히 독립 세션이다.

## RViz 통합 미리보기

2026년 10월 17일 연결부를 비교하기 위해 **검증 미통과 로비–2층 후보를 시각화 전용으로 내보냈다.** 과거의 `accepted=false`, 검증 기준과 실패 이유는 변경하지 않았다. 최종 길찾기 지도 채택과 미리보기 생성을 분리하기 위해 exporter에 명시적인 `--unvalidated-preview` 옵션을 추가했다. 옵션이 없으면 거부된 성분은 여전히 내보내지 않는다.

사용한 포즈는 `joint_disjoint_hypothesis/component_0/`의 5연결 후보다. 검증 위치 잔차 17.185cm가 남는 상태 그대로이며, 8연결 후보보다 이 값이 작아서 비교용으로 선택했다. 이는 검증 통과나 일반적인 우수성을 의미하지 않는다. 외부, 외부 브리지, 서브 계단에는 임의 변환을 추가하지 않았다.

새 출력은 해당 성분 아래 `preview_clouds/`에만 저장했다. 원본 RGB-D 프레임에 공동 최적화 포즈를 적용하고 confidence≥50, 깊이 0.25–4m, frame/pixel stride=1, voxel=5cm로 재생성했다.

| 점군 | 사용 프레임 | 점 수 |
|---|---:|---:|
| floor2_candidate.ply | 1,215 | 2,581,975 |
| lobby_candidate.ply | 949 | 1,951,004 |
| combined_candidate.ply | 위 두 세션의 공동 좌표 점군을 다시 5cm voxel 집계 | 4,401,703 |

공동 성분의 포즈가 없는 기존 단절 노드 2층 495/496/497 및 로비 175/185/319/320/321/398/727/736/904는 제외했다. 제외 ID와 모든 export 설정은 `export_manifest.json` 및 개별 JSON에 있다. 두 원본 DB의 export 후 SHA-256은 기존 기록과 일치했다.

RViz는 `map_preview` 좌표계에서 통합 RGB 점군과 두 세션의 궤적을 표시한다. 로비 주황색·2층 청록색 점군은 별도 선택 표시이며, 층간 높이나 궤적에 인위적인 Z 오프셋을 넣지 않았다. 경고 이름과 텍스트 marker로 `UNVALIDATED PREVIEW`를 표시한다. 통합 cloud의 ROS publisher 1개 및 RViz subscriber 1개를 확인했고, 실제 창에서 점군·궤적이 그려지는 것을 확인했다. 화면 캡처는 `preview_clouds/rviz_preview_screenshot.png`이다.

```bash
# 최초 생성 명령. 같은 preview_clouds가 있으면 덮어쓰지 않고 중단한다.
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 /usr/bin/python3 src/export_joint_candidate.py \
  outputs/student_union_20261017_revalidated/joint_disjoint_hypothesis \
  --unvalidated-preview --candidate-only

# 생성된 미리보기를 다시 열기
bash scripts/view_joint_preview.sh
```

새 viewer는 manifest의 미검증 표시, `navigation_ready=false`, 완료 상태, 성분의 원래 거부 결과와 포함 세션을 확인한 뒤 실행한다. 점군 메시지는 한 번 구성해 캐시하고 10초마다 발행한다. RViz가 닫히면 viewer가 자신이 실행한 publisher를 정리한다. 관련 파일은 `src/view_joint_preview.py`, `scripts/view_joint_preview.sh`, `tests/test_view_joint_preview.py`, `tests/test_export_joint_candidate.py`이며 전체 테스트는 **94 passed**이다. 이전 지표 JSON은 재검증 완료 시점의 87-test 기록으로 유지한다.

## 알고리즘 근거

- [RTAB-Map Robust Graph Optimization](https://github.com/introlab/rtabmap/wiki/Robust-Graph-Optimization): robust 최적화와 odometry covariance 선택의 영향.
- [Open3D Global Registration](https://www.open3d.org/html/tutorial/pipelines/global_registration.html): 대응 기반 전역 초기화와 국소 ICP의 역할 구분. 본 실행이 Open3D 구현을 사용했다는 뜻은 아니다.
- [Open3D Multiway Registration](https://www.open3d.org/docs/release/tutorial/pipelines/multiway_registration.html): 다중 기하의 pose graph 연결 및 공동 최적화 개념.
- [Kornia 0.7.1 LoFTR 구현](https://kornia.readthedocs.io/en/v0.7.1/_modules/kornia/feature/loftr/loftr.html), [LoFTR 논문](https://arxiv.org/abs/2104.00680): 학습 기반 대응 생성과 공식 pretrained model의 출처.
