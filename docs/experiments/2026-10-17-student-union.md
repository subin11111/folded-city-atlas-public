# 제1학생회관 외부·1층 로비 독립 최적화 실험 — 2026-10-17

## 결론과 적용 범위

두 원본 DB를 읽기 전용으로 검증하고, 각각의 포즈 그래프를 재최적화한 뒤 프레임별 RGB-D로 점군을 재생성했다. **두 지도를 강제로 연결하지 않았다.** 전체 1,312개 RGB 프레임을 검색하고 후보 240쌍을 정밀 검사했지만 검증된 cross-session edge는 0개였다. 통합 변환·공동 포즈 그래프 최적화·병합 지도는 생성하지 않았다.

새 최적화 결과는 **실험 후보**다. 외부의 일부 내부 기하 지표는 개선됐지만, 로비의 held-out 기하 지표는 앱 저장 최적화본보다 좋지 않다. solver 수렴이나 목적함수 감소를 실제 지도 정확도 향상의 증거로 대체하지 않는다. 원본 DB, 앱 저장 포즈, 새 후보를 모두 유지한다.

- 최종 검토 실행: `outputs/student_union_20261017_reviewed/`
- 이전 마스크 실험: `outputs/student_union_20261017/` — 비교·실패 기록으로 보존, 최종 지도 아님
- Git에 보존하는 수치·파라미터·출처: [student_union_20261017_metrics.json](student_union_20261017_metrics.json)
- 코드: `src/audit_sessions.py`, `src/optimize_session.py`, `src/check_session_overlap.py`, `src/export_rtabmap_db.py`
- 원본 DB·영상·모델 가중치·점군은 `.gitignore`로 제외하며, 로컬 출력 파일의 SHA256은 최종 실행의 `artifact_manifest.json`에 기록한다.

## 1. 입력 데이터와 기존 설명의 정정

| 항목 | 외부 | 1층 로비 |
|---|---:|---:|
| 파일 | `261016-234854-outside.db` | `261017-000622-1f.db` |
| Node/RGB-D 프레임 | 354 | 958 |
| RGB·depth·confidence 디코딩 | 전부 정상 | 전부 정상 |
| 주 odometry 연결 성분 | 353노드 | 949노드 |
| 분리된 단독 노드 | 1 | 9 |
| 앱에 저장된 최적화 포즈 | 353 | 949 |
| GNSS reported accuracy 중앙값 | 14.246m | 17.974m |
| 보수적 GNSS 필터 통과 | 35 | 1 |

`Node.pose`는 저장된 ARKit/VIO odometry 초기값이다. `Admin.opt_ids/opt_poses`에는 이와 다른 **앱 저장 최적화 포즈가 이미 있었다.** 따라서 “완전히 최적화되지 않은 raw 지도”라는 이전 설명은 정확하지 않다. 여기서 원본/raw DB는 수정하지 않은 수집 DB를 뜻하며, `cloud_raw_filtered.ply`의 raw는 `Node.pose` 기준을 뜻한다.

RGB는 720×960, depth/confidence는 192×256이다. depth는 4채널 PNG 바이트를 little-endian float32로 해석하며, confidence는 zlib 압축을 해제한다. 프레임별 camera intrinsic과 optical→node local transform을 사용한다. RGB와 depth의 독립 촬영 timestamp가 이 스키마에 없어 **두 센서의 실제 동기 오차까지 검증한 것은 아니다.** 별도 timestamped 가속도·자이로 raw stream도 없으므로 IMU 재적분은 수행하지 않았다.

`ground_truth_pose`는 필드가 존재하지만 모든 값이 null-transform sentinel이다. 측량 기준값이나 독립 ground truth가 없으므로 절대 위치 정확도/ATE를 주장하지 않는다. 저장된 `Optimizer/PriorsIgnored=true`는 설정 스냅샷이지 모든 과거 실행에서 GNSS를 사용하지 않았다는 증명은 아니다. `RGBD/ForceOdom3DoF=true`도 `Reg/Force3DoF`와 함께 해석해야 하며, 현재 저장값 `Reg/Force3DoF=false`만으로 z를 강제로 평탄화했다고 판단하지 않았다.

원본 SHA256:

```text
outside e5a5c517bbddf2ec5b6877bbf38c67e0b88baeda0c4be57b784d60c05949dc91
lobby   1d39c5d4a2983c1ade05947bc205074d40cb1f77f5aca63df5519110bf762a8c
```

각 DB의 `PRAGMA quick_check`, 스키마, 모든 프레임 디코딩, timestamp 단조성, calibration, link transform/information matrix, GNSS를 검사했다. 감사 CSV/JSON은 최종 실행의 `audit/`에 있다. 0행렬 회전을 SVD로 유효한 회전처럼 복구하지 않고 **먼저 무효 link로 거부**했다. 분리 노드에 가까운 ID의 포즈 보정을 임의 전파하지 않는다. 점군 비교에서는 앱 포즈와 공통인 주 연결 성분만 사용한다.

## 2. GNSS 처리와 사용하지 않은 이유

위도·경도를 WGS84 ECEF로 변환한 다음 공통 ENU 좌표로 바꿔 진단했다. GNSS 고도의 기준면이 확인되지 않아 변환에는 ellipsoid height=0을 사용하고, 원래 altitude 값은 별도 보존했다. ENU의 surface-up은 지구 곡률 항일 뿐 건물 층 높이가 아니다.

진단용 보수적 필터: reported accuracy ≤10m, 측정 age −0.1~1.5s, 반복 timestamp/좌표 제거, 비현실적인 보행 속도 >3m/s 제외. 이 필터는 위치 정확성을 보증하지 않는다. 보고된 accuracy는 **실측 오차도, 검증된 1σ covariance도 아니다.** 실내 데이터와 반복·상관된 fix를 독립 제약처럼 쓰지 않았다. 최종 세션 최적화와 연결 검사에서 **GNSS prior 수는 0**이다. 지도를 GNSS 위치에 맞춰 억지로 붙이지 않았다.

## 3. 동적 객체 마스크 실험 및 폐기 결정

`src/mask_dynamic_objects.py`로 YOLOv8n-seg를 전체 프레임에 적용해 실험했다.

- 가중치: 공식 Ultralytics assets의 `yolov8n-seg.pt`, SHA256 `a7cd8f929e1903d78a12a48efecab430209f18dc46cb96c3599a5980c63c423c`
- Ultralytics 8.3.91, PyTorch 1.13.1+cpu, CPU 추론
- confidence 0.35, image size 640, class IDs 0/1/2/3/5/7/15/16: 사람·자전거·자동차·오토바이·버스·트럭·고양이·개
- segmentation polygon을 RGB 해상도에 rasterize하고 5픽셀 dilation; depth에는 nearest-neighbor resize
- 외부 24/354, 로비 81/958프레임에서 탐지. 평균 RGB 마스크 비율은 약 0.570% / 0.422%

`mask_review.jpg`를 직접 확인하니 외부 기둥·건물·그림자, 로비 자판기·유리·안내판이 사람/차량으로 오탐됐다. 예를 들어 외부172번은 약63%, 로비668번은 약58%를 제외했다. 평균 제외율이 작아도 특정 프레임의 정적 구조를 크게 훼손한다. **최종 reviewed 실행에서는 마스크를 사용하지 않도록 결정했다.** 이는 동적 객체를 완벽히 제거했다는 뜻이 아니며 움직이는 사람의 잔상은 남을 수 있다.

최종 no-mask로 두 최적화와 covariance 대조 실험을 모두 다시 실행했다. 채택된 루프와 원래 측정값이 같아 궤적은 이전 마스크 실험과 바이트 단위로 같았지만, 전체 유효 depth 픽셀에서 이전 마스크가 외부77,560개 / 로비113,237개를 제외했으므로 점군은 다시 생성했다. 마스크 파일·프레임별 탐지·모델 checksum·실행 로그는 이전 실행 경로에 보존한다.

## 4. 세션 내부 루프 검증

이번 실행은 **기존 저장 link를 검증하는 보수적 재최적화**다. 모든 연속 프레임의 visual odometry를 재추정하거나 새 intra-session loop를 탐색하지 않았다. EKF/ESKF, 연속 프레임 point-to-plane ICP, GICP는 실행하지 않았다.

1. 유효한 type0 odometry link를 읽고 역방향 중복은 한 방향만 보존한다.
2. 저장된 type1/type2 loop를 검사한다. 잘못된 회전·비양정치 정보행렬·분리된 odometry 성분 사이 edge를 거부한다.
3. depth 0.25~4m, confidence ≥50, pixel stride2, 6cm voxel로 로컬 점군을 만든다. deterministic voxel hash로 약20% 점을 ICP fitting에서 제외해 기하 검증에 남긴다.
4. 저장된 loop transform을 초기값으로 **양방향 trimmed point-to-point ICP**를 각각 실행한다. cKDTree nearest neighbor, SVD/Kabsch rigid fit, reflection correction을 사용한다.
5. 최대20회/방향, 대응거리0.22m, 대응 중 거리 하위80% 사용. 평가 overlap 거리0.15m, 양방향 overlap ≥0.45, RMSE ≤0.075m, 초기값 대비 변화 ≤0.25m/5°, 왕복 일관성 ≤0.10m/2°로 검증한다.
6. ICP는 **loop 채택/거부용**이다. ICP 보정값을 근거 없는 covariance와 함께 graph 측정으로 대체하지 않는다. 채택 loop에는 저장된 원래 측정과 full information matrix를 쓴다.

원본 유효 loop 후보 34쌍 중 외부11쌍이 통과했다. 로비는60쌍 중36쌍이 통과했다. 통과 loop가10개 이상이면 결정적 순서에서 매5번째를 graph fitting에서 제외한다. 외부는9개 학습+2개 held-out, 로비는29개 학습+7개 held-out이다. 이 held-out도 같은 데이터/후보 검증을 공유하는 내부 일관성 검사이지 완전 독립 시험집합은 아니다.

## 5. 강건한 세션별 포즈 그래프 최적화

### 상태와 좌표계

`T_w_i`는 node i→세션 world, `Z_i_j`는 node j→node i 변환이다. edge error는 다음으로 계산한다.

```text
E_ij = inverse(Z_ij) · inverse(T_w_i) · T_w_j
r_ij = [translation(E_ij), log_SO3(rotation(E_ij))]
```

노드마다 x/y/z/yaw 보정 **4자유도**를 사용한다. 회전은 `R'_i = Rz(delta_yaw_i) R_i`, 이동은 `t'_i = t_i + delta_t_i`다. 저장된 roll/pitch 및 `R_i.T @ world_up`을 유지한다. 중력 방향을 재추정하는 IMU factor를 새로 넣은 것은 아니며, 이 방법은 roll/pitch bias를 고치지 못한다. z는 자유 변수이고 단일 높이에 고정하지 않는다. 각 odometry 연결 성분의 첫 노드를 독립 고정해 gauge를 제거한다.

### 불확실성·목적함수·solver

원래6×6 full covariance를 유지한다. VIO 과신에 대한 민감도를 보기 위해 odometry covariance에 이동3cm·회전0.5°의 대각 model covariance를 더했다.

```text
Sigma_odom_effective = inverse(Omega_stored)
                     + diag([0.03²]*3 + [radians(0.5)²]*3)
Omega_effective = inverse(Sigma_odom_effective)
e = chol(Omega_effective).T · r
minimize sum_edges Huber_delta=3(||e||)
```

3cm/0.5°는 **실측 센서 covariance가 아니라 명시적 모델 가정**이다. loop 측정 covariance는 변경하지 않는다. 정보행렬을 이 translation/rotation-vector tangent residual에 사용하는 것은 근사이며 RTAB-Map native optimizer의 완전한 복제가 아니다.

Huber는 scalar별로 따로 적용하지 않고 **whitened 6D edge 블록 norm**에 적용한다. SciPy sparse `least_squares(method='trf', tr_solver='lsmr')`를 사용한다. Huber를 residual norm 변환으로 구현했으므로 solver의 `loss='linear'`와 중복 적용하지 않는다. 보정 bounds는 x/y ±3m, z ±1m, yaw ±20°, 최대300 evaluations. Jacobian sparsity·`x_scale='jac'` 적용; 나머지 정확한 허용오차는 소스와 실행 JSON에 남긴다.

수렴 여부, bounds 도달, held-out/인접 프레임 기하 일관성을 검사하고 후보 상태를 기록했다. 이번 두 실행은 수렴하고 bounds에 닿지 않았다. **이 guard는 `Node.pose` 대비 내부 일관성 검사이며, 앱 저장본보다 우수함을 보증하지 않는다.**

### covariance 대조 실험

같은 코드·루프·강건 손실에서 추가 model sigma를 둘 다0으로 바꾼 결과를 `stored_covariance_baseline/`에 남겼다. 원래 저장 covariance만 쓰면 최대 위치 보정이 외부2.75cm / 로비3.90cm에 그쳤다. 주 실행에서는 각각63.73cm /91.77cm다. 결과가 불확실성 가정에 민감하다는 증거이므로, 더 크게 움직였다는 이유로 더 정확하다고 판단하지 않는다.

### 지면과 z 오차 처리

이번에는 바닥을 한 높이로 묶거나 실외 경사면 법선을 수직 방향으로 강제하지 않았다. 외부의 턱·계단·경사를 보존하고 로비도 동일한 보수적 기준으로 비교했다. 카메라 extrinsic을 재보정하지 않았다.

`floor_observations.csv`는 **진단 전용**이다. 포즈 아래0.4~2.2m 후보에서4cm 높이 histogram의 최빈 band를 고르고 ±8cm 점으로 평면 least squares를 수행한다.3cm 잔차 trimming을3회 반복한다. RANSAC이 아니며 최적화 factor로 사용하지 않았다. 바닥이 아닌 다른 수평면을 포함할 수 있고, 장소/층이 다른 모든 관측 높이의 분산을 한 번에 줄이는 것은 목표가 아니다. 따라서 “지면 z 오차 완전 해결”이나 “평탄화 완료”로 해석하면 안 된다.

## 6. 최적화 결과와 한계

| 지표 | 외부 | 로비 |
|---|---:|---:|
| 사용 odometry edge | 352 | 948 |
| 통과 loop / 학습 / held-out | 11 / 9 / 2 | 36 / 29 / 7 |
| Huber 목적함수 전→후 | 10.418→2.484 | 20.842→6.839 |
| solver evaluations | 18 | 63 |
| 위치 보정 중앙값 / 최대 | 0.189 / 0.637m | 0.434 / 0.918m |
| held-out 기하 RMSE: Node.pose | 0.18869m | 0.16620m |
| held-out 기하 RMSE: 앱 저장본 | 0.16595m | **0.15907m** |
| held-out 기하 RMSE: 새 후보 | 0.16540m | 0.16602m |
| held-out overlap: Node.pose→새 후보 | 0.2720→0.3813 | 0.5529→0.5489 |

여기의 기하 RMSE는 withheld voxel sample 사이 **양방향 nearest-neighbor 거리를0.20m에서 clip한 RMSE의 pair 중앙값**이다. 실제 지도 절대오차가 아니다. overlap은 양방향 포함률의 조화평균이다. 점군이 완전히 다른 경우도 clip 때문에 숫자가 제한된다. 작은 수치 차이를 정확도 향상으로 확대 해석하지 않는다.

외부 held-out 저장 loop의 이동 잔차 중앙값은 오히려0.987→1.021m로 증가했다. 로비는0.258→0.191m로 감소했지만 기하 RMSE가 앱 기준보다 나쁘다. **모든 지표가 개선된 것은 아니다.** 앱 저장본과 새 후보를 동시에 남긴 이유다. 궤적 그림은 임의 best-fit 정렬 없이 저장된 좌표로 표시하므로 앱/odometry의 전체 gauge 차이도 포함될 수 있다. 상대 edge/기하 지표는 이를 구분해서 해석한다.

## 7. 점군 전체 재생성과 비교

완성된 점군을 하나의 rigid transform으로 이동시키지 않았다. 각 프레임에서 confidence/depth를 필터하고 다음 순서로 back-project한다.

```text
p_optical = depth × inverse(K_depth) × [u,v,1]
p_world   = T_w_node × T_node_optical × p_optical
```

RGB는 depth 위치에 대응시켜 색을 부여한다. 최종 조건은 모든 주 성분 프레임 사용(frame stride1), pixel stride1, depth0.25~4m, confidence≥50, global5cm voxel, 추가 semantic mask 없음, 추가 extrinsic correction 없음이다. 세 포즈 소스에 같은 원본 프레임/필터를 적용한다. Statistical outlier removal, TSDF·mesh는 이번 범위에서 실행하지 않았다.

각 세션 출력:

- `cloud_raw_filtered.ply`: Node.pose odometry
- `cloud_app_filtered.ply`: DB의 앱 저장 최적화 포즈
- `cloud_optimized_filtered.ply`: 새 독립 PGO 후보
- `raw_main_trajectory.csv`, `optimized_main_trajectory.csv`: 시각화/점군용 주 연결 성분
- `candidate_trajectory.csv`, 전체 `*_trajectory.csv`: 진단용 전체 포즈; 분리 노드 포함 여부 주의
- `trajectory_comparison.png`, `cloud_comparison.png`: 원본/앱/후처리 비교
- `export_*.json`: 사용 프레임, 제외 ID, 점 개수, 필터, 좌표 범위

최종 PLY 점 수는 아래와 같다. voxel 점 수가 줄었다고 곧바로 정확도가 높아진 것은 아니다.

| 세션 | Node.pose | 앱 저장본 | 새 후보 |
|---|---:|---:|---:|
| 외부, 353프레임 | 689,663 | 654,550 | 634,662 |
| 로비, 949프레임 | 2,060,669 | 1,991,172 | 1,995,008 |

비교 PNG는 공통 축 범위(0.5~99.5percentile)와 최대150,000개 표시 샘플을 사용한다. **이 clipping/subsampling은 그림만의 설정**이며 저장된 PLY의 유효 voxel을 삭제하지 않는다. 두 세션의 PNG를 직접 확인했으며 두께가 있는 중복 지면/구조가 완전히 사라진 상태는 아니다.

## 8. 외부–로비 연결 검사: 구현과 실제 실행을 구분

GNSS가 아니라 공통 영상/metric depth 근거를 탐색했다. `src/check_session_overlap.py`의 전체 파라미터와 실제 단계별 실행 횟수는 `cross_session/overlap_report.json` 및 Git의 metrics JSON에 보존한다.

실제로 실행한 단계:

1. 전체1,312 RGB 프레임: RootSIFT, FLANN randomized KD-tree, weighted frame voting. 검색 영상 최대변480, 프레임당최대600특징/인덱스300특징, neighbor8, FLANN trees5/checks96. 외부 프레임당 상위3 후보를 모아1,062쌍을 만들었다.
2. 시간적 중복 억제3초로 후보를 분산해240쌍을 원해상도에서 검사했다. 최대2,500특징, 상호 nearest-neighbor Lowe ratio0.75, 최소20개 상호 대응.
3. depth0.25~5m와 confidence≥50,3×3 patch에서 최소5개 유효 픽셀/깊이 범위≤0.12m를 적용해3D 대응을 만들도록 했다. 양쪽에 유효한 대응 최소20개 필요.

결과는237쌍 RGB 상호 대응 부족,3쌍 유효 depth 대응 부족이다. 검증 edge0개, 독립 일치 edge0개다.

다음 단계들은 **구현했지만 이번 실제 후보는 앞 단계에서 전부 탈락하여 실행하지 않았다**. 합성시험은 rigid fit·RANSAC·퇴화·좌표계·합의 일부를 검증했다. checker의 ICP/held-out 전체 경로는 별도 합성시험하지 않았다:

- seeded 3D SE(3) RANSAC(pair seed=`20261017 + outside_id × 10007 + lobby_id`,2,500회,8cm threshold), Kabsch refit 및 reflection 방지
- 3D singular-value/condition 및 RGB convex-hull 면적 기반 퇴화 검사
- trimmed point-to-point ICP 최대30회,15cm 대응, 가까운80% 유지(먼20% 제거); symmetric overlap≥0.4, RMSE≤7cm, 보정≤15cm/5°
- `max(5, ceil(0.25*N))`개 held-out 특징 대응 검사, 최소5inlier·비율≥0.5·RMSE≤7cm
- 변환된 세션 z축 불일치≤10°(raw IMU 잔차 검사 아님), 양쪽 세션 각각 시간≥8초이면서 위치간격≥0.75m로 분리된 일관된 연결 최소3개, 세션 변환 일치≤0.35m/5°

특징 held-out은 RANSAC에서 제외하지만 dense ICP에 해당 픽셀이 포함될 수 있으므로 완전 독립적인 센서 검증은 아니다. 단일 평면의 모양만 맞춘 결과도 보수적으로 거부한다.

구체적인 실패 근거: 외부91–로비30의36개 RGB 특징은 유리문 너머 먼 나무/조명에 대응한다. 외부 depth는5.20~7.98m이고 전부confidence<50인 반면 로비 depth는2.37~2.47m다. 외부33–로비30의27개 특징도 같은 문제다. **비슷한 RGB 특징이 양쪽 depth에서 같은 표면을 측정했다는 뜻은 아니다.** 마스크 없이 재확인한 값은 `glass_depth_diagnostic.json`, 이미지는 `candidate_contact_sheet.jpg`와 `matches/`에 있다.

240쌍의 제한된 검색에서 신뢰 가능한 미터 단위 연결을 얻지 못했다는 결론이지 **물리적 중첩이 전혀 없다는 증명은 아니다.** threshold를 낮춰 가짜 연결을 만들지 않았고, GNSS/수동 눈대중으로 최종 변환을 만들지 않았다.

이전 마스크 사용 실험은120후보 검사를 완료했다. 확대한240후보 실험은205번째 검사 이후 SIGTERM으로 중단됐으며 완성된 결과처럼 사용하지 않았다. 최종 판정은 모든240쌍이 완료된 **마스크 없는 reviewed 실행** 기준이다.

## 9. 재현·검증·RViz

기본 처리 환경: Ubuntu/ROS2 Humble, Python3.10.12, NumPy1.21.5, SciPy1.8.0, OpenCV4.5.4, Matplotlib3.5.1. 범용 의존성 범위는 `requirements-processing.txt`, 정확한 환경은 이 문서와 실행 provenance JSON을 함께 참조한다. YOLO는 선택 실험에만 필요하다. 기존 시스템 Python/ROS 환경을 일괄 업그레이드할 필요는 없다.

```bash
cd folded-city-atlas
# 새 출력 폴더에 재현; 원본 DB는 read-only
./scripts/process_student_union.sh --run outputs/student_union_reproduction
# 전체 실행 종료 후 checksum/추적용 요약 생성
/usr/bin/python3 src/finalize_experiment.py \
  --run outputs/student_union_reproduction \
  --evidence outputs/student_union_reproduction/metrics.json
# 단위/합성 테스트
OPENBLAS_NUM_THREADS=1 /usr/bin/python3 -m pytest -q tests
# 최종 검토본의 두 세션을 각각 표시
./scripts/view_session.sh outside
./scripts/view_session.sh lobby
```

기본 파이프라인은 mask 단계를 건너뛴다. `--use-semantic-masks`는 QC가 필요한 선택 실험이며 현재 자료의 최종 설정으로 권장하지 않는다. 기존 실행 경로는 자동으로 덮어쓰지 않도록 audit manifest를 확인한다. 특정 단계 재실행은 `--start-at`/`--stop-after`로 명시하며, 중요한 결과는 새 `--run` 경로로 보존한다.

단계는 감사→선택 마스크→독립 최적화·covariance 대조→점군3종→연결검사→비교그림이다. 병렬 조사로 직접 실행한 최적화/연결의 정확한 명령은 해당 report의 `provenance.command`/`command`에 있다. 최종 최적화의 앱 reference 경로가 이전 실행 `audit/`를 가리키지만 동일 원본 DB에서 추출된 내용이며 최종 audit CSV와 일치함을 확인한다. driver 실행 명령과 종료상태는 `commands.jsonl`, stdout/stderr는 `logs/`에 남긴다.

RViz에서는 세션별 `/folded_city/outside/*`, `/folded_city/lobby/*` topic과 각각 `outside_map`, `lobby_map` frame을 사용한다. **두 frame 사이 TF는 발행하지 않는다.** Raw Cloud(반투명 적색), Optimized Cloud(RGB), App Saved Cloud(RGB, 기본off)를 전환한다. raw/후보/app 궤적은 청록/주황/초록이며 세 궤적에 인위적인 z-offset을 주지 않는다.

검증 결과: 단위/합성 테스트 **23개 통과**, 변경 Python 파일 컴파일과 shell 구문 검사 통과. 테스트는 무효 transform 거부, full covariance/Huber, 성분별 anchor, gravity 보존, 기하/pose convention, 데이터 감사, GNSS/스키마 decode, 특징/SE(3) 검증, mask/export 및 누락 pose 처리를 다룬다. 단위테스트 성공은 실데이터의 절대 정합 정확도를 증명하지 않는다.

실외 RViz를 실행해 OpenGL 초기화와 publisher 시작을 확인했다. ROS2 transient-local subscriber로 외부 cloud width634,662 / 로비 cloud width1,995,008 및 두 app 궤적의 `outside_map`/`lobby_map` header를 실제 수신했다. 로비는 publisher-only smoke test 후 해당 시험 publisher를 종료했다. 실외 비교 화면은 열어 두었다. GUI 픽셀을 자동 검증한 것은 아니며 이미지 품질 평가는 위 PNG와 수치 비교 기준이다.

## 10. 아직 하지 않은 것과 다음 연결 조건

이번에 하지 않은 작업: EKF/ESKF, IMU preintegration, 매 연속 프레임 RGB-D odometry 재추정, point-to-plane/GICP, 앱 native optimizer의 완전 재현, floor/plane/GNSS factor 추가, calibration 재추정, 강제 지면 제거/평탄화, joint PGO, TSDF/mesh, 보행 가능 영역·경로 그래프. 기존 계획은 전체 로드맵이며, 이 목록을 실행 완료로 간주하지 않는다.

현재 자료로 추가 개선하려면 raw/app/새 후보의 중첩 부분을 수동 검토하고 신뢰 가능한 정적 면/측정 기준을 먼저 확보해야 한다. 새 층을 촬영할 때는 층만 별도 촬영하는 것에 더해 **기존1층의 특징 많은 근거리 정적 구조→계단→2층**을 끊기지 않게 관측한 연결 구간이 도움이 된다. 외부–로비도 양쪽에서 같은 불투명 문틀·벽·기둥을 depth 유효 범위에서 중첩 촬영하는 bridge가 가장 직접적인 보강이다. 유리 너머 먼 RGB 풍경만 공유하는 것은 충분하지 않다. 추가 촬영/수동 제약 없이 현재 검증을 통과한 것처럼 병합하지 않는다.

## 알고리즘·포맷 근거

- [RTAB-Map SQLite 저장 구현](https://github.com/introlab/rtabmap/blob/master/corelib/src/DBDriverSqlite3.cpp): Node/Data/Link/Admin 해석
- [RTAB-Map 파라미터 정의](https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h): Force3DoF 및 prior 설정 의미
- [RTAB-Map Robust Graph Optimization](https://github.com/introlab/rtabmap/wiki/Robust-Graph-Optimization): 강건 graph 최적화 배경; 이번 SciPy block-Huber 구현과 동일한 실행이라는 뜻은 아님
- [SciPy least_squares](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html): sparse TRF/LSMR 및 robust objective
- [Open3D Multiway registration](https://www.open3d.org/docs/release/tutorial/pipelines/multiway_registration.html): pairwise edge와 전체 graph 최적화 구분; 이번 실행은 Open3D solver를 사용하지 않음
- [Ultralytics segmentation](https://docs.ultralytics.com/tasks/segment/): 선택 semantic mask 모델; 실제 야간 자료 QC 결과로 최종 비활성화
