# 추가 브리지·2층 세션 독립 재최적화 — 2026-10-17

## 범위와 결과

추가 수집한 세 DB를 **각각 독립 처리**했다. 이전 두 파일의 최종 검토본과 동일한 최적화·점군 필터를 적용했다. 원본 DB는 읽기 전용이며 이전 실행 폴더도 덮어쓰지 않았다. 이 단계는 개별 재최적화와 점군 재생성으로 범위를 정해 **세션 간 특징 검색·정합·공동 최적화·TF·지도 병합은 실행하지 않았다.** 브리지 파일이 추가됐다는 사실만으로 연결 성공을 가정하지 않는다.

최종 결과: `outputs/student_union_20261017_added_reviewed/`

| 세션 ID | 원본 DB | 수집 범위 |
|---|---|---|
| `bridge_out1f` | `261017-011331-bridgeout1f.db` | 1층 로비와 바깥 연결 |
| `floor2` | `261017-013501-2f.db` | 1층 로비 일부, 1층–2층 메인 계단, 2층 전체 |
| `bridge_12_sub` | `261017-013904-bridg12_2.db` | 1층–2층 서브 계단, 1·2층 엘리베이터 문 앞과 계단 앞 연결 구간 |

각각 원본 포즈·앱 저장 최적화 포즈·새 오프라인 포즈로 만든 RGB 점군을 보존했다. 새 후보 3개 모두 solver 수렴 및 이전과 동일한 내부 일관성 guard를 통과했다. **절대 위치 정확도를 검증하거나 모든 정합 오차를 해결했다는 뜻은 아니다.**

## 동일하게 적용한 과정·알고리즘

수식과 상세 해석은 [이전 실험 기록](2026-10-17-student-union.md)을 따른다. 이번에 `src/optimize_session.py`와 `src/export_rtabmap_db.py`는 이전 커밋 `b3c9122` 대비 **수정하지 않았다**. 확장한 코드는 임의 세션 수 감사·실행 관리·RViz 설정·결과 요약뿐이다.

1. SQLite 무결성, RGB/depth/confidence/calibration 전체 디코딩, timestamp/GNSS/앱 포즈, graph 연결 성분을 감사했다.
2. 기존 type0 odometry edge와 type1/type2 loop 후보를 읽었다. 무효 변환과 중복 방향을 제외하고 기존 양방향 trimmed **point-to-point ICP**로 loop를 검증했다.
3. 로컬 기하 조건: depth 0.25~4m, confidence≥50, pixel stride2, 6cm voxel, ICP 최대20회/방향, 대응0.22m, 가까운80% 유지. overlap≥0.45, RMSE≤7.5cm, 초기값 대비 변화≤25cm/5°, 왕복 일관성≤10cm/2°. ICP는 채택 판단용이며 측정은 원본 저장 edge를 유지했다.
4. x/y/z/yaw의 **4자유도 포즈 그래프**, 성분별 기준점 고정, full information matrix, whitened 6D edge별 **Huber(delta3)**, sparse SciPy TRF/LSMR를 적용했다. 원래 roll/pitch를 보존하며 새 IMU raw 적분을 한 것은 아니다.
5. odometry covariance에 이동3cm/회전0.5°의 model covariance를 추가했다. 이는 검증된 센서 분산이 아닌 모델 가정이다. 이를0으로 둔 `stored_covariance_baseline/` 대조 실험도 세 개 모두 수행했다.
6. 실제 계단의 z 변화를 유지했다. 지면 평탄화/plane factor, GNSS prior, 추가 extrinsic 보정, RANSAC 지면 제거는 적용하지 않았다. ±1m z bound는 **원본 대비 보정량**이지 절대 높이 제한이 아니다.
7. 최적화된 각 프레임 포즈와 저장된 calibration을 사용해 RGB-D를 다시 back-project했다. 완성된 점군을 통째로 이동시킨 것이 아니다. frame stride1/pixel stride1, depth0.25~4m, confidence≥50, **5cm voxel**을 세 비교본에 동일 적용했다.
8. 이전 야간 자료에서 오탐으로 폐기했던 semantic mask는 이번에도 사용하지 않았다. 새 YOLO 추론을 실행하지 않았으며 동적 객체가 완전히 제거됐다고 주장하지 않는다.
9. 궤적/점군 비교 PNG, CSV/JSON 진단, 명령·종료코드·소스 SHA, RViz 설정을 생성했다. TSDF/mesh·길찾기 생성은 이번 범위에 포함하지 않았다.

설정 원본은 `config/student_union_added_sessions.json`, 정확한 수치·파라미터는 [추적용 metrics JSON](student_union_added_20261017_metrics.json)에 보존한다.

## 입력 감사와 지도 프레임 선택

| 항목 | 외부–로비 브리지 | 메인 계단·2층 | 서브 계단 브리지 |
|---|---:|---:|---:|
| 전체 노드 | 134 | 1,218 | 211 |
| 주 연결 성분 / 앱 저장 포즈 | 134 / 134 | 1,215 / 1,215 | 211 / 211 |
| RGB-D 디코딩 실패 | 0 | 0 | 0 |
| GNSS records | 130 | 1,216 | 207 |
| reported accuracy 중앙값 | 14.415m | 16.640m | 18.177m |
| 진단용 GNSS 필터 통과 | 1 | 0 | 1 |
| 원본 궤적 z 범위 | 0.237m | 6.024m | 4.643m |

총1,563프레임을 모두 검사했다. GNSS의 reported accuracy는 실측 오차나 검증된1σ가 아니다. prior로 사용하지 않았다. 모든 `ground_truth_pose`는 null-transform sentinel이므로 독립 측량 기준은 없다. RGB와 depth의 별도 timestamp가 없어서 실제 센서 동기 오차까지 검증한 것은 아니다.

`floor2`의495·496·497은 null-transform link가 있는 고립 노드다. 임의로 보정을 전파하지 않고 전체 궤적에는 원본 포즈로 보존했다. 세 비교 점군은 모두 동일한 주 성분1,215프레임만 사용했다. 다른 두 세션은 전 프레임을 사용했다. 각 앱 포즈와 raw/optimized main CSV의 ID 집합이 같은지도 검사했다.

## 독립 최적화 결과

| 항목 | 외부–로비 브리지 | 메인 계단·2층 | 서브 계단 브리지 |
|---|---:|---:|---:|
| 유효 저장 loop 후보 | 1 | 63 | 2 |
| 기하 검증 통과 | 1 | 42 | 1 |
| 최적화 학습 / held-out loop | 1 / 0 | 34 / 8 | 1 / 0 |
| Huber 비용 전→후 | 0.6524→0.6242 | 44.9623→5.3709 | 1.8553→0.3536 |
| solver evaluations | 7 | 182 | 12 |
| 이동 보정 중앙값 / 최대 | 0.0208 / 0.0650m | 0.5113 / 1.6865m | 0.0819 / 0.2744m |
| 최대 절대 z 보정 | 0.00065m | 0.71356m | 0.04729m |
| 궤적 z 범위 전→후 | 0.2374→0.2380m | 6.0238→5.4322m | 4.6434→4.6618m |

세 주 실행 및 세 covariance 대조 실행은 수렴했으며, 주 실행은 correction bounds에 닿지 않았다. 최적화의 높이 보정은 계단을 평면으로 대체하는 작업과 다르다. 궤적 z 범위에는 실제 이동·휴대 높이·잔여 드리프트가 함께 들어가므로 **건물의 측량 층고로 해석하면 안 된다.**

### 메인 계단·2층

graph fitting에서 제외한8개 loop pair의 결과:

| 지표 | Node.pose 원본 | 앱 저장 최적화본 | 새 후보 |
|---|---:|---:|---:|
| clipped 기하 RMSE 중앙값 | 0.175015m | 0.155571m | 0.148640m |
| 양방향 overlap 중앙값 | 0.3443 | 0.5712 | 0.6105 |
| 저장 loop 이동 잔차 중앙값 | 0.555952m | 0.466109m | 0.129838m |

인접61쌍의 clipped RMSE는0.140356→0.140298m로 큰 변화가 없다. 내부 검증에서는 새 후보가 개선됐으나, 전역 측량 ground truth가 없어 절대 정확도를 보증할 수 없다. 최대1.69m의 포즈 보정도 실제 오차의 측정값이 아니다.

앱 저장 포즈의 첫 노드는 원본 기준점보다 약(+0.0759,−0.0878,+0.7377)m 이동했고 회전도0.1436° 다르다. 그림은 이전과 같이 추가 best-fit/anchor 정렬 없이 저장 좌표로 표시한다. 따라서 **앱/원본의 절대 overlay 차이에는 전역 좌표 기준 차이가 포함되며, 이를 모두 정합 오차라고 볼 수 없다.** 위 상대 edge·기하 지표는 이 전역 기준 차이에 영향받지 않는다.

### 두 브리지의 제한

각각 채택 loop가1개뿐이다. 기존 규칙(통과 loop≥10일 때 매5번째를 보류)에 따라 **별도 held-out loop는0개**다. 아래 점군 점 샘플은 ICP fitting에서 제외됐지만 graph 학습에 사용한 동일 loop pair에 대한 평가이므로, 독립된 loop 검증 성공으로 표현하지 않는다.

| 동일 학습 loop의 withheld-point clipped RMSE | Node.pose | 앱 저장본 | 새 후보 |
|---|---:|---:|---:|
| 외부–로비 브리지 | 0.181763m | 0.181771m | 0.181291m |
| 서브 계단 브리지 | 0.165847m | 0.165365m | 0.156461m |

외부–로비의 변화는 매우 작고, 두 브리지 모두 인접 프레임 기하 지표는 사실상 같다. 서브 계단의75↔209 loop는 ICP 보정0.299m/6.67°가 기존 한도0.25m/5°를 넘어 거부했다. 연결이 더 많아 보이도록 임계값을 낮추지 않았다.

이 문서의 기하 RMSE는 양방향 nearest-neighbor 거리를 **0.20m에서 clip**한 pair별 RMSE의 중앙값이며, overlap은 양방향 포함률의 조화평균이다. 실제 지도 오차/ATE가 아니다. 동일 데이터에 기반한 내부 검사라는 제한도 유지된다.

### covariance 대조 실험

추가 model covariance를0으로 했을 때 최대 포즈 보정은 외부–로비0.000133m, 메인 계단·2층0.145455m, 서브 계단0.008188m였다. 주 실행의0.065034/1.686452/0.274354m와 차이가 크므로 불확실성 가정에 민감하다. 원래 full matrix, 대조 결과, 후보를 모두 보존했다.

## 점군·그림·RViz 결과

| 세션 | 원본 점군 | 앱 저장본 점군 | 새 후보 점군 |
|---|---:|---:|---:|
| `bridge_out1f` | 233,670 | 233,686 | 234,142 |
| `floor2` | 3,023,789 | 2,847,302 | 2,800,708 |
| `bridge_12_sub` | 500,622 | 490,585 | 494,790 |

각 세션 폴더의 파일:

- `cloud_raw_filtered.ply`, `cloud_app_filtered.ply`, `cloud_optimized_filtered.ply`
- `raw_main_trajectory.csv`, `optimized_main_trajectory.csv`, 전체 `raw_trajectory.csv`/`optimized_trajectory.csv`, `candidate_trajectory.csv`
- `optimization.json`, `validated_edges.json`, `loop_validation.csv`, `geometry_validation.csv`, `pose_corrections.csv`, `floor_observations.csv`
- `export_raw.json`, `export_app.json`, `export_optimized.json`: 사용 프레임/점 수/필터/범위
- `trajectory_comparison.png`, `cloud_comparison.png`, `plot_settings.json`
- `comparison.rviz`, `stored_covariance_baseline/`

세 점군 비교 PNG를 직접 확인했다. 계단 구간과 층간 구조가 남아 있으며, 잔여 두께·중복·노이즈가 완전히 제거된 것은 아니다. 그림의 공통 percentile 범위와 최대150,000점 표시 제한은 시각화만의 설정이다. PLY는 모든 유효 voxel을 보존한다.

RViz 설정은 `--config-only`로 생성하고 독립 frame/topic 및 Path z-offset0을 검증했다. 이번 실행에서 새 GUI나 publisher를 띄우지는 않았다. 기존 세션의 창/프로세스에도 개입하지 않았다.

## 재현과 검증 기록

```bash
cd folded-city-atlas
# 새 폴더에 세 세션 모두 독립 재현
./scripts/process_independent_sessions.sh --run outputs/added_sessions_reproduction
# 수치 기록·입력/출력 SHA256 생성
/usr/bin/python3 src/finalize_experiment.py \
  --run outputs/added_sessions_reproduction \
  --sessions bridge_out1f floor2 bridge_12_sub --independent \
  --evidence outputs/added_sessions_reproduction/metrics.json
# 완료된 검토본 보기: 세션명만 바꾸어 선택
./scripts/view_session.sh floor2 --run outputs/student_union_20261017_added_reviewed
```

실제 실행은 세DB를 한 번에 감사한 뒤 `--sessions <이름> --start-at optimize`로 각 세션을 병렬 실행했다. 명령·종료코드·실행 시간·thread 환경·소스 SHA는 `commands_*.jsonl`, 로그는 `logs/`, 전체 입력 매핑은 `run_definition.json`에 보존했다. 최적화 JSON에도 원본/코드 SHA와 정확한 명령이 있다. driver의 초기 metadata 동시생성 lock과 재개 전 DB checksum 검사도 추가했다. 최적화·export 알고리즘과 파라미터는 바뀌지 않았다.

검증: **테스트49개 통과**, 변경 Python 파일 컴파일·shell 구문 검사 통과. 추가로9개 PLY의 실제 vertex 수/유한 좌표, 동일 frame coverage/필터, 원본 DB SHA, 성분별 보존, RViz frame/topic/offset을 검사했다. raw/optimized quaternion으로 계산한 local gravity 방향 차이는 최대약1.2×10⁻¹⁵로, roll/pitch 보존을 확인했다. 새 결과에 대한 독립 측량 검증은 수행하지 않았다.

원본 SHA256:

```text
bridge_out1f  bb51298a6bf29e16eb8e8f58f8bf5cd0694f8217ff61152b7e78d78acd8a306b
floor2        bd15632bad4bc253461b6ce2dc0c4304d3024785983dddbc968bdb83e48f9c6e
bridge_12_sub d72fab3480c7cba541d0c29084b49c93e021f70760b860334807a4d0bde781f5
```

출력별 checksum과 실행에 사용된 코드/설정 inventory는 `artifact_manifest.json`, Git용 요약은 [metrics JSON](student_union_added_20261017_metrics.json)에 있다. DB 원본·영상·점군·로그·PDF는 계속 Git 제외 대상이다. 기존 두 세션을 포함한 세션 간 연결은 이후 별도 검증 단계로 남겨 둔다.
