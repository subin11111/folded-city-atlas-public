# 외부와 서브 계단을 포함한 전체 세션 미리보기

## 결과

2026년 10월 17일 외부, 외부–로비 브리지, 1층 로비, 메인 계단과 2층, 서브 계단의 다섯 세션을 같은 좌표계에 배치해 RViz에 표시했다. 통합 점군은 **5,143,710점**이다. 기존 로비–2층 공동 최적화 후보를 기준으로 나머지 세션의 강체 배치를 추가한 결과이며, **새로운 전체 포즈 그래프 최적화나 검증을 통과한 최종 지도는 아니다.**

원본 DB 다섯 개와 이전 검증 판정은 그대로 유지했다. 모든 추가 변환은 xyz 이동과 yaw 회전만 사용해 각 세션의 내부 형상과 높이 변화가 보존된다. 바닥 평탄화, 높이 스케일 변경, GNSS를 이용한 강제 배치, 사람의 좌표 추측은 사용하지 않았다.

전체 산출물은 `outputs/student_union_20261017_all_preview/`에 있다. 아래 상대 경로는 이 디렉터리 기준이다.

| 세션 | 사용 포즈 수 | 점 수 | 배치 근거 |
|---|---:|---:|---|
| lobby | 949 | 1,951,004 | 기존 로비–2층 공동 최적화 후보 |
| floor2 | 1,215 | 2,581,975 | 기존 후보의 메인 계단·2층 |
| bridge_out1f | 134 | 234,142 | 로비와 같은 출입문 관측 |
| outside | 353 | 634,662 | 외부 브리지와 같은 출입구 패널·문틀 |
| bridge_12_sub | 211 | 471,654 | 로비와 2층의 두 공통 구역 |

표의 개별 점군을 공동 좌표로 옮긴 후 5cm voxel로 합쳤으므로 통합 점 수는 단순 합보다 적다. 색상은 voxel 내 점들의 평균이다. 추가 세션은 이전에 프레임별로 재구성한 점군을 강체 이동시켰으며, 이번에 원본 깊이로 전부 다시 재생성한 것은 아니다.

## 로비와 외부 브리지

구현은 `src/explore_lobby_bridge_attachment.py`, 결과는 `lobby_bridge/attachment.json`이다.

기존 상위 대응 영상에서 같은 문·벽·안내판을 확인한 뒤, 로비11↔브리지23, 39↔79, 49↔38 주변의 59개 프레임쌍을 추가 검사했다. CLAHE와 신뢰 깊이 영역의 RootSIFT, 양방향 Lowe ratio 0.75, 3D RANSAC, 평면 대응 관측 가능성, 검증 깊이 이웃 제외, 양방향 trimmed point-to-point ICP와 재투영 검사를 재사용했다. 깊이 confidence≥50과 기존 엄격한 기준은 바꾸지 않았다.

새로운 엄격 통과 쌍은 0개였다. 로비39↔브리지79는 3D와 ICP 검사를 통과하지만 재투영 중앙값이 7.70px로 6px 기준을 넘었다. 미리보기의 위치 근거로 이 한 쌍을 사용했다. 4DoF 상대 포즈 잔차를 SciPy least_squares의 성분별 soft-L1로 맞췄으며, 이동 0.10m·회전 3°는 센서의 실측 공분산이 아닌 모델 표준편차다.

배치 후 같은 쌍의 검증 대응 8개 RMSE는 4.05cm지만, 이는 배치에 쓴 프레임쌍에서 나온 진단이다. 사용하지 않은 로비11↔브리지23에서는 위치 잔차 **22.22cm**, 회전 **5.82°**, 검증 대응 RMSE **28.00cm**가 남았다. 이 쌍 자체도 역방향 ICP와 재투영 검사에 실패했으므로 원인과 정확도를 단정할 수 없다. 한쪽 문에서 맞는 위치가 브리지 전체에서 일관되게 검증된 것은 아니다.

## 외부와 외부 브리지

구현은 `src/explore_outside_bridge.py`, `src/compose_outside_bridge_preview.py`다. `outside_bridge/`에 원래 실패 기준, 대응점 NPZ, 영상과 후보 변환을 보존했다.

외부259–267과 브리지110–125의 같은 출입구 주변 **144쌍**을 검사했다. 같은 측면 패널, 문틀, 벽돌 기둥과 유도블록을 영상으로 확인했지만 야간 조명·유리 반사·좁은 특징 영역 때문에 엄격 통과는 없었다. 이후 SIFT 263↔117, 262↔117, 263↔118 및 이전 LoFTR 263↔118의 학습 인라이어를 사용해 별도 배치 가설을 만들었다. 이들은 같은 장소의 상관된 관측이며 네 개의 독립 연결로 계산하지 않는다.

세 가지 대안을 비교했다.

1. 기존 LoFTR 측정의 6DoF 변환.
2. 기존 포즈의 위쪽 방향을 유지한 xyz+yaw 강체 정합.
3. 같은 프레임에 대한 모든 매처·쌍의 검증 depth 픽셀 ±3 이웃을 합집합으로 제외한 xyz+yaw 정합.

실제 통합에는 세 번째 `alternatives.gravity_union_disjoint_4d`를 사용했다. 합집합 제외 후 학습 대응은 63개에서 49개로 줄었다. 프레임쌍마다 총 제곱 가중치를 같게 하고 Huber scale=0.08m, 최대 200회, 종료 허용오차 1e-11로 추정했다. z는 자유로운 이동량이며 지면을 같은 높이로 압축하지 않는다. 기존 포즈의 중력 방향을 유지하는 가정이지 새 IMU 관측을 추가한 것은 아니다.

이 대안의 검증 대응 RMSE는 각각 **9.79/6.90/6.03/13.47cm**다. 정합에 직접 쓰지 않은 인접 프레임 점군의 symmetric overlap은 **44.9%**, 15cm 안쪽 최근접점 거리의 RMSE는 **8.69cm**였다. 공간적으로 가까운 동일 센서 관측이므로 독립적인 측량 정확도는 아니다. 일부 대응이 7cm 기준을 넘고 독립적인 여러 연결 구역도 없어 strict_accepted 및 eligible_for_joint는 false로 남는다.

`attachment_candidate.json`의 주 선택 필드는 초기 4DoF 결과를 보존한다. 실제 사용한 합집합 제외 대안은 `bundle/manifest.json`의 `outside_variant`와 `T_outside_bridge_used`가 명시한다. 해당 변환은 브리지 지도 → 외부 지도 방향이므로, 통합 좌표로는 다음처럼 합성했다.

`T_joint_outside = T_joint_bridge × inverse(T_outside_bridge)`

## 서브 계단

구현은 `src/explore_substairs_attachment.py`, 결과는 `substairs/attachment_report.json`이다. 앞선 내부 루프 재측정의 보수적 4DoF `candidate_nominal` 포즈·점군을 사용했다.

로비462↔서브36과 2층553↔서브159의 두 구역으로 단일 xyz+yaw 변환을 맞췄다. 이동 0.10m/회전 3° 모델과 edge block-Huber δ=3, 최대 250회, 종료 허용오차 1e-11을 사용했다. 내부 포즈나 roll/pitch는 변경하지 않았다.

| 대응 | 역할 | 배치 후 위치 잔차 | 회전 잔차 |
|---|---|---:|---:|
| 로비462↔서브36 | 학습 | 4.14cm | 1.56° |
| 2층553↔서브159 | 학습 | 4.14cm | 2.96° |
| 2층560↔서브162 | 미사용 audit | 7.67cm | 3.10° |
| 로비443↔서브60 | 미사용 audit | 14.95cm | 1.47° |

각 층의 단일 앵커가 제안하는 세션 원점 변환은 약 0.7m 차이가 났다. 원점은 회전과 앵커까지의 거리에 영향을 받으므로, 최종 배치는 원점 이동 차이만이 아니라 관측 노드의 상대 잔차를 최소화했다. 작은 학습 잔차가 정합 성공을 뜻하지는 않는다. 미사용 두 쌍의 점군 clipped RMSE는 직접 측정 배치 대비 **5.47cm/5.02cm 악화**돼 기존 +2cm guard를 통과하지 못했다.

두 2층 관측은 같은 문 주변이라 독립 구역으로 중복 계산하지 않았고, 엘리베이터 audit는 이전 dense ICP 픽셀 재사용의 한계가 있다. 독립 연결 구역 세 개와 별도의 독립 검증 구역을 확보하지 못했다. 계단의 카메라 Z 범위 **4.595226m는 배치 전후 동일**하며, 이것은 실측 층고가 아니라 높이 변화 보존의 확인이다.

## 통합과 RViz

`src/build_all_session_preview.py`는 위의 명시적인 변환과 포즈 출처를 읽어 `bundle/`을 만든다. 변환의 정규직교성·det=1·위쪽 방향을 검사하고 배율 변경이나 뒤집기를 거부한다. 브리지의 좌표 출처가 서로 같은지도 확인한다. 새 글로벌 최적화, 수동 좌표 보정, 지면 제거는 하지 않는다.

주요 파일:

- `bundle/combined.ply`: 전체 미리보기 점군.
- `bundle/manifest.json`: 세션별 변환, 출처·해시, 점 수, 높이 범위, 연결 감사 및 원본 DB 보존 검사.
- `bundle/placement_overview.png`: 세션별 색상 평면·측면 배치.
- `bundle/rviz_all_preview_screenshot.png`: 실제 표시 화면.
- `bundle/EXPLORATORY_ALL_SESSIONS.rviz`: 표시 설정.

새 RViz 창의 Global Status가 OK이고, 통합 PointCloud2 publisher/subscriber가 연결된 상태에서 점군과 다섯 궤적이 표시되는 것을 확인했다. 프레임은 `map_all_preview`이며, `preview_axes`로의 identity TF만 추가했다. 운영용 map/odom과 연결되지 않는다. 기존 두 세션 RViz 창은 비교용으로 남겼다.

RGB 통합 점군이 기본 표시다. 이를 끄고 세션별 `exploratory placement (tint)`를 켜면 로비 주황, 2층 청록, 외부 브리지 분홍, 외부 초록, 서브 계단 보라색으로 구분된다. 경고는 디스플레이 이름에 표시하고, 지도를 가리는 텍스트 marker는 기본으로 꺼 뒀다.

```bash
# 이미 만든 전체 미리보기 다시 열기
bash scripts/view_all_sessions_preview.sh

# 재현: 다음 다섯 명령은 기존 원본과 이전 재검증 산출물을 사용한다.
# fresh_run은 새 디렉터리여야 한다.
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 /usr/bin/python3 src/explore_lobby_bridge_attachment.py --output outputs/fresh_run/lobby_bridge
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 /usr/bin/python3 src/explore_outside_bridge.py --output outputs/fresh_run/outside_bridge
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 /usr/bin/python3 src/compose_outside_bridge_preview.py --output outputs/fresh_run/outside_bridge
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 /usr/bin/python3 src/explore_substairs_attachment.py --output outputs/fresh_run/substairs
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 /usr/bin/python3 src/build_all_session_preview.py --run outputs/fresh_run
```

첫 통합 실행은 마지막 해시 검사에서 경로 타입 처리 오류로 중단됐다. 해당 미완료 출력은 `bundle_incomplete_hash_trial/`로 옮겨 보존했고, 타입 처리를 수정한 뒤 새 `bundle/`에서 전 과정을 다시 실행했다. RViz는 완료 manifest가 있는 결과만 읽는다.

원본 DB 다섯 개의 최종 SHA-256은 이전 기록과 모두 일치했다. 전체 테스트 **119개 통과**와 `git diff --check`를 확인했다. 실행 코드·테스트·문서는 로컬 변경이며 commit/push는 하지 않았다. DB, 점군과 대용량 실험 산출물은 기존 Git 제외 정책을 유지했다.

현재 결과는 다섯 지도의 연결 모습을 함께 관찰하는 용도로는 사용할 수 있지만, 벽·출입문 주변의 중복과 높이·방향 불일치가 남아 있다. `Global Status: OK`는 RViz의 데이터 수신·좌표 표시 상태이며 지도의 정합 품질 승인 표시가 아니다.
