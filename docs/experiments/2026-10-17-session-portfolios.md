# 다섯 기록: 원본 → 전처리만 → 최적화, 전체 후보 갤러리

## 비교 자료

세션별 후보 선택을 위해 모든 비교본을 나란히 구성했다.
한 사진의 위 행은 XY 평면도, 아래 행은 XZ 측면도다. 색은 높이이며 같은 기록의
모든 후보가 같은 축·높이 색 범위를 사용한다. 세션을 새로 합치지는 않았다.

| 기록 | 원본→전처리→최적화 3단계 | 전체 후보 XY | 전체 후보 XZ |
|---|---|---|---|
| 외부 | [3단계](../assets/session-portfolios/outside/three_stage.png) | [전체 XY](../assets/session-portfolios/outside/portfolio_xy.png) | [전체 XZ](../assets/session-portfolios/outside/portfolio_xz.png) |
| 1층 로비 | [3단계](../assets/session-portfolios/lobby/three_stage.png) | [전체 XY](../assets/session-portfolios/lobby/portfolio_xy.png) | [전체 XZ](../assets/session-portfolios/lobby/portfolio_xz.png) |
| 외부–1층 브리지 | [3단계](../assets/session-portfolios/bridge_out1f/three_stage.png) | [전체 XY](../assets/session-portfolios/bridge_out1f/portfolio_xy.png) | [전체 XZ](../assets/session-portfolios/bridge_out1f/portfolio_xz.png) |
| 메인 계단·2층 기록 | [3단계](../assets/session-portfolios/floor2/three_stage.png) | [전체 XY](../assets/session-portfolios/floor2/portfolio_xy.png) | [전체 XZ](../assets/session-portfolios/floor2/portfolio_xz.png) |
| 서브 계단 기록 | [3단계](../assets/session-portfolios/bridge_12_sub/three_stage.png) | [전체 XY](../assets/session-portfolios/bridge_12_sub/portfolio_xy.png) | [전체 XZ](../assets/session-portfolios/bridge_12_sub/portfolio_xz.png) |

[다섯 기록 한 번에 보기](../assets/session-portfolios/overview.png).
원본·전처리·과거/신규 후보를 포함해 **91개 점군 지도,사진112장**을 생성·검증했다.
이는 서로 다른 optimizer91종이라는 뜻은 아니다. 전체 후보의 큰 개별 사진은
`docs/assets/session-portfolios/<기록>/<후보ID>.png`에 있다.
예: [2층 N02](../assets/session-portfolios/floor2/N02.png),
[2층 F03](../assets/session-portfolios/floor2/F03.png).

**3단계 사진의 마지막 열은 비교 예시 N02(DCS)이지 자동으로 뽑은 우승 후보가 아니다.**
`외부=N03, 로비=J02, 외부브리지=O00, 2층=F03, 서브계단=N04`처럼 서로 다른
후보 ID를 골라도 된다. 이 예시는 추천 결과가 아닌 선택 형식 예시다.
후보를 선택할 때는 전체 형상과 함께 벽·바닥 중복, 계단 높이, 예약 관측과 연결 지표를 비교한다.

## 1. “원본”과 “전처리만”을 구분한 방법

- **O_FULL:** DB `Admin.opt_ids/opt_poses`에 저장된 폰 앱 최적화 포즈와, RGB·depth·
  보정값이 있는 전체 해당 프레임으로 새로 재생성. 과거 RViz 창의 스크린샷을
  재사용하거나 raw `Node.pose`를 앱 최적화 포즈로 바꾸어 부른 것이 아니다.
- **O00:** 동일 폰 앱 포즈 중 기존 품질 A/B 프레임만 사용하는 공정 비교용 원본.
- **P00:** O00와 **정확히 동일한 포즈**, 동일한 A/B 프레임에 기록별 depth 전처리만
  적용. 벽을 이동하거나 지면을 평면으로 대체하지 않았다.
- **N02:** P00와 동일한 카메라 점·색·프레임에 새 DCS 최적화 포즈를 적용한 예시.

`O_FULL→P00`에는 기존 품질 검사로 C 프레임을 제외한 효과가 포함된다.
그 효과를 포즈 개선으로 오해하지 않도록 `O00→P00→N02`의 **동일 프레임 대조**도
모든 기록에 추가했다: 각 폴더의 `matched_three_stage.png`.

| 기록 | 폰 앱 저장 전체 프레임 | 공통 A/B 프레임 |
|---|---:|---:|
| 외부 | 353 | 305 |
| 로비 | 949 | 800 |
| 외부 브리지 | 134 | 86 |
| 메인 계단·2층 | 1,215 | 925 |
| 서브 계단 | 211 | 135 |

원본에도 기본적인 물리적 유효성 범위(depth0.25~4m, confidence≥50)는 적용했다.
따라서 “원본”은 **앱 저장 포즈 기반의 유효 RGB-D 점군**이며, 모든 잘못된 depth까지
무조건 포함한 센서 바이트 덤프나 앱 내부 mesh와 바이트 동일한 산출물은 아니다.
기존 A/B 품질 분류는 재사용했으며 이번에 새로 품질 등급을 학습하지 않았다.

## 2. 적용한 후보 ID와 알고리즘

이번에 새로 실행한 최적화는 **28개**다. 다른 입력에서 실행한 과거 결과를 새 실행으로
세지 않는다. 새 방법은 모두 교차 세션 연결을 제외한 **세션별 submap 그래프**에
적용하고, 고정된 submap 내부 프레임을 anchor 보정으로 펼쳤다. 전체 프레임 BA와
같은 방법이라고 부르지 않는다.

| 후보 | 내용 | 이번 계산 여부 |
|---|---|---|
| O_FULL | 폰 앱 저장 포즈 전체 원본 | 포즈 재사용, 점군 새 재생성 |
| O00 | 폰 앱 저장 포즈·동일 A/B 원본 | 포즈 재사용, 점군 새 재생성 |
| P00 | 기록별 전처리만, 앱 포즈 그대로 | 전처리·점군 새 계산 |
| H00 | 기존 RViz에 사용하던 incumbent | 포즈 재사용, 같은 새 전처리로 재생성 |
| H01 | 이전 세션별 4DoF | 포즈 재사용 |
| H02 | 이전 독립 Huber | 포즈 재사용 |
| H03 | 이전 독립 DCS | 포즈 재사용 |
| H04 | 이전 독립 구역 공유 switch | 포즈 재사용 |
| N01 | 새 전처리 측정 + 6DoF block-Huber | **새 실행** |
| N02 | 같은 새 그래프 + 6DoF DCS | **새 실행** |
| N03 | 같은 새 그래프 + 구역 공유 switch | **새 실행** |
| N04 | 기존 코드의 4DoF(x/y/z/yaw) 최적화 | **새 실행** |
| N05 | 기존 코드의 SE(3)+기준 자세 prior | **새 실행** |
| N06 | N05+국소 수평면 법선 | 로비·2층·서브 계단에서 **새 실행** |
| F01 | 다중 프레임 RGB-D BA→전체 frame PGO | **2층 과거 포즈 재사용** |
| F02 | 수동 구조물 약한 제약→전체 frame PGO | **2층 과거 포즈 재사용** |
| F03 | 644↔1035 제약 제외한 frame 감사 후보 | **2층 과거 진단 포즈 재사용** |
| J01 | 과거 joint `pgo4` | 과거 통합 가설의 포즈 재사용 |
| J02 | 과거 joint `pgo6_attitude` | 과거 통합 가설의 포즈 재사용 |
| J03 | 과거 joint `pgo6_normals` | 과거 통합 가설의 포즈 재사용 |
| J04 | 과거 joint `pgo6_planes` | 과거 통합 가설의 포즈 재사용 |

외부·외부 브리지는 경사 가능성이 있어 N06 수평면 가정을 새로 강제하지 않았다.
기존 BA·수동 대응·같은 유한 평면 쌍은 자료가 있는 2층 결과를 F/J로 보여 준다.
나머지 네 기록에 존재하지 않는 수동 관측/BA를 만들어 적용했다고 주장하지 않는다.
EKF·새 IMU·새 GNSS prior·새 동적 객체 segmentation은 이번에 추가하지 않았다.

J 후보는 과거 joint 의존성을 가지고, F 후보는 frame 수준 변형을 허용했던 결과다.
N 후보와 입력·자유도가 다르므로 이 비교는 **실제 후보 선택용 포트폴리오**이지,
오직 loss 이름 하나만 바꾼 순수 통제 실험은 아니다. 각 후보의 원래 한계도 유지한다.

## 3. 기록별 전처리 실험

설정: `config/student_union_session_preprocessing.json`.
기록마다2개씩, 총10개 profile을 시험했다. 원래 depth 값을 smoothing하거나
이동하지 않고 불안정한 픽셀만 제외하는 방식이다.

1. float32 depth PNG와 보정값을 복원한다.
2. confidence의 실제0/50/100 값을 읽는다. optional12-byte trailer가 있어도 이를
   무시하고 기존 픽셀을 읽으며, 형식이 다르다고 전부 confidence100으로 만들지 않는다.
3. 범위/신뢰도 마스크 후, 3×3 median과 차이가 큰 고립 depth를 제외한다.
4. profile에 따라 국소 depth discontinuity를 추가 제외한다. 모든 profile에서
   계단·경사를 한 평면으로 덮는 처리는 없다.

median 차이 gate는 `고정 gate + 0.015 × depth`이고, 경계 gate가 있는 경우에는
3×3 유효 depth 범위에 `고정 gate + 0.02 × depth`를 적용한다.
**median 값으로 원래 depth를 교체하는 필터가 아니다.**

| 기록 | 비교한2개 profile | 현재 갤러리의 전처리 |
|---|---|---|
| 외부 | coverage / high-confidence100 | confidence50,4m,median gate15cm |
| 로비 | boundary / conservative | confidence50,4m,median gate15cm |
| 외부 브리지 | transition coverage / near3.5m | confidence50,4m,median gate18cm |
| 2층 | corridor boundary / high-confidence100 | confidence50,4m,median10cm+경계20cm |
| 서브 계단 | preserve edges3.5m / full range4m | confidence50,3.5m,median22cm,경계 제거 없음 |

profile별로 동일한 기존 대응 구간과 변경하지 않은 baseline 테스트 voxel을 사용했다.
포즈 초기화 국소 ICP의 평균 clipped RMSE를 비교하고, 평균 픽셀 유지율65% 이상인
profile 중 선택했다. 따라서 **상관된 개발 자료에서 고른 후보**이지 기록별 전처리의
전역 최적해를 입증한 것이 아니다. 같은 profile이 외부·로비에 선택될 수 있으며,
다르게 보이게 하려고 억지로 필터를 강하게 하지 않았다.

실제로 선택된 느슨한 profile은 외부·로비·외부 브리지에서 픽셀을 거의 제거하지
않았다. 해당 기록의 큰 전처리 차이는 기존 A/B 프레임 선택이다. 더 강한 profile이
정합 근거를 잃거나 국소 검사에서 낫지 않아 선택되지 않은 사실까지 기록했다.
전체 A/B 입력 픽셀 유지율은 외부100%,로비99.9999%,외부 브리지100%,
2층99.7397%,서브 계단91.9995%다. 반올림 전 값은 `verification.json`에 있다.

## 4. 전처리 결과를 최적화에 반영한 방법

전처리를 단순히 마지막 점군 청소에만 적용하지 않았다.

- 기존 학습 재방문 쌍과 최대12개 native odometry anchor 쌍을 국소 검토 대상으로
  잡았다. **예약 edge는 fitting 대상으로 쓰지 않았다.**
- 각 방문의 중심±1프레임을 같은 품질 run 안에서만 고정 포즈로 합쳤다.
- voxel6cm의 공간 hash로 학습/테스트를 분리했다. 전처리한 학습점으로만 기존
  `optimize_session.trimmed_icp`를 실행했다:20회, 최대 대응22cm,거리 하위80%,
  SVD rigid fit, 양방향 계산.
- 테스트는 필터로 지우지 않은 원래 baseline test cloud다. overlap≥30%,
  clipped RMSE 악화≤1mm,overlap 악화≤1%p,양방향 cycle<5cm/<2°,
  원래 포즈 대비 보정<35cm/<5°와 실질적 개선 조건을 검사했다.
- 게이트 통과는 **기존 위치 추정 근처의 국소 기하 개선**이다. RGB를 사용한 새로운
  장소 식별이나 반복 벽의 독립 동일성 증명으로 취급하지 않았다. 인접 프레임의
  물리 표본과 depth 잡음은 여전히 상관되어 있다.
- 재방문 측정 교체 시 원래 제약을 보관했다. native odometry는 삭제하지 않고,
  통과한 순차 깊이 측정만 약한 등록 제약으로 추가했다.

| 기록 | 통과해 반영한 국소 측정 | 기존 재방문 측정 교체 | 추가 순차 측정 |
|---|---:|---:|---:|
| 외부 | 5 | 3 | 2 |
| 로비 | 11 | 10 | 1 |
| 외부 브리지 | 0 | 0 | 0 |
| 2층 | 12 | 11 | 1 |
| 서브 계단 | 2 | 1 | 1 |

재방문 측정 모델은 translation σ15cm/rotation5°, 추가 순차 측정은20cm/6°의
보수적인 등방 **실험 모델**이다. 센서에서 추정·보정한 공분산이나 rank-aware
정보행렬이라고 부르지 않는다. 이 선택으로 원래 약한 native loop보다 강해지는
경우가 있으므로, 결과와 주변 악화를 함께 확인하고 자동 채택하지 않았다.

2층 N 후보는 최신 frame 그래프의 재방문 제약을 submap anchor로 다시 변환하고,
감사에서 문제였던644↔1035의 원래 ICP 제약을 제외한 실험이다. 원본은 보존했다.
이 N 실험에서는656↔1055도 국소 ICP 재검토·교체 대상에 포함됐다. 기존 BA 결과를
유지한 F01/F02/F03와 다를 수 있으며 어느 쪽이 좋은지 사진·고정 패치로 비교한다.
N 후보를 기존 BA를 그대로 보존한 유일한 후속 결과라고 부르지 않는다.

### 실행한 optimizer

N01/N02/N03는 `continue_floor2_graph.continue_graph`의 기존6DoF backend를 사용한다.
outer12회,inner 최대200평가,LSMR5,000회,3-point Jacobian이다. 기존 graph gauge와
componentwise correction bound를 유지하고 물리 구역의 상관 정보 정규화를 적용했다.
보존된 예약 edge는 목적함수에서 제외한다.

N04는 `optimize_geometry_stage.optimize_four_dof` 및 기존 `optimize_session.optimize`를
사용했다. N05/N06는 기존 `optimize_normals` 코드를 사용했다. 이 legacy 실행은
최대300평가와 원래 solver tolerance/bound를 유지했고, 선언된 구역의 측정 수로
정보를 나누어 같은 구역의 중복 투표를 줄였다.

N05의2° 자세 prior는 seed 자세의 regularization이며 새 IMU 중력 측정이 아니다.
N06는 최대30개 anchor에서 기존 RANSAC/SVD 수평면 추출을 수행하고, camera 중심
기준0.4~2.4m 아래의 조건부 표면 법선만 사용했다. 로비28개·2층26개·서브계단17개다.
tabletop/landing 의미의 불확실성이 남아 있는 **별도 후보**이며 전역 바닥 높이를
제약하지 않는다. 계단 디딤판들을 한 높이로 묶지 않았다.

새 실행28개는 모두 마지막 inner solve 성공·active bound0개다. robust15개 중
strict outer 기준은9개 충족,로비/2층6개 미달이다. legacy13개에는 같은 outer
반복 정의가 없어 이를 strict outer 통과로 세지 않았다. 지도 정확도 합격은 별개다.

외부 브리지는 통과한 새 측정0개이고 graph가 chain이라 여러 N 결과가 거의 같다.
같은 포즈의 동일 렌더는 별도 개선으로 집계하지 않았다.

## 5. 사진과 수치의 공정성

- DB를 read-only로 열고 각 포즈로 RGB-D를 다시 재투영했다. 기존 완성 점군을
  비강체로 보기 좋게 펴거나 어긋난 벽만 삭제하지 않았다.
- 모든 export는 pixel stride2,frame stride1,voxel5cm다. 즉 모든 선택 프레임을
  사용하되 depth 픽셀은 가로·세로2칸 간격이며 앱의 원래 고밀도 export와 점 수는 다르다.
- 각 P00/H/N/F/J 후보의 **카메라 좌표 점·프레임·입력 점 수와 hash가 동일**하다.
  최적화별 voxel 출력 점 수 차이는 포즈 차이에 따른 것으로 원래 관측을 다르게 골라서
  만든 차이가 아니다. O_FULL/O00 원본 참조만 명시된 예외다.
- 표시 좌표는 첫 공통 A/B 카메라에서 단 한 번의 rigid gauge로 맞췄다.
  후보별 추가 ICP best-fit 정렬 없이 비교했다.
- 모든 후보를 포함하는 전체 min/max로 축·색 범위를 고정했다. percentile crop으로
  어긋난 꼬리나 나쁜 후보를 밖으로 감추지 않았다. 실제 PLY는 sampling하지 않았다.
- 사진의 scatter는 개별 최대35만점,전체 후보판24만점의 결정적 표시 표본이다.
  PLY에는 전체 voxel 결과를 남겼다.
- 고정 패치 평가에서는 후보 전처리를 적용하지 않은 원래 관측을 사용했다.
  따라서 P00가 점을 지운다고 평가 오차가 저절로 좋아지지 않는다.
- 외부16개·로비36개·2층38개 같은 과거 패치로 중앙값/P90/최대를 기록했다.
  두 브리지는 이 고정 검증 재고가0개라 `NO fixed-patch validation`으로 표시했다.
  오류0으로 표기하지 않았다.

이 패치들은 반복 사용한 개발 관측이며 일부가 학습 제약·문제 선택과 상관되어 있다.
P90 한 숫자나 눈으로 예쁜 지도가 측량 정확도·안전한 길찾기를 보장하지 않는다.
전체 point-to-plane 간격과 실제 동일 장소/가시 영역이 맞는지도 계속 확인해야 한다.

## 6. 사진을 고르는 순서

1. 해당 기록의 `three_stage.png`로 전체 모양을 본다.
2. `matched_three_stage.png`로 frame 제외 효과와 포즈 개선을 구분한다.
3. `portfolio_xy.png`에서 겹친 벽·기둥·복도 모양을 비교한다.
4. `portfolio_xz.png`에서 바닥 높이·이중 바닥·계단 단차를 비교한다.
5. 후보 ID의 개별 사진으로 확대하고 숫자/수렴 경고를 본다.
6. 기록별로 원하는 ID를 남긴다. 서로 다른 후보의 X/Y/Z 성분을 잘라 조립하는
   선택은 하지 않고, 선택한 일관된 SE(3) 포즈 전체를 사용한다.

기계 판독 수치는 [candidate_metrics.csv](../assets/session-portfolios/candidate_metrics.csv),
모든 입력·사진·point support 검증 결과는 [verification.json](../assets/session-portfolios/verification.json).
각 기록의 `summary.json`에는 profile 시험 전체,graph/source,solver 상태,pose/cloud
경로·hash,고정 패치별 값이 있다. **winner는 비워 두었다.**

## 7. 재현·파일 경로·보존

원본 계산·후보 PLY: `outputs/student_union_20261017_session_portfolios_v1/`.
예를 들어 새2층 DCS 점군은 `floor2/clouds/N02.ply`,폰 앱 전체는
`floor2/clouds/O_FULL.ply`,전처리만은 `floor2/clouds/P00.ply`다.

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
python3 src/build_session_portfolios.py \
  --output outputs/student_union_20261017_session_portfolios_NEW --workers 3

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
python3 src/render_session_portfolios.py \
  --run outputs/student_union_20261017_session_portfolios_NEW \
  --output outputs/student_union_20261017_session_portfolios_NEW/gallery --workers 2

python3 src/validate_session_portfolios.py \
  --run outputs/student_union_20261017_session_portfolios_NEW \
  --assets outputs/student_union_20261017_session_portfolios_NEW/gallery
```

각 출력 경로는 새 경로여야 한다. 모든 실제 실행 입력은 hash로 묶었고
`source_snapshot.py`, `render_source_snapshot.py`, `verified_source_snapshots/`에 소스를
보존했다. 원본 DB·기존 RViz·기존 선택 지도는 유지했다. 원본data·대용량outputs·PDF의
Git 제외 정책도 유지한다.

테스트: 전체 `python3 -m pytest -q` **348 passed**. 새5개 테스트는 confidence trailer,
결측 confidence 거부,계단 경계 보존,고립 spike 제외,입력 depth 불변,표시 gauge의
내부 상대 포즈 보존을 검사한다. 코드 테스트 통과와 지도 평가 및 최종 선택은 별개다.
