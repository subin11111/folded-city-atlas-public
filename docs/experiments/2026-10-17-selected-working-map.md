# 선택 지도 확정과 다섯 세션 강체 통합

## 결과와 확정 범위

전체 형상 검토를 거쳐 **작업용 지도 버전**을 `student_union_selected_20261017_v1`로 고정했다.
2층·메인 계단은 명시적으로 선택한 J04를 사용했다. 나머지 기록은 직전 육안
shortlist 안에서 아래 대표 후보를 선정했다. 모든 기록에 J04를 적용한 것은 아니다.

| 기록 | 확정 후보 | 선택 이유 | 사용 포즈 / 점 수 |
|---|---|---|---:|
| 외부 | J01 | 선호한 J계열 중 고정 패치 P90이 가장 낮은 대표. 모든 후보의 수치 최선이라는 뜻은 아님 | 305 / 464,565 |
| 로비 | N05 | N04/N05/SE3 선호 범위. 자세 prior가 있는 SE3를 사용하되 N06의 추가 법선 변형은 피함 | 800 / 1,430,215 |
| 외부 브리지 | H03 | 선호한 H02와 포즈가 정확히 같은 DCS 대표 | 86 / 153,559 |
| 2층·메인 계단 | **J04** | 전체 형상 검토를 우선한 선택. 기존 국소 악화 수치는 그대로 보존 | 925 / 1,754,344 |
| 서브 계단 | H03 | 선호한 H/NEW SE3 중 보수적인 형상. 기존 학습 루프의 관측 정합과 계단 형상 보존 기준 | 135 / 261,829 |

총 **2,251포즈**, 개별 점군 합계 **4,064,512점**, 최종 5cm voxel 통합 **3,493,208점**이다.
후보마다 이미 재생성해 둔 색상 점군을 선택한 뒤 강체 이동했다. 원본 depth를 전부
다시 해석하거나, 새로운 객체 제거·내부 포즈 최적화를 실행한 것으로 세지 않는다.

**선택 지도 확정과 정합 정확도 승인은 다르다.** 현재 지도는 이후 표면 모델·의미
지도 개발의 기준 버전으로 사용할 수 있지만, 연결 가설의 엄격한 검증과 실제 길찾기
안전성은 미완료다. 기존 보고서의 `eligible_for_joint=false`를 true로 변경하지 않았다.

- 현재 선택: [`config/current_map.json`](../../config/current_map.json)
- 후보 선택 근거: [`config/student_union_selected_map_20261017.json`](../../config/student_union_selected_map_20261017.json)
- 실제 산출물: `outputs/student_union_selected_20261017_v1/`
- 보관용 수치·그림: [`selected-map-review`](../assets/selected-map-review/)
- 실행 보고서 사본: [`summary.json`](../assets/selected-map-review/summary.json)

![다섯 세션을 구분한 통합 지도](../assets/selected-map-review/selected_map_sessions.png)

초록=외부 J01, 주황=로비 N05, 분홍=외부 브리지 H03, 파랑=2층·메인 계단 J04,
보라=서브 계단 H03이다. XY에서 1층과 2층이 겹치는 것은 곧바로 잘못된 중첩이
아니다. XZ와 RViz 3D를 함께 확인해야 한다.

## 1. 선택 후보 내부를 고정

선택한 J04 내부 형상을 유지하기 위해 통합 변수는 **세션별 xyz 이동과 yaw 회전 4개뿐**이며, 2층 J04의 변환은
identity로 고정했다. 프레임별 포즈, 내부 odometry, 내부 루프, 평면 factor는
다시 풀지 않았다. “새 전체 프레임 pose-graph 최적화”가 아니라 **계층적 강체
배치 최적화**다.

각 세션의 모든 프레임에 동일한 `T_working_selected`를 왼쪽에서 곱했다.

`P_working[i] = T_session × P_selected[i]`

따라서 `inverse(P[i]) × P[j]`는 통합 전후 같다. 최대 상대행렬 차이는
**1.42e-14**, 카메라 Z 범위 차이는 **8.88e-16m**였다. 이는 수치적으로 모양과
높이 변화가 유지됐다는 확인이며, 실측 정확도나 실제 층고를 뜻하지 않는다.
바닥 평탄화·계단 압축·축별 배율 변경·국소 벽 삭제는 하지 않았다.

![같은 입력 점으로 본 연결 전후](../assets/selected-map-review/selected_map_before_after.png)

## 2. 로비·2층·서브 계단의 배치

`graph_final/used_constraints.json`의 기존 교차 세션 측정값을 사용했다.
학습과 audit 역할을 그대로 유지하고, 선택 A/B 포즈에 없는 endpoint는 제외했다.

- 로비↔2층 학습: 112↔14, 317↔1215, 337↔66.
- 로비↔서브 계단 학습: 462↔36, 495↔85.
- 2층↔서브 계단 학습: 553↔159.
- 로비↔2층 audit: 327↔51.
- 로비↔서브 계단 추가 audit: 443↔60. 기존 엘리베이터 관측이며 fitting에 넣지 않음.
- 172↔1158 학습과 560↔162 audit는 C등급 endpoint가 선택 포즈에 없어 제외.
  특히 hard boundary를 넘어 보간해서 복원하지 않았다.

잔차는 `inverse(T_measurement) × inverse(P_a) × P_b`의 이동·회전이다.
σ이동15cm, σ회전5°로 whitening한 뒤 **edge block-Huber δ=3**를 적용했다.
같은 세션 연결에 포함된 측정 수의 제곱근으로 residual을 나눠 연결 하나에
프레임 수만큼 중복 투표가 생기지 않게 했다. 이는 상관관계에 대한 보수적
휴리스틱이며 관측들이 실제 독립이라는 판정도, 보정된 센서 공분산도 아니다.

SciPy TRF, 최대600평가, `ftol=xtol=gtol=1e-11`, 이동 한계 ±5/±5/±3m,
yaw ±20°를 사용했다. 2층 고정, 로비·서브 계단 8변수로14평가 후 수렴했고
활성 한계는0개였다. 목적함수44.0243→1.2403. audit 값으로 모델·가중치·
후보를 다시 선택하지 않았다.

## 3. 외부 브리지와 출입문

기존 로비39↔브리지79의 RGB-D 상대 측정을 사용했다. 이 측정은 과거 재투영
엄격 기준을 통과하지 못했던 한 출입문 가설이다. 새 독립 연결로 승격하지 않았다.

브리지79는 회전 step으로 C등급 처리돼 점군에서 빠져 있다. 이를 점군에 다시
넣지 않고 **연결 계산에만 필요한 보조 포즈**를 만들었다. 양옆77·80은 같은
`bridge_out1f:s0012`이며 사이에 hard boundary가 없다. 고정 앱 포즈에 대한
선택 포즈의 왼쪽 강체 보정량을 timestamp 비율로 translation 선형 보간·rotation
SLERP해79에 적용했다. 같은 submap 안, 최대5노드 간격, timestamp 단조 조건을
강제한다. hard boundary나 다른 submap을 넘는 보간은 거부한다.

이는 새 측정이 아닌 **조건부 포즈 전달 가정**이다. 모델 σ이동30cm·회전8°로
약하게 두고 로비는 이미 결정한 배치에 고정해 브리지만 xyz/yaw로 배치했다.
브리지79의 RGB-D는 최종 점군·선택 궤적에 포함하지 않았다.
대체 audit 11↔23은 브리지23도 제외 프레임이라 복원하지 않고 누락으로 기록했다.
따라서 출입문 fitting 잔차가 작다고 브리지 전체 연결이 검증됐다고 할 수 없다.

## 4. 외부와 외부 브리지

이전 MAP-to-MAP 변환을 그대로 복사하지 않았다. 내부 후보 포즈가 바뀌면 같은
원본 프레임도 지도 안 위치가 달라지므로 **현재 선택 포즈에서 원본 대응점을 다시
투영해** 강체 변환을 재추정했다.

- 기존 SIFT 263↔117,262↔117,263↔118과 LoFTR263↔118의 저장된 학습 인라이어 사용.
- 모든 pair/matcher의 예약 depth 픽셀 ±3 이웃의 **합집합**을 학습에서 제외.
- 학습점은 총49개. 프레임쌍/매처별 총 제곱 가중치를 같게 설정.
- xyz+yaw, 3D point-block Huber8cm, 최대400평가, 종료 허용오차1e-11.
- 같은 출입구의 상관된 관측1구역이며, 네 독립 장소라고 계산하지 않음.
- 방향: `T_selected_outside_selected_bridge`는 bridge 좌표를 outside로 변환.
- 합성: `T_working_outside = T_working_bridge × inverse(T_outside_bridge)`.

예약 대응 RMSE는 SIFT순서대로9.75/6.57/5.94cm, LoFTR13.53cm다.
엄격7cm 기준을 넘는 관측이 남고 서로 독립된 여러 구역도 부족하므로
`strict_accepted=false`를 유지했다. GNSS로 강제 이동하거나 depth confidence를
완화하지 않았다.

## 5. 연결별 잔차

| 연결 | 역할 | 위치 잔차 | 회전 잔차 | 직접 측정→통합 배치의 clipped RMSE |
|---|---|---:|---:|---:|
| 로비112↔2층14 | 학습 | 3.55cm | 0.66° | 5.35→6.17cm |
| 로비317↔2층1215 | 학습 | 10.48cm | 1.77° | 12.81→14.21cm |
| 로비337↔2층66 | 학습 | 11.83cm | 1.62° | 9.08→10.50cm |
| 로비327↔2층51 | audit | **13.11cm** | **3.94°** | 11.53→12.11cm |
| 로비462↔서브36 | 학습 | 17.76cm | 0.88° | **6.56→9.34cm** |
| 2층553↔서브159 | 학습 | 7.53cm | 2.85° | 10.50→11.49cm |
| 로비495↔서브85 | 학습 | 17.12cm | 1.82° | 9.15→10.88cm |
| 로비443↔서브60 | audit | **19.86cm** | 1.88° | **6.71→13.50cm** |
| 로비39↔외부 브리지79 | 한 출입문 학습/보조포즈 | 약0cm | 0.78° | 7.53→7.54cm |

로비462↔서브36와 로비443↔서브60는 직접 측정 대비 RMSE+2cm guard를
통과하지 못했다. 로비↔2층 audit의 회전3.94°도 기존3° 중앙 기준을 넘는다.
강체로 해결되지 않는 내부 불일치가 남아 있으며 **전체 정합 성공이나 길찾기
검증 완료로 선언하지 않는다**. 선택한 내부 형상을 보존한 현재 지도다.

![연결의 직접 측정과 실제 통합 잔차](../assets/selected-map-review/connection_audit.png)

**J04의656↔1055 패치54.42cm도 그대로 남아 있다.** 그 사실과 전체 형상 선호를
함께 기록하고 이번 선택으로 덮어쓰지 않았다.
[해당 패치의 위치·원본 사진](../assets/visual-shortlist-review/wall_656_1055_locator.png).
거리와 clipped RMSE는 서로 다른 지표이며 측량 정답은 아니다.

![높이 색상으로 본 통합 지도](../assets/selected-map-review/selected_map_height.png)

## 6. 저장·실행·보존

```bash
# 현재 선택 지도 확인/표시
bash scripts/view_current_map.sh --check-only
bash scripts/view_current_map.sh

# 같은 선택을 새 폴더로 재현(기존 출력 덮어쓰기 거부)
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 /usr/bin/python3 -s src/assemble_selected_map.py \
  --output outputs/student_union_selected_reproduction \
  --assets docs/assets/selected-map-reproduction
```

통합 결과에는 `combined.ply`, 세션별5개PLY·궤적CSV,
`SELECTED_WORKING_MAP.rviz`, `manifest.json`이 있다. 프레임은
`map_selected_working`, topic은 `/folded_city/selected_working/*`이며 기존
preview와 별도다. 운영 `map/odom/base_link` TF를 연결하지 않는다.
기존 RViz publisher와 새 publisher가 같은 topic에서 지도를 섞지 않도록 분리했다.

저장한 RViz 설정의 동일한 Git 보관용 복사본은
[`config/student_union_selected_working.rviz`](../../config/student_union_selected_working.rviz)에 있다.
지도 PLY 자체는 기존 정책대로 로컬 `outputs/`에만 보관한다.

이 환경에서는 Humble 전체 setup의 의존성 파일 탐색이 I/O 대기 상태로 지연됐다.
전용 실행 파일은 설치된 Humble의 Python/동적 라이브러리/패키지 검색 경로를
해당 프로세스에만 지정해 실행한다. 원본 환경 설정이나 시스템 파일은 변경하지 않는다.

실제 RViz에서 merged RGB와 다섯 궤적이 표시되고 **Global Status: Ok**인 것을
확인했다. merged topic은 publisher/subscriber 각1개, 다섯 궤적도 각각1/1이다.
세션별 tint 점군은 기본 off라 각각 publisher1/subscriber0이다. 이 상태는 표시와
수신 확인이며 지도의 정확도 검증은 아니다.
[표시 확인·뷰어 해시](../assets/selected-map-review/display_verification.json).

![실제 RViz 표시 화면](../assets/selected-map-review/rviz_selected_working_map.png)

원본 DB5개·선택 후보·기존 측정/보고서·이전 지도는 해시로 보존 확인했다.
현재 지도 포인터만 새로 추가했으며 기존 기록을 삭제·덮어쓰기하지 않았다.
코드/수치/사진은 Git 보관 대상, DB/PLY/실행 출력은 기존 제외 규칙 유지다.
Git에는 코드, 선택 설정, 실험 문서와 비교 자료를 보관하고 원본 DB와 대형 지도 출력은 별도로 관리한다.

전체 테스트 **366 passed**. 통합 신규7개는 강체 높이·상대포즈 보존, audit의 fitting
비개입, 끊긴 학습 그래프 거부, 보조포즈의 같은 submap 제한·hard boundary 거부,
측정 변환 방향을 검사한다. 뷰어 신규7개는 명시적 선택 여부, frame/topic 격리,
승인 플래그 미승격, 통합 점군 hash, 후보 바꿔치기 거부를 검사한다.
테스트 통과는 실제 정합 정확도 승인을 뜻하지 않는다.
