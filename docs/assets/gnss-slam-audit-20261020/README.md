# 실측 GNSS와 웹 지도 위치 대조 자료

[분석 방법과 결과](../../experiments/2026-10-20-gnss-slam-comparison.md)

다섯 원본 DB의 고유 측정 시각 1,099개 가운데 1,098개를 동시각 SLAM 웹 지도 위치와 대응했다. 기준은 공개 사진 189개의 위치와 일치하는 웹 제작 포즈 복원본이다. 원래 선택 포즈 CSV나 독립 측량 정답이 아니다.

| 자료 | 내용 |
| --- | --- |
| `summary.json` | 구간별 원본과 필터의 동일 시각 차이, 좌표 일관성 |
| `paired-input.json` | 읽기 전용 DB에서 추출한 고유 시각과 보간 기준 |
| `comparison.json` | 운영 코드로 계산한 칼만, RTS와 UI 상태 |
| `unique-fixes.csv` | 고유 시각별 좌표와 기준 위치 차이 |
| `comparison.geojson` | GNSS 위경도와 SLAM 기준의 거친 글로벌 배치 |
| `registration-sensitivity.json` | 실외 기록의 강건 SE(2) 재계산과 시간 구간 민감도 |
| `all-session-overlay.png` | 복도 지도와 다섯 구간의 궤적 중첩 |
| `residual-time.png` | 같은 시각의 위치 차이 |
| `residual-summary.png` | 공통 시각 RMSE |
| `corridor-ambiguity.png` | 보행면 근처여도 기준 위치와 다른 사례 |
| `registration-diagnostics.png` | 화면 이동 제거와 세션별 차이 |
| `multilevel-discrepancy.png` | SLAM 높이 위에 올린 GNSS 수평 차이 |
| `indoor-report.png` | 보고서용 1층 및 2층 비교 |
| `web-*.png` | 실제 로컬 웹의 실측 비교 화면 |
| `measured-replay.gif` | 실제 비교 화면을 28시점에서 촬영한 1층 재생 |
| `browser-verification.json` | 다섯 구간, 좌표 전환, 재생과 모바일 검사 |

생성형 이미지나 미관측 장면 합성은 사용하지 않았다. GNSS 높이 및 층은 추정하지 않았다. 비교용 3차원 그림의 Z는 동시각 SLAM 높이다. HTML 화면의 기준점도 새 방문자의 실시간 위치가 아니다.
