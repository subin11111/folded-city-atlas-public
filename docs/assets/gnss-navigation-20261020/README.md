# 현재 위치와 GNSS 보정 자료

2026-10-20의 코드와 실제 로컬 브라우저에서 만든 자료다. 생성형 장면 이미지를 사용하지 않았다. 그래프는 저장 JSON을 읽는 Matplotlib, 재현 애니메이션은 실제 브라우저 PNG를 인코딩한 Pillow 결과다.

| 파일 | 내용 |
| --- | --- |
| [filter-pipeline.png](filter-pipeline.png) | 위치 처리 및 후보정 흐름 |
| [recorded-plan.png](recorded-plan.png), [recorded-time.png](recorded-time.png) | 실측 야외 338개 고유 시각의 원본, 칼만 및 RTS |
| [synthetic-plan.png](synthetic-plan.png), [synthetic-time.png](synthetic-time.png) | 합성 기준 궤적, 원본 오차, NIS 및 수신 반경 |
| [synthetic-report.png](synthetic-report.png) | 보고서용 합성 오차와 NIS 비교 |
| [desktop-gnss.png](desktop-gnss.png) | 1440×960 웹 화면, 합성 입력 |
| [mobile-gnss.png](mobile-gnss.png) | 390×844 모바일 뷰포트, 2배 캡처 |
| [outlier-rejection.png](outlier-rejection.png) | 주입한 위치 튐의 갱신 거부 |
| [entry-handoff.png](entry-handoff.png) | 미등록 접근로와 입구 수동 확인 |
| [gnss-replay.gif](gnss-replay.gif) | 실제 모바일 UI 24프레임, 합성 입력임을 표시 |
| [filter_results.json](filter_results.json) | 합성 기준, 필터 상태, 공분산, RTS 및 수치 |
| [recorded-filter-results.json](recorded-filter-results.json) | 실측 원본 처리 결과, 절대 기준 위치 없음 |
| [demo-browser-export.json](demo-browser-export.json) | 브라우저 내보내기로 저장한 합성 로그 |
| [browser-verification.json](browser-verification.json) | 수신에서 실내 경로 응답까지 확인 기록 |

시험 입력과 숫자 계산 범위, 센서 권한 시험의 대역 사용은 [실험 기록](../../experiments/2026-10-20-gnss-navigation.md)에 정리했다. 실측 결과에는 독립 기준 위치가 없어 절대 오차 수치를 붙이지 않는다.
