# 연구 보고서 그림

2026년 10월 20일 작성한 최종 보고서의 원본 그림을 README에도 사용한다. 센서 관측과 보행 격자의 계산 결과로 제작했으며 파일별 SHA-256은 [manifest.json](manifest.json)에 있다.

| 파일 | 입력과 표현 |
| --- | --- |
| `design_process.png` | 개념설계, 기본설계, 상세설계와 구현의 기술 도식 |
| `destinations.png` | 직접 촬영한 출입구와 목적지 사진 |
| `rgb_depth.png` | 로비 DB 112번의 RGB와 유효 LiDAR 깊이 |
| `gnss_accuracy.png` | 초기 실외와 로비 DB의 GPS 필드에 저장된 보고 정확도 |
| `navigation_grid.png` | 현재 `navigation.json`의 층별 격자와 정책별 경로 |
| `navigation_3d.png` | 동일 미터 좌표의 층간 경로를 투영한 도표 |
| `route_distances.png` | 같은 외부 출발점에서 목적지별 계단 정책 거리 비교 |
| `loading-animation.gif` | 웹의 실제 로딩 화면을 캡처한 3.4초 반복 애니메이션 |

GNSS 보고 정확도는 실측 오차가 아니며 경로 거리는 지도 모델에서 계산한 값이다. 사진의 내용이나 지도 장면을 이미지 생성 모델로 만들지 않았다.

[연구 보고서](../../../README.md) / [전체 자료](../../EVIDENCE.md)
