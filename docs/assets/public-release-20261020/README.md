# 웹 화면과 관측 처리 자료

[전체 연구 보고서](../../../README.md) / [시각 자료 전체](../../EVIDENCE.md)

현재 작업 트리의 로컬 웹을 직접 조작해 캡처했다. 파일별 해시와 기준 커밋은 manifest.json에 기록한다. 기준 커밋 위의 미커밋 수정도 포함된 캡처이며, 배포 커밋을 촬영했다고 소급해서 표시하지 않는다.

| 자료 | 의미 |
| --- | --- |
| 01-campus.jpg, 02-route-comparison.jpg | 캠퍼스와 계산 경로 |
| 03-prime-photo.jpg, 04-prime-3d.jpg | PRIME HALL 앞 실사 사진과 3D |
| 05-mobile-route.jpg ~ 07-mobile-map.jpg | 390×844 모바일 목록·지도 |
| 08-measured-gnss.jpg | 학생회관에서 수집한 실측 GNSS의 재생 |
| 10-indoor-osm.jpg | 실내 모델 주위에도 배경지도를 유지한 화면 |
| 11-grounded-indoor.jpg | 2026-10-21, 실내 1층 기준을 배경 평면에 정합하고 4.15m 층간 높이를 보존한 화면. 1280×720 |
| navigation-prime.mp4, navigation-prime.gif | 실제 브라우저 캡처 13장을 편집한 사진 안내 시연 |
| navigation-capture.json | 각 캡처의 화면 상태 기록 |
| sift-preprocessing.png, rootsift-descriptor.png | 영상 전처리와 특징 기술자 처리 |
| ransac-image-matches.png, ransac-metric-consensus.png | 실제 영상·깊이 대응의 기하 검사 |
| sensor-floor-transition.png, sensor-imu-motion.png | 추가 약 112초 보행 로그의 기압·IMU |
| sensor-registration-offset.png | 기존 지도 배치 오프셋 비교 |

시연의 이동은 A* 계산 경로를 버튼으로 따라가는 가상 안내다. 현장 보행이나 카메라 없는 자동 실내 측위 영상이 아니다. GNSS 재생과 IMU·기압 그림은 실제 기록이며 합성 위치 시험과 구분한다.
