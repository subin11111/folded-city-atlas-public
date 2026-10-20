# 웹 지도 화면 기록

[연구 보고서](../../../README.md) / [최신 모바일 화면](../mobile-journey-20261020/README.md) / [자료 전체](../../EVIDENCE.md)

대상: [Folded City Production](https://folded-city-atlas-subin11111s-projects.vercel.app/)

- 웹 소스 기준: `dc45c6a84bb8af2c10d9d35c9a1ca809be0ccdc3`.
- 배포 릴리스 기준일: 2026-10-19. 캡처 작업은 한국 시간 10월 19–20일에 진행했다.
- 환경: 기존 인증이 있는 브라우저의 실제 화면. 일반 화면 1280×720, 사진 검사 1265×712, 마지막 두 화면 848×871의 캡처 원본 크기를 유지했다.
- 서비스의 실제 UI를 캡처했으며 이미지 후처리는 적용하지 않았다.
- 08/09는 서비스의 실제 밝기 전환을 사용했다. 같은 `bridge_out1f:41` 사진이다.
- 모든 지도 좌표·사진 탐색 위치는 현재 프로토타입의 가상 위치이며 실시간 사용자 측위가 아니다.

| 파일 | 화면·확인 내용 |
| --- | --- |
| [01-campus.jpg](01-campus.jpg) | 첫 화면, OSM 기반 캠퍼스 48개 건물·실사 카드·검색 |
| [02-indoor-3d.jpg](02-indoor-3d.jpg) | 학생회관 실내 선택, 스캔 GLB·장소·층 선택 |
| [03-route-3d.jpg](03-route-3d.jpg) | 고전음악감상실 3D 경로와 경로 재생 컨트롤 |
| [04-route-options.jpg](04-route-options.jpg) | 41.8m 최단 도보·47.9m 메인·41.8m 서브·36.4m 승강기 후보 |
| [05-floorplan.jpg](05-floorplan.jpg) | 동일 목적지의 평면 경로와 비교 패널 |
| [06-photo-entrance.jpg](06-photo-entrance.jpg) | 입구 실제 사진·추천 이동·남은 거리·미니맵 |
| [07-photo-lobby.jpg](07-photo-lobby.jpg) | 출입문 통과 후 1층 실제 사진, 남은 거리 갱신 |
| [08-photo-original.jpg](08-photo-original.jpg) | 밝게 보기 끔, 원본 JPEG 표시 |
| [09-photo-corrected.jpg](09-photo-corrected.jpg) | 밝게 보기 켬, v3 파생본 표시 |
| [10-menu.jpg](10-menu.jpg) | 일반 메뉴, 연구 정보는 접힌 상태 |
| [11-project-info.jpg](11-project-info.jpg) | 연구 정보 펼침, 가상 측위·사진 대응·경로 검증 설명 |
| [12-photo-audit.jpg](12-photo-audit.jpg) | RGB 2,875개 전수 자동 검사·원본 목록 |
| [13-search-cu.jpg](13-search-cu.jpg) | CU 검색 결과와 실제 매장 사진 카드 |
| [14-cu-route.jpg](14-cu-route.jpg) | CU까지 44.3m의 1층 경로 |
| [15-elevator-route.jpg](15-elevator-route.jpg) | 음악감상실까지 계단 제외·승강기 후보 경로 |
| [16-floor2.jpg](16-floor2.jpg) | 2F 선택 상태와 2층 3D 표시 |

거리는 해당 버전의 보행 격자와 등록 연결에서 계산했다. 현장 실측 거리나 이동 시간과는 다르다. 데스크톱 화면은 이 배포본의 기록이며 최신 모바일 구성은 위 링크에서 확인할 수 있다.
