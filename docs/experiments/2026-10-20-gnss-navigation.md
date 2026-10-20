# 휴대폰 현재 위치와 GNSS 보정

## 구현 범위

브라우저 위치 API로 현재 위치를 받고 카메라 지도와 같은 미터 좌표에 표시한다. GNSS 기반 실외 접근과 수동 입구 확인 이후의 실내 경로를 한 화면에서 연결한다. 새 방문자의 영상 기반 실내 자동 재위치 추정이나 RTK를 구현한 것은 아니다.

기존 지도 제작의 GNSS factor 수는 여전히 0이다. 이번 필터는 만들어진 지도 위에서 새 위치 수신을 처리하는 별도의 클라이언트 모듈이다. 현재 위치와 사용자가 저장하는 로그는 서버에 보내지 않는다.

## 상태 추정

`gnss-filter.js`는 상태 `[x,y,vx,vy]`의 CV Kalman, Joseph 공분산 갱신과 RTS 평활화를 구현한다. 배열 기반 행렬 연산과 역행렬은 4차원 이하이며 서버나 별도 학습 모델 없이 동작한다. 공분산 대칭성, 역행렬, RTS 구간 경계를 Node 테스트로 확인한다.

| 설정 | 값과 의미 |
| --- | --- |
| 가속도 표준편차 | 1.2m/s² |
| 관측 H | XY 위치만 관측 |
| 브라우저 R | 95% 반경 / 2.44774683, 등방성 Gaussian 가정, σ 최소 1.5m |
| 기존 iOS 로그 R | 신뢰수준을 단정하지 않고 보고 반경을 σ로 사용 |
| 혁신 판정 | NIS ≤ χ²(2, 99%) = 9.21034 |
| 속도 판정 | 마지막 수락 원 수신과의 거리 ≤ 4m/s × Δt + 양쪽 보고 반경 |
| 저정확도 | 50m 초과 측정 갱신 거부, 25m 초과 진행 안내 중단 |
| 오래된 실시간 입력 | 수신 시각이 현재 시각과 15초 초과 차이이면 사용하지 않음 |
| 단절 | 정상 수신 후 10초이면 stale, 15초 초과이면 새 필터 구간 |
| 헤딩 | 원형 보간 가중치 0.3, 나침반 3초 우선 유지, 표시 유효시간 5초 |
| 나침반 | iOS webkitCompassHeading, 보고 나침반 오차 40° 초과이면 사용하지 않음 |
| 이동 방향 | 수락된 위치의 제공 heading, 제공 speed ≥ 0.7m/s |

R의 가정, 과정 잡음과 속도 상한은 고정 설계값이다. 지도 정합 오차나 일정하게 치우친 다중경로 편향을 이 모델이 자동으로 해결하지는 않는다. 나침반 자북과 GNSS 이동 방향 진북을 화면에 구분하며 자기 편차의 정밀 보상은 적용하지 않았다.

거부된 관측은 원본 로그에 남는다. 실시간 마커는 마지막 정상 위치에 머물며, 필터 내부 예측은 별도로 저장한다. RTS는 이 저장 상태와 공분산을 역방향으로 계산하고 수신 공백의 경계를 넘지 않는다.

## 실외 경로

`outdoor-routing.js`는 OSM의 연결된 도로 선분을 분할해 출발점과 도착 접근점을 삽입하고 Dijkstra를 실행한다. 허용 유형은 footway, path, pedestrian, residential, service, living_street, steps, track이다. `foot=no/private`, 허용 근거 없는 `access=no/private`는 사용하지 않는다. layer가 다른 교차점도 합치지 않는다.

도로 그래프의 최단 거리와 현재 위치에서 도로까지의 이격, 도로에서 입구까지의 이격은 별도 값이다. 현재 학생회관 접근 지점은 가장 가까운 도로와 37.2417m 떨어져 있다. 이 구간에는 선을 그리거나 통행 연결을 합성하지 않는다. 도로 안내가 끝나면 현장에서 입구를 확인하고 기존 실내 A*를 시작한다. 승강기 후보를 포함한 실내 경로 정책은 기존 방식대로 유지한다.

## 실측과 합성 입력

실측 야외 DB를 읽기 전용 SQLite로 열어 GPS blob의 6개 double을 해독했다. 양의 정확도, 유효 좌표와 고유 GNSS 시각을 기준으로 338개를 추출했다. 지도 정합의 기존 290개 표본과 선정 기준이 다르다. 입력 DB SHA-256은 `e5a5c517bbddf2ec5b6877bbf38c67e0b88baeda0c4be57b784d60c05949dc91`이다.

338개는 새 필터 기준을 모두 통과했다. 독립 기준 위치가 없어 절대 오차를 계산하지 않는다. [실측 원본 및 보정](../assets/gnss-navigation-20261020/recorded-plan.png)과 [좌표 시간 변화](../assets/gnss-navigation-20261020/recorded-time.png)는 평활화 형태를 보여준다.

합성 입력은 연결된 약 90.71m의 OSM 경로와 seed 20261020으로 생성한다. 기준 경로 주변 표준편차 2.2m의 독립 XY 잡음, 위치 튐 3회, 보고 반경 85m인 수신 1회, 70~87초 입력 누락을 포함한다. 누락 전후의 시각 차이는 19초다.

| 합성 입력 평가 | 값 |
| --- | ---: |
| 입력 수 | 97 |
| 수락 / 거부 | 93 / 4 |
| 공통 RMSE 평가 시각 수 | 96 |
| 원 수신 RMSE | 13.5698m |
| 칼만 필터 내부 상태 RMSE | 2.3572m |
| RTS RMSE | 1.4054m |

보고 반경 85m인 입력은 상태 기록을 만들기 전에 거부해 RMSE의 공통 시각에서 빠진다. 혁신으로 거부한 3개 시각의 칼만 내부 예측은 비교에 남고, 화면 마커 유지와 구분한다. 실제 휴대폰의 정확도 향상률로 해석할 수 없다.

## 실행

```sh
cd web
npm ci
python -X utf8 server.py --port 8775
```

다른 터미널에서 합성 입력과 처리 결과를 재생성한다.

```sh
cd web
npm run gnss:demo
node scripts/analyze-recorded-gnss.mjs
node --test test_gnss.mjs
```

원본 DB가 있을 때만 추출을 다시 실행한다. 공개 웹의 재현에는 포함된 고유 수신 JSON을 사용한다.

```sh
python scripts/extract-gnss.py /path/to/261016-234854-outside.db public/data/gnss_outside_recorded.json
```

그림 생성에는 `matplotlib`, `numpy`, `Pillow`가 필요하다. `GNSS_PLOT_DEPS`는 별도 설치 폴더를 사용할 때만 지정한다.

```sh
python scripts/plot-gnss.py
```

브라우저 캡처는 Playwright 및 설치된 Edge를 사용한다. 별도 패키지 디렉터리는 `PLAYWRIGHT_NODE_MODULES`, 다른 Chromium 채널은 `BROWSER_CHANNEL`, 서버 주소는 `GNSS_TEST_URL`로 설정한다. 기본 URL은 `http://127.0.0.1:8773`이다.

```sh
node scripts/verify-gnss-browser.mjs
python scripts/plot-gnss.py
```

두 번째 그림 생성 명령은 브라우저가 기록한 24개 실제 모바일 화면을 GIF로 인코딩한다. 입력이 재현 자료임을 나타내는 UI 표시는 제거하지 않는다.

## 브라우저 확인

10개 시나리오를 확인했다. 위치 API 콜백에서 ENU 및 칼만과 마커까지, 방향 허용과 자북 라벨, clearWatch, 주입 이상치, 공백 분리, 원본 및 후보정 저장, 저장 로그 불러오기, 입구 확인 이후 `/api/route` HTTP 200, 모바일 넘침, 권한 거부 메시지를 포함한다.

실시간 위치 API에는 브라우저 모의 좌표를 사용했다. 데스크톱 방향 센서가 없으므로 헤딩 허용 분기는 권한 응답 대역과 방향 이벤트 입력으로 시험했다. 실제 iPhone 나침반 수신 시험은 아니다. 결과는 [검증 JSON](../assets/gnss-navigation-20261020/browser-verification.json)에 남겼다.

기존 Node 15개에 GNSS 7개를 더해 Node 22개, Python 29개가 통과했다. cold-cache 모바일 요청에서는 로컬 서버 기본 연결 대기열이 부족해 초기 파일 요청이 지연되어 대기열을 64로 늘렸다. 이후 같은 흐름의 실제 브라우저 캡처를 완료했다.

## 근거

- [W3C Geolocation](https://www.w3.org/TR/geolocation/)
- [MDN accuracy](https://developer.mozilla.org/en-US/docs/Web/API/GeolocationCoordinates/accuracy)
- [MDN heading](https://developer.mozilla.org/en-US/docs/Web/API/GeolocationCoordinates/heading)
- [Apple DeviceOrientationEvent](https://developer.apple.com/documentation/webkitjs/deviceorientationevent)
- [FilterPy KalmanFilter 및 RTS](https://filterpy.readthedocs.io/en/latest/kalman/KalmanFilter.html)
