# 장소 원장과 실내 주소 설계

## 장소 정보와 표시 모델

원본 RTAB-Map DB나 GLB 삼각형 번호 대신 별도의 장소 원장을 둔다. 현재 웹 구현은 `web/public/data/catalog.json`에 장소, 별칭, 근거 사진과 보행 접근점을 저장한다. 아래 SQLite 관계형 구조는 여러 건물로 확장할 때의 설계안이다.

이 문서의 예시 JSON과 초기 사진 수량은 2026년 10월 19일 설계 시점의 기록이다. 현재 등록 상태는 [장소 원장](../web/public/data/catalog.json), 전체 사진은 [사진 원장](../web/public/data/photo_library.json)에서 확인한다.

표시 예시는 **건국대학교 → 제1학생회관 → 2층 → PRIME HALL**이다. 건물 도로명주소,
호실번호, 장소명, 지도 좌표, 실제 들어가는 문은 서로 다른 정보로 저장한다.

## 필요한 데이터

| 테이블 | 주요 정보 | 역할 |
| --- | --- | --- |
| buildings | 영구 building_id, 명칭/별칭, 검증된 도로명주소, 외부 지도 연결 | 외부 주소와 실내 주소의 부모 |
| levels | 영구 level_id, building_id, 표시명1층/2층, 수직 순서 | 같은 XY의 다른 층을 구분 |
| places | 영구 place_id, level_id, 이름, 호실번호 또는 null, 분류, 별칭 | 방·매장·시설 검색 |
| place_anchors | place_id, 선택 지도 버전, 좌표계, 미터 XYZ, 공간 영역, 사진 증거 | 지도상 표식과 공간의 위치 |
| portals | 영구 portal_id, 두 공간/층, 문/계단/엘리베이터, 문 위치·방향·폭·검증상태 | 실제 통과 가능한 연결 |
| place_portals | place_id ↔ portal_id | 장소에 출입문이 여러 개일 수 있음 |
| route_nodes / edges | 보행 좌표·층·연결·계단/접근성·검증상태 | 검색 결과를 실제 안내 목적지로 연결 |
| observations | 장소/문 ID, 촬영세션·노드·사진·검토자/상태 | 표시/연결 판단의 근거와 수정 이력 |

SQLite에서는 관계를 외래키로 보호하고 연결별 `PRAGMA foreign_keys=ON`을 켠다.
공간 후보 검색이 필요하면 R*Tree로 영역 검색을 보조할 수 있다. R*Tree 겹침은
같은 방/출입구라는 증명이 아니므로 최종 영역/층/접근성 검사는 별도다.
[SQLite 공식 외래키 문서](https://www.sqlite.org/foreignkeys.html),
[공식 R*Tree 문서](https://www.sqlite.org/rtree.html).

## 현재 프로젝트에서 특히 중요한 구분

1. **촬영 로그5개 ≠ 층5개**다. `floor2` 촬영에도 1층 일부·계단이 들어 있으므로
   session명을 level_id로 쓰지 않는다. 한 장소는 여러 세션의 관측을 가질 수 있다.
2. **GLB 삼각형 번호 ≠ 영구 주소**다. 이번 파편/중복 제거로 순서가 바뀐다.
   기존 `semantic_objects.json`에도 오래된 `current_triangle_indices`가 있어 최종
   정리 파생본에서는 이 번호를 무효화하고 원본 이력으로 따로 보존했다. 안정적인
   object_id·world_boundary·원본사진은 유지했다. 클릭용 face→place_id 바인딩은
   최종 웹 메시를 내보낼 때 그 버전에 맞춰 다시 생성한다.
3. **표식 위치 ≠ 길찾기 목적지**다. 상점 가운데 표식을 두더라도 경로는 복도 쪽
   출입문 앞 보행 노드에서 끝나야 한다. 단순 최근접점으로 벽을 뚫고 안내하지 않는다.
4. **표시용 Z ≠ 측량한 층고/실제 보행 Z**다. 현재 외관 모델의 층 높이 정규화와
   선택된 SLAM 지도 좌표계를 구별한다. 원장은 선택 지도 버전에 연결한 미터 좌표와
   층ID를 갖고, 표시 Z/Three.js 축 변환은 파생값으로 관리한다.
5. **문이 사진에서 닫힘 ≠ 통행 불가**다. 엘리베이터/문은 개폐 가능성, 실제 접근성,
   현재 운영정보, 통행 검증을 구별한다. 촬영순서 edge도 보행 승인과 다르다.
6. **보기 좋게 메운 바닥/천장 ≠ 보행 가능한 면**이다. 설비 공간과 중앙 로비의
   빈 공간을 경로 그래프에 자동으로 넣지 않는다.

## 장소 하나의 웹 내보내기 예시

아래 ID는 설명용 예시다. 임의의 호실번호나 좌표를 실제 값처럼 채우지 않았다.

```json
{
  "place_id": "place_prime_hall",
  "building_id": "konkuk_student_union_1",
  "level_id": "konkuk_student_union_1_f2",
  "name": "PRIME HALL",
  "aliases": ["프라임홀"],
  "unit_code": null,
  "indoor_address": "건국대학교 제1학생회관 2층 PRIME HALL",
  "map_version": "student_union_selected_20261017_v1",
  "anchor_map_m": null,
  "anchor_status": "NEEDS_VERIFIED_CANONICAL_MAP_BINDING",
  "entrance_portal_ids": [],
  "routing_status": "UNVERIFIED",
  "evidence": [{"semantic_object_id": "floor2_prime_hall_complete_fascia", "session": "floor2", "node": 781}]
}
```

영구 내부 ID는 이름 변경/폐점/호실명 수정으로 바꾸지 않는다. 필요하면 UUID를
내부 키로 사용하고 읽기 쉬운 slug는 외부 URL용으로 따로 둔다. 텍스트 실내주소는
부모 건물·층·장소 이름에서 생성하고, 실제 호실번호가 확인되었을 때만 추가한다.
운영 종료/층 이동은 이력을 남기며 삭제·재활용으로 다른 장소와 섞지 않는다.

## 확장 시 등록 절차

1. 기존27개와 이번 보완 벽2개를 포함한 의미 물체29개, 사진1,199개에서
   **장소/출입문/표지 물체**를 구분한다. 벽 보완2개는 새 주소나 매장이 아니다.
   같은 장소의 여러 표지를 별개 매장으로 만들지 않는다.
2. 장소 관리자가 건물명·층·호실/매장명·별칭을 확인한다. 공식 도로명주소는 검증된
   별도 출처로 채우고 사진만 보고 법정 주소/호실을 단정하지 않는다.
3. 지도에서 표식 위치와 실제 출입문을 각각 지정하고 동일한 선택 지도 버전에 연결한다.
4. 현장 확인 또는 충분한 관측으로 문/계단/복도 보행 연결을 승인한다.
5. 웹의 검색/층 선택/장소 클릭/실내외 전이를 이 ID로 묶는다. 메시를 다시 정리해도
   주소 원장은 유지하고 그 버전의 클릭 바인딩만 재생성한다.
