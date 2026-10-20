# Ubuntu와 Windows 개발환경 이관

ROS 2 처리와 웹 실행은 필요한 환경이 다르다. 웹은 Windows의 Python과 Node.js로 실행할 수 있고, ROS 재생과 RViz는 Ubuntu 22.04 및 ROS 2 Humble 환경을 사용한다.

[연구 상태](../README.md) / [연구 과정](../README_HISTORY.md) / [웹 실행](../web/README.md)

## 저장소와 별도 자료

| 자료 | 보관 위치와 역할 |
| --- | --- |
| 소스, 설정과 테스트 | Git 저장소 |
| 실험 수치와 비교 그림 | `docs/experiments/`, `docs/assets/` |
| 웹 실행용 모델과 사진 | `web/public/` |
| 원본 DB 5개 | 별도 이관본의 `project/data/raw/rtabmap/` |
| 선택 포즈와 통합 점군 | `project/outputs/student_union_selected_20261017_v1/` |
| 최종 표시 모델 | `project/outputs/student_union_final_mesh_v9/` |
| ROS 재생 결과 | `project/outputs/student_union_slam_replay_v3/` |
| 환경 목록 | 이관본의 `handoff/environment/` |

원본 DB와 최종 산출물이 필요한 작업은 별도 로컬 이관본을 사용한다. 과거 대형 중간 출력은 이관본에 모두 들어 있지 않으므로 실험에 따라 다시 계산해야 한다. 기존 실험 문서와 manifest의 역사적 경로는 원 실행의 출처 정보다.

## 전체 이관본 복원

이관본은 `full_development.tar.zst`, SHA-256 파일과 `START_HERE.md`로 구성된다. 파일 목록과 해시는 `handoff/file_inventory.json`에 있다. `.partial`만 있는 파일은 완료본이 아니다.

새 빈 디렉터리에서 복원한다.

```bash
sha256sum -c full_development.tar.zst.sha256
mkdir restored-project
tar -I 'zstd -d --long=30' -xf full_development.tar.zst -C restored-project
cd restored-project
python3 handoff/verify_full_handoff.py --project project \
  --inventory handoff/file_inventory.json --rehydrate
```

압축본의 hardlink를 독립 파일로 복원하고 파일 해시와 실행 권한을 확인한다. 원래 실행 경로를 참조하는 도구는 새 경로를 설정하거나 해당 입력을 준비한다. 원본 manifest를 일괄 치환해 출처 해시를 바꾸지는 않는다.

## Windows 웹 실행

웹만 실행할 때는 Git 저장소의 `web/`이면 충분하다. ROS, CUDA와 원본 DB는 필요하지 않다.

```powershell
cd web
npm ci
npm run build
npm test
python -X utf8 server.py --port 8767
```

## Ubuntu와 WSL2

기존 Ubuntu 또는 WSL2 Ubuntu 22.04에 ROS 2 Humble과 처리 의존성을 설치한다. Conda 환경 자체를 운영체제 사이에 복사하기보다 환경 목록을 기준으로 새로 구성한다.

```bash
bash scripts/view_current_map.sh --check-only
bash scripts/view_current_map.sh

ROS_DOMAIN_ID=83 bash scripts/with_ros.sh /usr/bin/python3 -s src/run_slam_replay.py \
  --output outputs/student_union_slam_replay_restored --batch 40 --max-evaluations 180 --hold 600
```

RViz와 rqt 실행은 [재생 실험 문서](experiments/2026-10-19-cartographic-replay-windows.md)를 참고한다. 기존 출력과 다른 이름을 사용하고 같은 ROS 도메인에서 과거 재생을 동시에 실행하지 않는다.

## 표시용 뷰어 이관본

`student_union_windows_handoff_20261019`에는 상대경로 GLB, 지도, 영상과 별도 native viewer가 있다. 압축을 모두 푼 다음 포함된 `requirements-viewer.txt`로 환경을 만든다. 현재 웹 클라이언트와는 별도의 오프라인 뷰어다.

## 공개 자료와 로컬 자료

이관본의 센서 DB, 개인 대화와 환경별 자격증명은 로컬 자료다. 공개 저장소에는 소스, 설정, 실험 문서와 공개 범위를 확인한 시각자료를 둔다. `.gitignore`와 배포 제외 규칙은 이미 커밋된 과거 파일을 삭제하지 않으므로, 공개 전에는 Git 이력도 따로 확인한다.
