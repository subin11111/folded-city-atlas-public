# 2층 RGB-D BA 실행 스크립트

국소 BA의 수치 해법 비교와 그림 생성에 사용한 스크립트 사본이다. 원 실행본과 바이트가 같으며 일반 알고리즘과 테스트는 저장소의 `src/`, `tests/`에 있다.

[실험 방법과 결과](../../2026-10-17-floor2-revisit-ba.md) / [연구 보고서](../../../../README.md)

이 스크립트들은 원래 실행 위치를 기준으로 경로를 계산한다. **이 폴더에서 직접
실행하는 entrypoint가 아니다.** 입력 데이터와 이전 단계 산출물을 복원한 뒤 아래
원래 위치에 배치하여 사용한다. 기존 출력 폴더가 있으면 덮어쓰지 않고 실패한다.

- `run_dense_bundle.py`, `run_dense_bundle_previous.py`, `plot_bundle_numerics.py`,
  `export_integrated_clouds.py`: `outputs/student_union_20261017_floor2_revisit_v2/`
- `plot_numerical_progress.py`: 위 경로의 `convergence/`

두 `run_dense_bundle*`은 동일 관측·목적함수의 scalar/vectorized residual 및 Jacobian
검사, 실제 dense exact solver, singular values와 입력 해시 기록까지 포함한다.
배포 환경의 SciPy 버전은 실험 기록의 1.8.0과 다를 수 있으며, 사용하는
`scipy.optimize._numdiff`는 private API라 버전 호환을 확인해야 한다.

일반 continuation entrypoint는 `src/continue_floor2_revisit_ba.py`다. 이 파일의
직접 풀이 기본값(2-point dense 차분)과, 당시 별도 runner가 사용한 colored 3-point
차분을 구분한다. 당시 수치 결과는 이 동결 runner와 각 출력의 report/snapshot이 기준이다.
