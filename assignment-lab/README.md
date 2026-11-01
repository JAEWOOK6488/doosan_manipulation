# Assignment Lab

Week9 MuJoCo 과제의 작업용 사본과 개선안이다. 다운로드 폴더의 원본 자료는
수정하지 않았다. 이 사본의 시뮬레이터에는 결과 기록과 불확실성 감사를 위한
선택적 계측 훅을 추가했으며, 기본 실행의 동역학 조건은 원본과 같다.

실제 제출용 파일은 상위 폴더의 `IC-PBL_submission.zip`이다. 원본 키트 위에
풀어 쓰는 드롭인 패키지이며 `sim/`, `model/`을 포함하지 않는다.

## 최종 실행

```bash
.venv/bin/python run.py --ctrl hover_servo --detector hybrid_detector --box 8 --viewer
.venv/bin/python evaluate.py --ctrl hover_servo --detector hybrid_detector --out final_eval.txt
.venv/bin/python verify_solution.py
.venv/bin/python build_submission.py
```

## 전체 감사

```bash
.venv/bin/python robustness_sweep.py
.venv/bin/python timing_audit.py
.venv/bin/python startup_audit.py
.venv/bin/python sensitivity_audit.py
.venv/bin/python compute_benchmark.py
.venv/bin/python uncertainty_audit.py
.venv/bin/python monte_carlo_audit.py
.venv/bin/python make_figures.py
.venv/bin/python final_audit.py
```

한 명령으로 실행하려면 빠른 점검은 `.venv/bin/python reproduce.py --quick`,
전체 결과 재생성은 `.venv/bin/python reproduce.py --full`을 사용한다.

충돌 성공 판정은 같은 조건의 무충돌 실행과 비교한다. 검출 시각 주변 ±20 ms에
기준 경보가 있으면 `구분 불가`로 세며, 요약 JSON의
`collision_reference_alarm_cases` 또는 `reference_alarm_cases`에서 확인할 수 있다.

- `scheduled_servo.py`: 연속 하중 스케줄
- `hover_servo.py`: 최종 제어기(연속 하중 + 1.25 mm 놓기 호버)
- `hybrid_detector.py`: 최종 상태별 혼합 충돌 감지기
- `robustness_sweep.py`: 공개 조건 밖 88조건 시험
- `timing_audit.py`: 하중 ON/OFF 주변 15 N 전 방향 감지 경계 감사
- `startup_audit.py`: 관측기 기동·교정 0.1–1.3 s의 15 N 전 방향 감사
- `sensitivity_audit.py`: 전체 사이클 12시각의 10/15 N 전 방향 감도 지도
- `dense_sensitivity_audit.py`: 전체 사이클 0.1 s 간격의 15 N 전 방향 탐색 감사
- `compute_benchmark.py`: 제어기·감지기 콜백 median/p99/max 실행시간 측정
- `uncertainty_audit.py`: 센서 잡음·오프셋·드리프트·마찰 불일치 감사
- `monte_carlo_audit.py`: 연속 질량·COM·임의 3D 충돌 60조건 감사
- `motor_gap_study.py`: 모터 전류 기반 토크와 JTS 단순 차이의 한계 실험
- `height_robustness_study.py`: 호버 높이와 작업대 높이 오차 절충 실험
- `dual_encoder_detector.py`: 출력측 엔코더가 제공될 때의 분석 전용 감지기
- `fused_encoder_detector.py`: JTS와 두 엔코더 감지기의 분석 전용 병렬 결합
- `make_figures.py`: 비교 그림 재생성
- `verify_solution.py`: 제어·오경보·전이 구간 15 N 전 방향 회귀 검증
- `final_audit.py`: 제출 ZIP·체크섬·결과 요약·보고서 링크 일관성 검사
- `build_submission.py`: 고정 메타데이터로 재현 가능한 제출 ZIP·SHA-256 생성
- `reproduce.py`: 빠른 제출 점검 또는 전체 감사 파이프라인 실행
- `REPORT.md`: 원리, 결과, 한계
- `ONE_PAGE_SUMMARY.md`: 발표 직전 확인용 목표·방법·결과·한계 1쪽 요약
- `COMPLIANCE_MATRIX.md`: 실습 가이드 요구사항과 검증 증거 대응표
- `PRESENTATION_OUTLINE.md`: 발표용 9개 슬라이드 흐름
- `DEMO_SCRIPT.md`: 대시보드 성공·실패 사례를 보여 주는 5분 발표 동선
- `PRESENTATION_QA.md`: 과제 목표·센서·모델·제어주기·한계 예상 질의응답
- `SENSOR_CHANNELS.md`: 공식 콜백, 센서 단위·지연·잡음, 분석 채널 데이터 사전
- `PAPER_NOTES.md`: 9편의 문헌 원리와 구현·검증 결과를 연결한 발표용 대응표
- `FINAL_STATUS.md`: 제출 해시, 확정 검증 결과, 센서 해석을 모은 최종 상태표
- `final_eval_unmodified_sim.txt`: 다운로드 폴더의 수정되지 않은 원본
  `sim/`·`model/`에서 재실행한 공식 평가 결과
- `original_sim_checksums.txt`: 원본 재평가에 사용한 `sim/*.py` SHA-256 기록
- `original_model_checksums.txt`: 원본 재평가에 사용한 `model/` 전체 SHA-256 기록

`adaptive_servo.py`, `load_aware_detector.py`, `gravity_comp_detector.py`,
`projected_detector.py`는 실패와 중간 아이디어를 재현하기 위해 남긴 실험안이다.
