# Advanced Manipulation IC-PBL Workspace

Doosan M1013의 물체 이송 충격과 충돌 오경보를 줄이는 MuJoCo 기반 제어·감지
프로젝트다. 1차 결과와 물체 형상·관성·접촉 환경까지 확장한 2차 강건성 결과를
함께 보존한다.

## Quick Start

다른 컴퓨터에서 저장소를 복제하고 가상환경을 만든 뒤 2차 결과를 검증한다.

```bash
git clone https://github.com/JAEWOOK6488/doosan_manipulation.git
cd doosan_manipulation/assignment-lab
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python verify_phase2.py
```

기본 시뮬레이션과 2차 검출기는 다음과 같이 실행한다.

```bash
.venv/bin/python run.py --ctrl hover_servo --detector robust_detector_v2 --box 8 --viewer
.venv/bin/python evaluate.py --ctrl hover_servo --detector robust_detector_v2 \
  --out results/phase2/final_eval_v2.txt
```

## 제출

- `IC-PBL_submission.zip`: 원본 `icpbl_sim`에 풀어 쓰는 최종 제출 패키지
- `IC-PBL_submission.sha256`: 압축 파일 무결성 확인값

압축 파일에는 제어기·충돌 감지기와 보조 모듈만 있으며, 수정 금지 대상인
`sim/`과 `model/`은 들어 있지 않다.

## 개발·검증

- `assignment-lab/`: 알고리즘, 공식 평가, 강건성 감사, 보고서, 발표 개요
- `mujoco-dashboard/`: http://127.0.0.1:8765 로 실행 중인 로컬 분석 대시보드
- `submission/`: 제출 패키지의 기본 파일명 진입점과 사용 안내

가장 먼저 볼 문서는 다음과 같다.

1. `assignment-lab/REPORT.md`
2. `assignment-lab/ONE_PAGE_SUMMARY.md`
3. `assignment-lab/FINAL_STATUS.md`
4. `assignment-lab/PHASE2_REPORT.md`
5. `assignment-lab/COMPLIANCE_MATRIX.md`
6. `assignment-lab/PRESENTATION_OUTLINE.md`
7. `assignment-lab/PRESENTATION_QA.md`
8. `assignment-lab/PAPER_NOTES.md`
9. `assignment-lab/SENSOR_CHANNELS.md`
10. `assignment-lab/DEMO_SCRIPT.md`
11. `assignment-lab/final_eval_unmodified_sim.txt`

`assignment-lab/final_audit.py`는 ZIP 내용·체크섬·평가 요약·보고서 링크를
검사하며 결과와 환경 버전을 `results/final_audit_manifest.json`에 기록한다.
ZIP은 `assignment-lab/build_submission.py`로 재현 가능하게 생성한다.

가상환경 `.venv`는 각 컴퓨터에서 새로 만들며 Git에 저장하지 않는다. 1차 상태는
`snapshots/phase1_2026-11-01/`, 2차 수치 결과는
`assignment-lab/results/phase2/`에 있다.
