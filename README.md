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

> **가정 주의:** 기존 2차의 94.1% 결과는 payload 질량·COM을 제어기와
> 검출기에 설정한 조건이다. 두 값을 완전히 숨긴 재검증에서는 1,000회 성공률이
> 51.5%였다. 같은 미지 조건의 1차 기준안 34.6%보다는 16.9%p 개선됐지만,
> 현재 센서 구성만으로 미지 payload 강건성을 확보했다고 주장할 수 없다.
> 같은 1,000조건에서 2차 시스템끼리 직접 비교하면 질량·COM을 아는 경우는
> 89.6%, 모르는 경우는 51.5%로 **38.1%p 차이**가 났다.
> 자세한 결과는 `assignment-lab/UNKNOWN_PAYLOAD_REPORT.md`에 있다.

이후 미지 payload를 처음부터 목표로 관측기를 다시 설계했다. 정상 실행 300개로
시간가변 잔차 범위를 학습하고 독립 1,000조건에서 기존 미지 2차 52.8% 대비
3차 **83.5%**를 얻었다. 상세한 데이터 분리와 한계는
`assignment-lab/UNKNOWN_PAYLOAD_V3_REPORT.md`에 있다.

기본 시뮬레이션과 2차 검출기는 다음과 같이 실행한다.

```bash
.venv/bin/python run.py --ctrl hover_servo --detector robust_detector_v2 --box 8 --viewer
.venv/bin/python evaluate.py --ctrl hover_servo --detector robust_detector_v2 \
  --out results/phase2/final_eval_v2.txt

# 질량·COM 비공개 수정안과 1,000회 재검증
.venv/bin/python run.py --ctrl unknown_payload_servo \
  --detector unknown_payload_detector --box 8 --viewer
.venv/bin/python unknown_payload_audit.py --cases 1000 --workers 8
.venv/bin/python compare_phase1_unknown_phase2.py --cases 1000 --workers 8
.venv/bin/python compare_known_unknown_phase2.py --cases 1000 --workers 8
.venv/bin/python plot_known_unknown_phase2.py

# 미지 payload 3차: 정상 프로필 학습 후 독립 holdout 비교
.venv/bin/python train_unknown_normal_profile.py --cases 300 --seed 73129 --workers 8
.venv/bin/python unknown_payload_holdout_audit.py --cases 1000 --seed 20261201 --workers 8
.venv/bin/python unknown_payload_v3_audit.py --cases 1000 --seed 20261201 --workers 8
.venv/bin/python analyze_unknown_v3.py
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
5. `assignment-lab/EXTENDED_ROBUSTNESS_REPORT.md`
6. `assignment-lab/PAIRED_1000_REPORT.md`
7. `assignment-lab/UNKNOWN_PAYLOAD_REPORT.md`
8. `assignment-lab/UNKNOWN_PAYLOAD_V3_REPORT.md`
9. `assignment-lab/COMPLIANCE_MATRIX.md`
10. `assignment-lab/PRESENTATION_OUTLINE.md`
11. `assignment-lab/PRESENTATION_QA.md`
12. `assignment-lab/PAPER_NOTES.md`
13. `assignment-lab/SENSOR_CHANNELS.md`
14. `assignment-lab/DEMO_SCRIPT.md`
15. `assignment-lab/final_eval_unmodified_sim.txt`

`assignment-lab/final_audit.py`는 ZIP 내용·체크섬·평가 요약·보고서 링크를
검사하며 결과와 환경 버전을 `results/final_audit_manifest.json`에 기록한다.
ZIP은 `assignment-lab/build_submission.py`로 재현 가능하게 생성한다.

가상환경 `.venv`는 각 컴퓨터에서 새로 만들며 Git에 저장하지 않는다. 1차 상태는
`snapshots/phase1_2026-11-01/`, 2차 수치 결과는
`assignment-lab/results/phase2/`에 있다.
