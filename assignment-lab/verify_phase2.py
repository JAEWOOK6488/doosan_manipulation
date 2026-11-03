"""2차 결과 파일과 1차 스냅샷 보존 상태를 빠르게 검증한다."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent
PHASE2 = ROOT / "results" / "phase2"
SNAPSHOT = ROOT.parent / "snapshots" / "phase1_2026-11-01"


def load(name: str) -> dict:
    return json.loads((PHASE2 / name).read_text(encoding="utf-8"))


def main() -> None:
    fixed = load("environment_robustness_v2_summary.json")
    random = load("environment_monte_carlo_v2_summary.json")
    cycle = load("sensitivity_audit_v2_summary.json")
    extended = load("environment_monte_carlo_extended_v2_summary.json")
    paired = json.loads(
        (PHASE2 / "paired_1000" / "paired_comparison_summary.json").read_text(encoding="utf-8")
    )

    checks = {
        "고정 환경 무충돌 오경보 0/17": fixed["no_collision_false_alarm_scenarios"] == 0,
        "고정 환경 15 N 구분 51/51": fixed["collision_distinguished"] == 51,
        "고정 환경 발산 0": fixed["no_collision_diverged_scenarios"] == 0,
        "무작위 환경 구분 54/60": random["distinguished"] == 54,
        "무작위 환경 기준 오경보 0": random["reference_false_alarm_cases"] == 0,
        "무작위 환경 선행경보 0": random["alarm_before_cases"] == 0,
        "무작위 환경 발산 0": random["diverged_cases"] == 0,
        "전 사이클 15 N 70/72": cycle["groups"]["15N"]["detected"] == 70,
        "전 사이클 10 N 53/72": cycle["groups"]["10N"]["detected"] == 53,
        "확장 환경 500조건 저장": extended["cases"] == 500,
        "확장 환경 구분 475/500": extended["distinguished"] == 475,
        "확장 환경 기준 오경보 11/500": extended["reference_false_alarm_cases"] == 11,
        "확장 환경 발산 0": extended["diverged_cases"] == 0,
        "쌍대 비교 1,000조건/4,000실행": paired["cases"] == 1000 and paired["episodes"] == 4000,
        "1차 쌍대 구분 773/1,000": paired["v1"]["distinguished"] == 773,
        "2차 쌍대 구분 941/1,000": paired["v2"]["distinguished"] == 941,
        "2차 무충돌 오경보 24/1,000": paired["v2"]["reference_false_alarm_cases"] == 24,
        "쌍대 비교 발산 0": paired["v1"]["diverged_cases"] == 0 and paired["v2"]["diverged_cases"] == 0,
        "표준 평가 저장": (PHASE2 / "final_eval_v2.txt").is_file(),
        "2차 보고서 저장": (ROOT / "PHASE2_REPORT.md").is_file(),
    }

    archive = SNAPSHOT / "phase1_assignment_lab.tar.gz"
    expected = (SNAPSHOT / "SHA256SUMS").read_text(encoding="utf-8").split()[0]
    actual = hashlib.sha256(archive.read_bytes()).hexdigest()
    checks["1차 스냅샷 SHA-256 유지"] = actual == expected

    for label, passed in checks.items():
        print(f"[{'PASS' if passed else 'FAIL'}] {label}")
    failed = [label for label, passed in checks.items() if not passed]
    if failed:
        raise SystemExit(f"검증 실패 {len(failed)}건: {', '.join(failed)}")
    print(f"\n2차 검증 {len(checks)}/{len(checks)} 통과")


if __name__ == "__main__":
    main()
