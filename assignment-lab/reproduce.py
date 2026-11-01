"""최종 결과를 빠르게 점검하거나 전체 감사를 순서대로 재생성한다."""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


HERE = Path(__file__).resolve().parent


QUICK = [
    ["verify_solution.py"],
    ["compute_benchmark.py"],
    ["make_figures.py"],
    ["build_submission.py"],
    ["final_audit.py"],
]

FULL = [
    ["evaluate.py", "--ctrl", "hover_servo", "--detector", "hybrid_detector",
     "--out", "final_eval.txt"],
    ["verify_solution.py"],
    ["startup_audit.py"],
    ["sensitivity_audit.py"],
    ["dense_sensitivity_audit.py"],
    ["timing_audit.py"],
    ["robustness_sweep.py"],
    ["uncertainty_audit.py"],
    ["monte_carlo_audit.py"],
    ["motor_gap_study.py"],
    ["height_robustness_study.py"],
    ["timing_audit.py", "--detector", "dual_encoder_detector"],
    ["timing_audit.py", "--detector", "fused_encoder_detector"],
    ["robustness_sweep.py", "--detector", "fused_encoder_detector"],
    ["compute_benchmark.py"],
    ["make_figures.py"],
    ["build_submission.py"],
    ["final_audit.py"],
]


def main() -> None:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--quick", action="store_true", help="짧은 제출 회귀·패키지 점검")
    mode.add_argument("--full", action="store_true", help="모든 평가·강건성 결과 재생성")
    args = parser.parse_args()
    plan = QUICK if args.quick else FULL
    for index, command in enumerate(plan, 1):
        print(f"\n[{index}/{len(plan)}] {' '.join(command)}", flush=True)
        subprocess.run([sys.executable, *command], cwd=HERE, check=True)


if __name__ == "__main__":
    main()
