"""같은 1,000조건에서 미지-payload 1차 기준안과 2차를 비교한다.

두 안 모두 실제 mass/COM을 받지 않고 동일한 JTS 추정 제어기를 쓴다. 1차는
미지 payload에 필요한 이동 기준선만 공통 적용한 비교 기준안이다.
"""
from __future__ import annotations

import argparse
import csv
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

import unknown_payload_phase1_detector
import unknown_payload_servo
from sim.runner import run_episode
from unknown_payload_audit import OUT, make_case


UNKNOWN_CSV = OUT / "unknown_payload_1000.csv"


def evaluate_phase1(case: dict) -> dict:
    common = dict(
        box_mass=case["mass_kg"], box_com=case["com"],
        plant_uncertainty=case["uncertainty"],
    )
    reference_result, reference_log = run_episode(
        unknown_payload_servo.Controller(),
        detector=unknown_payload_phase1_detector.Detector(), **common
    )
    vector = case["force_N"] * np.asarray(case["direction"])
    result, _ = run_episode(
        unknown_payload_servo.Controller(),
        detector=unknown_payload_phase1_detector.Detector(),
        collision=(case["collision_time_s"], vector), **common
    )
    delay = result["detect_delay_ms"]
    overlap = False
    if delay is not None and not result["alarm_before_collision"]:
        k = int(round((case["collision_time_s"] + delay / 1000.0) / 0.001))
        overlap = bool(reference_log["detect"][max(0, k - 20):k + 21].any())
    distinguished = bool(
        delay is not None and not result["alarm_before_collision"] and not overlap
    )
    if result["alarm_before_collision"]:
        failure = "alarm_before"
    elif delay is None:
        failure = "missed"
    elif overlap:
        failure = "reference_overlap"
    else:
        failure = ""
    return {
        "case": case["case"], "p1_distinguished": distinguished,
        "p1_delay_ms": delay, "p1_failure_type": failure,
        "p1_reference_false_alarms": reference_result["false_alarms"],
        "p1_rms_mm": result["rms_mm"], "p1_max_mm": result["max_mm"],
        "p1_diverged": bool(result["diverged"] or reference_result["diverged"]),
    }


def as_bool(value: str) -> bool:
    return value == "True"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=20261104)
    parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args()

    with UNKNOWN_CSV.open(encoding="utf-8") as file:
        unknown_rows = {int(row["case"]): row for row in csv.DictReader(file)}
    cases = [make_case(i, args.seed) for i in range(args.cases)]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        phase1_rows = list(pool.map(evaluate_phase1, cases, chunksize=2))

    rows = []
    for p1 in phase1_rows:
        u = unknown_rows[p1["case"]]
        rows.append({
            "case": p1["case"], "phase": u["phase"],
            "mass_kg": float(u["mass_kg"]), "force_N": float(u["force_N"]),
            "collision_time_s": float(u["collision_time_s"]),
            **{k: v for k, v in p1.items() if k != "case"},
            "p2_unknown_distinguished": as_bool(u["distinguished"]),
            "p2_unknown_delay_ms": u["delay_ms"],
            "p2_unknown_failure_type": u["failure_type"],
            "p2_unknown_reference_false_alarms": int(u["reference_false_alarms"]),
            "p2_unknown_rms_mm": float(u["rms_mm"]),
            "p2_unknown_max_mm": float(u["max_mm"]),
            "p2_unknown_diverged": as_bool(u["diverged"]),
        })

    output_csv = OUT / "phase1_vs_unknown_phase2_1000.csv"
    with output_csv.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=rows[0].keys(), lineterminator="\n")
        writer.writeheader(); writer.writerows(rows)

    p1_success = sum(r["p1_distinguished"] for r in rows)
    p2_success = sum(r["p2_unknown_distinguished"] for r in rows)
    p1_only = sum(r["p1_distinguished"] and not r["p2_unknown_distinguished"] for r in rows)
    p2_only = sum(not r["p1_distinguished"] and r["p2_unknown_distinguished"] for r in rows)
    phase_summary = {}
    for phase in ("startup_or_approach", "pickup_transition", "carry", "place_transition", "return"):
        group = [r for r in rows if r["phase"] == phase]
        phase_summary[phase] = {
            "cases": len(group),
            "phase1_unknown_payload_pct": 100 * sum(r["p1_distinguished"] for r in group) / len(group),
            "phase2_unknown_payload_pct": 100 * sum(r["p2_unknown_distinguished"] for r in group) / len(group),
        }
    summary = {
        "comparison_warning": "both hide true mass/COM; phase1 is an adapted unknown-payload baseline",
        "cases": len(rows),
        "phase1_unknown_payload_success": p1_success,
        "phase1_unknown_payload_pct": 100 * p1_success / len(rows),
        "phase2_unknown_payload_success": p2_success,
        "phase2_unknown_payload_pct": 100 * p2_success / len(rows),
        "difference_percentage_points": 100 * (p2_success - p1_success) / len(rows),
        "phase1_only_success": p1_only,
        "phase2_only_success": p2_only,
        "phase1_reference_false_alarm_cases": sum(r["p1_reference_false_alarms"] > 0 for r in rows),
        "phase2_unknown_reference_false_alarm_cases": sum(r["p2_unknown_reference_false_alarms"] > 0 for r in rows),
        "by_phase": phase_summary,
    }
    (OUT / "phase1_vs_unknown_phase2_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
