"""같은 1,000조건에서 2차 검출기의 payload 정보 유무 효과를 비교한다."""
from __future__ import annotations

import argparse
import csv
import json
from concurrent.futures import ProcessPoolExecutor

import numpy as np

import hover_servo
import robust_detector_v2
from sim.runner import run_episode
from unknown_payload_audit import OUT, make_case


def evaluate_known(case: dict) -> dict:
    common = dict(
        box_mass=case["mass_kg"], box_com=case["com"],
        plant_uncertainty=case["uncertainty"],
    )
    ref_result, ref_log = run_episode(
        hover_servo.Controller(), detector=robust_detector_v2.Detector(), **common
    )
    vector = case["force_N"] * np.asarray(case["direction"])
    result, _ = run_episode(
        hover_servo.Controller(), detector=robust_detector_v2.Detector(),
        collision=(case["collision_time_s"], vector), **common
    )
    delay = result["detect_delay_ms"]
    overlap = False
    if delay is not None and not result["alarm_before_collision"]:
        k = int(round((case["collision_time_s"] + delay / 1000.0) / 0.001))
        overlap = bool(ref_log["detect"][max(0, k - 20):k + 21].any())
    success = bool(delay is not None and not result["alarm_before_collision"] and not overlap)
    return {
        "case": case["case"], "known_success": success,
        "known_delay_ms": delay,
        "known_reference_false_alarms": ref_result["false_alarms"],
        "known_alarm_before": result["alarm_before_collision"],
        "known_reference_overlap": overlap,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=20261104)
    parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args()
    unknown_path = OUT / "unknown_payload_1000.csv"
    with unknown_path.open(encoding="utf-8") as file:
        unknown = {int(r["case"]): r for r in csv.DictReader(file)}
    cases = [make_case(i, args.seed) for i in range(args.cases)]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        known = list(pool.map(evaluate_known, cases, chunksize=2))

    rows = []
    for k in known:
        u = unknown[k["case"]]
        rows.append({
            "case": k["case"], "phase": u["phase"],
            "mass_kg": float(u["mass_kg"]), "force_N": float(u["force_N"]),
            **{x: y for x, y in k.items() if x != "case"},
            "unknown_success": u["distinguished"] == "True",
            "unknown_delay_ms": u["delay_ms"],
            "unknown_reference_false_alarms": int(u["reference_false_alarms"]),
        })
    with (OUT / "phase2_known_vs_unknown_1000.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys(), lineterminator="\n")
        writer.writeheader(); writer.writerows(rows)

    by_phase = {}
    for phase in ("startup_or_approach", "pickup_transition", "carry", "place_transition", "return"):
        group = [r for r in rows if r["phase"] == phase]
        by_phase[phase] = {
            "cases": len(group),
            "known_pct": 100 * sum(r["known_success"] for r in group) / len(group),
            "unknown_pct": 100 * sum(r["unknown_success"] for r in group) / len(group),
        }
    known_n = sum(r["known_success"] for r in rows)
    unknown_n = sum(r["unknown_success"] for r in rows)
    summary = {
        "cases": len(rows),
        "known_success": known_n, "known_pct": 100 * known_n / len(rows),
        "unknown_success": unknown_n, "unknown_pct": 100 * unknown_n / len(rows),
        "knowledge_gain_percentage_points": 100 * (known_n - unknown_n) / len(rows),
        "known_only_success": sum(r["known_success"] and not r["unknown_success"] for r in rows),
        "unknown_only_success": sum(not r["known_success"] and r["unknown_success"] for r in rows),
        "known_reference_false_alarm_cases": sum(r["known_reference_false_alarms"] > 0 for r in rows),
        "unknown_reference_false_alarm_cases": sum(r["unknown_reference_false_alarms"] > 0 for r in rows),
        "by_phase": by_phase,
    }
    (OUT / "phase2_known_vs_unknown_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
