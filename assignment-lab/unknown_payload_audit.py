"""질량·COM 비공개 조건의 1,000회 강건성 감사.

plant만 실제 payload를 안다. Controller/Detector에는 UNKNOWN_PAYLOAD 플래그로
실제 box_mass/box_com 전달을 차단한다.
"""
from __future__ import annotations

import argparse
import csv
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

import unknown_payload_detector
import unknown_payload_servo
from paired_robustness_comparison import FORCE_BINS, PHASES, phase_for_index
from sim.runner import run_episode


OUT = Path(__file__).resolve().parent / "results" / "unknown_payload"


def make_case(index: int, seed: int) -> dict:
    rng = np.random.default_rng(seed + 104729 * index)
    phase, time_low, time_high = phase_for_index(index)
    force_bin, force_low, force_high = FORCE_BINS[index % len(FORCE_BINS)]
    mass = float(rng.uniform(0.5, 10.0))
    size_scale = rng.uniform([0.60, 0.60, 0.45], [1.35, 1.35, 1.55])
    half = np.array([0.30, 0.20, 0.18]) * size_scale / 2
    com = tuple(rng.uniform(-0.45 * half, 0.45 * half))
    friction = [
        float(np.exp(rng.uniform(np.log(0.15), np.log(1.8)))),
        float(np.exp(rng.uniform(np.log(0.001), np.log(0.015)))),
        float(np.exp(rng.uniform(np.log(0.00002), np.log(0.0003)))),
    ]
    uncertainty = {
        "box_size_scale": size_scale.tolist(),
        "box_inertia_scale": float(np.exp(rng.uniform(np.log(0.45), np.log(2.2)))),
        "table_time_scale": float(np.exp(rng.uniform(np.log(0.35), np.log(2.5)))),
        "table_damping_scale": float(rng.uniform(0.65, 1.8)),
        "contact_friction": friction,
        "suction_time_scale": float(np.exp(rng.uniform(np.log(0.5), np.log(3.0)))),
        "place_height_offset_m": float(rng.uniform(-0.002, 0.002)),
        "noise_scale": float(rng.uniform(1.0, 3.0)),
        "offset_scale": float(rng.uniform(0.5, 1.8)),
        "drift_scale": float(rng.uniform(1.0, 4.0)),
        "friction_scale": float(rng.uniform(0.75, 1.35)),
        "seed": int(rng.integers(1, 1_000_000)),
    }
    direction = rng.normal(size=3)
    direction /= np.linalg.norm(direction)
    force = float(rng.uniform(force_low, force_high))
    collision_time = float(rng.uniform(time_low, time_high))
    return {
        "case": index, "phase": phase, "force_bin": force_bin,
        "mass_kg": mass, "com": com, "uncertainty": uncertainty,
        "collision_time_s": collision_time, "force_N": force,
        "direction": direction.tolist(),
    }


def run_case(case: dict) -> dict:
    common = dict(
        box_mass=case["mass_kg"], box_com=case["com"],
        plant_uncertainty=case["uncertainty"],
    )
    reference_ctrl = unknown_payload_servo.Controller()
    reference_result, reference_log = run_episode(
        reference_ctrl, detector=unknown_payload_detector.Detector(), **common
    )
    collision_ctrl = unknown_payload_servo.Controller()
    vector = case["force_N"] * np.asarray(case["direction"])
    result, _ = run_episode(
        collision_ctrl, detector=unknown_payload_detector.Detector(),
        collision=(case["collision_time_s"], vector), **common
    )
    delay = result["detect_delay_ms"]
    reference_overlap = False
    if delay is not None and not result["alarm_before_collision"]:
        k = int(round((case["collision_time_s"] + delay / 1000.0) / 0.001))
        reference_overlap = bool(
            reference_log["detect"][max(0, k - 20):k + 21].any()
        )
    distinguished = bool(
        delay is not None and not result["alarm_before_collision"]
        and not reference_overlap
    )
    if result["alarm_before_collision"]:
        failure = "alarm_before"
    elif delay is None:
        failure = "missed"
    elif reference_overlap:
        failure = "reference_overlap"
    else:
        failure = ""
    com = np.asarray(case["com"])
    row = {
        "case": case["case"], "phase": case["phase"],
        "force_bin": case["force_bin"], "mass_kg": case["mass_kg"],
        "com_x_m": com[0], "com_y_m": com[1], "com_z_m": com[2],
        "collision_time_s": case["collision_time_s"], "force_N": case["force_N"],
        "dir_x": case["direction"][0], "dir_y": case["direction"][1],
        "dir_z": case["direction"][2], "distinguished": distinguished,
        "delay_ms": delay, "failure_type": failure,
        "reference_false_alarms": reference_result["false_alarms"],
        "collision_false_alarms": result["false_alarms"],
        "estimated_mass_kg": reference_ctrl.estimated_mass,
        "mass_abs_error_kg": abs(reference_ctrl.estimated_mass - case["mass_kg"]),
        "estimated_com_x_m": reference_ctrl.estimated_com[0],
        "estimated_com_y_m": reference_ctrl.estimated_com[1],
        "rms_mm": result["rms_mm"], "max_mm": result["max_mm"],
        "diverged": bool(result["diverged"] or reference_result["diverged"]),
    }
    return row


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=20261104)
    parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args()
    cases = [make_case(i, args.seed) for i in range(args.cases)]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        rows = list(pool.map(run_case, cases, chunksize=2))
    OUT.mkdir(parents=True, exist_ok=True)
    with (OUT / "unknown_payload_1000.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys(), lineterminator="\n")
        writer.writeheader(); writer.writerows(rows)

    phase_summary = {}
    for name, *_ in PHASES:
        group = [r for r in rows if r["phase"] == name]
        phase_summary[name] = {
            "cases": len(group),
            "detected": sum(r["distinguished"] for r in group),
            "rate_pct": (
                100 * sum(r["distinguished"] for r in group) / len(group)
                if group else None
            ),
        }
    delays = [r["delay_ms"] for r in rows if r["distinguished"]]
    summary = {
        "assumption": "controller and detector receive neither true mass nor true COM",
        "cases": len(rows), "distinguished": sum(r["distinguished"] for r in rows),
        "success_rate_pct": 100 * sum(r["distinguished"] for r in rows) / len(rows),
        "reference_false_alarm_cases": sum(r["reference_false_alarms"] > 0 for r in rows),
        "missed": sum(r["failure_type"] == "missed" for r in rows),
        "alarm_before": sum(r["failure_type"] == "alarm_before" for r in rows),
        "reference_overlap": sum(r["failure_type"] == "reference_overlap" for r in rows),
        "median_delay_ms": float(np.median(delays)) if delays else None,
        "mass_mae_kg": float(np.mean([r["mass_abs_error_kg"] for r in rows])),
        "rms_tracking_median_mm": float(np.median([r["rms_mm"] for r in rows])),
        "max_tracking_p95_mm": float(np.percentile([r["max_mm"] for r in rows], 95)),
        "diverged": sum(r["diverged"] for r in rows),
        "by_phase": phase_summary,
    }
    (OUT / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
