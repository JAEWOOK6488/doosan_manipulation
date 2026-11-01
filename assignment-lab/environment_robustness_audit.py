"""Phase 2: 물체 형상·접촉 환경 변화에 대한 재현 가능한 강건성 감사."""
from __future__ import annotations

import csv
import argparse
import importlib
import json
from pathlib import Path

import numpy as np

import hover_servo
import hybrid_detector
from sim.runner import run_episode


OUT = Path(__file__).resolve().parent / "results" / "phase2"
OUT.mkdir(parents=True, exist_ok=True)

# 질량·COM은 과제 가정대로 알려 주되, 명목 모델이 직접 표현하지 않는 형상,
# 관성, 접촉재료, 작업대 오차, 흡착 강성, 센서 비이상성을 바꾼다.
SCENARIOS = [
    ("reference", 8.0, (0.02, -0.01, 0.03), {}),
    ("small_dense", 8.0, (0.01, -0.01, 0.01), {"box_size_scale": [0.60, 0.65, 0.65]}),
    ("large_light", 3.0, (-0.04, 0.03, 0.02), {"box_size_scale": [1.25, 1.25, 1.15]}),
    ("flat_wide", 5.0, (0.05, -0.04, -0.02), {"box_size_scale": [1.35, 1.25, 0.45]}),
    ("tall_narrow", 6.0, (-0.03, 0.02, 0.07), {"box_size_scale": [0.65, 0.70, 1.55]}),
    ("low_inertia", 7.0, (0.03, 0.02, 0.03), {"box_inertia_scale": 0.45}),
    ("high_inertia", 7.0, (-0.04, -0.03, 0.04), {"box_inertia_scale": 2.2}),
    ("soft_table", 8.0, (0.02, -0.01, 0.03), {"table_time_scale": 2.5, "table_damping_scale": 0.65}),
    ("hard_table", 8.0, (0.02, -0.01, 0.03), {"table_time_scale": 0.35, "table_damping_scale": 1.6}),
    ("low_friction", 4.0, (0.04, 0.03, 0.02), {"contact_friction": [0.15, 0.001, 0.00002]}),
    ("high_friction", 4.0, (-0.04, -0.03, 0.02), {"contact_friction": [1.8, 0.015, 0.0003]}),
    ("place_low_2mm", 8.0, (0.02, -0.01, 0.03), {"place_height_offset_m": -0.002}),
    ("place_high_2mm", 8.0, (0.02, -0.01, 0.03), {"place_height_offset_m": 0.002}),
    ("soft_suction", 6.0, (0.04, 0.03, 0.04), {"suction_time_scale": 3.0}),
    ("sensor_environment", 8.0, (-0.04, 0.04, 0.05), {
        "noise_scale": 3.0, "offset_scale": 1.8, "drift_scale": 4.0,
        "friction_scale": 1.35, "seed": 31,
    }),
    ("combined_soft", 5.5, (0.05, -0.04, 0.06), {
        "box_size_scale": [1.25, 0.70, 1.25], "box_inertia_scale": 1.8,
        "table_time_scale": 2.2, "table_damping_scale": 0.7,
        "contact_friction": [0.25, 0.002, 0.00003], "suction_time_scale": 2.5,
        "place_height_offset_m": -0.0015, "noise_scale": 2.0,
        "offset_scale": 1.5, "drift_scale": 2.5, "friction_scale": 1.25, "seed": 37,
    }),
    ("combined_hard", 7.5, (-0.05, 0.04, 0.05), {
        "box_size_scale": [0.70, 1.30, 0.75], "box_inertia_scale": 0.6,
        "table_time_scale": 0.4, "table_damping_scale": 1.8,
        "contact_friction": [1.7, 0.012, 0.00025], "suction_time_scale": 0.6,
        "place_height_offset_m": 0.0015, "noise_scale": 2.0,
        "offset_scale": 0.5, "drift_scale": 3.0, "friction_scale": 0.75, "seed": 41,
    }),
]

COLLISIONS = [
    ("pickup_-X", 1.42, np.array([-15.0, 0.0, 0.0])),
    ("carry_+Y", 3.00, np.array([0.0, 15.0, 0.0])),
    ("place_+Z", 3.86, np.array([0.0, 0.0, 15.0])),
]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", default="hover_servo")
    parser.add_argument("--detector", default="hybrid_detector")
    parser.add_argument("--tag", default="environment_robustness")
    args = parser.parse_args()
    controller_cls = importlib.import_module(args.controller).Controller
    detector_cls = importlib.import_module(args.detector).Detector
    rows = []
    for name, mass, com, uncertainty in SCENARIOS:
        reference_result, reference = run_episode(
            controller_cls(), box_mass=mass, box_com=com,
            detector=detector_cls(), plant_uncertainty=uncertainty,
        )
        rows.append({
            "scenario": name, "mass_kg": mass, "case": "no_collision",
            "detected": "", "distinguishable": "", "delay_ms": "",
            "alarm_before": False, "reference_alarm": False,
            "false_alarms": reference_result["false_alarms"],
            "rms_mm": reference_result["rms_mm"], "max_mm": reference_result["max_mm"],
            "diverged": reference_result["diverged"],
        })
        for case, time_s, force in COLLISIONS:
            result, _ = run_episode(
                controller_cls(), box_mass=mass, box_com=com,
                detector=detector_cls(), collision=(time_s, force),
                plant_uncertainty=uncertainty,
            )
            delay = result["detect_delay_ms"]
            reference_alarm = False
            if delay is not None and not result["alarm_before_collision"]:
                k = int(round((time_s + delay / 1000.0) / 0.001))
                reference_alarm = bool(reference["detect"][max(0, k - 20):k + 21].any())
            distinguishable = bool(
                delay is not None and not result["alarm_before_collision"] and not reference_alarm
            )
            rows.append({
                "scenario": name, "mass_kg": mass, "case": case,
                "detected": delay is not None, "distinguishable": distinguishable,
                "delay_ms": delay, "alarm_before": result["alarm_before_collision"],
                "reference_alarm": reference_alarm, "false_alarms": result["false_alarms"],
                "rms_mm": result["rms_mm"], "max_mm": result["max_mm"],
                "diverged": result["diverged"],
            })
        print(f"  {name} complete", flush=True)

    csv_path = OUT / f"{args.tag}.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=rows[0].keys())
        writer.writeheader(); writer.writerows(rows)

    nominal = [r for r in rows if r["case"] == "no_collision"]
    collisions = [r for r in rows if r["case"] != "no_collision"]
    failures = [r for r in collisions if not r["distinguishable"]]
    summary = {
        "scenarios": len(SCENARIOS), "cases": len(rows),
        "controller": args.controller, "detector": args.detector,
        "no_collision_false_alarm_scenarios": sum(r["false_alarms"] > 0 for r in nominal),
        "no_collision_diverged_scenarios": sum(r["diverged"] for r in nominal),
        "collision_cases": len(collisions),
        "collision_distinguished": sum(r["distinguishable"] for r in collisions),
        "collision_missed": len(failures),
        "collision_alarm_before": sum(r["alarm_before"] for r in collisions),
        "collision_reference_alarm": sum(r["reference_alarm"] for r in collisions),
        "max_delay_ms": max((r["delay_ms"] for r in collisions if r["delay_ms"] is not None), default=None),
        "max_rms_mm": max(r["rms_mm"] for r in rows),
        "max_error_mm": max(r["max_mm"] for r in rows),
        "failures": failures,
    }
    json_path = OUT / f"{args.tag}_summary.json"
    json_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
