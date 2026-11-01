"""Phase 2: 연속 물체·환경 분포에서의 쌍대 무충돌/충돌 Monte Carlo 감사."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

import hover_servo
import robust_detector_v2
from sim.runner import run_episode


OUT = Path(__file__).resolve().parent / "results" / "phase2"
OUT.mkdir(parents=True, exist_ok=True)


def sample_collision_time(rng: np.random.Generator, index: int) -> float:
    region = index % 4
    if region == 0:
        return float(rng.uniform(0.7, 5.4))
    if region == 1:
        return float(rng.uniform(1.32, 1.70))
    if region == 2:
        return float(rng.uniform(3.72, 4.12))
    return float(rng.uniform(2.0, 3.5))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", type=int, default=60)
    parser.add_argument("--seed", type=int, default=20261102)
    args = parser.parse_args()
    rng = np.random.default_rng(args.seed)
    rows = []

    for i in range(args.cases):
        mass = float(rng.uniform(0.5, 8.0))
        size_scale = rng.uniform([0.60, 0.60, 0.45], [1.35, 1.35, 1.55])
        half = np.array([0.30, 0.20, 0.18]) * size_scale / 2
        com = tuple(rng.uniform(-0.45 * half, 0.45 * half))
        uncertainty = {
            "box_size_scale": size_scale.tolist(),
            "box_inertia_scale": float(np.exp(rng.uniform(np.log(0.45), np.log(2.2)))),
            "table_time_scale": float(np.exp(rng.uniform(np.log(0.35), np.log(2.5)))),
            "table_damping_scale": float(rng.uniform(0.65, 1.8)),
            "contact_friction": [
                float(np.exp(rng.uniform(np.log(0.15), np.log(1.8)))),
                float(np.exp(rng.uniform(np.log(0.001), np.log(0.015)))),
                float(np.exp(rng.uniform(np.log(0.00002), np.log(0.0003)))),
            ],
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
        force = float(rng.uniform(15.0, 60.0))
        time_s = sample_collision_time(rng, i)
        collision = (time_s, force * direction)

        reference_result, reference = run_episode(
            hover_servo.Controller(), box_mass=mass, box_com=com,
            detector=robust_detector_v2.Detector(), plant_uncertainty=uncertainty,
        )
        result, _ = run_episode(
            hover_servo.Controller(), box_mass=mass, box_com=com,
            detector=robust_detector_v2.Detector(), collision=collision,
            plant_uncertainty=uncertainty,
        )
        delay = result["detect_delay_ms"]
        reference_alarm = False
        if delay is not None and not result["alarm_before_collision"]:
            k = int(round((time_s + delay / 1000.0) / 0.001))
            reference_alarm = bool(
                reference["detect"][max(0, k - 20):min(len(reference["detect"]), k + 21)].any()
            )
        distinguished = bool(
            delay is not None and not result["alarm_before_collision"] and not reference_alarm
        )
        rows.append({
            "case": i, "mass_kg": mass,
            "size_x_scale": size_scale[0], "size_y_scale": size_scale[1],
            "size_z_scale": size_scale[2],
            "com_x_m": com[0], "com_y_m": com[1], "com_z_m": com[2],
            "inertia_scale": uncertainty["box_inertia_scale"],
            "table_time_scale": uncertainty["table_time_scale"],
            "table_damping_scale": uncertainty["table_damping_scale"],
            "friction_slide": uncertainty["contact_friction"][0],
            "suction_time_scale": uncertainty["suction_time_scale"],
            "place_height_mm": 1000 * uncertainty["place_height_offset_m"],
            "noise_scale": uncertainty["noise_scale"],
            "offset_scale": uncertainty["offset_scale"],
            "drift_scale": uncertainty["drift_scale"],
            "robot_friction_scale": uncertainty["friction_scale"],
            "collision_time_s": time_s, "force_N": force,
            "dir_x": direction[0], "dir_y": direction[1], "dir_z": direction[2],
            "distinguished": distinguished, "delay_ms": delay,
            "alarm_before": result["alarm_before_collision"],
            "reference_alarm": reference_alarm,
            "reference_false_alarms": reference_result["false_alarms"],
            "collision_false_alarms": result["false_alarms"],
            "rms_mm": result["rms_mm"], "max_mm": result["max_mm"],
            "diverged": result["diverged"],
        })
        if (i + 1) % 10 == 0:
            print(f"  {i + 1}/{args.cases} cases complete", flush=True)

    path = OUT / "environment_monte_carlo_v2.csv"
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=rows[0].keys())
        writer.writeheader(); writer.writerows(rows)
    good = [r for r in rows if r["distinguished"]]
    failures = [r for r in rows if not r["distinguished"]]
    summary = {
        "seed": args.seed, "cases": len(rows), "distinguished": len(good),
        "missed": len(failures),
        "reference_false_alarm_cases": sum(r["reference_false_alarms"] > 0 for r in rows),
        "collision_false_alarm_cases": sum(r["collision_false_alarms"] > 0 for r in rows),
        "alarm_before_cases": sum(r["alarm_before"] for r in rows),
        "reference_alarm_cases": sum(r["reference_alarm"] for r in rows),
        "diverged_cases": sum(r["diverged"] for r in rows),
        "delay_ms_median": float(np.median([r["delay_ms"] for r in good])) if good else None,
        "delay_ms_max": max((r["delay_ms"] for r in good), default=None),
        "rms_mm_max": max(r["rms_mm"] for r in rows),
        "max_error_mm": max(r["max_mm"] for r in rows),
        "failures": failures,
    }
    (OUT / "environment_monte_carlo_v2_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
