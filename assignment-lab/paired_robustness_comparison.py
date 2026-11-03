"""고정 테스트 세트에서 1차와 2차 검출기를 같은 조건으로 쌍대 비교한다."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

import hover_servo
import hybrid_detector
import robust_detector_v2
from sim.runner import run_episode


OUT = Path(__file__).resolve().parent / "results" / "phase2" / "paired_1000"
OUT.mkdir(parents=True, exist_ok=True)

PHASES = (
    ("startup_or_approach", 15, 0.10, 1.32),
    ("pickup_transition", 20, 1.32, 1.70),
    ("carry", 25, 1.70, 3.60),
    ("place_transition", 30, 3.60, 4.20),
    ("return", 10, 4.20, 5.70),
)
FORCE_BINS = (
    ("15_20N", 15.0, 20.0),
    ("20_30N", 20.0, 30.0),
    ("30_45N", 30.0, 45.0),
    ("45_60N", 45.0, 60.0),
)


def phase_for_index(index: int) -> tuple[str, float, float]:
    slot = index % 100
    edge = 0
    for name, count, low, high in PHASES:
        edge += count
        if slot < edge:
            return name, low, high
    raise AssertionError("phase allocation")


def evaluate_pair(detector_cls, mass, com, uncertainty, collision):
    reference_result, reference = run_episode(
        hover_servo.Controller(), box_mass=mass, box_com=com,
        detector=detector_cls(), plant_uncertainty=uncertainty,
    )
    result, _ = run_episode(
        hover_servo.Controller(), box_mass=mass, box_com=com,
        detector=detector_cls(), collision=collision,
        plant_uncertainty=uncertainty,
    )
    time_s = collision[0]
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
    if result["alarm_before_collision"]:
        failure_type = "alarm_before"
    elif delay is None:
        failure_type = "missed"
    elif reference_alarm:
        failure_type = "reference_overlap"
    else:
        failure_type = ""
    return {
        "distinguished": distinguished,
        "delay_ms": delay,
        "failure_type": failure_type,
        "alarm_before": result["alarm_before_collision"],
        "reference_alarm": reference_alarm,
        "reference_false_alarms": reference_result["false_alarms"],
        "collision_false_alarms": result["false_alarms"],
        "rms_mm": result["rms_mm"],
        "max_mm": result["max_mm"],
        "diverged": result["diverged"] or reference_result["diverged"],
    }


def prefixed(prefix: str, values: dict) -> dict:
    return {f"{prefix}_{key}": value for key, value in values.items()}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", type=int, default=100)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--tag", required=True)
    args = parser.parse_args()
    rng = np.random.default_rng(args.seed)
    rows = []

    for i in range(args.cases):
        global_index = (args.seed - 20261110) * 100 + i
        phase, time_low, time_high = phase_for_index(global_index)
        force_bin, force_low, force_high = FORCE_BINS[global_index % len(FORCE_BINS)]
        mass = float(rng.uniform(0.5, 8.0))
        size_scale = rng.uniform([0.60, 0.60, 0.45], [1.35, 1.35, 1.55])
        half = np.array([0.30, 0.20, 0.18]) * size_scale / 2
        com = tuple(rng.uniform(-0.45 * half, 0.45 * half))
        contact_friction = [
            float(np.exp(rng.uniform(np.log(0.15), np.log(1.8)))),
            float(np.exp(rng.uniform(np.log(0.001), np.log(0.015)))),
            float(np.exp(rng.uniform(np.log(0.00002), np.log(0.0003)))),
        ]
        uncertainty = {
            "box_size_scale": size_scale.tolist(),
            "box_inertia_scale": float(np.exp(rng.uniform(np.log(0.45), np.log(2.2)))),
            "table_time_scale": float(np.exp(rng.uniform(np.log(0.35), np.log(2.5)))),
            "table_damping_scale": float(rng.uniform(0.65, 1.8)),
            "contact_friction": contact_friction,
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
        time_s = float(rng.uniform(time_low, time_high))
        collision = (time_s, force * direction)

        v1 = evaluate_pair(hybrid_detector.Detector, mass, com, uncertainty, collision)
        v2 = evaluate_pair(robust_detector_v2.Detector, mass, com, uncertainty, collision)
        rows.append({
            "seed": args.seed, "case": i, "global_case": global_index,
            "phase": phase, "force_bin": force_bin,
            "mass_kg": mass,
            "size_x_scale": size_scale[0], "size_y_scale": size_scale[1],
            "size_z_scale": size_scale[2],
            "com_x_m": com[0], "com_y_m": com[1], "com_z_m": com[2],
            "inertia_scale": uncertainty["box_inertia_scale"],
            "table_time_scale": uncertainty["table_time_scale"],
            "table_damping_scale": uncertainty["table_damping_scale"],
            "friction_slide": contact_friction[0],
            "friction_torsional": contact_friction[1],
            "friction_rolling": contact_friction[2],
            "suction_time_scale": uncertainty["suction_time_scale"],
            "place_height_mm": 1000 * uncertainty["place_height_offset_m"],
            "noise_scale": uncertainty["noise_scale"],
            "offset_scale": uncertainty["offset_scale"],
            "drift_scale": uncertainty["drift_scale"],
            "robot_friction_scale": uncertainty["friction_scale"],
            "plant_seed": uncertainty["seed"],
            "collision_time_s": time_s, "force_N": force,
            "dir_x": direction[0], "dir_y": direction[1], "dir_z": direction[2],
            **prefixed("v1", v1), **prefixed("v2", v2),
        })
        if (i + 1) % 10 == 0:
            print(f"  {i + 1}/{args.cases} paired cases complete", flush=True)

    csv_path = OUT / f"{args.tag}.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=rows[0].keys(), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    compact = {
        "seed": args.seed, "cases": len(rows),
        "v1_distinguished": sum(row["v1_distinguished"] for row in rows),
        "v2_distinguished": sum(row["v2_distinguished"] for row in rows),
        "v1_reference_false_alarm_cases": sum(row["v1_reference_false_alarms"] > 0 for row in rows),
        "v2_reference_false_alarm_cases": sum(row["v2_reference_false_alarms"] > 0 for row in rows),
        "v1_diverged": sum(row["v1_diverged"] for row in rows),
        "v2_diverged": sum(row["v2_diverged"] for row in rows),
    }
    (OUT / f"{args.tag}_summary.json").write_text(
        json.dumps(compact, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(compact, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
