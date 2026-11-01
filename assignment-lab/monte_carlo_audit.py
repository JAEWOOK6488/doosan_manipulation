"""격자 사이 임의 질량·COM·충돌 조건의 재현 가능한 강건성 감사."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

import hover_servo
from hybrid_detector import Detector
from sim.runner import run_episode


OUT = Path(__file__).resolve().parent / "results"
OUT.mkdir(exist_ok=True)


def collision_time(rng: np.random.Generator, index: int) -> float:
    """일반 구간과 집기·놓기 주변을 비슷한 수로 뽑는다."""
    region = index % 3
    if region == 0:
        return float(rng.uniform(0.7, 5.4))
    if region == 1:
        return float(rng.uniform(1.30, 1.70))
    return float(rng.uniform(3.70, 4.10))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", type=int, default=60)
    parser.add_argument("--seed", type=int, default=20261101)
    args = parser.parse_args()
    if args.cases <= 0:
        raise ValueError("--cases must be positive")

    rng = np.random.default_rng(args.seed)
    rows = []
    for i in range(args.cases):
        mass = float(rng.uniform(0.0, 8.0))
        # 박스 반치수보다 충분히 안쪽인 연속 COM 범위.
        com = tuple(rng.uniform([-0.06, -0.05, -0.03], [0.06, 0.05, 0.06]))
        direction = rng.normal(size=3)
        direction /= np.linalg.norm(direction)
        force = float(rng.uniform(15.0, 60.0))
        time_s = collision_time(rng, i)
        collision = (time_s, force * direction)

        _, reference = run_episode(
            hover_servo.Controller(), box_mass=mass, box_com=com,
            detector=Detector(),
        )
        result, _ = run_episode(
            hover_servo.Controller(), box_mass=mass, box_com=com,
            detector=Detector(), collision=collision,
        )
        delay = result["detect_delay_ms"]
        reference_alarm = False
        if delay is not None and not result["alarm_before_collision"]:
            k = int(round((time_s + delay / 1000.0) / 0.001))
            reference_alarm = bool(
                reference["detect"][max(0, k - 20):min(len(reference["detect"]), k + 21)].any()
            )
        detected = bool(
            delay is not None
            and not result["alarm_before_collision"]
            and not reference_alarm
        )
        rows.append({
            "case": i,
            "mass_kg": mass,
            "com_x_m": com[0], "com_y_m": com[1], "com_z_m": com[2],
            "collision_time_s": time_s, "force_N": force,
            "dir_x": direction[0], "dir_y": direction[1], "dir_z": direction[2],
            "detected": detected, "delay_ms": delay,
            "alarm_before": result["alarm_before_collision"],
            "reference_alarm": reference_alarm,
            "false_alarms": result["false_alarms"],
            "rms_mm": result["rms_mm"], "max_mm": result["max_mm"],
            "diverged": result["diverged"],
        })
        if (i + 1) % 10 == 0:
            print(f"  {i + 1}/{args.cases} cases complete", flush=True)

    with (OUT / "monte_carlo_audit.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    good = [row for row in rows if row["detected"]]
    misses = [row for row in rows if not row["detected"]]
    summary = {
        "seed": args.seed,
        "cases": len(rows),
        "detected": len(good),
        "missed": len(misses),
        "alarm_before_cases": sum(row["alarm_before"] for row in rows),
        "reference_alarm_cases": sum(row["reference_alarm"] for row in rows),
        "false_alarm_cases": sum(row["false_alarms"] > 0 for row in rows),
        "diverged_cases": sum(row["diverged"] for row in rows),
        "delay_ms_median": float(np.median([row["delay_ms"] for row in good])) if good else None,
        "delay_ms_max": max((row["delay_ms"] for row in good), default=None),
        "rms_mm_max": max(row["rms_mm"] for row in rows),
        "max_error_mm": max(row["max_mm"] for row in rows),
        "misses": misses,
    }
    (OUT / "monte_carlo_audit_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n"
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
