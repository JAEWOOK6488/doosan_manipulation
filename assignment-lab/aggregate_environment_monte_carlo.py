"""여러 환경 Monte Carlo 배치를 합쳐 통계와 실패 집중 구간을 계산한다."""
from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path

import numpy as np


OUT = Path(__file__).resolve().parent / "results" / "phase2"
BOOL_FIELDS = (
    "distinguished", "alarm_before", "reference_alarm", "diverged",
)
INT_FIELDS = ("seed", "case", "reference_false_alarms", "collision_false_alarms")
FLOAT_FIELDS = (
    "mass_kg", "size_x_scale", "size_y_scale", "size_z_scale",
    "com_x_m", "com_y_m", "com_z_m", "inertia_scale",
    "table_time_scale", "table_damping_scale", "friction_slide",
    "suction_time_scale", "place_height_mm", "noise_scale", "offset_scale",
    "drift_scale", "robot_friction_scale", "collision_time_s", "force_N",
    "dir_x", "dir_y", "dir_z", "delay_ms", "rms_mm", "max_mm",
)


def parse_bool(value: str) -> bool:
    return value.strip().lower() == "true"


def wilson(successes: int, total: int, z: float = 1.959963984540054) -> list[float]:
    p = successes / total
    denom = 1.0 + z * z / total
    centre = (p + z * z / (2 * total)) / denom
    radius = z * math.sqrt(p * (1 - p) / total + z * z / (4 * total * total)) / denom
    return [centre - radius, centre + radius]


def phase(time_s: float) -> str:
    if time_s < 1.32:
        return "startup_or_approach"
    if time_s < 1.70:
        return "pickup_transition"
    if time_s < 3.60:
        return "carry"
    if time_s < 4.20:
        return "place_transition"
    return "return"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("inputs", nargs="+")
    parser.add_argument("--tag", default="environment_monte_carlo_extended_v2")
    args = parser.parse_args()

    rows = []
    for raw_path in args.inputs:
        path = Path(raw_path)
        with path.open(newline="", encoding="utf-8") as file:
            for row in csv.DictReader(file):
                for key in BOOL_FIELDS:
                    row[key] = parse_bool(row[key])
                for key in INT_FIELDS:
                    row[key] = int(row[key])
                for key in FLOAT_FIELDS:
                    row[key] = None if row[key] == "" else float(row[key])
                row["phase"] = phase(row["collision_time_s"])
                rows.append(row)

    if not rows:
        raise SystemExit("입력 결과가 없습니다")
    keys = list(rows[0].keys())
    with (OUT / f"{args.tag}.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=keys, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)

    good = [row for row in rows if row["distinguished"]]
    failures = [row for row in rows if not row["distinguished"]]
    per_seed = {}
    for seed in sorted({row["seed"] for row in rows}):
        selected = [row for row in rows if row["seed"] == seed]
        detected = sum(row["distinguished"] for row in selected)
        per_seed[str(seed)] = {
            "cases": len(selected), "distinguished": detected,
            "missed": len(selected) - detected,
        }
    summary = {
        "input_files": [str(Path(p)) for p in args.inputs],
        "seeds": sorted({row["seed"] for row in rows}),
        "cases": len(rows), "distinguished": len(good), "missed": len(failures),
        "distinguished_rate": len(good) / len(rows),
        "distinguished_rate_wilson_95": wilson(len(good), len(rows)),
        "reference_false_alarm_cases": sum(row["reference_false_alarms"] > 0 for row in rows),
        "collision_false_alarm_cases": sum(row["collision_false_alarms"] > 0 for row in rows),
        "alarm_before_cases": sum(row["alarm_before"] for row in rows),
        "reference_alarm_cases": sum(row["reference_alarm"] for row in rows),
        "diverged_cases": sum(row["diverged"] for row in rows),
        "delay_ms_median": float(np.median([row["delay_ms"] for row in good])),
        "delay_ms_p95": float(np.percentile([row["delay_ms"] for row in good], 95)),
        "delay_ms_max": max(row["delay_ms"] for row in good),
        "misses_by_phase": dict(Counter(row["phase"] for row in failures)),
        "per_seed": per_seed,
        "failures": failures,
    }
    (OUT / f"{args.tag}_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps({key: value for key, value in summary.items() if key != "failures"}, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
