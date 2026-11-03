"""1차/2차 1,000조건 쌍대 시험의 통계와 실패 목록을 생성한다."""
from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path

import numpy as np


OUT = Path(__file__).resolve().parent / "results" / "phase2" / "paired_1000"


def boolean(value: str) -> bool:
    return value.strip().lower() == "true"


def wilson(successes: int, total: int, z: float = 1.959963984540054) -> list[float]:
    p = successes / total
    denom = 1 + z * z / total
    centre = (p + z * z / (2 * total)) / denom
    radius = z * math.sqrt(p * (1 - p) / total + z * z / (4 * total * total)) / denom
    return [centre - radius, centre + radius]


def mcnemar_exact(v1_only: int, v2_only: int) -> float:
    discordant = v1_only + v2_only
    if discordant == 0:
        return 1.0
    tail = sum(math.comb(discordant, k) for k in range(min(v1_only, v2_only) + 1))
    return min(1.0, 2.0 * tail / (2 ** discordant))


def metric(rows: list[dict], prefix: str) -> dict:
    success = [row for row in rows if row[f"{prefix}_distinguished"]]
    delays = [row[f"{prefix}_delay_ms"] for row in success]
    return {
        "distinguished": len(success),
        "distinguished_rate": len(success) / len(rows),
        "distinguished_rate_wilson_95": wilson(len(success), len(rows)),
        "missed": sum(row[f"{prefix}_failure_type"] == "missed" for row in rows),
        "alarm_before": sum(row[f"{prefix}_failure_type"] == "alarm_before" for row in rows),
        "reference_overlap": sum(row[f"{prefix}_failure_type"] == "reference_overlap" for row in rows),
        "reference_false_alarm_cases": sum(row[f"{prefix}_reference_false_alarms"] > 0 for row in rows),
        "collision_false_alarm_cases": sum(row[f"{prefix}_collision_false_alarms"] > 0 for row in rows),
        "diverged_cases": sum(row[f"{prefix}_diverged"] for row in rows),
        "delay_ms_median": float(np.median(delays)),
        "delay_ms_p95": float(np.percentile(delays, 95)),
        "delay_ms_max": max(delays),
        "delay_over_100ms": sum(delay > 100 for delay in delays),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("inputs", nargs="+")
    args = parser.parse_args()
    rows = []
    for raw in args.inputs:
        with Path(raw).open(newline="", encoding="utf-8") as file:
            for row in csv.DictReader(file):
                for prefix in ("v1", "v2"):
                    for key in ("distinguished", "alarm_before", "reference_alarm", "diverged"):
                        row[f"{prefix}_{key}"] = boolean(row[f"{prefix}_{key}"])
                    for key in ("reference_false_alarms", "collision_false_alarms"):
                        row[f"{prefix}_{key}"] = int(row[f"{prefix}_{key}"])
                    for key in ("delay_ms", "rms_mm", "max_mm"):
                        value = row[f"{prefix}_{key}"]
                        row[f"{prefix}_{key}"] = None if value == "" else float(value)
                row["force_N"] = float(row["force_N"])
                row["collision_time_s"] = float(row["collision_time_s"])
                rows.append(row)

    v1_only = sum(row["v1_distinguished"] and not row["v2_distinguished"] for row in rows)
    v2_only = sum(row["v2_distinguished"] and not row["v1_distinguished"] for row in rows)
    both = sum(row["v1_distinguished"] and row["v2_distinguished"] for row in rows)
    neither = len(rows) - both - v1_only - v2_only
    phase_table = {}
    for phase in ("startup_or_approach", "pickup_transition", "carry", "place_transition", "return"):
        selected = [row for row in rows if row["phase"] == phase]
        phase_table[phase] = {
            "cases": len(selected),
            "v1_distinguished": sum(row["v1_distinguished"] for row in selected),
            "v2_distinguished": sum(row["v2_distinguished"] for row in selected),
        }
    force_table = {}
    for force_bin in ("15_20N", "20_30N", "30_45N", "45_60N"):
        selected = [row for row in rows if row["force_bin"] == force_bin]
        force_table[force_bin] = {
            "cases": len(selected),
            "v1_distinguished": sum(row["v1_distinguished"] for row in selected),
            "v2_distinguished": sum(row["v2_distinguished"] for row in selected),
        }
    v2_failures = [
        row for row in rows
        if not row["v2_distinguished"] or row["v2_reference_false_alarms"] > 0
        or (row["v2_delay_ms"] is not None and row["v2_delay_ms"] > 100)
    ]
    summary = {
        "cases": len(rows),
        "episodes": 4 * len(rows),
        "seeds": sorted({int(row["seed"]) for row in rows}),
        "v1": metric(rows, "v1"),
        "v2": metric(rows, "v2"),
        "paired_contingency": {
            "both_success": both, "v1_only_success": v1_only,
            "v2_only_success": v2_only, "neither_success": neither,
            "mcnemar_exact_two_sided_p": mcnemar_exact(v1_only, v2_only),
        },
        "by_phase": phase_table,
        "by_force": force_table,
        "v2_failure_types": dict(Counter(
            row["v2_failure_type"] or "success_with_other_flag" for row in v2_failures
        )),
        "v2_failure_cases": v2_failures,
    }
    OUT.mkdir(parents=True, exist_ok=True)
    with (OUT / "paired_comparison_all.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=rows[0].keys(), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    with (OUT / "v2_failure_cases.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=rows[0].keys(), lineterminator="\n")
        writer.writeheader()
        writer.writerows(v2_failures)
    (OUT / "paired_comparison_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    printable = {key: value for key, value in summary.items() if key != "v2_failure_cases"}
    print(json.dumps(printable, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
