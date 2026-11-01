"""Phase 2 검출기의 전체 사이클 10/15 N, 6방향 감도 지도."""
from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

import hover_servo
from robust_detector_v2 import Detector
from sim.runner import run_episode


OUT = Path(__file__).resolve().parent / "results" / "phase2"
TIMES = (0.10, 0.50, 0.90, 1.30, 1.42, 2.00, 3.00, 3.70, 3.86, 4.10, 4.80, 5.50)
FORCES = (10.0, 15.0)
DIRECTIONS = {
    "+X": (1, 0, 0), "-X": (-1, 0, 0),
    "+Y": (0, 1, 0), "-Y": (0, -1, 0),
    "+Z": (0, 0, 1), "-Z": (0, 0, -1),
}


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    _, reference = run_episode(
        hover_servo.Controller(), box_mass=8.0, detector=Detector()
    )
    rows = []
    for force in FORCES:
        for time_s in TIMES:
            for name, direction in DIRECTIONS.items():
                result, _ = run_episode(
                    hover_servo.Controller(), box_mass=8.0, detector=Detector(),
                    collision=(time_s, force * np.asarray(direction)),
                )
                delay = result["detect_delay_ms"]
                reference_alarm = False
                if delay is not None and not result["alarm_before_collision"]:
                    k = int(round((time_s + delay / 1000.0) / 0.001))
                    reference_alarm = bool(
                        reference["detect"][max(0, k - 20):min(len(reference["detect"]), k + 21)].any()
                    )
                distinguishable = bool(
                    delay is not None
                    and not result["alarm_before_collision"]
                    and not reference_alarm
                )
                rows.append({
                    "force_N": force, "time_s": time_s, "direction": name,
                    "detected": distinguishable, "delay_ms": delay,
                    "alarm_before": result["alarm_before_collision"],
                    "reference_alarm": reference_alarm,
                    "false_alarms": result["false_alarms"],
                })
            print(f"  {force:.0f} N @ {time_s:.2f} s complete", flush=True)

    with (OUT / "sensitivity_audit_v2.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    groups = {}
    for force in FORCES:
        selected = [row for row in rows if row["force_N"] == force]
        good = [row for row in selected if row["detected"]]
        groups[f"{force:.0f}N"] = {
            "cases": len(selected), "detected": len(good),
            "missed": len(selected) - len(good),
            "alarm_before_cases": sum(row["alarm_before"] for row in selected),
            "reference_alarm_cases": sum(row["reference_alarm"] for row in selected),
            "false_alarm_cases": sum(row["false_alarms"] > 0 for row in selected),
            "delay_ms_median": float(np.median([row["delay_ms"] for row in good])) if good else None,
            "delay_ms_max": max((row["delay_ms"] for row in good), default=None),
            "misses": [
                {key: row[key] for key in ("time_s", "direction")}
                for row in selected if not row["detected"]
            ],
        }
    summary = {"mass_kg": 8.0, "groups": groups}
    (OUT / "sensitivity_audit_v2_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
