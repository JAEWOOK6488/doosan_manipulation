"""관측기 기동·교정 구간의 15 N 전 방향 충돌 감사를 수행한다."""
from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

import hover_servo
from hybrid_detector import Detector
from sim.runner import run_episode


OUT = Path(__file__).resolve().parent / "results"
TIMES = (0.10, 0.30, 0.50, 0.70, 0.90, 1.10, 1.20, 1.30)
DIRECTIONS = {
    "+X": (1, 0, 0), "-X": (-1, 0, 0),
    "+Y": (0, 1, 0), "-Y": (0, -1, 0),
    "+Z": (0, 0, 1), "-Z": (0, 0, -1),
}


def main() -> None:
    _, reference = run_episode(
        hover_servo.Controller(), box_mass=8.0, detector=Detector()
    )
    rows = []
    for time_s in TIMES:
        for name, direction in DIRECTIONS.items():
            collision = (time_s, 15.0 * np.asarray(direction))
            result, _ = run_episode(
                hover_servo.Controller(), box_mass=8.0,
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
                "time_s": time_s, "direction": name,
                "detected": detected, "delay_ms": delay,
                "alarm_before": result["alarm_before_collision"],
                "reference_alarm": reference_alarm,
                "false_alarms": result["false_alarms"],
            })
        print(f"  {time_s:.2f} s complete", flush=True)

    with (OUT / "startup_audit.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    good = [row for row in rows if row["detected"]]
    summary = {
        "cases": len(rows),
        "detected": len(good),
        "missed": len(rows) - len(good),
        "alarm_before_cases": sum(row["alarm_before"] for row in rows),
        "reference_alarm_cases": sum(row["reference_alarm"] for row in rows),
        "false_alarm_cases": sum(row["false_alarms"] > 0 for row in rows),
        "delay_ms_median": float(np.median([row["delay_ms"] for row in good])) if good else None,
        "delay_ms_max": max((row["delay_ms"] for row in good), default=None),
        "misses": [row for row in rows if not row["detected"]],
    }
    (OUT / "startup_audit_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n"
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
