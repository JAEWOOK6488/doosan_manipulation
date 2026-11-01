"""15 N 충돌을 전체 사이클에서 0.1 s 간격으로 탐색한다.

대표 12시각 감사 사이의 사각지대를 찾는 추가 시험이다. 결과가 전 연속시간의
수학적 보장을 뜻하지는 않으며, 같은 조건의 무충돌 실행과 검출 시각을 대조한다.
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

import hover_servo
from hybrid_detector import Detector
from sim.runner import run_episode


OUT = Path(__file__).resolve().parent / "results"
TIMES = np.round(np.arange(0.1, 5.81, 0.1), 2)
DIRECTIONS = {
    "+X": (15, 0, 0), "-X": (-15, 0, 0),
    "+Y": (0, 15, 0), "-Y": (0, -15, 0),
    "+Z": (0, 0, 15), "-Z": (0, 0, -15),
}


def main() -> None:
    _, reference = run_episode(
        hover_servo.Controller(), box_mass=8.0, detector=Detector()
    )
    rows = []
    for index, time_s in enumerate(TIMES, 1):
        for name, vector in DIRECTIONS.items():
            result, _ = run_episode(
                hover_servo.Controller(), box_mass=8.0, detector=Detector(),
                collision=(float(time_s), np.asarray(vector, dtype=float)),
            )
            delay = result["detect_delay_ms"]
            reference_alarm = False
            if delay is not None and not result["alarm_before_collision"]:
                k = int(round((time_s + delay / 1000.0) / 0.001))
                reference_alarm = bool(
                    reference["detect"][max(0, k - 20):min(len(reference["detect"]), k + 21)].any()
                )
            rows.append({
                "time_s": float(time_s), "direction": name,
                "distinguishable": bool(
                    delay is not None
                    and not result["alarm_before_collision"]
                    and not reference_alarm
                ),
                "delay_ms": delay,
                "alarm_before": result["alarm_before_collision"],
                "reference_alarm": reference_alarm,
                "false_alarms": result["false_alarms"],
            })
        if index % 5 == 0 or index == len(TIMES):
            print(f"  {index}/{len(TIMES)} times complete", flush=True)

    OUT.mkdir(exist_ok=True)
    csv_path = OUT / "dense_sensitivity_audit.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    good = [row for row in rows if row["distinguishable"]]
    summary = {
        "force_N": 15.0,
        "mass_kg": 8.0,
        "time_step_s": 0.1,
        "cases": len(rows),
        "detected": len(good),
        "missed": len(rows) - len(good),
        "alarm_before_cases": sum(row["alarm_before"] for row in rows),
        "reference_alarm_cases": sum(row["reference_alarm"] for row in rows),
        "false_alarm_cases": sum(row["false_alarms"] > 0 for row in rows),
        "delay_ms_median": float(np.median([row["delay_ms"] for row in good])) if good else None,
        "delay_ms_max": max((row["delay_ms"] for row in good), default=None),
        "misses": [
            {key: row[key] for key in ("time_s", "direction")}
            for row in rows if not row["distinguishable"]
        ],
    }
    (OUT / "dense_sensitivity_audit_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
