"""15 N 충돌이 하중 전환 시각과 겹칠 때의 감지 경계를 감사한다.

강건성 스윕과 달리 이 시험은 임계값 튜닝용 점수를 만들지 않는다. 집기·놓기
명령 바로 전후를 촘촘히 훑어 검출이 물리적으로 모호해지는 시각을 보고한다.
"""
import csv
import argparse
import importlib
import json
from pathlib import Path

import numpy as np

import hover_servo
from sim.runner import run_episode


OUT = Path(__file__).resolve().parent / "results"
OUT.mkdir(exist_ok=True)
ap = argparse.ArgumentParser()
ap.add_argument("--detector", default="hybrid_detector")
args = ap.parse_args()
Detector = importlib.import_module(args.detector).Detector
suffix = "" if args.detector == "hybrid_detector" else f"_{args.detector}"
DIRECTIONS = {
    "+X": (15, 0, 0), "-X": (-15, 0, 0),
    "+Y": (0, 15, 0), "-Y": (0, -15, 0),
    "+Z": (0, 0, 15), "-Z": (0, 0, -15),
}
TIMES = {
    "grasp": (1.34, 1.36, 1.38, 1.40, 1.42, 1.44, 1.46, 1.50),
    "release": (3.80, 3.84, 3.86, 3.88, 3.90, 3.92, 3.94, 3.98),
}

rows = []
# 같은 질량·COM·제어기에서 충돌만 뺀 기준 실행. 전이 자체가 같은 시각에
# 경보를 만드는 경우를 충돌 검출로 잘못 세지 않기 위해 모든 조건과 대조한다.
_, reference_log = run_episode(
    hover_servo.Controller(),
    box_mass=8.0,
    box_com=(0.02, -0.01, 0.03),
    detector=Detector(),
)
reference_detect = reference_log["detect"]
for phase, times in TIMES.items():
    for time_s in times:
        for direction, vector in DIRECTIONS.items():
            result, _ = run_episode(
                hover_servo.Controller(),
                box_mass=8.0,
                box_com=(0.02, -0.01, 0.03),
                detector=Detector(),
                collision=(time_s, vector),
            )
            delay = result["detect_delay_ms"]
            reference_alarm = False
            if delay is not None and not result["alarm_before_collision"]:
                k = int(round((time_s + delay / 1000.0) / 0.001))
                reference_alarm = bool(
                    reference_detect[max(0, k - 20):min(len(reference_detect), k + 21)].any()
                )
            rows.append({
                "phase": phase,
                "collision_time_s": time_s,
                "direction": direction,
                "detected": delay is not None,
                "distinguishable": (
                    delay is not None
                    and not result["alarm_before_collision"]
                    and not reference_alarm
                ),
                "reference_alarm": reference_alarm,
                "delay_ms": delay,
                "alarm_before": result["alarm_before_collision"],
                "false_alarms": result["false_alarms"],
            })
        print(f"  {phase} {time_s:.2f} s complete", flush=True)

with (OUT / f"timing_audit{suffix}.csv").open("w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=rows[0].keys())
    writer.writeheader()
    writer.writerows(rows)

detected = [r for r in rows if r["distinguishable"]]
summary = {
    "cases": len(rows),
    "detected": len(detected),
    "missed": len(rows) - len(detected),
    "alarm_before": sum(r["alarm_before"] for r in rows),
    "reference_alarm_cases": sum(r["reference_alarm"] for r in rows),
    "false_alarm_cases": sum(r["false_alarms"] > 0 for r in rows),
    "delay_ms_median": float(np.median([r["delay_ms"] for r in detected])),
    "delay_ms_max": max(r["delay_ms"] for r in detected),
    "misses": [
        {k: r[k] for k in ("phase", "collision_time_s", "direction")}
        for r in rows if not r["distinguishable"]
    ],
}
(OUT / f"timing_audit{suffix}_summary.json").write_text(
    json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
)
print(json.dumps(summary, indent=2, ensure_ascii=False))
