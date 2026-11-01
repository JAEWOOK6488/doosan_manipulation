"""공개 평가 밖의 질량·COM·충돌 방향 강건성 시험."""
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
rows = []
reference_detect = {}


def run_case(kind, mass, com, collision=None):
    key = (float(mass), tuple(float(x) for x in com))
    result, log = run_episode(
        hover_servo.Controller(),
        box_mass=mass,
        box_com=com,
        detector=Detector(),
        collision=collision,
    )
    if collision is None:
        reference_detect[key] = log["detect"].copy()
    elif key not in reference_detect:
        _, reference = run_episode(
            hover_servo.Controller(), box_mass=mass, box_com=com,
            detector=Detector(),
        )
        reference_detect[key] = reference["detect"].copy()
    delay = result.get("detect_delay_ms")
    reference_alarm = False
    if collision is not None and delay is not None and not result.get("alarm_before_collision", False):
        k = int(round((collision[0] + delay / 1000.0) / 0.001))
        ref = reference_detect[key]
        reference_alarm = bool(ref[max(0, k - 20):min(len(ref), k + 21)].any())
    row = {
        "kind": kind,
        "mass_kg": mass,
        "com_x_m": com[0],
        "com_y_m": com[1],
        "com_z_m": com[2],
        "collision_time_s": None if collision is None else collision[0],
        "fx_N": 0 if collision is None else collision[1][0],
        "fy_N": 0 if collision is None else collision[1][1],
        "fz_N": 0 if collision is None else collision[1][2],
        "false_alarms": result["false_alarms"],
        "delay_ms": delay,
        "alarm_before": result.get("alarm_before_collision", False),
        "reference_alarm": reference_alarm,
        "distinguishable": (
            collision is None or (
                delay is not None
                and not result.get("alarm_before_collision", False)
                and not reference_alarm
            )
        ),
        "rms_mm": result["rms_mm"],
        "max_mm": result["max_mm"],
        "diverged": result["diverged"],
    }
    rows.append(row)
    if len(rows) % 10 == 0:
        print(f"  {len(rows)} cases complete", flush=True)


coms = [
    (0.02, -0.01, 0.03),
    (-0.03, 0.02, 0.01),
    (0.05, 0.00, 0.05),
    (0.00, -0.04, -0.02),
]

# 충돌 없는 일반화: 공개 평가에 없는 질량과 COM.
for mass in (1.0, 3.0, 5.0, 7.0):
    for com in coms:
        run_case("nominal", mass, com)

# 세 축 양/음 방향, 두 힘 크기, 세 작업 상태.
directions = {
    "+X": (1, 0, 0), "-X": (-1, 0, 0),
    "+Y": (0, 1, 0), "-Y": (0, -1, 0),
    "+Z": (0, 0, 1), "-Z": (0, 0, -1),
}
for time_s in (1.42, 3.00, 3.86):
    for force in (15.0, 40.0):
        for direction in directions.values():
            vector = [force * x for x in direction]
            run_case("direction", 8.0, coms[0], (time_s, vector))

# 공개하지 않은 질량/COM 조합에서 40 N 충돌.
for mass, com in zip((1.0, 3.0, 5.0, 7.0), coms):
    for time_s in (1.42, 3.00, 3.86):
        for vector in ([40, 0, 0], [0, 40, 0], [0, 0, 40]):
            run_case("hidden", mass, com, (time_s, vector))

with (OUT / f"robustness{suffix}.csv").open("w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=rows[0].keys())
    writer.writeheader()
    writer.writerows(rows)

nominal = [r for r in rows if r["kind"] == "nominal"]
collisions = [r for r in rows if r["kind"] != "nominal"]
summary = {
    "cases": len(rows),
    "nominal_cases": len(nominal),
    "nominal_false_alarm_cases": sum(r["false_alarms"] > 0 for r in nominal),
    "nominal_diverged_cases": sum(r["diverged"] for r in nominal),
    "nominal_rms_mm_max": max(r["rms_mm"] for r in nominal),
    "nominal_max_mm_max": max(r["max_mm"] for r in nominal),
    "collision_cases": len(collisions),
    "collision_detected": sum(r["distinguishable"] for r in collisions),
    "collision_missed": sum(not r["distinguishable"] for r in collisions),
    "collision_alarm_before": sum(r["alarm_before"] for r in collisions),
    "collision_reference_alarm_cases": sum(r["reference_alarm"] for r in collisions),
    "collision_false_alarm_cases": sum(r["false_alarms"] > 0 for r in collisions),
    "delay_ms_median": float(np.median([r["delay_ms"] for r in collisions if r["distinguishable"]])),
    "delay_ms_max": max(r["delay_ms"] for r in collisions if r["distinguishable"]),
}
(OUT / f"robustness{suffix}_summary.json").write_text(
    json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
)
print(json.dumps(summary, indent=2, ensure_ascii=False))
