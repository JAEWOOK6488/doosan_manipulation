"""고정 놓기 호버 높이와 작업대 높이 오차의 절충을 정량화한다."""
import json
from pathlib import Path

import numpy as np

import hover_servo
import hybrid_detector
from sim.runner import run_episode


OUT = Path(__file__).resolve().parent / "results" / "hover_height_sweep.json"
HOVER_HEIGHTS_MM = (0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0)
TABLE_OFFSETS_MM = (-3, -2, -1, 0, 1, 2, 3)

rows = []
for hover_mm in HOVER_HEIGHTS_MM:
    hover_servo.HOVER_M = hover_mm / 1000.0
    forces, rms_values, false_alarms = [], [], 0
    for offset_mm in TABLE_OFFSETS_MM:
        result, log = run_episode(
            hover_servo.Controller(), box_mass=8,
            detector=hybrid_detector.Detector(),
            plant_uncertainty={"place_height_offset_m": offset_mm / 1000.0},
        )
        release = (log["t"] >= 3.7) & (log["t"] < 4.1)
        forces.append(float(log["f_table"][release].max()))
        rms_values.append(result["rms_mm"])
        false_alarms += result["false_alarms"]
    row = {
        "hover_mm": hover_mm,
        "max_force_N": max(forces),
        "mean_force_N": float(np.mean(forces)),
        "nominal_force_N": forces[TABLE_OFFSETS_MM.index(0)],
        "false_alarms_total": false_alarms,
        "max_rms_mm": max(rms_values),
        "forces_N": forces,
    }
    rows.append(row)
    print(row, flush=True)

OUT.write_text(json.dumps({
    "table_offsets_mm": TABLE_OFFSETS_MM,
    "rows": rows,
}, indent=2), encoding="utf-8")
print(OUT)
