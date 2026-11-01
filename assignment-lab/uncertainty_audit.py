"""센서 오차·마찰 모델 불일치에 대한 최종안 강건성 감사."""
import csv
import json
from pathlib import Path

import hover_servo
import hybrid_detector
from sim.runner import run_episode


OUT = Path(__file__).resolve().parent / "results"
OUT.mkdir(exist_ok=True)
scenarios = [("reference", {})]
scenarios += [(f"noise2_seed{s}", {"noise_scale": 2.0, "seed": s}) for s in (1, 2, 3, 11)]
scenarios += [("noise3_seed13", {"noise_scale": 3.0, "seed": 13})]
scenarios += [(f"offset{x:g}", {"offset_scale": x}) for x in (0.0, 0.5, 1.5, 2.0)]
scenarios += [(f"drift{x:g}", {"drift_scale": x}) for x in (0.0, 3.0, 5.0)]
scenarios += [(f"friction{x:g}", {"friction_scale": x}) for x in (0.7, 1.3)]
scenarios += [("combined", {
    "noise_scale": 2.0, "offset_scale": 1.5, "drift_scale": 3.0,
    "friction_scale": 1.3, "seed": 19,
})]

collisions = (
    ("pre_grasp_-Z", (1.36, [0, 0, -15])),
    ("carry_+Y", (3.00, [0, 15, 0])),
    ("pre_release_+Z", (3.84, [0, 0, 15])),
)
rows = []
for name, uncertainty in scenarios:
    result, reference_log = run_episode(
        hover_servo.Controller(), box_mass=8, detector=hybrid_detector.Detector(),
        plant_uncertainty=uncertainty,
    )
    rows.append({
        "scenario": name, "case": "no_collision", "detected": "",
        "distinguishable": "", "reference_alarm": False,
        "delay_ms": "", "alarm_before": False,
        "false_alarms": result["false_alarms"], "rms_mm": result["rms_mm"],
        "max_mm": result["max_mm"], "diverged": result["diverged"],
    })
    for case, collision in collisions:
        result, _ = run_episode(
            hover_servo.Controller(), box_mass=8,
            detector=hybrid_detector.Detector(), collision=collision,
            plant_uncertainty=uncertainty,
        )
        delay = result["detect_delay_ms"]
        reference_alarm = False
        if delay is not None and not result["alarm_before_collision"]:
            k = int(round((collision[0] + delay / 1000.0) / 0.001))
            ref = reference_log["detect"]
            reference_alarm = bool(ref[max(0, k - 20):min(len(ref), k + 21)].any())
        rows.append({
            "scenario": name, "case": case,
            "detected": delay is not None,
            "distinguishable": (
                delay is not None
                and not result["alarm_before_collision"]
                and not reference_alarm
            ),
            "reference_alarm": reference_alarm,
            "delay_ms": delay,
            "alarm_before": result["alarm_before_collision"],
            "false_alarms": result["false_alarms"], "rms_mm": result["rms_mm"],
            "max_mm": result["max_mm"], "diverged": result["diverged"],
        })
    print(f"  {name} complete", flush=True)

with (OUT / "uncertainty_audit.csv").open("w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=rows[0].keys())
    writer.writeheader()
    writer.writerows(rows)

nominal = [r for r in rows if r["case"] == "no_collision"]
collision_rows = [r for r in rows if r["case"] != "no_collision"]
summary = {
    "scenarios": len(scenarios),
    "cases": len(rows),
    "nominal_false_alarm_scenarios": sum(r["false_alarms"] > 0 for r in nominal),
    "nominal_diverged_scenarios": sum(r["diverged"] for r in nominal),
    "collision_cases": len(collision_rows),
    "collision_detected": sum(r["distinguishable"] for r in collision_rows),
    "collision_missed": sum(not r["distinguishable"] for r in collision_rows),
    "collision_alarm_before": sum(r["alarm_before"] for r in collision_rows),
    "collision_reference_alarm_cases": sum(r["reference_alarm"] for r in collision_rows),
    "collision_false_alarm_cases": sum(r["false_alarms"] > 0 for r in collision_rows),
    "max_delay_ms": max(
        r["delay_ms"] for r in collision_rows if r["delay_ms"] not in ("", None)
    ),
    "max_rms_mm": max(r["rms_mm"] for r in rows),
    "max_error_mm": max(r["max_mm"] for r in rows),
}
(OUT / "uncertainty_audit_summary.json").write_text(
    json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
)
print(json.dumps(summary, indent=2, ensure_ascii=False))
