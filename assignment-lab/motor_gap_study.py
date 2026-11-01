"""모터 전류 기반 토크와 JTS의 단순 차이가 충돌 판별에 충분한지 확인한다."""
import json
from pathlib import Path

import numpy as np

import hover_servo
import hybrid_detector
from sim.runner import run_episode


OUT = Path(__file__).resolve().parent / "results" / "motor_jts_gap_study.json"
MASS = 8.0
FORCE = [0.0, 10.0, 0.0]
TIMES = (1.42, 3.00, 3.86)
LAG = 20


def lag_change(x):
    return np.vstack((np.zeros((LAG, x.shape[1])), x[LAG:] - x[:-LAG]))


_, baseline = run_episode(
    hover_servo.Controller(), box_mass=MASS, detector=hybrid_detector.Detector()
)
baseline_gap = baseline["tau_m"] - baseline["tau_s"]
baseline_change = lag_change(baseline_gap)

cases = []
for collision_time in TIMES:
    result, collision = run_episode(
        hover_servo.Controller(), box_mass=MASS,
        detector=hybrid_detector.Detector(),
        collision=(collision_time, FORCE),
    )
    gap = collision["tau_m"] - collision["tau_s"]
    change = lag_change(gap)
    k = int(round(collision_time / 0.001))
    window = slice(k, k + 140)
    cases.append({
        "collision_time_s": collision_time,
        "detect_delay_ms": result["detect_delay_ms"],
        "baseline_gap_change_max_Nm": np.max(
            np.abs(baseline_change[window]), axis=0
        ).tolist(),
        "collision_gap_change_max_Nm": np.max(
            np.abs(change[window]), axis=0
        ).tolist(),
        "collision_increment_max_Nm": np.max(
            np.abs(gap[window] - baseline_gap[window]), axis=0
        ).tolist(),
    })

summary = {
    "mass_kg": MASS,
    "collision_force_N": FORCE,
    "lag_ms": LAG,
    "interpretation": (
        "Normal load transfer can dominate tau_m-tau_s changes; a fixed threshold "
        "on the raw gap is not a reliable 10 N collision discriminator."
    ),
    "cases": cases,
}
OUT.write_text(json.dumps(summary, indent=2), encoding="utf-8")
print(OUT)
