"""관절 토크 반복 데이터로 집기·놓기 하중 전달률 alpha(t)를 식별한다.

동일 센서 시드의 빈손 실행과 하중 실행을 짝지어 JTS 차이를 구하고,
명목 payload 토크 방향에 최소제곱 투영한다. 시뮬레이터 내부의 실제 하중률은
사용하지 않으므로 같은 절차를 실제 장비 로그에도 적용할 수 있다.
"""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import hover_servo
from sim.nominal import Nominal
from sim.runner import run_episode
from sim.trajectory import T_GRASP, T_RELEASE, reference


HERE = Path(__file__).resolve().parent


def moving_average(x, width=9):
    kernel = np.ones(width) / width
    return np.convolve(x, kernel, mode="same")


def pava(y, increasing=True):
    """동일 가중치 isotonic regression (pool-adjacent-violators)."""
    z = np.asarray(y, float) * (1.0 if increasing else -1.0)
    levels, weights, starts, ends = [], [], [], []
    for i, value in enumerate(z):
        levels.append(float(value)); weights.append(1.0)
        starts.append(i); ends.append(i)
        while len(levels) >= 2 and levels[-2] > levels[-1]:
            w = weights[-2] + weights[-1]
            v = (levels[-2] * weights[-2] + levels[-1] * weights[-1]) / w
            levels[-2:] = [v]; weights[-2:] = [w]
            ends[-2:] = [ends[-1]]; starts.pop()
    out = np.empty(len(z))
    for value, start, end in zip(levels, starts, ends):
        out[start:end + 1] = value
    return out * (1.0 if increasing else -1.0)


def payload_direction(mass, com, times):
    empty, loaded = Nominal(), Nominal()
    loaded.set_payload(mass, com)
    out = []
    for t in times:
        q, qd, qdd = reference(float(t))
        out.append(loaded.inverse_dynamics(q, qd, qdd)
                   - empty.inverse_dynamics(q, qd, qdd))
    return np.asarray(out)


def estimate_alpha(empty_log, loaded_log, mass, com):
    t = loaded_log["t"]
    delta = loaded_log["tau_s"] - empty_log["tau_s"]
    direction = payload_direction(mass, com, t)
    raw = np.sum(delta * direction, axis=1) / (
        np.sum(direction * direction, axis=1) + 1e-9
    )
    baseline = np.median(raw[(t >= 1.20) & (t < 1.35)])
    full = np.median(raw[(t >= 2.00) & (t < 3.30)])
    scale = max(full - baseline, 0.1)
    alpha = moving_average((raw - baseline) / scale)
    return np.clip(alpha, -0.15, 1.15)


def crossing_time(t_ms, curve, level, rising=True):
    idx = np.flatnonzero(curve >= level) if rising else np.flatnonzero(curve <= level)
    return float(t_ms[idx[0]]) if len(idx) else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=12)
    ap.add_argument("--seed", type=int, default=20261130)
    ap.add_argument("--out", default="results/load_transfer_identification")
    args = ap.parse_args()
    out = HERE / args.out
    out.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(args.seed)
    masses = (2.0, 4.0, 6.0, 8.0)
    records = []
    print(f"collecting {args.repeats} paired sets x {len(masses)} masses", flush=True)

    for repeat in range(args.repeats):
        sensor_seed = args.seed + repeat
        uncertainty = {
            "seed": sensor_seed,
            "noise_scale": float(rng.uniform(0.8, 1.6)),
            "offset_scale": float(rng.uniform(0.8, 1.3)),
            "drift_scale": float(rng.uniform(0.8, 1.8)),
            "friction_scale": float(rng.uniform(0.9, 1.1)),
            "table_time_scale": float(rng.uniform(0.75, 1.45)),
            "table_damping_scale": float(rng.uniform(0.8, 1.3)),
            "suction_time_scale": float(rng.uniform(0.7, 1.6)),
            "place_height_offset_m": float(rng.uniform(-0.001, 0.001)),
        }
        _, empty_log = run_episode(
            hover_servo.Controller(), box_mass=0.0,
            plant_uncertainty=uncertainty,
        )
        for mass in masses:
            com = tuple(rng.uniform([-0.04, -0.04, -0.025], [0.04, 0.04, 0.025]))
            _, loaded_log = run_episode(
                hover_servo.Controller(), box_mass=mass, box_com=com,
                plant_uncertainty=uncertainty,
            )
            records.append({
                "repeat": repeat, "mass": mass, "com": com,
                "t": loaded_log["t"],
                "alpha": estimate_alpha(empty_log, loaded_log, mass, com),
            })
        print(f"  paired set {repeat + 1}/{args.repeats}", flush=True)

    t = records[0]["t"]
    all_alpha = np.stack([r["alpha"] for r in records])
    pick_mask = (t >= T_GRASP) & (t <= T_GRASP + 0.120)
    place_mask = (t >= T_RELEASE - 0.120) & (t <= T_RELEASE)
    pick_ms = 1e3 * (t[pick_mask] - T_GRASP)
    place_ms = 1e3 * (t[place_mask] - T_RELEASE)

    def summarize(mask, increasing):
        x = all_alpha[:, mask]
        med = np.median(x, axis=0)
        fit = np.clip(pava(med, increasing=increasing), 0.0, 1.0)
        if increasing:
            fit[0] = 0.0; fit[-1] = 1.0
        else:
            fit[0] = 1.0; fit[-1] = 0.0
        return fit, np.quantile(x, 0.10, axis=0), np.quantile(x, 0.90, axis=0)

    pick, pick_lo, pick_hi = summarize(pick_mask, True)
    place, place_lo, place_hi = summarize(place_mask, False)

    # 5 ms 간격의 작은 테이블로 저장한다. 실행 시 np.interp로 연속 보간한다.
    pick_idx = np.unique(np.r_[np.arange(0, len(pick_ms), 5), len(pick_ms) - 1])
    place_idx = np.unique(np.r_[np.arange(0, len(place_ms), 5), len(place_ms) - 1])
    profile = {
        "source": "paired JTS projection; no simulator-internal load fraction",
        "repeats": args.repeats,
        "loaded_episodes": len(records),
        "masses_kg": list(masses),
        "pick_t_ms": np.round(pick_ms[pick_idx], 3).tolist(),
        "pick_alpha": np.round(pick[pick_idx], 6).tolist(),
        "place_t_ms": np.round(place_ms[place_idx], 3).tolist(),
        "place_alpha": np.round(place[place_idx], 6).tolist(),
    }
    (out / "learned_profile.json").write_text(
        json.dumps(profile, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    with (out / "identified_curves.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["transition", "relative_time_ms", "alpha_fit", "alpha_p10", "alpha_p90"])
        for name, tm, fit, lo, hi in (
            ("pick", pick_ms, pick, pick_lo, pick_hi),
            ("place", place_ms, place, place_lo, place_hi),
        ):
            w.writerows(zip([name] * len(tm), tm, fit, lo, hi))

    smooth_pick = np.array([hover_servo._smoothstep(x / 25.0) for x in pick_ms])
    release_lead_ms = 42.0 + 1.4 * np.mean(masses)
    smooth_place = np.array([
        1.0 - hover_servo._smoothstep((x + release_lead_ms) / release_lead_ms)
        for x in place_ms
    ])
    rmse_pick = float(np.sqrt(np.mean((pick - smooth_pick) ** 2)))
    rmse_place = float(np.sqrt(np.mean((place - smooth_place) ** 2)))
    metrics = {
        "pick_t10_ms": crossing_time(pick_ms, pick, 0.10, True),
        "pick_t50_ms": crossing_time(pick_ms, pick, 0.50, True),
        "pick_t90_ms": crossing_time(pick_ms, pick, 0.90, True),
        "place_t90_ms": crossing_time(place_ms, place, 0.90, False),
        "place_t50_ms": crossing_time(place_ms, place, 0.50, False),
        "place_t10_ms": crossing_time(place_ms, place, 0.10, False),
        "smoothstep_rmse_pick": rmse_pick,
        "smoothstep_rmse_place": rmse_place,
    }
    (out / "metrics.json").write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.6))
    panels = [
        (axes[0], pick_ms, pick, pick_lo, pick_hi, smooth_pick, "Pick transition"),
        (axes[1], place_ms, place, place_lo, place_hi, smooth_place, "Place transition"),
    ]
    for ax, tm, fit, lo, hi, scheduled, title in panels:
        ax.fill_between(tm, lo, hi, color="#9FB3C8", alpha=0.28, label="10–90% data range")
        ax.plot(tm, fit, color="#E76F51", lw=2.6, label="data-fitted profile")
        ax.plot(tm, scheduled, color="#2A9D8F", lw=2.0, ls="--", label="current smoothstep")
        ax.set(title=title, xlabel="time relative to event [ms]", ylabel="load fraction alpha")
        ax.set_ylim(-0.05, 1.05); ax.grid(alpha=0.25); ax.legend(fontsize=8)
    fig.suptitle(f"Load-transfer identification from {len(records)} loaded episodes")
    fig.tight_layout(); fig.savefig(out / "identified_profile_comparison.png", dpi=180)
    plt.close(fig)
    print(json.dumps(metrics, ensure_ascii=False, indent=2))
    print("saved", out)


if __name__ == "__main__":
    main()
