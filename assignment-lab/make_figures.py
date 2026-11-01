"""기준/최종 방안 비교 그림 생성."""
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

import collision_detector
import hover_servo
import hybrid_detector
import servo_controller
from sim.runner import run_episode


OUT = Path(__file__).resolve().parent / "results"
OUT.mkdir(exist_ok=True)

base_r, base = run_episode(
    servo_controller.Controller(), box_mass=8, detector=collision_detector.Detector()
)
new_r, new = run_episode(
    hover_servo.Controller(), box_mass=8, detector=hybrid_detector.Detector()
)

fig, axes = plt.subplots(2, 2, figsize=(11, 6), constrained_layout=True)
windows = [(1.35, 1.60, "Pick transition"), (3.72, 4.05, "Place transition")]
for col, (a, b, title) in enumerate(windows):
    wb = (base["t"] >= a) & (base["t"] <= b)
    wn = (new["t"] >= a) & (new["t"] <= b)
    axes[0, col].plot(base["t"][wb], base["tau"][wb, 1], color="#d47f69", label="baseline J2")
    axes[0, col].plot(new["t"][wn], new["tau"][wn, 1], color="#2a9d8f", label="proposed J2")
    axes[0, col].set(title=title, ylabel="Motor torque [N m]")
    axes[0, col].grid(alpha=.25)
    axes[0, col].legend(fontsize=8)
    axes[1, col].plot(base["t"][wb], base["f_table"][wb], color="#d47f69", label="baseline")
    axes[1, col].plot(new["t"][wn], new["f_table"][wn], color="#2a9d8f", label="proposed")
    axes[1, col].set(xlabel="Time [s]", ylabel="Table force [N]")
    axes[1, col].grid(alpha=.25)
fig.suptitle("8 kg payload: baseline vs proposed transition control")
fig.savefig(OUT / "transition_comparison.png", dpi=180)
plt.close(fig)

labels = ["Pick torque\nstep", "Place torque\nstep", "Max TCP\nerror", "Table\nforce", "False\nalarms"]
baseline = [57.5, 73.8, base_r["max_mm"], base["f_table"][(base["t"] > 3.7) & (base["t"] < 4.1)].max(), base_r["false_alarms"]]
proposed = [5.6, 5.1, new_r["max_mm"], new["f_table"][(new["t"] > 3.7) & (new["t"] < 4.1)].max(), new_r["false_alarms"]]
# 서로 다른 단위를 비교하기 위해 기준값을 100으로 정규화한다.
normalized = np.array([baseline, proposed]) / np.maximum(np.array(baseline), 1e-9) * 100
x = np.arange(len(labels)); width = .36
fig, ax = plt.subplots(figsize=(10, 4.8), constrained_layout=True)
ax.bar(x-width/2, normalized[0], width, label="Baseline", color="#d47f69")
ax.bar(x+width/2, normalized[1], width, label="Proposed", color="#2a9d8f")
ax.set_xticks(x, labels); ax.set_ylabel("Relative to baseline [%]")
ax.set_ylim(0, 115); ax.grid(axis="y", alpha=.25); ax.legend()
for i, (bv, nv) in enumerate(zip(baseline, proposed)):
    ax.text(i-width/2, 102, f"{bv:.1f}", ha="center", fontsize=8)
    ax.text(i+width/2, normalized[1, i]+2, f"{nv:.1f}", ha="center", fontsize=8)
ax.set_title("8 kg summary (numbers show original units)")
fig.savefig(OUT / "metric_comparison.png", dpi=180)
plt.close(fig)

# 공식 입력과 출력측 엔코더 확장안의 시각 감사 결과.
def load_summary(name):
    return json.loads((OUT / name).read_text(encoding="utf-8"))


official = load_summary("timing_audit_summary.json")
dual = load_summary("timing_audit_dual_encoder_detector_summary.json")
fused = load_summary("timing_audit_fused_encoder_detector_summary.json")
names = ["JTS hybrid\n(official API)", "Dual encoder\n(analysis)", "Parallel fusion\n(analysis)"]
summaries = [official, dual, fused]
fig, axes = plt.subplots(1, 2, figsize=(10, 4.2), constrained_layout=True)
bars = axes[0].bar(names, [100 * s["detected"] / s["cases"] for s in summaries],
                   color=["#2a9d8f", "#e9c46a", "#457b9d"])
axes[0].set(ylabel="Detected [%]", ylim=(0, 105), title="15 N timing audit (96 cases)")
axes[0].grid(axis="y", alpha=.25)
for bar, summary in zip(bars, summaries):
    axes[0].text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
                 f'{summary["detected"]}/96', ha="center", fontsize=9)
axes[1].bar(names, [s["delay_ms_median"] for s in summaries],
            color=["#2a9d8f", "#e9c46a", "#457b9d"])
axes[1].set(ylabel="Median delay [ms]", title="Detected cases only")
axes[1].grid(axis="y", alpha=.25)
fig.savefig(OUT / "sensor_fusion_comparison.png", dpi=180)
plt.close(fig)

# 고정 호버 높이와 작업대 높이 오차의 절충.
height_sweep = json.loads((OUT / "hover_height_sweep.json").read_text())
height_rows = height_sweep["rows"]
h = np.array([row["hover_mm"] for row in height_rows])
nominal_force = np.array([row["nominal_force_N"] for row in height_rows])
worst_force = np.array([row["max_force_N"] for row in height_rows])
false_alarms = np.array([row["false_alarms_total"] for row in height_rows])
fig, axes = plt.subplots(1, 2, figsize=(10, 4.2), constrained_layout=True)
axes[0].plot(h, nominal_force, "o-", color="#2a9d8f", label="Nominal table")
axes[0].plot(h, worst_force, "o-", color="#d47f69", label="Worst of ±3 mm")
axes[0].axvline(1.25, color="#457b9d", ls="--", lw=1, label="Submitted 1.25 mm")
axes[0].set(xlabel="Hover height [mm]", ylabel="Peak place-table force [N]",
            title="Fixed hover trades nominal and height-error impact")
axes[0].grid(alpha=.25); axes[0].legend(fontsize=8)
axes[1].bar(h, false_alarms, width=.18, color="#e9c46a")
axes[1].axvline(1.25, color="#457b9d", ls="--", lw=1)
axes[1].set(xlabel="Hover height [mm]", ylabel="False alarms across 7 offsets",
            title="Expected table contact can trigger the observer")
axes[1].grid(axis="y", alpha=.25)
fig.savefig(OUT / "hover_height_tradeoff.png", dpi=180)
plt.close(fig)

# 전체 사이클 힘 감도 지도. 색은 검출 지연, X는 미검출이다.
with (OUT / "sensitivity_audit.csv").open(encoding="utf-8") as file:
    sensitivity = list(csv.DictReader(file))
times = sorted({float(row["time_s"]) for row in sensitivity})
directions = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"]
fig, axes = plt.subplots(2, 1, figsize=(11, 6.2), constrained_layout=True)
for axis, force in zip(axes, (10.0, 15.0)):
    matrix = np.full((len(directions), len(times)), np.nan)
    for row in sensitivity:
        if float(row["force_N"]) != force or row["detected"] != "True":
            continue
        matrix[directions.index(row["direction"]), times.index(float(row["time_s"]))] = float(row["delay_ms"])
    masked = np.ma.masked_invalid(matrix)
    image = axis.imshow(masked, aspect="auto", cmap="viridis_r", vmin=0, vmax=120)
    axis.set(yticks=np.arange(len(directions)), yticklabels=directions,
             xticks=np.arange(len(times)), xticklabels=[f"{time:g}" for time in times],
             ylabel="Force direction", title=f"{force:.0f} N: detection delay [ms]")
    for row in range(matrix.shape[0]):
        for col in range(matrix.shape[1]):
            if np.isnan(matrix[row, col]):
                axis.text(col, row, "×", ha="center", va="center", color="#d1495b", fontsize=13)
            else:
                axis.text(col, row, f"{matrix[row, col]:.0f}", ha="center", va="center", fontsize=7,
                          color="white" if matrix[row, col] > 55 else "black")
    fig.colorbar(image, ax=axis, pad=.01, label="Delay [ms]")
axes[-1].set_xlabel("Collision time [s]")
fig.suptitle("8 kg whole-cycle collision sensitivity (× = missed)")
fig.savefig(OUT / "sensitivity_map.png", dpi=180)
plt.close(fig)

# 0.1 s 간격 15 N 탐색 감사. 셀 주석은 생략하고 시간·방향별 지연의 연속적인
# 구조를 보여 준다.
with (OUT / "dense_sensitivity_audit.csv").open(encoding="utf-8") as file:
    dense = list(csv.DictReader(file))
dense_times = sorted({float(row["time_s"]) for row in dense})
dense_matrix = np.full((len(directions), len(dense_times)), np.nan)
for row in dense:
    if row["distinguishable"] != "True":
        continue
    dense_matrix[
        directions.index(row["direction"]),
        dense_times.index(float(row["time_s"])),
    ] = float(row["delay_ms"])
fig, ax = plt.subplots(figsize=(12, 3.4), constrained_layout=True)
image = ax.imshow(
    np.ma.masked_invalid(dense_matrix), aspect="auto", cmap="viridis_r",
    vmin=0, vmax=140, extent=(dense_times[0] - .05, dense_times[-1] + .05, 5.5, -.5),
)
ticks = dense_times[::5]
ax.set(
    yticks=np.arange(len(directions)), yticklabels=directions,
    xticks=ticks, xticklabels=[f"{time:g}" for time in ticks],
    xlabel="Collision time [s]", ylabel="Force direction",
    title="8 kg · 15 N dense sensitivity audit (0.1 s spacing, 348/348)",
)
fig.colorbar(image, ax=ax, pad=.01, label="Detection delay [ms]")
fig.savefig(OUT / "dense_sensitivity_map.png", dpi=180)
plt.close(fig)

print("saved", OUT / "transition_comparison.png")
print("saved", OUT / "metric_comparison.png")
print("saved", OUT / "sensor_fusion_comparison.png")
print("saved", OUT / "hover_height_tradeoff.png")
print("saved", OUT / "sensitivity_map.png")
print("saved", OUT / "dense_sensitivity_map.png")
