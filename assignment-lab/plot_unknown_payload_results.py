"""미지 payload 1,000회 감사의 발표용 요약 그림."""
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent
CSV = ROOT / "results" / "unknown_payload" / "unknown_payload_1000.csv"
OUT = ROOT / "results" / "unknown_payload" / "unknown_payload_summary.png"
with CSV.open(encoding="utf-8") as file:
    rows = list(csv.DictReader(file))

phases = ["startup_or_approach", "pickup_transition", "carry", "place_transition", "return"]
labels = ["Approach", "Pickup", "Carry", "Place", "Return"]
rates = []
for phase in phases:
    group = [r for r in rows if r["phase"] == phase]
    rates.append(100 * sum(r["distinguished"] == "True" for r in group) / len(group))

true_mass = np.array([float(r["mass_kg"]) for r in rows])
estimated_mass = np.array([float(r["estimated_mass_kg"]) for r in rows])
fig, axes = plt.subplots(1, 2, figsize=(11, 4.4))
colors = ["#3b82f6", "#ef4444", "#f59e0b", "#ef4444", "#3b82f6"]
axes[0].bar(labels, rates, color=colors)
axes[0].axhline(90, color="#64748b", ls="--", lw=1, label="90% reference")
axes[0].set_ylim(0, 105); axes[0].set_ylabel("Detection success [%]")
axes[0].set_title("Unknown-payload collision discrimination")
for i, value in enumerate(rates):
    axes[0].text(i, value + 2, f"{value:.1f}%", ha="center", fontsize=9)
axes[0].grid(axis="y", alpha=.25); axes[0].legend(fontsize=8)
axes[1].scatter(true_mass, estimated_mass, s=9, alpha=.28, color="#2563eb")
axes[1].plot([0, 10], [0, 10], color="#111827", lw=1, ls="--")
axes[1].set(xlabel="True mass [kg]", ylabel="Estimated mass [kg]",
            title="JTS-only mass identification")
axes[1].grid(alpha=.25)
fig.suptitle("1,000 randomized cases — true mass and COM hidden from controller/detector",
             fontsize=12, fontweight="bold")
fig.tight_layout()
fig.savefig(OUT, dpi=180, bbox_inches="tight")
print(OUT)
