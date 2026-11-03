"""질량·COM 비공개 1차 기준안과 2차 개선안의 1,000회 비교 그림."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parent / "results" / "unknown_payload"
data = json.loads((ROOT / "phase1_vs_unknown_phase2_summary.json").read_text())
phases = ["startup_or_approach", "pickup_transition", "carry", "place_transition", "return"]
labels = ["Approach", "Pickup", "Carry", "Place", "Return"]
p1 = [data["by_phase"][p]["phase1_unknown_payload_pct"] for p in phases]
p2 = [data["by_phase"][p]["phase2_unknown_payload_pct"] for p in phases]

fig, axes = plt.subplots(1, 3, figsize=(13, 4.4), gridspec_kw={"width_ratios": [1, 2.25, 1]})
axes[0].bar(["Phase 1\nunknown", "Phase 2\nunknown"],
            [data["phase1_unknown_payload_pct"], data["phase2_unknown_payload_pct"]],
            color=["#94a3b8", "#f97316"])
axes[0].set_ylim(0, 100); axes[0].set_ylabel("Collision discrimination [%]")
axes[0].set_title("Overall 1,000 cases")
for i, value in enumerate([data["phase1_unknown_payload_pct"], data["phase2_unknown_payload_pct"]]):
    axes[0].text(i, value + 2, f"{value:.1f}%", ha="center", fontweight="bold")

x = range(len(phases)); width = .37
axes[1].bar([i - width/2 for i in x], p1, width, label="Phase 1 unknown", color="#94a3b8")
axes[1].bar([i + width/2 for i in x], p2, width, label="Phase 2 unknown", color="#f97316")
axes[1].set_xticks(list(x), labels); axes[1].set_ylim(0, 105)
axes[1].set_title("Success by motion state"); axes[1].legend(fontsize=8)

false_cases = [data["phase1_reference_false_alarm_cases"], data["phase2_unknown_reference_false_alarm_cases"]]
axes[2].bar(["Phase 1", "Phase 2"], false_cases, color=["#94a3b8", "#14b8a6"])
axes[2].set_ylim(0, 300); axes[2].set_title("No-collision alarm cases")
for i, value in enumerate(false_cases):
    axes[2].text(i, value + 7, str(value), ha="center", fontweight="bold")
for ax in axes:
    ax.grid(axis="y", alpha=.25)
fig.suptitle("True mass and COM hidden from both methods — same 1,000 randomized cases",
             fontsize=12, fontweight="bold")
fig.tight_layout()
out = ROOT / "phase1_vs_unknown_phase2.png"
fig.savefig(out, dpi=180, bbox_inches="tight")
print(out)
