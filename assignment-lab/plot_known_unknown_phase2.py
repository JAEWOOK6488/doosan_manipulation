"""같은 1,000조건에서 payload 정보 유무의 효과를 그린다."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parent / "results" / "unknown_payload"
data = json.loads((ROOT / "phase2_known_vs_unknown_summary.json").read_text())
phases = ["startup_or_approach", "pickup_transition", "carry", "place_transition", "return"]
labels = ["Approach", "Pickup", "Carry", "Place", "Return"]
known = [data["by_phase"][p]["known_pct"] for p in phases]
unknown = [data["by_phase"][p]["unknown_pct"] for p in phases]

fig, axes = plt.subplots(1, 3, figsize=(13, 4.4), gridspec_kw={"width_ratios": [1, 2.25, 1]})
overall = [data["known_pct"], data["unknown_pct"]]
axes[0].bar(["Known\nmass + COM", "Unknown\nmass + COM"], overall,
            color=["#2563eb", "#f97316"])
axes[0].set_ylim(0, 100)
axes[0].set_ylabel("Collision discrimination [%]")
axes[0].set_title("Overall 1,000 cases")
for i, value in enumerate(overall):
    axes[0].text(i, value + 2, f"{value:.1f}%", ha="center", fontweight="bold")

x = range(len(phases)); width = .37
axes[1].bar([i - width/2 for i in x], known, width, label="Known mass + COM", color="#2563eb")
axes[1].bar([i + width/2 for i in x], unknown, width, label="Unknown mass + COM", color="#f97316")
axes[1].set_xticks(list(x), labels); axes[1].set_ylim(0, 105)
axes[1].set_title("Success by motion state"); axes[1].legend(fontsize=8)

false_cases = [data["known_reference_false_alarm_cases"], data["unknown_reference_false_alarm_cases"]]
axes[2].bar(["Known", "Unknown"], false_cases, color=["#2563eb", "#f97316"])
axes[2].set_ylim(0, max(false_cases) * 1.35)
axes[2].set_title("No-collision alarm cases")
for i, value in enumerate(false_cases):
    axes[2].text(i, value + 2, str(value), ha="center", fontweight="bold")
for ax in axes:
    ax.grid(axis="y", alpha=.25)
fig.suptitle("Value of payload information — same 1,000 randomized cases",
             fontsize=12, fontweight="bold")
fig.tight_layout()
out = ROOT / "phase2_known_vs_unknown.png"
fig.savefig(out, dpi=180, bbox_inches="tight")
print(out)
