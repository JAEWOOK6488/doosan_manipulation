"""독립 holdout 1,000조건의 기존 미지 2차와 3차를 비교한다."""
from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parent / "results"
OLD = ROOT / "unknown_payload_holdout"
NEW = ROOT / "unknown_payload_v3"


def read(path):
    with path.open(encoding="utf-8") as file:
        return list(csv.DictReader(file))


old = read(OLD / "unknown_payload_1000.csv")
new = read(NEW / "unknown_payload_1000.csv")
assert len(old) == len(new) == 1000
assert [r["case"] for r in old] == [r["case"] for r in new]

success = lambda r: r["distinguished"] == "True"
phases = ["startup_or_approach", "pickup_transition", "carry", "place_transition", "return"]
phase_labels = ["Approach", "Pickup", "Carry", "Place", "Return"]
force_bins = ["15_20N", "20_30N", "30_45N", "45_60N"]


def rate(rows, key=None, value=None):
    group = rows if key is None else [r for r in rows if r[key] == value]
    return 100 * sum(success(r) for r in group) / len(group)


summary = {
    "holdout_seed": 20261201,
    "cases": 1000,
    "training": {"normal_only_cases": 300, "seed": 73129},
    "development": {"cases": 100, "seed": 777},
    "phase2_unknown_pct": rate(old),
    "phase3_unknown_pct": rate(new),
    "gain_percentage_points": rate(new) - rate(old),
    "phase2_reference_false_alarm_cases": sum(int(r["reference_false_alarms"]) > 0 for r in old),
    "phase3_reference_false_alarm_cases": sum(int(r["reference_false_alarms"]) > 0 for r in new),
    "phase3_missed": sum(r["failure_type"] == "missed" for r in new),
    "phase3_alarm_before": sum(r["failure_type"] == "alarm_before" for r in new),
    "phase2_only_success": sum(success(a) and not success(b) for a, b in zip(old, new)),
    "phase3_only_success": sum(not success(a) and success(b) for a, b in zip(old, new)),
    "by_phase": {
        p: {"phase2_pct": rate(old, "phase", p), "phase3_pct": rate(new, "phase", p)}
        for p in phases
    },
    "by_force": {
        b: {"phase2_pct": rate(old, "force_bin", b), "phase3_pct": rate(new, "force_bin", b)}
        for b in force_bins
    },
}
(NEW / "phase2_vs_phase3_holdout_summary.json").write_text(
    json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
)

fig, axes = plt.subplots(1, 4, figsize=(16, 4.4), gridspec_kw={"width_ratios": [1, 2.2, 1.8, 1]})
old_color, new_color = "#94a3b8", "#2563eb"
overall = [summary["phase2_unknown_pct"], summary["phase3_unknown_pct"]]
axes[0].bar(["Phase 2", "Phase 3"], overall, color=[old_color, new_color])
axes[0].set_title("Overall holdout")
axes[0].set_ylabel("Collision discrimination [%]")
for i, v in enumerate(overall): axes[0].text(i, v + 2, f"{v:.1f}%", ha="center", fontweight="bold")

x = range(len(phases)); width = .37
p2 = [summary["by_phase"][p]["phase2_pct"] for p in phases]
p3 = [summary["by_phase"][p]["phase3_pct"] for p in phases]
axes[1].bar([i-width/2 for i in x], p2, width, color=old_color, label="Phase 2")
axes[1].bar([i+width/2 for i in x], p3, width, color=new_color, label="Phase 3")
axes[1].set_xticks(list(x), phase_labels); axes[1].set_title("By motion state"); axes[1].legend()

x = range(len(force_bins))
f2 = [summary["by_force"][b]["phase2_pct"] for b in force_bins]
f3 = [summary["by_force"][b]["phase3_pct"] for b in force_bins]
axes[2].bar([i-width/2 for i in x], f2, width, color=old_color)
axes[2].bar([i+width/2 for i in x], f3, width, color=new_color)
axes[2].set_xticks(list(x), [b.replace("_", "-") for b in force_bins]); axes[2].set_title("By collision force")

fa = [summary["phase2_reference_false_alarm_cases"], summary["phase3_reference_false_alarm_cases"]]
axes[3].bar(["Phase 2", "Phase 3"], fa, color=[old_color, new_color])
axes[3].set_title("No-collision alarm cases")
for i, v in enumerate(fa): axes[3].text(i, v + 2, str(v), ha="center", fontweight="bold")
for ax in axes:
    ax.set_ylim(0, 105); ax.grid(axis="y", alpha=.25)
fig.suptitle("Unknown mass and COM — independent 1,000-case holdout", fontweight="bold")
fig.tight_layout()
out = NEW / "phase2_vs_phase3_holdout.png"
fig.savefig(out, dpi=180, bbox_inches="tight")
print(json.dumps(summary, indent=2, ensure_ascii=False))
print(out)
