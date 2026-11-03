"""1차·2차 1,000조건 쌍대 비교 결과를 보고서용 그림으로 만든다."""
from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager


ROOT = Path(__file__).resolve().parent
DATA = ROOT / "results" / "phase2" / "paired_1000"
SUMMARY = DATA / "paired_comparison_summary.json"
ALL_CASES = DATA / "paired_comparison_all.csv"

V1 = "#73879C"
V2 = "#E76F51"
GRID = "#D9DEE3"


def setup_style() -> None:
    candidates = [
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJKkr-Regular.otf",
    ]
    for candidate in candidates:
        if Path(candidate).is_file():
            font_manager.fontManager.addfont(candidate)
            plt.rcParams["font.family"] = font_manager.FontProperties(fname=candidate).get_name()
            break
    plt.rcParams.update({
        "axes.unicode_minus": False,
        "axes.titleweight": "bold",
        "axes.edgecolor": "#AAB2BA",
        "axes.labelcolor": "#27313A",
        "text.color": "#27313A",
        "xtick.color": "#46525E",
        "ytick.color": "#46525E",
        "figure.facecolor": "white",
        "axes.facecolor": "white",
    })


def annotate_bars(ax, bars, suffix="%", decimals=1) -> None:
    for bar in bars:
        value = bar.get_height()
        ax.text(
            bar.get_x() + bar.get_width() / 2, value + max(ax.get_ylim()[1] * 0.012, 0.5),
            f"{value:.{decimals}f}{suffix}", ha="center", va="bottom", fontsize=9,
        )


def comparison_dashboard(summary: dict) -> Path:
    fig, axes = plt.subplots(2, 2, figsize=(14, 10), constrained_layout=True)
    fig.suptitle("1차·2차 충돌검출기 쌍대 비교 — 동일 1,000조건", fontsize=18, fontweight="bold")

    ax = axes[0, 0]
    labels = ["충돌 구분률\n(높을수록 좋음)", "무충돌 오경보율\n(낮을수록 좋음)"]
    v1 = [100 * summary["v1"]["distinguished_rate"], summary["v1"]["reference_false_alarm_cases"] / 10]
    v2 = [100 * summary["v2"]["distinguished_rate"], summary["v2"]["reference_false_alarm_cases"] / 10]
    x = np.arange(len(labels)); width = 0.34
    b1 = ax.bar(x - width / 2, v1, width, color=V1, label="1차 hybrid")
    b2 = ax.bar(x + width / 2, v2, width, color=V2, label="2차 robust v2")
    annotate_bars(ax, b1); annotate_bars(ax, b2)
    ax.set_xticks(x, labels); ax.set_ylim(0, 105); ax.set_ylabel("비율 [%]")
    ax.set_title("전체 성능"); ax.legend(frameon=False, loc="upper center")
    ax.grid(axis="y", color=GRID, linewidth=0.8); ax.set_axisbelow(True)
    ax.text(0.5, 0.50, "성공률 +16.8%p\n오경보 93.0% 감소", transform=ax.transAxes,
            ha="center", va="center", fontsize=11, fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.45", facecolor="#FFF4EC", edgecolor="#F0B89D"))

    ax = axes[0, 1]
    phase_keys = ["startup_or_approach", "pickup_transition", "carry", "place_transition", "return"]
    phase_labels = ["초기·접근", "집기 천이", "운반", "놓기 천이", "복귀"]
    v1_phase = [100 * summary["by_phase"][k]["v1_distinguished"] / summary["by_phase"][k]["cases"] for k in phase_keys]
    v2_phase = [100 * summary["by_phase"][k]["v2_distinguished"] / summary["by_phase"][k]["cases"] for k in phase_keys]
    x = np.arange(len(phase_keys)); width = 0.36
    b1 = ax.bar(x - width / 2, v1_phase, width, color=V1, label="1차")
    b2 = ax.bar(x + width / 2, v2_phase, width, color=V2, label="2차")
    annotate_bars(ax, b1); annotate_bars(ax, b2)
    ax.set_xticks(x, phase_labels); ax.set_ylim(0, 108); ax.set_ylabel("충돌 구분률 [%]")
    ax.set_title("동작 구간별 구분률"); ax.legend(frameon=False)
    ax.grid(axis="y", color=GRID, linewidth=0.8); ax.set_axisbelow(True)

    ax = axes[1, 0]
    force_keys = ["15_20N", "20_30N", "30_45N", "45_60N"]
    force_labels = ["15–20 N", "20–30 N", "30–45 N", "45–60 N"]
    v1_force = [100 * summary["by_force"][k]["v1_distinguished"] / summary["by_force"][k]["cases"] for k in force_keys]
    v2_force = [100 * summary["by_force"][k]["v2_distinguished"] / summary["by_force"][k]["cases"] for k in force_keys]
    x = np.arange(len(force_keys)); width = 0.36
    b1 = ax.bar(x - width / 2, v1_force, width, color=V1, label="1차")
    b2 = ax.bar(x + width / 2, v2_force, width, color=V2, label="2차")
    annotate_bars(ax, b1); annotate_bars(ax, b2)
    ax.set_xticks(x, force_labels); ax.set_ylim(0, 108); ax.set_ylabel("충돌 구분률 [%]")
    ax.set_title("충돌력별 구분률"); ax.legend(frameon=False)
    ax.grid(axis="y", color=GRID, linewidth=0.8); ax.set_axisbelow(True)

    ax = axes[1, 1]
    error_labels = ["완전 미검출", "충돌 전 경보", "기준 경보 중첩"]
    v1_error = [summary["v1"]["missed"], summary["v1"]["alarm_before"], summary["v1"]["reference_overlap"]]
    v2_error = [summary["v2"]["missed"], summary["v2"]["alarm_before"], summary["v2"]["reference_overlap"]]
    x = np.arange(len(error_labels)); width = 0.36
    b1 = ax.bar(x - width / 2, v1_error, width, color=V1, label="1차")
    b2 = ax.bar(x + width / 2, v2_error, width, color=V2, label="2차")
    annotate_bars(ax, b1, suffix="건", decimals=0); annotate_bars(ax, b2, suffix="건", decimals=0)
    ax.set_xticks(x, error_labels); ax.set_ylim(0, 195); ax.set_ylabel("실패 조건 수")
    ax.set_title("구분 실패 원인"); ax.legend(frameon=False)
    ax.grid(axis="y", color=GRID, linewidth=0.8); ax.set_axisbelow(True)
    ax.text(0.98, 0.96, "검출 지연 중앙값\n1차 29.9 ms · 2차 30.3 ms",
            transform=ax.transAxes, ha="right", va="top", fontsize=9,
            bbox=dict(boxstyle="round,pad=0.35", facecolor="#F5F7F8", edgecolor="#CAD1D7"))

    path = DATA / "paired_comparison_overview.png"
    fig.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(fig)
    return path


def failure_map() -> Path:
    with ALL_CASES.open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    time = np.array([float(row["collision_time_s"]) for row in rows])
    force = np.array([float(row["force_N"]) for row in rows])
    kind = np.array([row["v2_failure_type"] for row in rows])

    fig, ax = plt.subplots(figsize=(13, 6.5), constrained_layout=True)
    spans = [
        (0.10, 1.32, "초기·접근", "#EEF5FA"),
        (1.32, 1.70, "집기 천이", "#FFF1E6"),
        (1.70, 3.60, "운반", "#F2F7F0"),
        (3.60, 4.20, "놓기 천이", "#FDEDEC"),
        (4.20, 5.70, "복귀", "#F4F0F8"),
    ]
    for start, end, label, color in spans:
        ax.axvspan(start, end, color=color, alpha=0.9, zorder=0)
        ax.text((start + end) / 2, 61.2, label, ha="center", va="bottom", fontsize=10)
    success = kind == ""
    missed = kind == "missed"
    early = kind == "alarm_before"
    ax.scatter(time[success], force[success], s=15, color="#A8B3BC", alpha=0.38,
               label=f"성공 {success.sum()}건", edgecolors="none")
    ax.scatter(time[missed], force[missed], s=54, color="#D1495B", marker="x", linewidths=1.8,
               label=f"완전 미검출 {missed.sum()}건")
    ax.scatter(time[early], force[early], s=48, facecolors="none", edgecolors="#7B2CBF",
               linewidths=1.6, label=f"충돌 전 경보 {early.sum()}건")
    ax.set_xlim(0.05, 5.75); ax.set_ylim(13, 63)
    ax.set_xlabel("충돌 시각 [s]"); ax.set_ylabel("충돌력 [N]")
    ax.set_title("2차 검출기의 실패 위치 — 놓기 천이에 미검출 집중", fontsize=15, fontweight="bold")
    ax.grid(color=GRID, linewidth=0.7, alpha=0.8); ax.set_axisbelow(True)
    ax.legend(frameon=True, facecolor="white", edgecolor="#D0D5DA", loc="lower left", ncol=3)
    ax.annotate("놓기 천이 미검출 30건", xy=(3.77, 21), xytext=(4.42, 31),
                arrowprops=dict(arrowstyle="->", color="#D1495B", lw=1.4),
                color="#A22F40", fontweight="bold")
    path = DATA / "v2_failure_map.png"
    fig.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(fig)
    return path


def main() -> None:
    setup_style()
    summary = json.loads(SUMMARY.read_text(encoding="utf-8"))
    for path in (comparison_dashboard(summary), failure_map()):
        print(path)


if __name__ == "__main__":
    main()
