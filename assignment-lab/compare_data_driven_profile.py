"""현재 smoothstep과 데이터 기반 하중 스케줄의 제어 성능을 비교한다."""
import json
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import data_driven_hover_servo
import hover_servo
from sim.runner import run_episode


HERE = Path(__file__).resolve().parent
OUT = HERE / "results/load_transfer_identification"


def metrics(controller, mass, seed):
    result, log = run_episode(
        controller, box_mass=mass,
        plant_uncertainty={"seed": seed, "noise_scale": 1.2},
    )
    t = log["t"]
    step = np.r_[0.0, np.abs(np.diff(log["tau"], axis=0)).max(axis=1)]
    pick = (t >= 1.35) & (t < 1.60)
    place = (t >= 3.70) & (t < 4.10)
    return {
        "rms_mm": result["rms_mm"],
        "max_mm": result["max_mm"],
        "pick_torque_step_Nm": float(step[pick].max()),
        "place_torque_step_Nm": float(step[place].max()),
        "table_force_N": float(log["f_table"][place].max()),
    }


rows = []
for mass in (2.0, 4.0, 6.0, 8.0):
    for repeat in range(5):
        seed = 20261250 + repeat
        for name, cls in (
            ("current_smoothstep", hover_servo.Controller),
            ("data_fitted", data_driven_hover_servo.Controller),
        ):
            rows.append({"profile": name, "mass_kg": mass, "repeat": repeat,
                         **metrics(cls(), mass, seed)})

summary = {}
for name in ("current_smoothstep", "data_fitted"):
    selected = [r for r in rows if r["profile"] == name]
    summary[name] = {
        key: float(np.mean([r[key] for r in selected]))
        for key in ("rms_mm", "max_mm", "pick_torque_step_Nm",
                    "place_torque_step_Nm", "table_force_N")
    }
OUT.mkdir(parents=True, exist_ok=True)
(OUT / "controller_comparison.json").write_text(
    json.dumps({"summary": summary, "runs": rows}, ensure_ascii=False, indent=2),
    encoding="utf-8",
)
with (OUT / "controller_comparison.csv").open("w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=rows[0].keys())
    writer.writeheader(); writer.writerows(rows)

changes = {
    key: 100.0 * (summary["data_fitted"][key] / summary["current_smoothstep"][key] - 1.0)
    for key in summary["current_smoothstep"]
}

fig, axes = plt.subplots(1, 3, figsize=(12, 4.2))
names = ["current\nsmoothstep", "data-fitted\nprofile"]
colors = ["#2A9D8F", "#E76F51"]
for ax, keys, labels, title in (
    (axes[0], ["pick_torque_step_Nm", "place_torque_step_Nm"], ["pick", "place"], "Max 1 ms torque step [N m]"),
    (axes[1], ["table_force_N"], ["place table"], "Peak table force [N]"),
    (axes[2], ["rms_mm", "max_mm"], ["RMS", "max"], "TCP tracking error [mm]"),
):
    width = 0.34
    x = np.arange(len(keys))
    for i, profile in enumerate(("current_smoothstep", "data_fitted")):
        vals = [summary[profile][k] for k in keys]
        bars = ax.bar(x + (i - 0.5) * width, vals, width, label=names[i], color=colors[i])
        ax.bar_label(bars, fmt="%.1f", fontsize=8, padding=2)
    ax.set_xticks(x, labels); ax.set_title(title); ax.grid(axis="y", alpha=0.25)
axes[0].legend(fontsize=8)
fig.suptitle("Directly replaying the identified load profile does not improve control")
fig.tight_layout(); fig.savefig(OUT / "controller_comparison.png", dpi=180)
plt.close(fig)

identified = json.loads((OUT / "metrics.json").read_text(encoding="utf-8"))
report = f"""# 데이터 기반 하중 전달률 식별 실험

## 목적

현재 25 ms smoothstep 대신 반복 관절토크 데이터에서 실제 하중 전달률
`alpha(t)`를 근사해 제어 성능이 좋아지는지 확인했다.

## 데이터와 식별 방법

- 질량 2/4/6/8 kg, 각 12조건: 하중 실행 48회
- 같은 센서 시드의 빈손 기준 실행 12회
- COM, 센서 잡음·오프셋·드리프트, 마찰, 작업대·흡착 특성, 놓기 높이 변화
- 빈손과 하중 실행의 JTS 차이를 명목 payload 토크 방향에 최소제곱 투영
- 시뮬레이터 내부의 실제 하중률은 식별 입력으로 사용하지 않음

식별된 대표 시각은 다음과 같다.

| 천이 | 10% | 50% | 90% |
|---|---:|---:|---:|
| 집기, grip ON 이후 | {identified['pick_t10_ms']:.0f} ms | {identified['pick_t50_ms']:.0f} ms | {identified['pick_t90_ms']:.0f} ms |
| 놓기, grip OFF 기준 | {identified['place_t10_ms']:.0f} ms | {identified['place_t50_ms']:.0f} ms | {identified['place_t90_ms']:.0f} ms |

집기는 현재 25 ms smoothstep보다 느려 90% 전달까지 약 53 ms가 걸렸다. 놓기는
grip OFF 약 35 ms 전부터 하중이 감소하기 시작했다.

## 제어기 직접 적용 결과

4개 질량 × 5회, 총 20개 쌍대 조건에서 기존 `hover_servo`와 식별 곡선을
그대로 사용하는 `data_driven_hover_servo`를 비교했다.

| 지표 | 현재 smoothstep | 데이터 곡선 직접 적용 | 변화 |
|---|---:|---:|---:|
| TCP RMS 오차 | {summary['current_smoothstep']['rms_mm']:.2f} mm | {summary['data_fitted']['rms_mm']:.2f} mm | {changes['rms_mm']:+.1f}% |
| TCP 최대 오차 | {summary['current_smoothstep']['max_mm']:.2f} mm | {summary['data_fitted']['max_mm']:.2f} mm | {changes['max_mm']:+.1f}% |
| 집기 토크 계단 | {summary['current_smoothstep']['pick_torque_step_Nm']:.2f} N·m | {summary['data_fitted']['pick_torque_step_Nm']:.2f} N·m | {changes['pick_torque_step_Nm']:+.1f}% |
| 놓기 토크 계단 | {summary['current_smoothstep']['place_torque_step_Nm']:.2f} N·m | {summary['data_fitted']['place_torque_step_Nm']:.2f} N·m | {changes['place_torque_step_Nm']:+.1f}% |
| 작업대 최대 힘 | {summary['current_smoothstep']['table_force_N']:.1f} N | {summary['data_fitted']['table_force_N']:.1f} N | {changes['table_force_N']:+.1f}% |

데이터 곡선을 그대로 적용하는 방안은 채택하지 않는다. 측정 곡선은 제어 명령에
대한 결과이며, 특히 놓기에서는 이미 작업대 접촉이 시작된 뒤의 반응을 포함한다.
이 곡선을 그대로 재생하면 하중 보상을 늦게 줄여 작업대를 더 오래 누르게 된다.
현재 스케줄은 실제 반응을 그대로 복사하기보다 접촉을 미리 예상하는 선행 보상이라
작업대 힘이 더 작았다.

## 다음 단계

데이터는 버리지 않고 다음 방식으로 활용하는 것이 적절하다.

1. 식별 곡선 자체를 명령으로 사용하지 않고 접촉 시작·종료 시각을 추정한다.
2. 추정된 접촉 시각보다 앞서 보상을 시작하는 예측 스케줄을 최적화한다.
3. 학습 데이터와 독립된 질량·COM·접촉 조건에서 토크 계단, 작업대 힘, 오경보를 함께 검증한다.
4. 실제 M1013에서는 외부 F/T 센서를 제어 입력이 아닌 정답값 측정에만 사용한다.
"""
(OUT / "REPORT.md").write_text(report, encoding="utf-8")
print(json.dumps(summary, ensure_ascii=False, indent=2))
print(json.dumps({"percent_changes": changes}, ensure_ascii=False, indent=2))
