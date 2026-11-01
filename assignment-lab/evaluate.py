"""표준 평가: python evaluate.py [--ctrl servo_controller] [--detector collision_detector]

발표·보고에 쓰는 숫자를 같은 조건으로 한 번에 만든다 (노트북 약 1~2 분). 결과는 화면과 evaluation.txt 에 저장.
  1) 제어 성능  : 박스 0 / 4 / 8 kg, 충돌 없음. TCP 오차, 토크 포화, 집기·놓기 때 토크 계단(1 ms 최대 변화),
                  놓을 때 작업대가 받는 최대 힘(박스가 부딪히는 세기)
     오감지      : 같은 실행에서 집기(1.35~1.6 s)·놓기(3.7~4.1 s)·그 외로 나눠 센다
  2) 충돌 감지   : 박스 0 / 8 kg, 집기 직후 1.42 s · 운반 중 3.0 s · 놓기 직전 3.86 s 에
                   플랜지에 수평 충돌 10 / 15 / 40 / 100 N
최종 평가(15주차)는 이 조건 외에 다른 박스 무게·무게중심, 충돌 시각·크기·방향으로도 한다.
"""
import argparse
import importlib
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim.runner import run_episode          # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--ctrl", default="servo_controller", help="제어기 파일 이름")
ap.add_argument("--detector", default="collision_detector", help="충돌 감지기 파일 이름")
ap.add_argument("--boxes", default="0,4,8", help="추종·오감지 평가 박스 질량 [kg]")
ap.add_argument("--col-boxes", default="0,8", help="충돌 감지 평가 박스 질량 [kg]")
ap.add_argument("--forces", default="10,15,40,100", help="충돌 외력 크기 [N] (수평 y 방향)")
ap.add_argument("--col-times", default="1.42,3.0,3.86", help="충돌 시각 [s] (집기 직후, 운반 중, 놓기 직전)")
ap.add_argument("--out", default="evaluation.txt")
a = ap.parse_args()
fl = lambda s: [float(x) for x in s.split(",")]

C, D = importlib.import_module(a.ctrl), importlib.import_module(a.detector)
lines = [f"평가: 제어기 {a.ctrl} ({getattr(C.Controller, 'MODE', 'torque')} 방식), 감지기 {a.detector} "
         f"(SOURCE={getattr(D.Detector, 'SOURCE', '?')})"]
t0 = time.time()


def run(box, col=None, with_log=False):
    print(f"  실행 중: 박스 {box:g} kg" + ("" if col is None else f", {col[0]:g} s {col[1][1]:g} N 충돌"), flush=True)
    r, log = run_episode(C.Controller(), box_mass=box, detector=D.Detector(), collision=col)
    return (r, log) if with_log else r


lines += ["", "[1] 제어 성능·오감지 (충돌 없음)  — 토크 계단 = 집기(1.35~1.6 s)·놓기(3.7~4.1 s) 중 1 ms 토크 변화 최대 [N m],",
          "    작업대 힘 = 놓을 때 작업대가 박스를 받치는 힘 최대 [N] (박스 무게보다 크면 부딪히며 내려놓은 것)",
          f"{'박스':>6} | {'RMS mm':>7} | {'최대 mm':>7} | {'포화 %':>6} | 토크 계단 집기/놓기 | 작업대 힘 | {'계산 us':>7} | 오감지 집기/놓기/그 외 | 상태"]
for box in fl(a.boxes):
    r, log = run(box, with_log=True)
    t, ft = log["t"], r["false_alarm_times"]
    pick = sum(1.35 <= x < 1.6 for x in ft); place = sum(3.7 <= x < 4.1 for x in ft)
    step = np.r_[0, np.abs(np.diff(log["tau"], axis=0)).max(1)]
    wp, wl = (t >= 1.35) & (t < 1.6), (t >= 3.7) & (t < 4.1)
    lines.append(f"{box:>4g}kg | {r['rms_mm']:>7.2f} | {r['max_mm']:>7.2f} | {r['saturation_pct']:>6.1f} | "
                 f"{step[wp].max():>8.1f} / {step[wl].max():>6.1f} | {log['f_table'][wl].max():>6.0f} N | {r['ctrl_time_us']:>7.0f} | "
                 f"{pick:>3} / {place:>3} / {len(ft) - pick - place:>3} 회      | " + ("발산" if r["diverged"] else "정상"))

forces = fl(a.forces)
lines += ["", "[2] 충돌 감지 지연 ms (수평 충돌)  — '놓침' = 미감지, '경보 중' = 충돌 전부터 경보라 판정 불가,",
          "    '구분 불가' = 충돌이 없어도 그 시각(±20 ms)에 경보가 나므로 충돌을 감지했다고 볼 수 없음",
          f"{'박스':>6} {'시각':>6} | " + " | ".join(f"{f:>6g} N" for f in forces) + " | 오감지 | 감지한 최소 충돌력"]
ref = {}
for box, ct in [(b, c) for b in fl(a.col_boxes) for c in fl(a.col_times)]:
    if box not in ref:                                    # 같은 박스, 충돌 없는 실행의 경보 (구분 불가 판정용)
        ref[box] = run(box, with_log=True)[1]["detect"]
    cells, fa, smallest = [], 0, None
    for f in forces:
        r = run(box, (ct, [0.0, f, 0.0]))
        fa = max(fa, r["false_alarms"])
        k = None if r["detect_delay_ms"] is None else int(round((ct + r["detect_delay_ms"] / 1e3) / 1e-3))
        if r["alarm_before_collision"]:
            cells.append("경보 중")
        elif r["detect_delay_ms"] is None:
            cells.append("놓침")
        elif ref[box][max(0, k - 20):k + 21].any():
            cells.append("구분 불가")
        else:
            cells.append(f"{r['detect_delay_ms']:.0f}")
            smallest = f if smallest is None else min(smallest, f)
    lines.append(f"{box:>4g}kg {ct:>5.2f}s | " + " | ".join(f"{c:>8}" for c in cells) + f" | {fa:>4}회 | "
                 + ("—" if smallest is None else f"{smallest:g} N"))

lines += ["", f"(계산 {time.time() - t0:.0f} 초)"]
print("\n".join(lines))
with open(a.out, "w", encoding="utf-8") as f:
    f.write("\n".join(lines) + "\n")
print(f"저장: {a.out}")
