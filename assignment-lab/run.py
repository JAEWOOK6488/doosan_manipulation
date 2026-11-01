"""실행: python run.py [--ctrl servo_controller|torque_controller] [--box 6] [--collision 3.0] [--plot result.png] [--snapshot] [--viewer]"""
import argparse
import numpy as np
import importlib
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim.runner import run_episode, plot          # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--ctrl", default="servo_controller",
                help="제어기 파일 이름 (servo_controller = 서보 방식, torque_controller = 토크 방식)")
ap.add_argument("--box", type=float, default=6.0, help="박스 질량 [kg] (0~8, 제어기·감지기에 알려 줌)")
ap.add_argument("--detector", default="collision_detector", help="충돌 감지기 파일 이름 (none 이면 생략)")
ap.add_argument("--collision", type=float, default=None, help="이 시각 [s] 에 플랜지에 외력을 가함 (충돌 흉내)")
ap.add_argument("--col-force", type=float, default=100.0, help="충돌 외력 크기 [N] (수평 y 방향)")
ap.add_argument("--pick-offset", default=None, help="박스 위치 오차 dx,dy,dz [mm] (예: 0,0,30 = 30 mm 높게) — 툴이 박스를 칠 수 있음")
ap.add_argument("--plot", default="result.png", help="결과 그래프 파일")
ap.add_argument("--csv", default=None, help="시계열을 CSV 로 저장 (예: data.csv, 엑셀로 열 수 있음)")
ap.add_argument("--snapshot", action="store_true", help="주요 순간 3D 이미지 저장 (snapshot_*.png)")
ap.add_argument("--viewer", action="store_true", help="3D 창으로 재생 (모니터 필요)")
a = ap.parse_args()

ctrl = importlib.import_module(a.ctrl).Controller()
det = None if a.detector == "none" else importlib.import_module(a.detector).Detector()
col = None if a.collision is None else (a.collision, [0.0, a.col_force, 0.0])
po = None if a.pick_offset is None else [float(x) / 1000 for x in a.pick_offset.split(",")]
r, log = run_episode(ctrl, box_mass=a.box, detector=det, collision=col, pick_offset=po)
print(f"[{a.ctrl} / {getattr(ctrl, 'MODE', 'torque')} 방식] 박스 {a.box:.1f} kg | TCP 오차 RMS {r['rms_mm']:.2f} mm, 최대 {r['max_mm']:.2f} mm | "
      f"토크 포화 {r['saturation_pct']:.1f}% | 토크 떨림 {r['torque_jitter']:.2f} | "
      f"제어 계산 {r['ctrl_time_us']:.0f} us | {'발산!' if r['diverged'] else '정상'}")
if det is not None:
    fa = ", ".join(f"{x:.3f}s" for x in r["false_alarm_times"][:5]) + (" …" if r["false_alarms"] > 5 else "")
    msg = f"충돌 감지 | 오감지 {r['false_alarms']}회" + (f" ({fa})" if fa else "")
    if col is not None:
        msg += (" | 충돌 전부터 경보 중 (감지 판정 불가)" if r["alarm_before_collision"] else
                f" | 충돌 감지 지연 {r['detect_delay_ms']:.0f} ms" if r["detect_delay_ms"] is not None else " | 충돌 미감지!")
    print(msg)
if po is not None:
    msg = f"박스 위치 오차 {a.pick_offset} mm | 툴 접촉력 최대 {log['f_tool'].max():.0f} N"
    if log["f_tool"].max() > 1.0:
        hit_t = log["t"][np.argmax(log["f_tool"] > 1.0)]
        cd = r.get("tool_contact_delay_ms")
        msg += f" (접촉 시작 {hit_t:.3f}s" + ("" if det is None else
               f", 감지 지연 {cd:.0f} ms" if cd is not None else ", 미감지!") + ")"
    if a.box > 0:                                   # 흡착 직전까지 박스가 밀린 거리 (작업대 위 자유 물체)
        from sim.trajectory import T_GRASP
        bq = log["plant"].m.joint("box_free").qposadr[0]
        kg = min(int(T_GRASP / 1e-3) - 1, len(log["t"]) - 1)
        msg += f" | 박스 밀림 {1e3 * np.linalg.norm(log['qpos'][kg, bq:bq + 3] - log['qpos'][0, bq:bq + 3]):.0f} mm"
    print(msg)
plot(log, a.plot)
print(f"그래프 저장: {a.plot}")
if a.csv:                                           # 1 ms 시계열 (작업대 힘·충돌 외력은 분석용, 제어기에는 주지 않음)
    from sim.trajectory import T_GRASP, T_RELEASE
    n = int(np.count_nonzero(log["t"])) + 1         # 발산으로 일찍 끝난 경우 그 시점까지
    t = log["t"][:n]
    cols = {"t_s": t, "grip": ((t >= T_GRASP) & (t < T_RELEASE)).astype(int), "tcp_err_mm": log["e_mm"][:n]}
    for j in range(6):
        cols[f"theta_motor{j+1}_rad"] = log["th"][:n, j]
        cols[f"q_link{j+1}_rad"] = log["q"][:n, j]
        cols[f"theta_dot{j+1}_rad_s"] = log["thd"][:n, j]
        cols[f"q_d{j+1}_rad"] = log["q_d"][:n, j]
    for j in range(6):
        cols[f"tau_motor{j+1}_Nm"] = log["tau_m"][:n, j]
        cols[f"tau_jts{j+1}_Nm"] = log["tau_s"][:n, j]
    for j in range(6):
        cols[f"r{j+1}_Nm"] = log["r"][:n, j]
    cols.update(detect=log["detect"][:n].astype(int), f_table_N=log["f_table"][:n],
                f_collision_N=log["f_ext"][:n], f_tool_N=log["f_tool"][:n])
    with open(a.csv, "w", encoding="utf-8-sig", newline="") as f:
        f.write(",".join(cols) + "\n")
        np.savetxt(f, np.column_stack(list(cols.values())), delimiter=",", fmt="%.6g")
    print(f"CSV 저장: {a.csv} ({n} 행 × {len(cols)} 열, 1 ms 간격)")

if a.snapshot or a.viewer:
    import time
    import mujoco
    from sim import trajectory as T
    m = log["plant"].m                        # 가상 로봇 모델 (작업대·박스·관절 처짐 포함)
    d = mujoco.MjData(m)
    frames = [(0.0, "start"), (T.T_GRASP + 0.3, "lift"), (3.0, "move"), (T.T_RELEASE + 0.3, "place")]
    if a.snapshot:
        import matplotlib.pyplot as plt
        ren = mujoco.Renderer(m, 480, 640)
        cam = mujoco.MjvCamera(); cam.lookat[:] = [0.25, 0.1, 0.3]; cam.distance = 2.8; cam.azimuth = 150; cam.elevation = -20
        for t, name in frames:
            k = min(int(t / 1e-3), len(log["qpos"]) - 1)
            d.qpos[:] = log["qpos"][k]; mujoco.mj_forward(m, d)
            ren.update_scene(d, camera=cam); plt.imsave(f"snapshot_{name}.png", ren.render())
        print("3D 이미지 저장: snapshot_*.png")
    if a.viewer:
        import mujoco.viewer
        print("3D 창 재생 중 — 창을 닫으면 끝납니다")
        with mujoco.viewer.launch_passive(m, d) as v:
            while v.is_running():                     # 창을 닫을 때까지 반복 재생
                for k in range(0, len(log["qpos"]), 10):
                    if not v.is_running():
                        break
                    d.qpos[:] = log["qpos"][k]; mujoco.mj_forward(m, d); v.sync(); time.sleep(0.01)
                time.sleep(1)
