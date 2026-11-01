"""1 사이클 실행과 평가."""
import time
import numpy as np
from . import config as C, trajectory as T
from .plant import Plant
from .nominal import Nominal
from .drive import Drive


COLLISION_RAMP, COLLISION_HOLD = 0.02, 0.08            # 충돌 외력: 20 ms 증가 → 80 ms 유지 → 20 ms 감소


def _with_box(mass, com):
    """박스(질량·무게중심·관성)를 정확히 넣은 명목 모델"""
    n = Nominal()
    n.set_payload(mass, com)
    return n


def run_episode(controller, box_mass=6.0, box_com=(0.02, -0.01, 0.03), detector=None,
                collision=None, pick_offset=None, plant_uncertainty=None):
    """controller.compute(t, th, thd, tau_s, grip, q_d, qd_d, qdd_d)
      controller.MODE == "servo"  : (pos, vel, vel_ff, tau_ff) 반환 → 1 주기 뒤 구동부가 토크 생성
      controller.MODE == "torque" : 관절 토크 tau[6] 반환 → 1 주기 뒤 적용
    detector  : 충돌 감지기 (update(t, th, thd, tau_m, tau_s, grip) -> (r, detected)), 없으면 생략
    collision : (시각 [s], 힘 [N] 3개) — 플랜지에 외력을 가해 충돌을 흉내낸다, 없으면 충돌 없음
    pick_offset : 박스·픽업 작업대 위치 오차 [m] 3개 — 툴이 박스를 칠 수 있다 (위치 교시 오류)
    반환: 지표 dict + 시계열 log."""
    servo = getattr(controller, "MODE", "torque") == "servo"
    nom, drive = Nominal(), Drive()
    controller.nom, controller.nom_box = nom, _with_box(box_mass, box_com)
    controller.box_mass, controller.box_com = float(box_mass), tuple(box_com)   # 과제 가정: 박스 값은 안다
    if servo:
        controller.drive = drive
    if hasattr(controller, "reset"):
        controller.reset()
    if detector is not None:
        detector.nom, detector.nom_box, detector.ctrl = Nominal(), _with_box(box_mass, box_com), controller
        detector.box_mass, detector.box_com = float(box_mass), tuple(box_com)
        if hasattr(detector, "reset"):
            detector.reset()
    q0 = T.reference(0.0)[0]
    plant = Plant(q0, box_mass, box_com, pick_offset, uncertainty=plant_uncertainty)
    pend = np.array([q0, np.zeros(6), np.zeros(6), np.zeros(6)]) if servo else np.zeros(6)
    n = int(round(T.T_END / C.DT))
    log = {k: np.zeros(n) for k in ("t", "e_mm", "t_ctrl")}
    log.update({k: np.zeros((n, 6)) for k in (
        "q", "q_d", "th", "thd", "tau", "tau_m", "tau_s"
    )})
    log["f_table"], log["f_tool"] = np.zeros(n), np.zeros(n)
    log["r"], log["detect"] = np.zeros((n, 6)), np.zeros(n, bool)
    log["f_residual"] = np.zeros((n, 3))
    log["f_ext"] = np.zeros(n)
    tau_m = np.zeros(6)
    log["qpos"] = np.zeros((n, plant.m.nq))
    ref_tcp = np.array([nom.tcp_position(T.reference(k * C.DT)[0]) for k in range(n)])
    th_prev, held, sat, dtau, diverged = None, False, 0, 0.0, False
    for k in range(n):
        t = k * C.DT
        grip = T.T_GRASP <= t < T.T_RELEASE
        if grip != held:                    # 흡착 ON/OFF — 하중은 작업대 접촉을 통해 서서히 넘어온다
            plant.grip(grip)
            held = grip
        f = np.zeros(3)
        if collision is not None:           # 사다리꼴 외력 (충돌)
            a = t - collision[0]
            w = min(a / COLLISION_RAMP, 1.0, (2 * COLLISION_RAMP + COLLISION_HOLD - a) / COLLISION_RAMP)
            f = max(w, 0.0) * np.asarray(collision[1], float)
        plant.push(f)
        th, tau_s = plant.sense()
        thd = np.zeros(6) if th_prev is None else (th - th_prev) / C.DT
        th_prev = th
        log["th"][k], log["thd"][k] = th, thd
        log["tau_m"][k], log["tau_s"][k] = tau_m, tau_s
        if detector is not None:
            # 분석 전용 감지기가 명시적으로 요청할 때만 시뮬레이터 내부의
            # 출력측 엔코더 상당값을 노출한다. 표준 감지기에는 주지 않는다.
            if getattr(detector, "USES_LINK_ENCODER", False):
                detector.q_link = plant.q.copy()
            r_ext, hit = detector.update(t, th.copy(), thd, tau_m.copy(), tau_s.copy(), int(grip))
            log["r"][k], log["detect"][k] = r_ext, hit
            if hasattr(detector, "force_residual"):
                log["f_residual"][k] = detector.force_residual
        q_d, qd_d, qdd_d = T.reference(t)
        c0 = time.perf_counter()
        out = controller.compute(t, th.copy(), thd, tau_s.copy(), int(grip), q_d, qd_d, qdd_d)
        log["t_ctrl"][k] = time.perf_counter() - c0
        out = np.asarray(out, float)
        if out.shape != ((4, 6) if servo else (6,)) or not np.all(np.isfinite(out)):
            raise ValueError(f"t={t:.3f}s: 제어기 출력은 유한한 값 "
                             f"{'(pos, vel, vel_ff, tau_ff) 각 6개' if servo else '6개'}여야 합니다 (받은 값: {out})")
        if servo:                           # 지령은 한 주기 늦게 구동부에 도착, 구동부는 현재 엔코더로 토크 계산
            tau, pend = drive.torque(pend, th, thd), out
        else:                               # 토크는 한 주기 늦게 적용
            tau, pend = pend, out
        tau = np.clip(tau, -C.TAU_MAX, C.TAU_MAX)
        sat += np.any(np.abs(tau) >= 0.999 * C.TAU_MAX)
        if k:
            dtau += np.abs(tau - log["tau"][k - 1]).mean()
        plant.step(tau)
        tau_m = tau
        log["f_ext"][k] = np.linalg.norm(f)
        e = 1e3 * np.linalg.norm(nom.tcp_position(plant.q) - ref_tcp[k])
        log["t"][k], log["e_mm"][k], log["q"][k], log["q_d"][k], log["tau"][k] = t, e, plant.q, q_d, tau
        log["f_table"][k], log["qpos"][k] = plant.table_force(), plant.d.qpos
        if plant.tool_contact:
            log["f_tool"][k] = plant.tool_force()
        if not np.isfinite(e) or np.abs(plant.d.qvel[plant.th_v]).max() > 10:   # 발산 (감지 후 정지 등으로 기준에서 멀어지는 것은 발산 아님)
            log["e_mm"][k:] = 500.0
            diverged = True
            n = k + 1
            break
    e = log["e_mm"]
    log["plant"] = plant
    res = dict(rms_mm=float(np.sqrt(np.mean(e**2))), max_mm=float(e.max()), diverged=diverged,
               saturation_pct=float(100 * sat / n), torque_jitter=float(dtau / n),
               ctrl_time_us=float(1e6 * np.median(log["t_ctrl"][:n])))
    if detector is not None:
        res.update(collision_eval(log, n, collision))
    return res, log


def collision_eval(log, n, collision):
    """오감지: 충돌 구간 밖의 감지 횟수(감지 시작 기준)·시각, 감지 지연: 충돌 시작 → 첫 감지
    툴이 박스·작업대를 친 접촉(픽업 위치 오류)도 진짜 충돌로 본다."""
    det, t = log["detect"][:n], log["t"][:n]
    rise = np.flatnonzero(det & ~np.r_[False, det[:-1]])
    true_win = np.zeros(n, bool)
    if collision is not None:
        true_win = (t >= collision[0]) & (t < collision[0] + 2 * COLLISION_RAMP + COLLISION_HOLD + 0.2)
    hit_tool = log["f_tool"][:n] > 1.0
    contact_delay = None
    if hit_tool.any():
        k0 = int(np.argmax(hit_tool))
        after = np.flatnonzero(det[k0:])
        contact_delay = float(1e3 * after[0] * (t[1] - t[0])) if len(after) else None
        last = np.flatnonzero(hit_tool)[-1]
        true_win[k0:min(n, last + int(0.3 / (t[1] - t[0])))] = True
    false_rise = [k for k in rise if not true_win[k]]
    delay, masked = None, False
    if collision is not None:
        k0 = int(np.searchsorted(t, collision[0]))
        masked = bool(k0 > 0 and det[k0 - 1])          # 충돌 전부터 이미 경보 중이면 감지 지연을 잴 수 없음
        hit = [k for k in rise if true_win[k]]
        if hit and not masked:
            delay = float(1e3 * (t[hit[0]] - collision[0]))
    return dict(false_alarms=len(false_rise), false_alarm_times=[float(t[k]) for k in false_rise],
                detect_delay_ms=delay, alarm_before_collision=masked, r_peak=np.abs(log["r"][:n]).max(0),
                tool_contact_delay_ms=contact_delay)


def plot(log, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    det = log["detect"].any() or np.abs(log["r"]).any()
    nr = 4 if det else 3
    fig, ax = plt.subplots(nr, 1, figsize=(9, 8 + 2.2 * det), sharex=True,
                           gridspec_kw=dict(height_ratios=[3, 3, 1.4, 2.2][:nr]))
    ax[0].plot(log["t"], log["e_mm"], lw=1.2)
    ax[0].axvspan(T.T_GRASP, T.T_RELEASE, color="orange", alpha=0.15, label="holding box")
    ax[0].set_ylabel("TCP error [mm]"); ax[0].legend(loc="upper right"); ax[0].grid(alpha=.3)
    for j in range(6):
        ax[1].plot(log["t"], log["tau"][:, j], lw=1, label=f"J{j+1}")
    ax[1].set_ylabel("joint torque [N m]"); ax[1].legend(ncol=6, fontsize=8); ax[1].grid(alpha=.3)
    ax[2].plot(log["t"], log["f_table"], lw=1.2, color="saddlebrown", label="table")
    if log["f_tool"].any():
        ax[2].plot(log["t"], log["f_tool"], lw=1.2, color="crimson", label="tool contact")
        ax[2].legend(fontsize=8)
    ax[2].set_ylabel("contact force [N]"); ax[2].grid(alpha=.3)
    if det:
        for j in range(6):
            ax[3].plot(log["t"], log["r"][:, j], lw=1, label=f"J{j+1}")
        on = log["detect"]
        ax[3].fill_between(log["t"], 0, 1, where=on, transform=ax[3].get_xaxis_transform(), color="red", alpha=0.18, label="detected")
        if log["f_ext"].any():
            ax[3].fill_between(log["t"], 0, 1, where=log["f_ext"] > 0, transform=ax[3].get_xaxis_transform(),
                               color="black", alpha=0.12, label="collision")
        ax[3].set_ylabel("observer r [N m]"); ax[3].legend(ncol=8, fontsize=7); ax[3].grid(alpha=.3)
    ax[-1].set_xlabel("time [s]")
    fig.tight_layout(); fig.savefig(path, dpi=120); plt.close(fig)
