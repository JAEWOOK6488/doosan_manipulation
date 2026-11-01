"""가상 로봇 (학생 수정 금지). 물리 계산은 모두 MuJoCo 가 한다.

J1~J3: 모터(관성 B, 마찰) --[감속기 강성 K, 감쇠 D]--> 링크   — 같은 축 힌지 2개 (sim/model.py)
       엔코더 = 모터 각도 θ,  관절 토크센서 = 스프링 토크 K(θ−q) + D(θd−qd)
J4~J6: 강체 (모터 관성 B 를 armature 로)
       관절 토크센서 = 링크로 전달되는 토크 = 모터 토크 − 마찰 − B·θdd
관절 토크센서(JTS)는 6축 모두, 마찰·모터 관성보다 링크 쪽에 있다 (M 시리즈와 같은 배치).
JTS 측정값 = 실제 토크 → 센서 내부 1차 필터 → 1 주기 지연 + 영점 오프셋 + 드리프트 + 백색 잡음 (sim/config.py)
박스:  픽업 작업대 위의 자유 물체. grip 신호 순간 흡착(weld 구속)이 붙고, 로봇이 들어 올리는 동안
       작업대가 받치던 힘이 접촉을 통해 로봇으로 넘어온다. 놓을 때는 적재 작업대 위에서 흡착을 끊는다.
적분: MuJoCo RK4, 1 ms.  쿨롱 마찰(tanh 평활화)만 매 주기 외력으로 넣는다.
"""
import numpy as np
import mujoco
from . import model, config as C, trajectory as T
from .nominal import Nominal


def _box_pose(nom, t):
    """기준 궤적 t 시각에 흡착된 박스 중심의 월드 위치·자세(쿼터니언)·회전행렬"""
    nom.d.qpos[:] = T.reference(t)[0]
    mujoco.mj_kinematics(nom.m, nom.d)
    b = nom.m.body("link_6").id
    R = nom.d.xmat[b].reshape(3, 3).copy()
    return nom.d.xpos[b] + R @ C.BOX_POS, nom.d.xquat[b].copy(), R


class Plant:
    def __init__(self, q0, box_mass=0.0, box_com=(0., 0., 0.), pick_offset=None,
                 uncertainty=None):
        """pick_offset: 박스·픽업 작업대를 기준 위치에서 옮겨 놓을 거리 [m] (위치 교시 오류 흉내).
        주면 툴과 박스·작업대의 접촉을 켠다 (툴이 박스를 칠 수 있음). 흡착 중에는 툴 접촉을 끈다."""
        uncertainty = {} if uncertainty is None else dict(uncertainty)
        self.viscous = C.VISCOUS * float(uncertainty.get("friction_scale", 1.0))
        self.coulomb = C.COULOMB * float(uncertainty.get("friction_scale", 1.0))
        self.jts_noise = C.JTS_NOISE * float(uncertainty.get("noise_scale", 1.0))
        self.jts_offset = C.JTS_OFFSET * float(uncertainty.get("offset_scale", 1.0))
        self.jts_drift = C.JTS_DRIFT * float(uncertainty.get("drift_scale", 1.0))
        self.jts_seed = int(uncertainty.get("seed", C.JTS_SEED))
        self.place_height_offset = float(
            uncertainty.get("place_height_offset_m", 0.0)
        )
        nom = Nominal()
        spec = model.build(elastic=True, spec_only=True)
        half = np.array(C.BOX_SIZE) / 2
        off = np.zeros(3) if pick_offset is None else np.asarray(pick_offset, float)
        for name, t in (("table_pick", T.T_GRASP), ("table_place", T.T_RELEASE)):
            p, _, R = _box_pose(nom, t)
            if name == "table_pick":
                p = p + off
            top = p[2] - np.abs(R[2]) @ half                               # 작업대 윗면 = 박스 밑면
            if name == "table_place":
                top += self.place_height_offset
            hz = (top - C.FLOOR_Z) / 2
            g = spec.worldbody.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, size=[C.TABLE_XY[0] / 2, C.TABLE_XY[1] / 2, hz],
                                        pos=[p[0], p[1], top - hz], rgba=[.55, .5, .45, 1])
            g.contype, g.conaffinity, g.solref, g.solimp = 2, 2, C.TABLE_SOLREF, C.TABLE_SOLIMP
        self.has_box = box_mass > 0
        if self.has_box:
            p, q, _ = _box_pose(nom, T.T_GRASP)
            p = p + off
            bx = spec.worldbody.add_body(name="box_free", pos=p, quat=q)
            bx.add_freejoint(name="box_free")
            g = bx.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=half, density=0, rgba=[.8, .55, .25, .9])
            g.contype, g.conaffinity = 2, 2
            a, w, h = C.BOX_SIZE
            bx.mass, bx.ipos, bx.explicitinertial = box_mass, list(box_com), True
            bx.inertia = box_mass / 12 * np.array([w**2 + h**2, a**2 + h**2, a**2 + w**2])
            eq = spec.add_equality(type=mujoco.mjtEq.mjEQ_WELD, name="suction", name1="link_6", name2="box_free",
                                   objtype=mujoco.mjtObj.mjOBJ_BODY, active=False)
            eq.solref = C.SUCTION_SOLREF
        if pick_offset is not None:                                # 툴이 박스·작업대에 부딪힐 수 있게
            tg = spec.geom("tool_geom")
            tg.contype, tg.conaffinity = 2, 2
        self.m = spec.compile()
        self.tool_g = self.m.geom("tool_geom").id
        self.tool_contact = pick_offset is not None
        self.d = mujoco.MjData(self.m)

        jid = lambda n: self.m.joint(n).id
        motor = [f"motor_{k + 1}" if C.ELASTIC[k] else f"joint_{k + 1}" for k in range(6)]
        defl = [f"joint_{k + 1}" for k in range(6) if C.ELASTIC[k]]
        self.th_q = self.m.jnt_qposadr[[jid(n) for n in motor]]
        self.th_v = self.m.jnt_dofadr[[jid(n) for n in motor]]
        self.ph_q = self.m.jnt_qposadr[[jid(n) for n in defl]]
        self.ph_v = self.m.jnt_dofadr[[jid(n) for n in defl]]
        self.E = C.ELASTIC.nonzero()[0]
        self.Rg = (~C.ELASTIC).nonzero()[0]
        self.tau_link = np.zeros(6)                                # 강체 관절의 링크 전달 토크 (토크센서값)
        self.rng = np.random.default_rng(self.jts_seed)
        self.jts_f = None                                          # JTS 내부 필터 상태
        self.jts_out = None                                        # 1 주기 지연된 출력
        self.k = 0
        q0 = np.asarray(q0, float)
        phi0 = -nom.gravity(q0)[self.E] / C.K[self.E]              # 정지 상태 처짐 (중력과 스프링 평형)
        self.d.qpos[self.th_q] = q0
        self.d.qpos[self.th_q[self.E]] -= phi0
        self.d.qpos[self.ph_q] = phi0
        if self.has_box:                                           # 박스를 작업대에 안착 (로봇은 고정)
            rq, rv = np.r_[self.th_q, self.ph_q], np.r_[self.th_v, self.ph_v]
            hold = self.d.qpos[rq].copy()
            for _ in range(int(0.5 / C.DT)):
                mujoco.mj_step(self.m, self.d)
                self.d.qpos[rq], self.d.qvel[rv] = hold, 0.
            self.d.time = 0.
        mujoco.mj_forward(self.m, self.d)

    def grip(self, on):
        """흡착 ON/OFF. ON 순간의 플랜지-박스 상대 자세로 결합한다."""
        if not self.has_box:
            return
        if self.tool_contact and on:                               # 흡착한 뒤에는 툴 접촉을 끈다 (놓을 때 겹쳐 있으면 튀므로 다시 켜지 않음)
            self.m.geom_contype[self.tool_g] = self.m.geom_conaffinity[self.tool_g] = 0
        if on:
            f, b = self.m.body("link_6").id, self.m.body("box_free").id
            Rf = self.d.xmat[f].reshape(3, 3)
            qinv, rel = np.zeros(4), np.zeros(4)
            mujoco.mju_negQuat(qinv, self.d.xquat[f])
            mujoco.mju_mulQuat(rel, qinv, self.d.xquat[b])
            self.m.eq_data[0, 0:3] = 0.                            # anchor
            self.m.eq_data[0, 3:6] = Rf.T @ (self.d.xpos[b] - self.d.xpos[f])
            self.m.eq_data[0, 6:10] = rel
            self.m.eq_data[0, 10] = 1.                             # torquescale
        self.d.eq_active[0] = int(on)

    def push(self, f):
        """플랜지(link_6)에 외력 f [N] (월드 좌표) — 충돌 흉내"""
        self.d.xfrc_applied[self.m.body("link_6").id, :3] = f

    def tool_force(self):
        """툴이 박스·작업대와 부딪히는 접촉력 크기 합 [N] (기록용)"""
        f, fn = np.zeros(6), 0.
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            if self.tool_g in (c.geom1, c.geom2):
                mujoco.mj_contactForce(self.m, self.d, i, f)
                fn += np.linalg.norm(f[:3])
        return fn

    def table_force(self):
        """작업대가 박스를 받치는 수직력 합 [N] (기록용)"""
        tables = (self.m.geom("table_pick").id, self.m.geom("table_place").id)
        f, fn = np.zeros(6), 0.
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            if c.geom1 in tables or c.geom2 in tables:
                mujoco.mj_contactForce(self.m, self.d, i, f)
                fn += f[0]
        return fn

    def step(self, tau):
        tau = np.clip(tau, -C.TAU_MAX, C.TAU_MAX)
        thd = self.d.qvel[self.th_v].copy()
        coulomb = self.coulomb * np.tanh(thd / C.FRICTION_EPS)
        self.d.qfrc_applied[:] = 0.
        self.d.qfrc_applied[self.th_v] = tau - coulomb
        mujoco.mj_step(self.m, self.d)
        thd1 = self.d.qvel[self.th_v]
        self.tau_link = tau - coulomb - self.viscous * (thd + thd1) / 2 - C.B * (thd1 - thd) / C.DT

    def sense(self):
        """(엔코더 각도[rad], 관절 토크센서[N m] 6축)"""
        th = self.d.qpos[self.th_q]
        return np.round(th / C.ENCODER_RES) * C.ENCODER_RES, self._jts()

    def true_torque(self):
        """센서를 거치지 않은 실제 관절 토크 (평가·검증용, 제어기에는 주지 않음)"""
        tau = self.tau_link.copy()
        tau[self.E] = -(C.K[self.E] * self.d.qpos[self.ph_q] + C.D[self.E] * self.d.qvel[self.ph_v])
        return tau

    def _jts(self):
        tau = self.true_torque()
        a = 1 - np.exp(-2 * np.pi * C.JTS_CUTOFF * C.DT)
        self.jts_f = tau if self.jts_f is None else self.jts_f + a * (tau - self.jts_f)
        prev = self.jts_f.copy() if self.jts_out is None else self.jts_out
        self.jts_out = self.jts_f.copy()
        t = self.k * C.DT
        self.k += 1
        return prev + self.jts_offset + self.jts_drift * t + self.jts_noise * self.rng.standard_normal(6)

    @property
    def q(self):
        """링크 각도 = 모터 각도 + 처짐"""
        q = self.d.qpos[self.th_q].copy()
        q[self.E] += self.d.qpos[self.ph_q]
        return q
