"""제어기에 제공하는 명목 모델 (강체, 그리퍼 포함, 박스는 기본 없음).

모든 함수는 링크 각도 q 기준이다. 학생 제어기는 self.nom 으로 사용한다.
"""
import numpy as np
import mujoco
from . import model, config as C


class Nominal:
    def __init__(self):
        self.m, self.d = model.build()
        self._M = np.zeros((6, 6))
        self.tau_max, self.qd_max = C.TAU_MAX.copy(), C.QD_MAX.copy()
        self.B, self.K, self.elastic = C.B.copy(), C.K.copy(), C.ELASTIC.copy()

    def set_payload(self, mass, com=(0., 0., 0.)):
        """추정한 박스 질량[kg]·COM[m](박스 중심 기준)을 명목 모델에 반영. mass=0 이면 제거."""
        model.set_box(self.m, mass, com)

    def _update(self, q, qd):
        self.d.qpos[:], self.d.qvel[:] = q, qd
        mujoco.mj_kinematics(self.m, self.d)
        mujoco.mj_comPos(self.m, self.d)
        mujoco.mj_comVel(self.m, self.d)

    def inverse_dynamics(self, q, qd, qdd):
        """tau = M(q) qdd + C(q,qd) qd + g(q)  [N m]"""
        self.d.qpos[:], self.d.qvel[:], self.d.qacc[:] = q, qd, qdd
        mujoco.mj_inverse(self.m, self.d)
        return self.d.qfrc_inverse.copy()

    def mass_matrix(self, q):
        self._update(q, np.zeros(6))
        mujoco.mj_crb(self.m, self.d)
        mujoco.mj_fullM(self.m, self.d, self._M)
        return self._M.copy()

    def bias(self, q, qd):
        """코리올리 + 중력 h(q, qd) [N m]"""
        self._update(q, qd)
        out = np.empty(6)
        mujoco.mj_rne(self.m, self.d, 0, out)
        return out

    def gravity(self, q):
        return self.bias(q, np.zeros(6))

    def tcp_position(self, q):
        """TCP(흡착면) 월드 위치 [m]"""
        self.d.qpos[:] = q
        mujoco.mj_kinematics(self.m, self.d)
        return self.d.site_xpos[self.m.site("tcp").id].copy()

    def jacobian(self, q):
        """TCP 선속도 자코비안 (3x6)"""
        self.d.qpos[:] = q
        mujoco.mj_kinematics(self.m, self.d)
        mujoco.mj_comPos(self.m, self.d)
        jp = np.zeros((3, 6))
        mujoco.mj_jacSite(self.m, self.d, jp, None, self.m.site("tcp").id)
        return jp
