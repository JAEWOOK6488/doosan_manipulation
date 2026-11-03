"""데이터 기반 하중 전달 곡선 + 기존 1.25 mm 호버."""
import numpy as np

from data_driven_servo import Controller as DataDrivenController
from scheduled_servo import _smoothstep
from sim.trajectory import T_RELEASE

MODE = "servo"
HOVER_M = 0.00125


class Controller(DataDrivenController):
    MODE = MODE

    def compute(self, t, th, thd, tau_s, grip, q_d, qd_d, qdd_d):
        pos, vel, vel_ff, tau_ff = super().compute(
            t, th, thd, tau_s, grip, q_d, qd_d, qdd_d
        )
        if t < T_RELEASE - 0.20:
            height = 0.0
        elif t < T_RELEASE - 0.05:
            height = HOVER_M * _smoothstep((t - (T_RELEASE - 0.20)) / 0.15)
        elif t < T_RELEASE + 0.10:
            height = HOVER_M
        elif t < T_RELEASE + 0.25:
            height = HOVER_M * (1.0 - _smoothstep((t - (T_RELEASE + 0.10)) / 0.15))
        else:
            height = 0.0
        if height:
            dq = np.linalg.pinv(self.nom.jacobian(q_d), rcond=1e-4) @ np.array([0.0, 0.0, height])
            pos = pos + dq
        return pos, vel, vel_ff, tau_ff
