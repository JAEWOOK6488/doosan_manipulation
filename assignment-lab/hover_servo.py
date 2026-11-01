"""연속 하중 보상 + 놓기 직전 미세 호버 궤적.

작업대에 박스를 누른 채 release하는 대신 TCP를 수직으로 소량 들어 박스를
놓는다. 높이는 접촉 충격과 누름 힘의 절충 대상으로 실험한다.
"""
import numpy as np
from scheduled_servo import Controller as ScheduledController, _smoothstep
from sim.trajectory import T_RELEASE

MODE = "servo"
HOVER_M = 0.00125


class Controller(ScheduledController):
    MODE = MODE

    def compute(self, t, th, thd, tau_s, grip, q_d, qd_d, qdd_d):
        pos, vel, vel_ff, tau_ff = super().compute(
            t, th, thd, tau_s, grip, q_d, qd_d, qdd_d
        )
        # 놓기 접근 중 150 ms에 걸쳐 상승 오프셋을 만들고 release 뒤 100 ms
        # 유지한 후 부드럽게 기준 궤적으로 복귀한다.
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
            J = self.nom.jacobian(q_d)
            dq = np.linalg.pinv(J, rcond=1e-4) @ np.array([0.0, 0.0, height])
            pos = pos + dq
        return pos, vel, vel_ff, tau_ff
