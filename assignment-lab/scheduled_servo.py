"""접촉 시각을 반영한 연속 하중 스케줄 기반 서보 제어기.

집기 하중은 grip 직후 약 25 ms 동안 증가하고, 놓기 하중은 grip OFF보다
약 45~53 ms 먼저 감소한다는 기준 실험의 구조를 모델에 포함한다.
"""
import numpy as np
from sim.trajectory import T_GRASP, T_RELEASE

MODE = "servo"
PICK_DURATION_S = 0.025


def _smoothstep(x):
    x = float(np.clip(x, 0.0, 1.0))
    return x * x * (3.0 - 2.0 * x)


class Controller:
    MODE = MODE

    def reset(self):
        self.alpha = 0.0

    def compute(self, t, th, thd, tau_s, grip, q_d, qd_d, qdd_d):
        pick_duration = PICK_DURATION_S
        # 가이드의 2/4/6/8 kg 접촉 시각(45/47/50/53 ms 전)을
        # 단순 선형화하되 공개되지 않은 질량에서도 연속이 되게 한다.
        release_lead = 0.042 + 0.0014 * self.box_mass
        if t < T_GRASP:
            self.alpha = 0.0
        elif t < T_GRASP + pick_duration:
            self.alpha = _smoothstep((t - T_GRASP) / pick_duration)
        elif t < T_RELEASE - release_lead:
            self.alpha = 1.0
        elif t < T_RELEASE:
            self.alpha = 1.0 - _smoothstep((t - (T_RELEASE - release_lead)) / release_lead)
        else:
            self.alpha = 0.0

        tau_empty = self.nom.inverse_dynamics(q_d, qd_d, qdd_d)
        tau_loaded = self.nom_box.inverse_dynamics(q_d, qd_d, qdd_d)
        self.payload_tau = tau_loaded - tau_empty
        tau_link = tau_empty + self.alpha * self.payload_tau
        return q_d, qd_d, qd_d, self.nom.B * qdd_d + tau_link
