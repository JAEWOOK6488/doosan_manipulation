"""하중 전달률 추정 기반 서보 제어기.

관절 토크센서에서 박스가 만드는 토크 방향 성분을 투영해 현재 실린 비율
alpha(0~1)를 추정한다. 피드포워드는 grip 계단 대신 alpha로 두 모델을
연속 보간한다. 추정값은 load_aware_detector와 공유된다.
"""
import numpy as np

MODE = "servo"


class Controller:
    MODE = MODE

    def reset(self):
        self.alpha = 0.0
        self.alpha_raw = 0.0
        self.sensor_bias = np.zeros(6)
        self.bias_ready = False

    def compute(self, t, th, thd, tau_s, grip, q_d, qd_d, qdd_d):
        tau_empty = self.nom.inverse_dynamics(q_d, qd_d, qdd_d)
        tau_loaded = self.nom_box.inverse_dynamics(q_d, qd_d, qdd_d)
        payload_tau = tau_loaded - tau_empty

        # 동작 전 구간에서 명목 모델과 JTS 사이의 고정 오차를 학습한다.
        if t < 1.20:
            err = tau_s - tau_empty
            gain = 0.01 if self.bias_ready else 1.0
            self.sensor_bias += gain * (err - self.sensor_bias)
            self.bias_ready = True

        denom = float(payload_tau @ payload_tau)
        if denom > 1e-8 and t >= 1.20:
            projection = float((tau_s - tau_empty - self.sensor_bias) @ payload_tau / denom)
            self.alpha_raw = float(np.clip(projection, 0.0, 1.0))

            # 약 8 ms의 1차 필터. 1 ms 토크 계단을 없애면서 20~30 ms
            # 하중 천이를 따라갈 수 있는 절충값이다.
            self.alpha += 0.12 * (self.alpha_raw - self.alpha)
            self.alpha = float(np.clip(self.alpha, 0.0, 1.0))

        tau_link = tau_empty + self.alpha * payload_tau
        tau_ff = self.nom.B * qdd_d + tau_link
        return q_d, qd_d, qd_d, tau_ff
