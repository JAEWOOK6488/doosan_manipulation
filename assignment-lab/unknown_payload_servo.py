"""질량·COM을 모르는 조건의 JTS 기반 서보 + 1.25 mm 놓기 호버."""
import numpy as np

from payload_estimator import PayloadEstimator
from scheduled_servo import _smoothstep
from sim.trajectory import T_GRASP, T_RELEASE

MODE = "servo"
HOVER_M = 0.00125


class Controller:
    MODE = MODE
    UNKNOWN_PAYLOAD = True

    def reset(self):
        # 알려지지 않은 초기 하중을 피드포워드가 식별하기 전까지 위치 서보가
        # 받아낼 수 있도록 허용된 구동부 속도루프 게인을 높인다.
        self.drive.kv *= 2.0
        self.drive.ki *= 2.0
        self.estimator = PayloadEstimator(self.nom)
        self.alpha = 0.0
        self.payload_tau = np.zeros(6)
        self.estimated_payload_tau = np.zeros(6)
        self.estimated_mass = 0.0
        self.estimated_com = np.zeros(3)
        self.q_prev = None
        self.v = np.zeros(6)

    def _link_angle(self, th, tau_s):
        return th - np.where(
            self.nom.elastic,
            tau_s / np.where(self.nom.elastic, self.nom.K, 1.0),
            0.0,
        )

    def compute(self, t, th, thd, tau_s, grip, q_d, qd_d, qdd_d):
        q = self._link_angle(th, tau_s)
        qd = np.zeros(6) if self.q_prev is None else (q - self.q_prev) / 1e-3
        self.q_prev = q.copy()
        self.v += 0.15 * (qd - self.v)
        self.estimator.observe(t, q, self.v, tau_s, qdd_d, bool(grip))
        self.estimated_mass = self.estimator.mass
        self.estimated_com = self.estimator.com.copy()

        # 추정 전에는 0에서 시작한다. 추정값이 생겨도 피드포워드 계단이 되지
        # 않도록 payload 토크 자체를 저역통과시킨다.
        self.estimated_payload_tau = (
            self.estimator.payload_torque(q) if grip else np.zeros(6)
        )
        if grip:
            model_payload = self.estimator.payload_torque(q_d)
            measured_payload = tau_s - self.nom.inverse_dynamics(q, self.v, qdd_d)
            # 초기 식별 전에는 JTS에서 직접 본 하중을 사용하고, 식별이 안정되면
            # 자세에 따라 일반화되는 모델 토크로 부드럽게 넘긴다.
            blend = min(self.estimator.samples / 250.0, 1.0)
            target_payload_tau = blend * model_payload + (1.0 - blend) * measured_payload
        else:
            target_payload_tau = np.zeros(6)
        self.payload_tau += 0.08 * (target_payload_tau - self.payload_tau)

        if t < T_GRASP:
            self.alpha = 0.0
        elif t < T_RELEASE - 0.05:
            self.alpha = 1.0
        elif t < T_RELEASE:
            self.alpha = 1.0 - _smoothstep((t - (T_RELEASE - 0.05)) / 0.05)
        else:
            self.alpha = 0.0

        tau_empty = self.nom.inverse_dynamics(q_d, qd_d, qdd_d)
        tau_link = tau_empty + self.alpha * self.payload_tau
        pos = q_d.copy()

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
            dq = np.linalg.pinv(self.nom.jacobian(q_d), rcond=1e-4) @ np.array(
                [0.0, 0.0, height]
            )
            pos += dq
        return pos, qd_d, qd_d, self.nom.B * qdd_d + tau_link
