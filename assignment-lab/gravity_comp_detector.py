"""하중 전달률로 중력 성분만 보상하는 운동량 관측기.

접촉 천이 중에는 박스가 로봇과 강체로 결합됐다고 보기 어려우므로 관성행렬을
순간적으로 바꾸지 않는다. 대신 가장 큰 모델 오차인 박스 중력 토크만 alpha로
연속 보상해 관성행렬 변화에서 생기는 인공 운동량 피크를 피한다.
"""
import numpy as np

SOURCE = "jts"


class Detector:
    SOURCE = SOURCE

    def reset(self):
        self.K_O = 50.0
        self.threshold = np.array([10., 18., 8., 3., 3., 2.])
        self.r = np.zeros(6)
        self.s = None
        self.q_prev = None
        self.M_prev = None
        self.v = np.zeros(6)

    def update(self, t, th, thd, tau_m, tau_s, grip):
        dt = 1e-3
        alpha = float(np.clip(getattr(self.ctrl, "alpha", float(grip)), 0.0, 1.0))
        q = th - np.where(
            self.nom.elastic,
            tau_s / np.where(self.nom.elastic, self.nom.K, 1.0),
            0.0,
        )
        qd = np.zeros(6) if self.q_prev is None else (q - self.q_prev) / dt
        self.q_prev = q
        self.v += 0.2 * (qd - self.v)

        M = self.nom.mass_matrix(q)
        p = M @ self.v
        if self.s is None:
            self.s, self.M_prev = p.copy(), M
        Mdot = (M - self.M_prev) / dt
        self.M_prev = M

        beta0 = self.nom.bias(q, self.v) - Mdot @ self.v
        payload_g = self.nom_box.gravity(q) - self.nom.gravity(q)
        beta = beta0 + alpha * payload_g
        self.s += (tau_s - beta + self.r) * dt
        self.r = self.K_O * (p - self.s)
        return self.r.copy(), bool(np.any(np.abs(self.r) > self.threshold))
