"""연속 하중 모델을 사용하는 운동량 관측기.

adaptive_servo가 추정한 alpha로 빈손/박스 모델의 관성 및 바이어스 항을
연속 보간한다. grip 신호의 불연속 모델 전환으로 생기는 가짜 잔차를
제거하는 것이 목적이다.
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

        M0 = self.nom.mass_matrix(q)
        M1 = self.nom_box.mass_matrix(q)
        M = M0 + alpha * (M1 - M0)
        p = M @ self.v
        if self.s is None:
            self.s, self.M_prev = p.copy(), M
        Mdot = (M - self.M_prev) / dt
        self.M_prev = M

        h0 = self.nom.bias(q, self.v)
        h1 = self.nom_box.bias(q, self.v)
        beta = h0 + alpha * (h1 - h0) - Mdot @ self.v
        self.s += (tau_s - beta + self.r) * dt
        self.r = self.K_O * (p - self.s)
        return self.r.copy(), bool(np.any(np.abs(self.r) > self.threshold))
