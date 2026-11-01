"""하중 방향 성분을 분리하는 운동량 기반 충돌 감지기.

빈손 모델의 잔차에서 박스 중력이 만드는 관절토크 방향 성분을 제거한다.
정상적인 하중 전달은 이 1차원 부분공간에 주로 나타나고, 수평 충돌은 다른
방향 성분을 남긴다는 점을 이용한다.
"""
import numpy as np

SOURCE = "jts"


class Detector:
    SOURCE = SOURCE

    def reset(self):
        self.K_O = 50.0
        # 투영 뒤 남는 정상 동작 분산에 맞춘 축별 한계. J4/J5는 빈손
        # 기준값보다 여유를 주되 수평 충돌을 주로 보는 J1/J3 감도는 유지한다.
        self.threshold = np.array([10., 18., 8., 4., 10., 2.])
        self.r_model = np.zeros(6)
        self.r = np.zeros(6)
        self.s = None
        self.q_prev = None
        self.M_prev = None
        self.v = np.zeros(6)

    def update(self, t, th, thd, tau_m, tau_s, grip):
        dt = 1e-3
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
        beta = self.nom.bias(q, self.v) - Mdot @ self.v
        self.s += (tau_s - beta + self.r_model) * dt
        self.r_model = self.K_O * (p - self.s)

        payload_g = getattr(
            self.ctrl,
            "payload_tau",
            self.nom_box.gravity(q) - self.nom.gravity(q),
        )
        denom = float(payload_g @ payload_g)
        if denom > 1e-8:
            self.r = self.r_model - payload_g * float(self.r_model @ payload_g) / denom
        else:
            self.r = self.r_model.copy()
        return self.r.copy(), bool(np.any(np.abs(self.r) > self.threshold))
