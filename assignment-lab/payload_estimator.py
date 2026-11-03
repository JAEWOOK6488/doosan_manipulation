"""JTS만으로 미지 payload의 중력 파라미터를 온라인 추정한다.

시뮬레이터의 실제 box_mass/box_com은 읽지 않는다. 빈손 역동역학과 JTS의
차이를 여러 자세에서 모아 theta=[m, m*cx, m*cy, m*cz]를 ridge least
squares로 식별한다. 충돌과 접촉이 섞이기 쉬운 집기/놓기 천이 구간은 학습에서
제외한다.
"""
import numpy as np

from sim.nominal import Nominal
from sim.trajectory import T_GRASP, T_RELEASE


class PayloadEstimator:
    """질량과 COM을 사전에 받지 않는 중력 payload 추정기."""

    def __init__(self, nominal):
        self.nom = nominal
        self._basis = []
        for com in ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0),
                    (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)):
            model = Nominal()
            model.set_payload(1.0, com)
            self._basis.append(model)
        self.reset()

    def reset(self):
        self.theta = np.zeros(4)
        self.normal = 2e-2 * np.eye(4)
        self.rhs = np.zeros(4)
        self.samples = 0
        self.ready = False
        self.last_fit_error = np.nan

    def regressor(self, q):
        """tau_payload = Y(q)[m,m*cx,m*cy,m*cz]의 중력 regressor."""
        g0 = self.nom.gravity(q)
        unit_center = self._basis[0].gravity(q) - g0
        columns = [unit_center]
        for model in self._basis[1:]:
            columns.append(model.gravity(q) - g0 - unit_center)
        return np.column_stack(columns)

    @property
    def mass(self):
        return float(self.theta[0])

    @property
    def com(self):
        if self.mass < 0.25:
            return np.zeros(3)
        return self.theta[1:] / self.mass

    def payload_torque(self, q):
        return self.regressor(q) @ self.theta

    def observe(self, t, q, qd, tau_s, qdd_reference, grip):
        """현재 JTS 표본을 누적한다. 실제 payload 속성은 입력받지 않는다."""
        if not grip:
            return
        # 집기 직후부터 실제로 전달되는 하중을 따라간다. 단일 표본이 아니라
        # 누적 최소제곱을 쓰므로 초기 자세에서 관측 가능한 조합부터 정해지고,
        # 운반 중 자세가 달라지며 나머지 파라미터가 식별된다.
        if not (T_GRASP + 0.025 <= t <= T_RELEASE - 0.40):
            return

        y = tau_s - self.nom.inverse_dynamics(q, qd, qdd_reference)
        Y = self.regressor(q)
        residual = y - Y @ self.theta

        # 외란 한 표본이 payload로 학습되는 것을 막는 Huber형 가중치.
        scale = max(float(np.linalg.norm(residual)), 1e-9)
        weight = min(1.0, 12.0 / scale)
        self.normal += weight * (Y.T @ Y)
        self.rhs += weight * (Y.T @ y)
        self.samples += 1

        if self.samples % 5:
            return
        theta = np.linalg.solve(self.normal, self.rhs)
        mass = float(np.clip(theta[0], 0.0, 12.0))
        if mass < 0.25:
            theta[:] = 0.0
        else:
            # 과제 물체 크기보다 넓은 물리적 경계. 실제값을 제공하는 경계가 아니다.
            moments = np.clip(theta[1:], -0.18 * mass, 0.18 * mass)
            # 이 궤적의 중력 데이터만으로 공구축 방향 COM은 조건수가 나빠
            # 동적 오차를 COM으로 과적합한다. 관측 가능한 수평 1차 모멘트만 쓴다.
            moments[2] = 0.0
            theta = np.r_[mass, moments]
        self.theta = 0.85 * self.theta + 0.15 * theta
        self.last_fit_error = float(np.sqrt(np.mean(residual * residual)))
        self.ready = self.samples >= 80 and self.mass >= 0.25
