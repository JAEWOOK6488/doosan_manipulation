"""학생 제어기 B — 토크 방식 (계산 토크 제어, computed torque control).

구동부를 쓰지 않고 관절 토크를 직접 만든다. 1 ms 마다 compute() 가 호출되고,
반환한 토크는 한 주기 뒤에 모터에 들어간다.

입력  t, th, thd, tau_s, grip, q_d, qd_d, qdd_d — servo_controller.py 와 같다.
출력  관절 토크 6개 [N m] (한계: 346, 346, 163, 50, 50, 50 — 넘으면 잘림)

self.nom, self.nom_box, self.box_mass, self.box_com : servo_controller.py 설명 참고.
주의: J1~J3 은 탄성 관절이다. 강체 모델로 만든 계산 토크 제어는 게인을 올리면
      진동·포화가 생긴다. 이 한계를 어떻게 넘을지가 과제의 일부다.
"""
import numpy as np

MODE = "torque"


class Controller:
    MODE = MODE

    def reset(self):
        wn = 20.0                                              # 목표 고유진동수 [rad/s], 감쇠비 1 (25 부터 J2·J3 진동)
        self.Kp, self.Kd = wn**2, 2 * wn
        self.v = np.zeros(6)

    def compute(self, t, th, thd, tau_s, grip, q_d, qd_d, qdd_d):
        self.v += 0.2 * (thd - self.v)                         # 속도 저역통과 필터
        a = qdd_d + self.Kd * (qd_d - self.v) + self.Kp * (q_d - th)
        M = self.nom.mass_matrix(th) + np.diag(self.nom.B)     # 링크 + 모터 관성 (피드백 이득은 박스 없는 모델로)
        tau = M @ a + self.nom.bias(th, self.v)                # tau = M a + h
        if grip:                                               # grip 신호에 맞춰 박스가 만드는 토크를 더한다
            tau += self.nom_box.inverse_dynamics(q_d, qd_d, qdd_d) - self.nom.inverse_dynamics(q_d, qd_d, qdd_d)
        return tau
