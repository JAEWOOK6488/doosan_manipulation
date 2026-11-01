"""학생 제어기 A — 서보 방식 (실제 두산 로봇과 같은 구조).

로봇 제어기(이 파일)는 모터를 직접 돌리지 않는다. 1 ms 마다 축별 지령 4가지를
관절 구동부(인버터, sim/drive.py)에 보내고, 구동부가 위치 P → 속도 PI 루프로 토크를 만든다.

  [이 파일] 로봇 제어기 ── pos, vel, vel_ff, tau_ff ──▶ [구동부] 위치 P → 속도 PI ──▶ 모터 토크

입력  t      : 시각 [s]
      th     : 관절 엔코더 각도 6개 [rad] (모터측, 분해능 2^18/rev)
      thd    : th 의 후진차분 속도 [rad/s] (노이즈 있음)
      tau_s  : 관절 토크센서(JTS) 6축 [N m] — 마찰·모터 관성보다 링크 쪽에서 잰 토크 (잡음·오프셋·드리프트·지연 있음)
      grip   : 1 = 박스를 잡고 있음, 0 = 빈 손
      q_d, qd_d, qdd_d : 기준 관절 각도·속도·가속도
출력  (pos, vel, vel_ff, tau_ff) — 각 6개
      pos    : 목표 각도 [rad]
      vel    : 목표 각속도 [rad/s]
      vel_ff : 속도 피드포워드 [rad/s]
      tau_ff : 모터 토크 피드포워드 [N m] (모터 관성 + 링크 토크)

self.nom     : 명목 모델 (박스 없음). inverse_dynamics, mass_matrix, bias, gravity,
               tcp_position, jacobian, set_payload(mass, com), B(모터 관성), K(관절 강성)
self.nom_box : 같은 모델에 박스(질량·무게중심·관성)를 정확히 넣은 것
self.box_mass, self.box_com : 박스 질량 [kg]·무게중심 [m] (과제 가정: 박스 값은 안다)
self.drive   : 구동부. 게인 kp, kv, ki 는 reset() 에서 바꿀 수 있다 (구조는 고정).
모르는 것은 하중 전달: 지금 박스 무게 중 얼마가 로봇에 실려 있는가 (grip 신호와 다르다).
"""
import numpy as np

MODE = "servo"


class Controller:
    MODE = MODE

    def reset(self):
        pass                                                   # 예: self.drive.kv *= 1.2

    def compute(self, t, th, thd, tau_s, grip, q_d, qd_d, qdd_d):
        nom = self.nom_box if grip else self.nom               # grip 신호에 맞춰 박스를 모델에 붙이고 뗀다
        tau_link = nom.inverse_dynamics(q_d, qd_d, qdd_d)       # 역동역학
        tau_ff = self.nom.B * qdd_d + tau_link                  # 모터 관성 포함
        return q_d, qd_d, qd_d, tau_ff
