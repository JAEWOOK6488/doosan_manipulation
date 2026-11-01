"""관절 구동부(인버터) — 축마다 독립인 위치 P → 속도 PI 루프.

상위 제어기(로봇 제어기)는 1 ms 마다 축별 지령 4가지를 보낸다.
  pos    : 목표 각도 [rad]
  vel    : 목표 각속도 [rad/s] (기준값으로 기록, 루프에는 쓰지 않음)
  vel_ff : 속도 피드포워드 [rad/s]
  tau_ff : 모터 토크 피드포워드 [N m] (링크 환산)
구동부는 엔코더를 직접 읽어 토크를 만든다.
  v_ref = vel_ff + kp (pos - th)
  tau   = tau_ff + kv (v_ref - v) + ki ∫(v_ref - v) dt
게인은 바꿀 수 있지만 구조는 바꾸지 않는다 (실제 로봇의 구동부 파라미터와 같은 위치).
"""
import numpy as np
from . import config as C


class Drive:
    def __init__(self):
        self.kp, self.kv, self.ki = C.DRIVE_KP, C.DRIVE_KV.copy(), C.DRIVE_KI.copy()
        self.v, self.I = np.zeros(6), np.zeros(6)

    def torque(self, cmd, th, thd):
        pos, vel, vel_ff, tau_ff = cmd
        self.v += C.DRIVE_VEL_FILTER * (thd - self.v)
        ev = vel_ff + self.kp * (pos - th) - self.v
        self.I += ev * C.DT
        return tau_ff + self.kv * ev + self.ki * self.I
