"""학생 충돌 감지기 — 운동량 관측기(momentum observer) 기반 외력 추정.

제어기와 별도로 1 ms 마다, 제어기 compute() 보다 먼저 update() 가 호출된다.
시뮬레이터는 감지 후 아무것도 하지 않는다. 감지 후 반응(정지·후퇴·순응 등)은 자유롭게 정한다.
  예) update() 에서 self.ctrl.collided = True 로 알리고, 제어기 compute() 에서 처리

입력  t, th, thd, tau_s, grip — 제어기 입력과 같다 (tau_s = 관절 토크센서 JTS 6축)
      tau_m : 직전 주기 모터 토크 [N m] (실제 로봇의 전류 기반 토크에 해당)
출력  (r, detected)
      r        : 추정 외력 토크 6개 [N m] (관측기 잔차)
      detected : True 면 충돌로 판단

self.nom     : 감지기 전용 명목 모델 (박스 없음)
self.nom_box : 같은 모델에 박스를 정확히 넣은 것, self.box_mass·self.box_com : 박스 값 (과제 가정: 안다)
self.ctrl    : 제어기 객체 (제어기가 추정한 값 등을 읽을 수 있음)
예시는 grip 신호에 맞춰 박스 모델로 바꾼다. 실제 하중은 신호와 다르게 넘어가므로 그 사이 r 이 커진다.

운동량 관측기 (De Luca 등):
  p = M(q) qd,   β = g(q) − C(q, qd)ᵀ qd = h(q, qd) − Ṁ qd
  r = K_O [ p − ∫(τ − β + r) dt − p(0) ]   →   r ≈ τ_ext 를 1차 지연(1/K_O)으로 추종

SOURCE = "jts"   (기본) 링크 쪽 관측기. τ = JTS 측정 토크, 링크 각도 q = θ − τ_s/K (탄성 관절),
                 M = 링크 관성만. JTS 는 마찰·모터 관성·감속기 탄성보다 링크 쪽이라 그 영향이 빠진다.
SOURCE = "motor" 모터 쪽 관측기. τ = 모터 토크, q = θ, M = 링크 + 모터 관성 (강체 가정).
                 마찰과 감속기 탄성이 그대로 외력처럼 보인다 (비교용).
"""
import numpy as np

SOURCE = "jts"


class Detector:
    SOURCE = SOURCE

    def reset(self):
        self.K_O = 50.0                                              # 관측기 이득 [1/s] (대역폭)
        if self.SOURCE == "jts":
            self.threshold = np.array([10., 18., 8., 3., 3., 2.])   # 축별 감지 임계값 [N m]
        else:
            self.threshold = np.array([30., 50., 20., 5., 6., 4.])
        self.r = np.zeros(6)
        self.s = None
        self.q_prev = None
        self.M_prev = None
        self.v = np.zeros(6)

    def update(self, t, th, thd, tau_m, tau_s, grip):
        dt = 1e-3
        nom = self.nom_box if grip else self.nom                     # grip 신호에 맞춰 박스 모델로 전환
        if self.SOURCE == "jts":
            q = th - np.where(nom.elastic, tau_s / np.where(nom.elastic, nom.K, 1.0), 0.0)   # 링크 각도
            qd = np.zeros(6) if self.q_prev is None else (q - self.q_prev) / dt
            self.q_prev = q
            tau, M_add = tau_s, 0.0
        else:
            q, qd, tau, M_add = th, thd, tau_m, np.diag(nom.B)
        self.v += 0.2 * (qd - self.v)                                # 속도 저역통과 필터
        M = nom.mass_matrix(q) + M_add
        p = M @ self.v
        if self.s is None:
            self.s, self.M_prev = p.copy(), M
        Mdot = (M - self.M_prev) / dt
        self.M_prev = M
        beta = nom.bias(q, self.v) - Mdot @ self.v                   # g − Cᵀ qd
        self.s += (tau - beta + self.r) * dt
        self.r = self.K_O * (p - self.s)
        return self.r.copy(), bool(np.any(np.abs(self.r) > self.threshold))
