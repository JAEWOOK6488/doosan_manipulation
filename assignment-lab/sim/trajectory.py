"""팔레타이징 1 사이클 기준 궤적 (관절공간, 경유점 사이 5차 다항식, 경유점에서 속도 0).

박스는 '픽업 위치' 도착 순간 잡고(grip=1), '내려놓기' 도착 순간 놓는다(grip=0).
"""
import numpy as np

WAYPOINTS = [            # (도착 시각 [s], 관절각 [deg])
    (0.0, [0, 0, 90, 0, 90, 0]),          # 대기
    (1.4, [-40, 25, 95, 0, 60, -40]),     # 픽업 위치  → 잡기
    (1.7, [-40, 15, 100, 0, 65, -40]),    # 들어올림
    (3.6, [55, 30, 75, 0, 75, 55]),       # 적재 위치 위
    (3.9, [55, 38, 72, 0, 70, 55]),       # 내려놓기   → 놓기
    (5.3, [0, 0, 90, 0, 90, 0]),          # 복귀
]
T_GRASP, T_RELEASE, T_END = 1.4, 3.9, 6.0


def reference(t, waypoints=WAYPOINTS):
    """t [s] 에서의 (q_d, qd_d, qdd_d) [rad, rad/s, rad/s^2]"""
    for (t0, a), (t1, b) in zip(waypoints[:-1], waypoints[1:]):
        if t <= t1:
            T = t1 - t0
            s = min(max((t - t0) / T, 0.0), 1.0)
            dq = np.deg2rad(np.subtract(b, a))
            q0 = np.deg2rad(a)
            return (q0 + dq * (10 * s**3 - 15 * s**4 + 6 * s**5),
                    dq * (30 * s**2 - 60 * s**3 + 30 * s**4) / T,
                    dq * (60 * s - 180 * s**2 + 120 * s**3) / T**2)
    return np.deg2rad(waypoints[-1][1]), np.zeros(6), np.zeros(6)
