"""분석 전용: 출력측 엔코더가 공식 제공될 때의 성능 상한 실험.

기본 제출안이 아니다. 현재 과제 콜백 밖의 `q_link`가 필요하며 runner가
`USES_LINK_ENCODER` 플래그를 보고 분석 채널을 넣어 준다.
"""
import numpy as np

from hybrid_detector import Detector as HybridDetector
from sim.config import D, ENCODER_RES


class Detector(HybridDetector):
    USES_LINK_ENCODER = True

    def _link_angle(self, th, tau_s):
        # 모터측과 같은 18-bit 분해능을 가정해 이상적인 연속 상태를 쓰지 않는다.
        return np.round(self.q_link / ENCODER_RES) * ENCODER_RES

    def _joint_torque(self, th, thd, q, qd, tau_s):
        torque = tau_s.copy()
        elastic = self.nom.elastic
        torque[elastic] = (
            self.nom.K[elastic] * (th[elastic] - q[elastic])
            + D[elastic] * (thd[elastic] - qd[elastic])
        )
        return torque
