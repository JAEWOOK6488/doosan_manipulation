"""미지 payload 공정 비교용 1차 검출 기준안.

실제 mass/COM은 받지 않는다. 미지 payload에 필요한 이동 기준선은 공통으로
사용하되, 2차의 확장 천이창·접촉방향 제거·20 ms 벡터평균은 끈다.
"""
from robust_detector_v2 import Detector as RobustDetector


class Detector(RobustDetector):
    UNKNOWN_PAYLOAD = True
    UNKNOWN_PHASE1_BASELINE = True
