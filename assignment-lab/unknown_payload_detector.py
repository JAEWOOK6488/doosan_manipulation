"""실제 질량·COM을 받지 않고 제어기의 JTS payload 추정만 쓰는 검출기."""
from robust_detector_v2 import Detector as RobustDetector


class Detector(RobustDetector):
    UNKNOWN_PAYLOAD = True
