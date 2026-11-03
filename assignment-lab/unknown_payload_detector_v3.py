"""미지 payload용 3차 시제품: nuisance-subspace + adaptive observer."""
from robust_detector_v2 import Detector as RobustDetector


class Detector(RobustDetector):
    UNKNOWN_PAYLOAD = True
    UNKNOWN_PAYLOAD_V3 = True
