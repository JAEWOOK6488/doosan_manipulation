"""3차 미지 payload 시제품을 기존과 동일한 감사 절차로 실행한다."""
from pathlib import Path

import unknown_payload_audit as audit
import unknown_payload_detector_v3


audit.unknown_payload_detector = unknown_payload_detector_v3
audit.OUT = Path(__file__).resolve().parent / "results" / "unknown_payload_v3"


if __name__ == "__main__":
    audit.main()
