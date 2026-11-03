"""기존 2차 미지 payload 검출기를 독립 holdout 조건에서 재평가한다."""
from pathlib import Path

import unknown_payload_audit as audit


audit.OUT = Path(__file__).resolve().parent / "results" / "unknown_payload_holdout"


if __name__ == "__main__":
    audit.main()
