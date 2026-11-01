"""검토된 파일만 포함하는 재현 가능한 제출 ZIP을 만든다."""
from __future__ import annotations

import hashlib
import io
import zipfile
from pathlib import Path


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
FILES = {
    "servo_controller.py": ROOT / "submission/servo_controller.py",
    "collision_detector.py": ROOT / "submission/collision_detector.py",
    "README_SUBMISSION.md": ROOT / "submission/README.md",
    "scheduled_servo.py": HERE / "scheduled_servo.py",
    "hover_servo.py": HERE / "hover_servo.py",
    "hybrid_detector.py": HERE / "hybrid_detector.py",
}


def main() -> None:
    buffer = io.BytesIO()
    with zipfile.ZipFile(
        buffer, "w", zipfile.ZIP_DEFLATED, compresslevel=9
    ) as archive:
        for name, path in FILES.items():
            info = zipfile.ZipInfo(name, date_time=(2026, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.create_system = 3
            info.external_attr = 0o100644 << 16
            archive.writestr(info, path.read_bytes(), compress_type=zipfile.ZIP_DEFLATED,
                             compresslevel=9)
    payload = buffer.getvalue()
    output = ROOT / "IC-PBL_submission.zip"
    output.write_bytes(payload)
    digest = hashlib.sha256(payload).hexdigest()
    (ROOT / "IC-PBL_submission.sha256").write_text(
        f"{digest}  IC-PBL_submission.zip\n"
    )
    print(f"{output}\nsha256 {digest}")


if __name__ == "__main__":
    main()
