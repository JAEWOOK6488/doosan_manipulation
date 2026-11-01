"""최종 제출물·결과·보고서의 일관성을 한 번에 검사한다."""
from __future__ import annotations

import hashlib
import json
import platform
import re
import sys
import zipfile
from datetime import datetime, timezone
from pathlib import Path

import mujoco
import numpy as np


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
ZIP = ROOT / "IC-PBL_submission.zip"
SHA = ROOT / "IC-PBL_submission.sha256"
RESULTS = HERE / "results"
ORIGINAL_ROOT = Path("/home/jwj/Downloads/Week9_Materials/icpbl_sim")
ORIGINAL_SIM = ORIGINAL_ROOT / "sim"
EXPECTED_FILES = {
    "servo_controller.py": ROOT / "submission/servo_controller.py",
    "collision_detector.py": ROOT / "submission/collision_detector.py",
    "README_SUBMISSION.md": ROOT / "submission/README.md",
    "scheduled_servo.py": HERE / "scheduled_servo.py",
    "hover_servo.py": HERE / "hover_servo.py",
    "hybrid_detector.py": HERE / "hybrid_detector.py",
}
FORBIDDEN_PY = re.compile(
    r"(?:from|import)\s+sim\.plant|plant\.(?:q|d|m)|\bq_link\b|"
    r"\bf_table\b|\bf_tool\b|\bqpos\b|\btrue_torque\b"
)


class Audit:
    def __init__(self):
        self.checks: list[dict[str, object]] = []

    def check(self, name: str, condition: bool, detail: str = "") -> None:
        self.checks.append({"name": name, "passed": bool(condition), "detail": detail})
        print(f"{'PASS' if condition else 'FAIL'}  {name}{': ' + detail if detail else ''}")

    def summary(self) -> bool:
        passed = sum(bool(x["passed"]) for x in self.checks)
        print(f"\n{passed}/{len(self.checks)} checks passed")
        return passed == len(self.checks)


def load(name: str) -> dict:
    return json.loads((RESULTS / name).read_text())


def hash_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check_summary(audit: Audit, title: str, data: dict, expected: dict) -> None:
    mismatches = [f"{key}={data.get(key)!r} (expected {value!r})"
                  for key, value in expected.items() if data.get(key) != value]
    audit.check(title, not mismatches, "; ".join(mismatches))


def evaluation_rows(text: str) -> tuple[list[list[str]], list[str]]:
    control = []
    collisions = []
    for line in text.splitlines():
        stripped = line.strip()
        if re.match(r"^(0|4|8)kg\s*\|", stripped):
            columns = [part.strip() for part in stripped.split("|")]
            control.append(columns[:6] + columns[7:])  # wall-clock 계산 us 제외
        elif re.match(r"^(0|8)kg\s+\d", stripped):
            collisions.append(" ".join(stripped.split()))
    return control, collisions


def main() -> int:
    audit = Audit()
    actual_sha = hash_file(ZIP)
    recorded_sha = SHA.read_text().split()[0]
    audit.check("submission checksum", actual_sha == recorded_sha, actual_sha)

    with zipfile.ZipFile(ZIP) as archive:
        names = archive.namelist()
        audit.check("submission file allow-list", set(names) == set(EXPECTED_FILES), ", ".join(names))
        metadata_ok = all(
            item.date_time == (2026, 1, 1, 0, 0, 0)
            and (item.external_attr >> 16) & 0o777 == 0o644
            for item in archive.infolist()
        )
        audit.check("reproducible ZIP metadata", metadata_ok)
        source_mismatches = [name for name, path in EXPECTED_FILES.items()
                             if name not in names or archive.read(name) != path.read_bytes()]
        audit.check("ZIP matches reviewed sources", not source_mismatches,
                    ", ".join(source_mismatches))
        forbidden_hits = []
        for name in names:
            if not name.endswith(".py"):
                continue
            text = archive.read(name).decode("utf-8")
            if FORBIDDEN_PY.search(text):
                forbidden_hits.append(name)
        audit.check("no simulator truth access in submitted Python", not forbidden_hits,
                    ", ".join(forbidden_hits))

    timing = load("timing_audit_summary.json")
    check_summary(audit, "96-case transition timing audit", timing, {
        "cases": 96, "detected": 96, "missed": 0, "alarm_before": 0,
        "reference_alarm_cases": 0, "false_alarm_cases": 0,
    })
    audit.check("transition delay bound", timing["delay_ms_max"] <= 200,
                f"max={timing['delay_ms_max']} ms")

    startup = load("startup_audit_summary.json")
    check_summary(audit, "48-case startup/calibration audit", startup, {
        "cases": 48, "detected": 48, "missed": 0,
        "alarm_before_cases": 0, "reference_alarm_cases": 0,
        "false_alarm_cases": 0,
    })

    sensitivity = load("sensitivity_audit_summary.json")["groups"]["15N"]
    check_summary(audit, "72-case whole-cycle 15 N sensitivity audit", sensitivity, {
        "cases": 72, "detected": 72, "missed": 0,
        "alarm_before_cases": 0, "reference_alarm_cases": 0,
        "false_alarm_cases": 0,
    })

    dense = load("dense_sensitivity_audit_summary.json")
    check_summary(audit, "348-case dense 15 N sensitivity audit", dense, {
        "force_N": 15.0, "mass_kg": 8.0, "time_step_s": 0.1,
        "cases": 348, "detected": 348, "missed": 0,
        "alarm_before_cases": 0, "reference_alarm_cases": 0,
        "false_alarm_cases": 0,
    })

    benchmark = load("compute_benchmark.json")["combined"]
    audit.check("controller and detector p99 below 1 ms",
                benchmark["controller"]["p99_us"] < 1000
                and benchmark["detector"]["p99_us"] < 1000,
                f"controller={benchmark['controller']['p99_us']:.1f} us, "
                f"detector={benchmark['detector']['p99_us']:.1f} us")

    robust = load("robustness_summary.json")
    check_summary(audit, "88-case robustness audit", robust, {
        "cases": 88, "nominal_false_alarm_cases": 0,
        "nominal_diverged_cases": 0, "collision_cases": 72,
        "collision_detected": 72, "collision_missed": 0,
        "collision_alarm_before": 0, "collision_reference_alarm_cases": 0,
        "collision_false_alarm_cases": 0,
    })

    uncertainty = load("uncertainty_audit_summary.json")
    check_summary(audit, "sensor/model uncertainty audit", uncertainty, {
        "scenarios": 16, "cases": 64, "nominal_false_alarm_scenarios": 0,
        "nominal_diverged_scenarios": 0, "collision_cases": 48,
        "collision_detected": 48, "collision_missed": 0,
        "collision_alarm_before": 0, "collision_reference_alarm_cases": 0,
        "collision_false_alarm_cases": 0,
    })

    monte_carlo = load("monte_carlo_audit_summary.json")
    check_summary(audit, "continuous-condition Monte Carlo audit", monte_carlo, {
        "seed": 20261101, "cases": 60, "detected": 60, "missed": 0,
        "alarm_before_cases": 0, "reference_alarm_cases": 0,
        "false_alarm_cases": 0, "diverged_cases": 0,
    })

    original = (HERE / "final_eval_unmodified_sim.txt").read_text()
    audit.check("unmodified simulator evaluation present",
                all(marker in original for marker in ("[1] 제어 성능", "[2] 충돌 감지", "8kg", "92 N")))
    work_eval = (HERE / "final_eval.txt").read_text()
    audit.check("work copy and original simulator metrics match",
                evaluation_rows(work_eval) == evaluation_rows(original))
    checksum_lines = (HERE / "original_sim_checksums.txt").read_text().splitlines()
    recorded = {
        parts[1]: parts[0]
        for line in checksum_lines
        if len(parts := line.split()) == 2
    }
    expected_names = {f"sim/{name}" for name in (
        "__init__.py", "config.py", "drive.py", "model.py",
        "nominal.py", "plant.py", "runner.py", "trajectory.py",
    )}
    original_hashes_match = ORIGINAL_SIM.is_dir() and all(
        hash_file(ORIGINAL_SIM / Path(name).name) == digest
        for name, digest in recorded.items()
    )
    audit.check(
        "original simulator checksum record",
        set(recorded) == expected_names and original_hashes_match,
    )

    model_lines = (HERE / "original_model_checksums.txt").read_text().splitlines()
    model_recorded = {
        parts[1]: parts[0]
        for line in model_lines
        if len(parts := line.split()) == 2
    }
    actual_model_names = {
        path.relative_to(ORIGINAL_ROOT).as_posix()
        for path in (ORIGINAL_ROOT / "model").rglob("*")
        if path.is_file()
    }
    model_hashes_match = all(
        hash_file(ORIGINAL_ROOT / name) == digest
        for name, digest in model_recorded.items()
    )
    audit.check(
        "original model checksum record",
        set(model_recorded) == actual_model_names and model_hashes_match,
    )

    missing = []
    markdown_files = [
        *HERE.glob("*.md"), ROOT / "README.md", ROOT / "submission/README.md",
        ROOT / "mujoco-dashboard/README.md",
    ]
    for document in markdown_files:
        links = re.findall(r"!?\[[^]]*\]\(([^)]+)\)", document.read_text())
        for link in links:
            if link.startswith(("http://", "https://", "#")):
                continue
            path = (document.parent / link.split("#", 1)[0]).resolve()
            if not path.exists():
                missing.append(f"{document.name}: {link}")
    audit.check("Markdown local links", not missing, ", ".join(missing))

    manifest = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "environment": {
            "python": platform.python_version(),
            "mujoco": mujoco.__version__,
            "numpy": np.__version__,
        },
        "submission_zip_sha256": actual_sha,
        "original_sim_checksums_sha256": hash_file(HERE / "original_sim_checksums.txt"),
        "original_model_checksums_sha256": hash_file(HERE / "original_model_checksums.txt"),
        "source_sha256": {name: hash_file(path) for name, path in EXPECTED_FILES.items()},
        "result_sha256": {
            name: hash_file(RESULTS / name) for name in (
                "timing_audit_summary.json", "robustness_summary.json",
                "uncertainty_audit_summary.json", "hover_height_sweep.json",
                "motor_jts_gap_study.json", "monte_carlo_audit_summary.json",
                "startup_audit_summary.json", "sensitivity_audit_summary.json",
                "dense_sensitivity_audit_summary.json", "compute_benchmark.json",
            )
        },
        "checks": audit.checks,
    }
    (RESULTS / "final_audit_manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n"
    )
    return 0 if audit.summary() else 1


if __name__ == "__main__":
    sys.exit(main())
