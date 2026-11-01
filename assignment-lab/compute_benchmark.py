"""최종 제어기와 감지기 콜백의 1 kHz 계산시간을 측정한다."""
from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np

import hover_servo
from hybrid_detector import Detector
from sim.runner import run_episode


OUT = Path(__file__).resolve().parent / "results/compute_benchmark.json"


class TimedDetector(Detector):
    def reset(self):
        super().reset()
        self.update_ns = []
        self.update_cpu_ns = []

    def update(self, *args):
        start = time.perf_counter_ns()
        cpu_start = time.thread_time_ns()
        result = super().update(*args)
        self.update_cpu_ns.append(time.thread_time_ns() - cpu_start)
        self.update_ns.append(time.perf_counter_ns() - start)
        return result


class TimedController(hover_servo.Controller):
    def reset(self):
        super().reset()
        self.compute_cpu_ns = []

    def compute(self, *args):
        start = time.thread_time_ns()
        result = super().compute(*args)
        self.compute_cpu_ns.append(time.thread_time_ns() - start)
        return result


def stats(values_us: np.ndarray) -> dict:
    return {
        "median_us": float(np.median(values_us)),
        "p99_us": float(np.percentile(values_us, 99)),
        "max_us": float(values_us.max()),
    }


def main() -> None:
    cases = {
        "no_collision": None,
        "carry_15N": (3.0, [0, 15, 0]),
        "pre_release_15N": (3.86, [0, 0, 15]),
    }
    rows = {}
    all_controller = []
    all_detector = []
    all_controller_cpu = []
    all_detector_cpu = []
    for name, collision in cases.items():
        detector = TimedDetector()
        controller = TimedController()
        _, log = run_episode(
            controller, box_mass=8.0,
            detector=detector, collision=collision,
        )
        controller_us = 1e6 * log["t_ctrl"]
        detector_us = 1e-3 * np.asarray(detector.update_ns)
        rows[name] = {
            "samples": len(detector_us),
            "controller": stats(controller_us),
            "controller_cpu": stats(1e-3 * np.asarray(controller.compute_cpu_ns)),
            "detector": stats(detector_us),
            "detector_cpu": stats(1e-3 * np.asarray(detector.update_cpu_ns)),
        }
        all_controller.append(controller_us)
        all_detector.append(detector_us)
        all_controller_cpu.append(1e-3 * np.asarray(controller.compute_cpu_ns))
        all_detector_cpu.append(1e-3 * np.asarray(detector.update_cpu_ns))
    rows["combined"] = {
        "samples": int(sum(map(len, all_detector))),
        "controller": stats(np.concatenate(all_controller)),
        "controller_cpu": stats(np.concatenate(all_controller_cpu)),
        "detector": stats(np.concatenate(all_detector)),
        "detector_cpu": stats(np.concatenate(all_detector_cpu)),
    }
    OUT.write_text(json.dumps(rows, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(rows, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
