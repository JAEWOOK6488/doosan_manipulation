"""충돌 없는 미지 payload 실행만으로 시간가변 정상 잔차 범위를 학습한다."""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

import unknown_payload_detector_v3
import unknown_payload_servo
from sim.runner import run_episode
from unknown_payload_audit import make_case


OUT = Path(__file__).resolve().parent / "results" / "unknown_payload_v3"


def normal_residual(args):
    index, seed = args
    case = make_case(index, seed)
    _, log = run_episode(
        unknown_payload_servo.Controller(),
        detector=unknown_payload_detector_v3.Detector(),
        box_mass=case["mass_kg"], box_com=case["com"],
        plant_uncertainty=case["uncertainty"],
    )
    return log["r"].astype(np.float32)


def moving_max(values: np.ndarray, radius: int) -> np.ndarray:
    padded = np.pad(values, ((radius, radius), (0, 0)), mode="edge")
    return np.maximum.reduce([padded[k:k + len(values)] for k in range(2 * radius + 1)])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", type=int, default=300)
    parser.add_argument("--seed", type=int, default=73129)
    parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args()
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        residuals = np.stack(list(pool.map(
            normal_residual, [(i, args.seed) for i in range(args.cases)], chunksize=2
        )))
    center = np.median(residuals, axis=0)
    deviation = np.abs(residuals - center)
    # 개발 집합의 99.5백분위에 50% 여유를 주고, ±8 ms의 최대값을 취해
    # 접촉 시점과 센서 지연의 작은 차이에도 정상 범위가 유지되게 한다.
    envelope = 1.50 * np.percentile(deviation, 99.5, axis=0)
    envelope = moving_max(envelope, 8)
    envelope = np.maximum(envelope, np.array([4., 6., 4., 1.5, 1.5, 1.0]))
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / "normal_profile.npz"
    np.savez_compressed(path, center=center.astype(np.float32),
                        envelope=envelope.astype(np.float32),
                        cases=args.cases, seed=args.seed)
    print(path)


if __name__ == "__main__":
    main()
