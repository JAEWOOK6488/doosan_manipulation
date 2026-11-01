"""최종안의 핵심 회귀 검증."""
import collision_detector
import hover_servo
import hybrid_detector
import servo_controller
from sim.runner import run_episode


baseline, _ = run_episode(
    servo_controller.Controller(), box_mass=8, detector=collision_detector.Detector()
)
final, log = run_episode(
    hover_servo.Controller(), box_mass=8, detector=hybrid_detector.Detector()
)
release = (log["t"] > 3.7) & (log["t"] < 4.1)
assert baseline["false_alarms"] == 7
assert final["false_alarms"] == 0
assert final["rms_mm"] < baseline["rms_mm"]
assert final["max_mm"] < baseline["max_mm"]
assert log["f_table"][release].max() < 95.0
assert final["ctrl_time_us"] < 1000.0

# 가장 어려운 하중 천이 구간에서 15 N을 여섯 방향 모두 구분해야 한다.
directions = (
    [15, 0, 0], [-15, 0, 0],
    [0, 15, 0], [0, -15, 0],
    [0, 0, 15], [0, 0, -15],
)
for t in (1.42, 3.86):
    for vector in directions:
        result, _ = run_episode(
            hover_servo.Controller(),
            box_mass=8,
            detector=hybrid_detector.Detector(),
            collision=(t, vector),
        )
        assert result["detect_delay_ms"] is not None
        assert not result["alarm_before_collision"]
        assert result["false_alarms"] == 0

# 운반 중에도 공개 평가와 같은 15 N 수평 충돌을 유지해야 한다.
result, _ = run_episode(
    hover_servo.Controller(),
    box_mass=8,
    detector=hybrid_detector.Detector(),
    collision=(3.0, [0, 15, 0]),
)
assert result["detect_delay_ms"] is not None
assert not result["alarm_before_collision"]
assert result["false_alarms"] == 0

# 시각 감사에서 가장 늦게 해결된 경계 사례를 고정한다.
for t, vector in (
    (1.34, [0, 0, -15]),
    (1.36, [-15, 0, 0]),
    (1.38, [0, 0, -15]),
    (3.84, [0, 0, 15]),
):
    result, _ = run_episode(
        hover_servo.Controller(),
        box_mass=8,
        detector=hybrid_detector.Detector(),
        collision=(t, vector),
    )
    assert result["detect_delay_ms"] is not None
    assert not result["alarm_before_collision"]
    assert result["false_alarms"] == 0

for t in (1.42, 3.0, 3.86):
    for vector in ([40, 0, 0], [0, 40, 0], [0, 0, 40]):
        result, _ = run_episode(
            hover_servo.Controller(),
            box_mass=8,
            detector=hybrid_detector.Detector(),
            collision=(t, vector),
        )
        assert result["detect_delay_ms"] is not None
        assert result["false_alarms"] == 0

print("PASS: control, false alarm, boundary/transition 15 N, XYZ 40 N checks")
