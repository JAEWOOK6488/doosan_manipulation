# IC-PBL 제출 파일

이 압축 파일을 **원본 `icpbl_sim` 폴더**에 풀어 기존
`servo_controller.py`, `collision_detector.py`를 교체한다. 함께 들어 있는
`scheduled_servo.py`, `hover_servo.py`, `hybrid_detector.py`는 같은 폴더에 둔다.

`sim/`과 `model/`은 수정하거나 덮어쓰지 않는다. 패키지에는 두 폴더가 들어 있지
않다.

기본 평가 명령은 그대로다.

```bash
.venv/bin/python evaluate.py --out evaluation.txt
```

Linux 작업 환경에서 원본 `sim/`로 재검증한 결과는
`assignment-lab/final_eval_unmodified_sim.txt`에 있다.
