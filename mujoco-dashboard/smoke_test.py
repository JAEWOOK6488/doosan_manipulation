"""실행 중인 로컬 대시보드 API의 핵심 계약을 빠르게 검사한다."""
from __future__ import annotations

import json
from urllib.error import HTTPError
from urllib.request import Request, urlopen


BASE = "http://127.0.0.1:8765"


def post(params: dict) -> dict:
    request = Request(
        BASE + "/api/run",
        json.dumps(params).encode(),
        {"Content-Type": "application/json"},
    )
    with urlopen(request, timeout=30) as response:
        return json.load(response)


def get(path: str) -> dict:
    with urlopen(BASE + path, timeout=30) as response:
        return json.load(response)


def expect_error(request: Request, status: int) -> None:
    try:
        urlopen(request, timeout=30)
    except HTTPError as error:
        assert error.code == status, (error.code, status)
    else:
        raise AssertionError(f"expected HTTP {status}")


def check_series(data: dict) -> None:
    required = {
        "t", "q", "q_d", "th", "thd", "tau", "tau_s", "tau_m",
        "r", "f_residual", "detect", "e_mm", "f_table", "f_ext",
    }
    assert required <= set(data["series"]), required - set(data["series"])
    assert all(len(data["series"][name]) == 6000 for name in required)


def main() -> None:
    common = {
        "mass": 8, "comX": 20, "comY": -10, "comZ": 30,
        "mode": "improved", "time": 3, "force": 40, "axis": "+Y",
    }
    nominal = post({**common, "collision": False})
    check_series(nominal)
    assert nominal["metrics"]["false_alarms"] == 0
    assert nominal["metrics"]["rms_mm"] < 6.1

    fixed = post({
        **common, "collision": True, "time": 0.6, "force": 15, "axis": "-X",
    })
    check_series(fixed)
    assert fixed["metrics"]["distinguishable"] is True
    assert fixed["metrics"]["detect_delay_ms"] <= 100

    custom = post({
        **common, "mass": 5.5, "comX": -35, "comY": 40, "comZ": 15,
        "collision": True, "time": 3, "force": 15, "axis": "-Z",
    })
    assert custom["metrics"]["distinguishable"] is True

    dynamics = get("/api/dynamics?k=3000")
    assert len(dynamics["M_empty"]) == len(dynamics["M_loaded"]) == 6
    assert all(len(row) == 6 for row in dynamics["M_empty"] + dynamics["M_loaded"])
    assert len(dynamics["g_empty"]) == len(dynamics["g_loaded"]) == 6
    assert len(dynamics["J_tcp"]) == 3
    assert all(len(row) == 6 for row in dynamics["J_tcp"])

    invalid = Request(
        BASE + "/api/run",
        json.dumps({**common, "mass": 9, "collision": False}).encode(),
        {"Content-Type": "application/json"},
    )
    expect_error(invalid, 400)
    cross_origin = Request(
        BASE + "/api/run",
        json.dumps({**common, "collision": False}).encode(),
        {"Content-Type": "application/json", "Origin": "https://example.com"},
    )
    expect_error(cross_origin, 403)
    expect_error(Request(BASE + "/api/not-found"), 404)

    # 브라우저를 열었을 때 이해하기 쉬운 기본 무충돌 상태로 돌려놓는다.
    post({**common, "collision": False})
    print("PASS: dashboard API, sensors, dense-fix, custom payload, dynamics, validation")


if __name__ == "__main__":
    main()
