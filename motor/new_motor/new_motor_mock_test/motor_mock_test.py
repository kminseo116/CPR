#!/usr/bin/env python3
"""실제 모터 없이 motor_main 알고리즘을 검증하는 모의시험 실행기.

실행:
    python3 motor_mock_test.py

시험 항목:
1. 로드셀 접촉 후 지정 횟수만큼 왕복하고 INITIAL_POS 복귀
2. /motor_stop 발생 시 INITIAL_POS 복귀 및 Servo ON 유지
3. 로드셀 정지 요청 시 INITIAL_POS 복귀 및 Servo ON 유지
4. 과전류 알람 시 초기 위치로 복귀하지 않고 Servo ON 유지
"""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
import sys
import types
from unittest.mock import patch


# ---------------------------------------------------------------------------
# 실제 장비용 라이브러리가 없어도 시험할 수 있도록 최소 모듈을 만든다.
# Jetson에 pyserial, rclpy, minimalmodbus가 설치되어 있어도 이 시험에서는
# 실제 포트나 ROS 노드를 열지 않는다.
# ---------------------------------------------------------------------------

serial_stub = types.ModuleType("serial")
serial_stub.PARITY_NONE = "N"
serial_stub.EIGHTBITS = 8
serial_stub.STOPBITS_ONE = 1
sys.modules.setdefault("serial", serial_stub)

rclpy_stub = types.ModuleType("rclpy")
rclpy_stub.init = lambda: None
rclpy_stub.ok = lambda: True
rclpy_stub.shutdown = lambda: None
rclpy_stub.spin_once = lambda *args, **kwargs: None
sys.modules.setdefault("rclpy", rclpy_stub)


class MotorError(RuntimeError):
    pass


@dataclass
class MotorStatus:
    position: int
    speed_rpm: int
    current_raw: int
    position_error: int
    alarm: int
    run_status: int


class MotorDriverTypeStub:
    @staticmethod
    def decode_alarm(alarm: int) -> list[str]:
        import motor_config as local_cfg
        names: list[str] = []
        if alarm & local_cfg.ALARM_OVERVOLTAGE:
            names.append("OVERVOLTAGE")
        if alarm & local_cfg.ALARM_UNDERVOLTAGE:
            names.append("UNDERVOLTAGE")
        if alarm & local_cfg.ALARM_OVERCURRENT:
            names.append("OVERCURRENT")
        if alarm & local_cfg.ALARM_POSITION_DEVIATION:
            names.append("POSITION_DEVIATION")
        return names


motor_driver_stub = types.ModuleType("motor_driver")
motor_driver_stub.MotorDriver = MotorDriverTypeStub
motor_driver_stub.MotorError = MotorError
motor_driver_stub.MotorStatus = MotorStatus
sys.modules["motor_driver"] = motor_driver_stub

motor_ros_stub = types.ModuleType("motor_ros_node")
motor_ros_stub.MotorRosBridge = object
sys.modules["motor_ros_node"] = motor_ros_stub

import motor_config as cfg
import motor_main as alg
from mock_components import MockMotorDriver, MockMotorRosBridge, MockScenario


@contextmanager
def fast_test_config():
    """시험 시간을 줄이기 위해 설정값을 임시로 변경한다."""
    original = {
        "REPEAT_COUNT": cfg.REPEAT_COUNT,
        "STATUS_PERIOD_S": cfg.STATUS_PERIOD_S,
        "CONTACT_SETTLE_S": cfg.CONTACT_SETTLE_S,
        "CURRENT_BASELINE_SAMPLE_COUNT": cfg.CURRENT_BASELINE_SAMPLE_COUNT,
        "CURRENT_BASELINE_SAMPLE_PERIOD_S": cfg.CURRENT_BASELINE_SAMPLE_PERIOD_S,
        "POSITION_TOLERANCE_UNITS": cfg.POSITION_TOLERANCE_UNITS,
    }

    cfg.REPEAT_COUNT = 3
    cfg.STATUS_PERIOD_S = 0.0
    cfg.CONTACT_SETTLE_S = 0.0
    cfg.CURRENT_BASELINE_SAMPLE_COUNT = 3
    cfg.CURRENT_BASELINE_SAMPLE_PERIOD_S = 0.0
    cfg.POSITION_TOLERANCE_UNITS = 1

    try:
        yield
    finally:
        for key, value in original.items():
            setattr(cfg, key, value)


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: actual={actual}, expected={expected}")


def assert_true(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def execute_run_once(scenario: MockScenario):
    initial_pos = 100_000
    driver = MockMotorDriver(scenario, initial_position=initial_pos)
    ros = MockMotorRosBridge(driver, scenario)

    driver.verify_connection()
    driver.reset()
    driver.setup_runtime()
    driver.enable()

    with patch("builtins.input", return_value="5.0"):
        alg.run_once(
            driver=driver,
            ros=ros,
            initial_pos=initial_pos,
        )

    return initial_pos, driver, ros


def test_normal_completion() -> None:
    scenario = MockScenario(
        name="정상 반복 완료",
        contact_offset_units=900,
    )
    initial_pos, driver, ros = execute_run_once(scenario)

    assert_equal(ros.compression_count, cfg.REPEAT_COUNT, "압박 횟수")
    assert_equal(driver.position, initial_pos, "정상 완료 후 INITIAL_POS")
    assert_true(driver.enabled, "정상 완료 후 Servo ON이어야 함")
    assert_equal(ros.contact_source, "LOADCELL", "접촉 검출 소스")
    assert_equal(ros.state, "STOPPED_SERVO_ON", "최종 상태")


def test_motor_stop() -> None:
    scenario = MockScenario(
        name="motor_stop 정상 정지",
        contact_offset_units=900,
        stop_mode="MOTOR_STOP",
        stop_after_compressions=1,
    )
    initial_pos, driver, ros = execute_run_once(scenario)

    assert_equal(driver.position, initial_pos, "/motor_stop 후 INITIAL_POS")
    assert_true(driver.enabled, "/motor_stop 후 Servo ON이어야 함")
    assert_equal(ros.state, "STOPPED_SERVO_ON", "최종 상태")


def test_loadcell_stop() -> None:
    scenario = MockScenario(
        name="로드셀 정지",
        contact_offset_units=900,
        stop_mode="LOADCELL_STOP",
        stop_after_compressions=1,
    )
    initial_pos, driver, ros = execute_run_once(scenario)

    assert_equal(driver.position, initial_pos, "로드셀 정지 후 INITIAL_POS")
    assert_true(driver.enabled, "로드셀 정지 후 Servo ON이어야 함")
    assert_equal(ros.state, "STOPPED_SERVO_ON", "최종 상태")


def test_overcurrent_fault() -> None:
    initial_pos = 100_000
    contact_offset = 900
    start_pos = initial_pos + contact_offset
    depth_units = alg.mm_to_position_units(50.0) * cfg.MOTOR_DIRECTION_SIGN
    end_pos = start_pos + depth_units

    scenario = MockScenario(
        name="과전류 고장",
        contact_offset_units=contact_offset,
        overcurrent_position=start_pos + abs(depth_units) // 3,
    )
    driver = MockMotorDriver(scenario, initial_position=initial_pos)
    ros = MockMotorRosBridge(driver, scenario)
    driver.enable()

    error_raised = False
    with patch("builtins.input", return_value="5.0"):
        try:
            alg.run_once(
                driver=driver,
                ros=ros,
                initial_pos=initial_pos,
            )
        except MotorError:
            error_raised = True

    assert_true(error_raised, "과전류에서 MotorError가 전달되어야 함")
    assert_true(driver.position != initial_pos, "과전류 시 자동 초기 복귀하면 안 됨")
    assert_true(driver.enabled, "과전류 시 Servo ON을 유지해야 함")
    assert_true(
        ros.state.startswith("FAULT_LATCHED_"),
        "과전류 시 FAULT_LATCHED 상태여야 함",
    )
    assert_true(driver.position < end_pos or driver.position > start_pos, "고장 위치 확인")


def main() -> None:
    tests = [
        test_normal_completion,
        test_motor_stop,
        test_loadcell_stop,
        test_overcurrent_fault,
    ]

    print("\n=== 모터 없는 CPR 알고리즘 Mock 테스트 ===")
    passed = 0

    with fast_test_config():
        for test in tests:
            print(f"\n--- {test.__name__} ---")
            try:
                test()
            except Exception as exc:
                print(f"[FAIL] {test.__name__}: {exc}")
            else:
                passed += 1
                print(f"[PASS] {test.__name__}")

    print(f"\n=== 결과: {passed}/{len(tests)} PASS ===")
    if passed != len(tests):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
