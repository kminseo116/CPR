#!/usr/bin/env python3
"""실제 motor_main_test.py의 안전 분기를 하드웨어 없이 검증한다."""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
import sys
import types
from unittest.mock import patch


# 실제 직렬통신/ROS 패키지 없이 대상 모듈을 import하기 위한 경계 stub.
serial_stub = types.ModuleType("serial")
serial_stub.PARITY_NONE = "N"
sys.modules.setdefault("serial", serial_stub)


class MotorError(RuntimeError):
    pass


@dataclass(frozen=True)
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
        return [] if alarm == 0 else [hex(alarm)]


driver_module = types.ModuleType("motor_driver")
driver_module.MotorDriver = MotorDriverTypeStub
driver_module.MotorError = MotorError
driver_module.MotorStatus = MotorStatus
sys.modules["motor_driver"] = driver_module

ros_module = types.ModuleType("motor_ros_node")
ros_module.MotorRosBridge = object
sys.modules["motor_ros_node"] = ros_module

import motor_config as cfg
import motor_main_test as target


class FakeDriver:
    def __init__(self) -> None:
        self.position = cfg.START_FEEDBACK_POSITION
        self.target = self.position
        self.running = False
        self.stop_count = 0

    @staticmethod
    def output_rpm_to_speed_setting(rpm: float) -> int:
        return round(rpm)

    def read_position(self) -> int:
        return self.position

    def move_incremental(self, command: int, speed: int) -> None:
        del speed
        delta = round(
            command
            * cfg.FEEDBACK_COUNTS_PER_MOTOR_REV
            / cfg.COMMAND_UNITS_PER_MOTOR_REV
        )
        self.target = self.position + delta
        self.running = True

    def read_status(self) -> MotorStatus:
        if self.running:
            remaining = self.target - self.position
            step = min(abs(remaining), 4000)
            self.position += step if remaining > 0 else -step
            if self.position == self.target:
                self.running = False
        error = round(
            (self.target - self.position)
            * cfg.COMMAND_UNITS_PER_MOTOR_REV
            / cfg.FEEDBACK_COUNTS_PER_MOTOR_REV
        )
        return MotorStatus(
            self.position,
            100 if self.running else 0,
            10,
            error,
            0,
            0,
        )

    def stop(self) -> None:
        self.stop_count += 1
        self.running = False
        self.target = self.position


class FakeRos:
    def __init__(self, driver: FakeDriver) -> None:
        self.driver = driver
        self.loadcell_total = 0.0
        self.motor_stop = False
        self.loadcell_stop_request = False
        self.contact_source = None
        self.states: list[str] = []
        self.contact_position = driver.position + 8000

    def spin_once(self, timeout_sec: float = 0.0) -> None:
        del timeout_sec
        if self.driver.position >= self.contact_position:
            self.loadcell_total = cfg.CONTACT_FORCE_N + 1.0

    def should_stop(self) -> bool:
        return self.motor_stop or self.loadcell_stop_request

    def is_loadcell_stale(self, timeout_s: float) -> bool:
        del timeout_s
        return False

    def clear_start_stop_flags(self) -> None:
        self.motor_stop = False
        self.loadcell_stop_request = False

    def publish_state(self, state: str) -> None:
        self.states.append(state)

    def publish_contact_source(self, source: str) -> None:
        self.contact_source = source

    def publish_motor_status(self, position: int, current: int) -> None:
        del position, current


@contextmanager
def no_sleep():
    with patch.object(target.time, "sleep", return_value=None):
        yield


def test_loadcell_contact() -> None:
    driver = FakeDriver()
    ros = FakeRos(driver)
    with no_sleep():
        position = target.search_contact_with_loadcell(driver, ros)
    assert ros.contact_source == "LOADCELL"
    assert position >= ros.contact_position
    assert driver.stop_count >= 1


def test_motor_stop_return_order() -> None:
    driver = FakeDriver()
    ros = FakeRos(driver)
    ros.motor_stop = True

    def verify_return(_driver, checked_ros, _position):
        assert not checked_ros.should_stop(), (
            "초기 복귀 전에 motor_stop 플래그가 해제되지 않았습니다"
        )

    with patch.object(target, "return_to_feedback_position", side_effect=verify_return):
        target.normal_stop_and_return(driver, ros, cfg.START_FEEDBACK_POSITION)


def test_ctrl_c_return_order() -> None:
    driver = FakeDriver()
    ros = FakeRos(driver)
    ros.motor_stop = True

    def verify_return(_driver, checked_ros, _position):
        assert not checked_ros.should_stop()

    with patch.object(target, "return_to_feedback_position", side_effect=verify_return):
        target.interrupt_stop_and_return(driver, ros, cfg.START_FEEDBACK_POSITION)


def test_general_error_calls_fault_stop_correctly() -> None:
    driver = FakeDriver()
    ros = FakeRos(driver)
    calls = []

    def record_fault(*args):
        calls.append(args)

    with (
        patch.object(target, "prepare_zero_start_position", return_value=cfg.START_FEEDBACK_POSITION),
        patch.object(target, "print_runtime_settings"),
        patch.object(target, "read_depth_cm", return_value=1.0),
        patch.object(target, "search_contact_with_loadcell", side_effect=RuntimeError("test fault")),
        patch.object(target, "fault_stop", side_effect=record_fault),
    ):
        try:
            target.run_once(driver, ros, cfg.START_FEEDBACK_POSITION)
        except RuntimeError as exc:
            assert str(exc) == "test fault"
    assert len(calls) == 1
    assert len(calls[0]) == 4, f"fault_stop 인자 수가 {len(calls[0])}개입니다"


def main() -> None:
    tests = (
        test_loadcell_contact,
        test_motor_stop_return_order,
        test_ctrl_c_return_order,
        test_general_error_calls_fault_stop_correctly,
    )
    passed = 0
    print("=== motor_main_test.py 무모터 안전 시뮬레이션 ===")
    for test in tests:
        try:
            test()
        except Exception as exc:
            print(f"[FAIL] {test.__name__}: {type(exc).__name__}: {exc}")
        else:
            passed += 1
            print(f"[PASS] {test.__name__}")
    print(f"=== 결과: {passed}/{len(tests)} PASS ===")
    raise SystemExit(0 if passed == len(tests) else 1)


if __name__ == "__main__":
    main()
