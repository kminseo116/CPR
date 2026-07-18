#!/usr/bin/env python3
"""실제 모터와 ROS 토픽 없이 CPR 알고리즘을 시험하기 위한 Mock 구성요소."""

from __future__ import annotations

from dataclasses import dataclass
import time

import motor_config as cfg
from motor_driver import MotorError, MotorStatus


@dataclass
class MockScenario:
    """한 번의 모의시험 조건."""

    name: str
    contact_offset_units: int
    stop_mode: str = "NONE"
    stop_after_compressions: int = 1
    overcurrent_position: int | None = None
    loadcell_stale: bool = False


class MockMotorDriver:
    """MotorDriver와 같은 인터페이스를 제공하는 가상 모터."""

    def __init__(self, scenario: MockScenario, initial_position: int = 100_000) -> None:
        self.scenario = scenario
        self.position = int(initial_position)
        self.initial_position = int(initial_position)
        self.target_position = int(initial_position)
        self.speed_rpm = 0
        self.enabled = False
        self.running = False
        self.stopped = True
        self.command_history: list[tuple[str, int | None]] = []

        self.contact_position = (
            self.initial_position
            + cfg.MOTOR_DIRECTION_SIGN * abs(scenario.contact_offset_units)
        )

    def verify_connection(self) -> None:
        self.command_history.append(("verify_connection", None))

    def reset(self) -> None:
        self.command_history.append(("reset", None))

    def setup_runtime(self) -> None:
        self.command_history.append(("setup_runtime", None))

    def enable(self) -> None:
        self.enabled = True
        self.command_history.append(("enable", None))

    def disable(self) -> None:
        self.stop()
        self.enabled = False
        self.command_history.append(("disable", None))

    def stop(self) -> None:
        self.running = False
        self.stopped = True
        self.target_position = self.position
        self.command_history.append(("stop", self.position))

    def move_absolute(self, target_position: int, speed_rpm: int) -> None:
        self.check_position_limit(target_position)
        self.target_position = int(target_position)
        self.speed_rpm = int(speed_rpm)
        self.running = True
        self.stopped = False
        self.command_history.append(("move", self.target_position))

    def read_position(self) -> int:
        return int(self.position)

    def _advance_motion(self) -> None:
        if not self.running:
            return

        remaining = self.target_position - self.position
        if remaining == 0:
            self.running = False
            self.stopped = True
            return

        # 한 번 읽을 때마다 목표까지 일정량 이동한다.
        step = max(1, abs(remaining) // 3)
        self.position += step if remaining > 0 else -step

        if abs(self.target_position - self.position) <= cfg.POSITION_TOLERANCE_UNITS:
            self.position = self.target_position
            self.running = False
            self.stopped = True

    def read_status(self) -> MotorStatus:
        self._advance_motion()

        alarm = 0
        current_raw = 20

        if self.scenario.overcurrent_position is not None:
            crossed = (
                self.position >= self.scenario.overcurrent_position
                if cfg.MOTOR_DIRECTION_SIGN > 0
                else self.position <= self.scenario.overcurrent_position
            )
            if crossed:
                alarm |= cfg.ALARM_OVERCURRENT
                current_raw = cfg.MAX_CURRENT_RAW + 100

        return MotorStatus(
            position=int(self.position),
            speed_rpm=self.speed_rpm if self.running else 0,
            current_raw=current_raw,
            position_error=int(self.target_position - self.position),
            alarm=alarm,
            run_status=(cfg.RUN_TARGET_REACHED if not self.running else 0),
        )

    @staticmethod
    def check_position_limit(position: int) -> None:
        if not cfg.MIN_POSITION_CMD <= int(position) <= cfg.MAX_POSITION_CMD:
            raise MotorError(f"Mock 위치 제한 초과: {position}")

    @staticmethod
    def decode_alarm(alarm: int) -> list[str]:
        names: list[str] = []
        if alarm & cfg.ALARM_OVERVOLTAGE:
            names.append("OVERVOLTAGE")
        if alarm & cfg.ALARM_UNDERVOLTAGE:
            names.append("UNDERVOLTAGE")
        if alarm & cfg.ALARM_OVERCURRENT:
            names.append("OVERCURRENT")
        if alarm & cfg.ALARM_POSITION_DEVIATION:
            names.append("POSITION_DEVIATION")
        return names

    @staticmethod
    def target_reached(status: MotorStatus, target_position: int) -> bool:
        return abs(status.position - target_position) <= cfg.POSITION_TOLERANCE_UNITS


class MockMotorRosBridge:
    """MotorRosBridge와 같은 인터페이스를 제공하는 가상 ROS 브리지."""

    def __init__(self, driver: MockMotorDriver, scenario: MockScenario) -> None:
        self.driver = driver
        self.scenario = scenario

        self.loadcell_total = 0.0
        self.loadcell_stop_request = False
        self.loadcell_status_code = 0
        self.motor_start = False
        self.motor_stop = False

        self.compression_count = 0
        self.compression_bpm = 0.0
        self.compression_depth_cm = 0.0
        self.cpr_start_time: float | None = None
        self.last_compression_time: float | None = None

        self.state = "IDLE"
        self.state_history: list[str] = []
        self.contact_source: str | None = None
        self.last_position = driver.position
        self.last_current_raw = 0

    def spin_once(self, timeout_sec: float = 0.0) -> None:
        del timeout_sec

        # 접촉 위치에 도달하면 가상의 로드셀 힘이 증가한다.
        reached_contact = (
            self.driver.position >= self.driver.contact_position
            if cfg.MOTOR_DIRECTION_SIGN > 0
            else self.driver.position <= self.driver.contact_position
        )
        self.loadcell_total = 6.0 if reached_contact else 0.0

        if (
            self.scenario.stop_mode == "MOTOR_STOP"
            and self.compression_count >= self.scenario.stop_after_compressions
        ):
            self.motor_stop = True

        if (
            self.scenario.stop_mode == "LOADCELL_STOP"
            and self.compression_count >= self.scenario.stop_after_compressions
        ):
            self.loadcell_stop_request = True

    def should_stop(self) -> bool:
        self.spin_once()
        return (
            self.motor_stop
            or self.loadcell_stop_request
            or self.loadcell_status_code >= 2
        )

    def is_loadcell_stale(self, timeout_s: float) -> bool:
        del timeout_s
        return self.scenario.loadcell_stale

    def clear_start_stop_flags(self) -> None:
        self.motor_start = False
        self.motor_stop = False
        self.loadcell_stop_request = False

    def publish_state(self, state: str) -> None:
        self.state = state
        self.state_history.append(state)
        print(f"[MOCK ROS STATE] {state}")

    def publish_contact_source(self, source: str) -> None:
        self.contact_source = source
        print(f"[MOCK CONTACT SOURCE] {source}")

    def publish_motor_status(self, position: int, current_raw: int) -> None:
        self.last_position = int(position)
        self.last_current_raw = int(current_raw)

    def reset_compression(self) -> None:
        self.compression_count = 0
        self.compression_bpm = 0.0
        self.compression_depth_cm = 0.0
        self.cpr_start_time = time.monotonic()
        self.last_compression_time = None

    def count_compression(self, depth_cm: float) -> None:
        now = time.monotonic()
        self.compression_count += 1
        if self.last_compression_time is not None:
            dt = now - self.last_compression_time
            if dt > 0:
                self.compression_bpm = 60.0 / dt
        self.last_compression_time = now
        self.compression_depth_cm = float(depth_cm)
        print(
            f"[MOCK CPR] count={self.compression_count}, "
            f"depth={self.compression_depth_cm:.2f} cm"
        )

    def destroy_node(self) -> None:
        pass
