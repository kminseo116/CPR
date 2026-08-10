#!/usr/bin/env python3
"""새 RS485 서보모터의 현재 상태를 읽기 전용으로 확인한다.

이 파일은 레지스터 쓰기, Servo ON/OFF, 모터 기동 명령을 보내지 않는다.
각 항목을 독립적으로 읽으므로 일부 주소가 실패해도 나머지는 계속 확인한다.
"""

from collections.abc import Callable
from typing import TypeVar

import motor_config as cfg
from motor_driver import MotorDriver


T = TypeVar("T")


CONTROL_MODE_NAMES = {
    0: "외부 펄스 위치 모드",
    1: "RS485 위치 모드",
    2: "RS485 속도 모드",
    3: "외부 단자 속도 모드",
    4: "RS485 토크 모드",
    5: "외부 단자 토크 모드",
}


def safe_read(
    label: str,
    address: int,
    operation: Callable[[], T],
) -> T | None:
    """한 항목의 통신 실패가 전체 모니터를 중단하지 않게 한다."""
    try:
        value = operation()
        print(f"[OK]   {label:<24} 0x{address:04X} = {value}")
        return value
    except Exception as error:
        print(f"[FAIL] {label:<24} 0x{address:04X} = {error}")
        return None


def decode_run_status(value: int | None) -> list[str]:
    """현재 설정에 정의된 운전 상태 비트를 문자열로 바꾼다."""
    if value is None:
        return []

    states: list[str] = []
    if value & cfg.RUN_TARGET_REACHED:
        states.append("TARGET_REACHED")
    if value & cfg.RUN_ENABLED:
        states.append("ENABLED")
    if value & cfg.RUN_RUNNING:
        states.append("RUNNING")
    if value & cfg.RUN_SPEED_REACHED:
        states.append("SPEED_REACHED")
    return states


def main() -> None:
    print("====================================================")
    print(" 새 RS485 서보모터 현재 상태 모니터 (읽기 전용)")
    print("====================================================")
    print(f"포트: {cfg.PORT}")
    print(f"장치 주소: {cfg.SLAVE_ID}")
    print(f"통신 속도: {cfg.BAUDRATE}")
    print("주의: 설정 변경 및 모터 기동 명령을 보내지 않습니다.")

    driver = MotorDriver()

    try:
        print("\n================ 제어 설정 ================")
        control_mode = safe_read(
            "제어 모드",
            cfg.REG_CONTROL_MODE,
            lambda: driver.read_u16(cfg.REG_CONTROL_MODE),
        )
        if control_mode is not None:
            print(
                "       제어 모드 해석: "
                + CONTROL_MODE_NAMES.get(control_mode, "알 수 없는 모드")
            )

        safe_read(
            "인에이블 방식",
            cfg.REG_ENABLE_METHOD,
            lambda: driver.read_u16(cfg.REG_ENABLE_METHOD),
        )
        safe_read(
            "내부 Servo 인에이블",
            cfg.REG_INTERNAL_ENABLE,
            lambda: driver.read_u16(cfg.REG_INTERNAL_ENABLE),
        )
        print(
            "[INFO] 위치 명령 모드                "
            f"0x{cfg.REG_POSITION_COMMAND_MODE:04X} "
            "= 읽기 미지원(명령용 레지스터)"
        )
        safe_read(
            "내부 속도 설정",
            cfg.REG_INTERNAL_SPEED,
            lambda: driver.read_u16(cfg.REG_INTERNAL_SPEED),
        )
        safe_read(
            "가속 설정",
            cfg.REG_ACCEL,
            lambda: driver.read_u16(cfg.REG_ACCEL),
        )
        safe_read(
            "감속 설정",
            cfg.REG_DECEL,
            lambda: driver.read_u16(cfg.REG_DECEL),
        )

        print("\n================ 현재 상태 ================")
        alarm = safe_read(
            "알람 상태",
            cfg.REG_ALARM_STATUS,
            lambda: driver.read_u16(cfg.REG_ALARM_STATUS),
        )
        if alarm is not None:
            alarms = driver.decode_alarm(alarm)
            print(
                "       알람 해석: "
                + (", ".join(alarms) if alarms else "알람 없음")
            )

        run_status = safe_read(
            "운전 상태",
            cfg.REG_RUN_STATUS,
            lambda: driver.read_u16(cfg.REG_RUN_STATUS),
        )
        states = decode_run_status(run_status)
        print(
            "       운전 상태 해석: "
            + (", ".join(states) if states else "활성 상태 비트 없음")
        )

        safe_read(
            "현재 속도(rpm)",
            cfg.REG_CURRENT_SPEED,
            lambda: driver.read_i16(cfg.REG_CURRENT_SPEED),
        )
        safe_read(
            "현재 전류(raw)",
            cfg.REG_CURRENT_CURRENT,
            lambda: driver.read_u16(cfg.REG_CURRENT_CURRENT),
        )
        safe_read(
            "현재 전압(raw)",
            cfg.REG_CURRENT_VOLTAGE,
            lambda: driver.read_u16(cfg.REG_CURRENT_VOLTAGE),
        )

        print("\n========== 설정 파일 기준 위치 모니터 ==========")
        safe_read(
            "현재 펄스 카운터",
            cfg.REG_CURRENT_PULSE_COUNT,
            lambda: driver.read_i32(cfg.REG_CURRENT_PULSE_COUNT),
        )
        safe_read(
            "현재 위치",
            cfg.REG_CURRENT_POSITION,
            driver.read_position,
        )
        safe_read(
            "현재 위치 오차",
            cfg.REG_POSITION_ERROR,
            lambda: driver.read_i32(cfg.REG_POSITION_ERROR),
        )

        print("\n========== 매뉴얼 주소 차이 진단 ==========")
        print("DWORD는 시작 주소부터 WORD 2개를 함께 읽습니다.")
        for address, label in (
            (0x003A, "상세설명 pulse 후보"),
            (0x003B, "목록 pulse 후보"),
            (0x003C, "상세설명 position 후보"),
            (0x003D, "목록 position 후보"),
            (0x003E, "상세설명 error 후보"),
            (0x003F, "목록 error 후보"),
        ):
            safe_read(
                label,
                address,
                lambda address=address: driver.read_i32(address),
            )

        print("\n====================================================")
        print("상태 확인 완료: 어떤 레지스터도 쓰지 않았습니다.")
        print("====================================================")
    finally:
        serial_port = driver.instrument.serial
        if serial_port.is_open:
            serial_port.close()


if __name__ == "__main__":
    main()
