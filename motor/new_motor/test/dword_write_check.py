#!/usr/bin/env python3
"""현재 위치를 절대 위치 명령 레지스터에 쓰기만 하는 시험."""

import motor_config as cfg
from motor_driver import MotorDriver


def main() -> None:
    driver = MotorDriver()

    try:
        driver.verify_connection()
        current_position = driver.read_position()

        print(f"현재 위치: {current_position}")
        print(
            f"0x{cfg.REG_ABSOLUTE_POSITION:04X}에 "
            "같은 값을 씁니다."
        )

        # 모터 기동 명령(REG_MOTOR_START)은 보내지 않는다.
        driver.write_i32(
            cfg.REG_ABSOLUTE_POSITION,
            current_position,
        )

        readback = driver.read_i32(cfg.REG_ABSOLUTE_POSITION)
        print(f"절대 위치 명령 읽기값: {readback}")

        if readback == current_position:
            print("DWORD 쓰기 및 읽기 검증 성공")
        else:
            print("DWORD 쓰기 검증 실패")
    finally:
        try:
            driver.stop()
        except Exception:
            pass


if __name__ == "__main__":
    main()
