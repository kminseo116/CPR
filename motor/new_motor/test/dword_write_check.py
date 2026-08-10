#!/usr/bin/env python3
"""현재 위치를 절대 위치 명령 레지스터에 쓰기만 하는 시험.

0x0042 명령 레지스터는 표준 읽기 응답을 제공하지 않으므로 다시 읽지 않는다.
모터 기동 명령은 보내지 않고, 현재 위치 모니터 레지스터만 확인한다.
"""

import motor_config as cfg
from motor_driver import MotorDriver


def main() -> None:
    # 이 파일은 통신 프레임 자체를 확인하는 진단 시험이므로 원문을 표시한다.
    cfg.SHOW_MODBUS_DEBUG = True
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

        position_after_write = driver.read_position()
        print(f"쓰기 후 현재 위치: {position_after_write}")
        print("DWORD 요청 프레임 및 에코 확인 성공")
        print("주의: 실제 명령 반영 여부는 소량 이동 시험에서 확인해야 합니다.")
    finally:
        try:
            driver.stop()
        except Exception:
            pass


if __name__ == "__main__":
    main()
