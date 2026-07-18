#!/usr/bin/env python3
"""모터 레지스터 통신 확인 도구.

무부하·저속 상태에서 DWORD WORD 순서와 모니터 주소를 확인할 때 사용한다.
"""

import motor_config as cfg
from motor_driver import MotorDriver


def main() -> None:
    motor = MotorDriver()
    motor.verify_connection()

    print("=== 모터 레지스터 확인 ===")
    print(f"DWORD_WORD_ORDER : {cfg.DWORD_WORD_ORDER}")
    print(f"alarm           : 0x{motor.read_u16(cfg.REG_ALARM_STATUS):04X}")
    print(f"run_status      : 0x{motor.read_u16(cfg.REG_RUN_STATUS):04X}")
    print(f"speed_rpm       : {motor.read_i16(cfg.REG_CURRENT_SPEED)}")
    print(f"current_raw     : {motor.read_u16(cfg.REG_CURRENT_CURRENT)}")
    print(f"position        : {motor.read_i32(cfg.REG_CURRENT_POSITION)}")
    print(f"pulse_count     : {motor.read_i32(cfg.REG_CURRENT_PULSE_COUNT)}")
    print(f"position_error  : {motor.read_i32(cfg.REG_POSITION_ERROR)}")


if __name__ == "__main__":
    main()
