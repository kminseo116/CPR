#!/usr/bin/env python3
# """모터 레지스터 통신 확인 도구.

# 무부하·저속 상태에서 DWORD WORD 순서와 모니터 주소를 확인할 때 사용한다.
# """

# import motor_config as cfg
# from motor_driver import MotorDriver


# def main() -> None:
#     motor = MotorDriver()
#     motor.verify_connection()

#     print("=== 모터 레지스터 확인 ===")
#     print(f"DWORD_WORD_ORDER : {cfg.DWORD_WORD_ORDER}")
#     print(f"alarm           : 0x{motor.read_u16(cfg.REG_ALARM_STATUS):04X}")
#     print(f"run_status      : 0x{motor.read_u16(cfg.REG_RUN_STATUS):04X}")
#     print(f"speed_rpm       : {motor.read_i16(cfg.REG_CURRENT_SPEED)}")
#     print(f"current_raw     : {motor.read_u16(cfg.REG_CURRENT_CURRENT)}")
#     print(f"position        : {motor.read_i32(cfg.REG_CURRENT_POSITION)}")
#     print(f"pulse_count     : {motor.read_i32(cfg.REG_CURRENT_PULSE_COUNT)}")
#     print(f"position_error  : {motor.read_i32(cfg.REG_POSITION_ERROR)}")


# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3


<<<<<<< Updated upstream
"""
매뉴얼에서 서로 다르게 적힌 위치 관련 레지스터 주소를 비교해서 어느 주소가 실제 위치값인지 확인하는 진단 코드
=======
운동 명령은 전송하지 않는다. 매뉴얼의 목록/상세 페이지에 서로 다르게
기재된 모니터 주소를 같은 실행에서 모두 읽어 비교한다.
>>>>>>> Stashed changes
"""

from motor_driver import MotorDriver


MONITOR_ADDRESS_SETS = {
    "register_list": {
        "pulse_count": 0x003B,
        "position": 0x003D,
        "position_error": 0x003F,
    },
    "detail_pages": {
        "pulse_count": 0x003A,
        "position": 0x003C,
        "position_error": 0x003E,
    },
}


def print_i32(motor: MotorDriver, label: str, address: int) -> None:
    """한 DWORD 주소의 값 또는 통신 오류를 출력한다."""
    try:
        value = motor.read_i32(address)
    except Exception as exc:
        print(f"{label:<28}: ERROR ({exc})")
        return

    print(f"{label:<28}: {value:>12}  [0x{address:04X}]")


def main() -> None:
    motor = MotorDriver()
    motor.verify_connection()

<<<<<<< Updated upstream
    print("=== 위치 레지스터 후보 비교 ===")

    print("\n[레지스터 목록 기준]")
    print(f"pulse  0x003B = {motor.read_i32(0x003B)}")
    print(f"pos    0x003D = {motor.read_i32(0x003D)}")
    print(f"error  0x003F = {motor.read_i32(0x003F)}")

    print("\n[상세 설명 기준]")
    print(f"pulse  0x003A = {motor.read_i32(0x003A)}")
    print(f"pos    0x003C = {motor.read_i32(0x003C)}")
    print(f"error  0x003E = {motor.read_i32(0x003E)}")
=======
    print("=== 모터 레지스터 확인 ===")
    print(f"DWORD_WORD_ORDER : {cfg.DWORD_WORD_ORDER}")
    control_mode = motor.read_u16(cfg.REG_CONTROL_MODE)
    mode_ok = control_mode == cfg.EXPECTED_CONTROL_MODE
    print(
        f"control_mode     : {control_mode} "
        f"({'OK' if mode_ok else 'BLOCK: expected 1'})"
    )
    print(f"alarm           : 0x{motor.read_u16(cfg.REG_ALARM_STATUS):04X}")
    print(f"run_status      : 0x{motor.read_u16(cfg.REG_RUN_STATUS):04X}")
    print(f"speed_rpm       : {motor.read_u16(cfg.REG_CURRENT_SPEED)}")
    print(f"current_raw     : {motor.read_u16(cfg.REG_CURRENT_CURRENT)}")
    print(f"accel_0x002E    : {motor.read_u16(cfg.REG_ACCEL)}")
    print(f"decel_0x002F    : {motor.read_u16(cfg.REG_DECEL)}")

    print("\n=== 충돌 주소 비교 (축을 움직이기 전/후 각각 실행) ===")
    for source, addresses in MONITOR_ADDRESS_SETS.items():
        print(f"[{source}]")
        for name, address in addresses.items():
            print_i32(motor, f"{source}.{name}", address)

    print("\n판정: 실제 축 이동량과 같은 방향·크기로 변하는 주소 세트를 선택하세요.")
>>>>>>> Stashed changes


if __name__ == "__main__":
    main()
