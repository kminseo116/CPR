"""
motor_diagnostics.py

오류 발생 시 모터 드라이버 상태를 확인하기 위한 진단 코드이다.
main에서 예외가 발생하면 diagnose_motor()를 호출해 현재 상태를 출력한다.
"""

import motor_config as cfg
import motor_driver as drv


def diagnose_motor(motor):
    """
    모터 상태 진단 출력.

    확인 항목:
    - 현재 위치
    - 현재 전류
    - 현재 속도
    - fault code
    - bus status
    - DI 상태
    - position error
    """
    print("\n========== MOTOR DIAGNOSTICS ==========")

    try:
        pos = drv.read_current_position(motor)
        print(f"[DIAG] current position: {pos}")
    except Exception as e:
        print(f"[DIAG ERROR] position read failed: {e}")

    try:
        current_a = drv.read_current_a(motor)
        print(f"[DIAG] current: {current_a:.3f} A")
    except Exception as e:
        print(f"[DIAG ERROR] current read failed: {e}")

    try:
        speed = drv.read_i16(motor, cfg.ADDR_P0B_00)
        print(f"[DIAG] speed: {speed} rpm")
    except Exception as e:
        print(f"[DIAG ERROR] speed read failed: {e}")

    try:
        fault = drv.read_u16(motor, cfg.ADDR_P02_34)
        print(f"[DIAG] fault code: {fault}")
    except Exception as e:
        print(f"[DIAG ERROR] fault read failed: {e}")

    try:
        bus_status = drv.read_u16(motor, cfg.ADDR_P0B_04)
        print(f"[DIAG] bus status: {bus_status}")
    except Exception as e:
        print(f"[DIAG ERROR] bus status read failed: {e}")

    try:
        di_status = drv.read_u16(motor, cfg.ADDR_P0B_03)
        print(f"[DIAG] DI status: {di_status}")
    except Exception as e:
        print(f"[DIAG ERROR] DI status read failed: {e}")

    try:
        pos_error = drv.read_i32(motor, cfg.ADDR_P0B_53)
        print(f"[DIAG] position error: {pos_error}")
    except Exception as e:
        print(f"[DIAG ERROR] position error read failed: {e}")

    print("=======================================\n")