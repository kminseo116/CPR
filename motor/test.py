#!/usr/bin/env python3
"""
check_p0201_abs_system.py

목적:
- 절대 위치 검출 시스템 설정값 P02-01 확인
- 현재 모터가 절대 위치 시스템으로 설정되어 있는지 판단
- 읽기만 수행
"""

import motor_config as cfg
import motor_driver as drv


ADDR_P02_01 = 0x0201  # Absolute position detection system 후보


def read_u16_fc3(motor, addr, label):
    try:
        value = motor.read_register(addr, 0, functioncode=3, signed=False)
        print(f"[OK] {label} U16 FC3 addr=0x{addr:04X} -> {value}")
        return value
    except Exception as e:
        print(f"[FAIL] {label} U16 FC3 addr=0x{addr:04X} -> {e}")
        return None


def read_u16_fc4(motor, addr, label):
    try:
        value = motor.read_register(addr, 0, functioncode=4, signed=False)
        print(f"[OK] {label} U16 FC4 addr=0x{addr:04X} -> {value}")
        return value
    except Exception as e:
        print(f"[FAIL] {label} U16 FC4 addr=0x{addr:04X} -> {e}")
        return None


def main():
    print("============================================")
    print(" P02-01 절대 위치 검출 시스템 확인")
    print("============================================")
    print(f"PORT     = {cfg.PORT}")
    print(f"SLAVE_ID = {cfg.SLAVE_ID}")

    motor = drv.create_motor_connection(cfg.PORT, cfg.SLAVE_ID)

    print("\n================ READ P02-01 ================")

    value_fc3 = read_u16_fc3(motor, ADDR_P02_01, "P02-01")
    value_fc4 = read_u16_fc4(motor, ADDR_P02_01, "P02-01")

    value = value_fc3 if value_fc3 is not None else value_fc4

    print("\n================ RESULT ================")

    if value is None:
        print("P02-01을 읽지 못했습니다.")
        print("주소가 다르거나, 해당 파라미터가 Modbus에서 읽기 제한일 수 있습니다.")
    elif value == 0:
        print("P02-01 = 0")
        print("현재 설정: 증분 시스템")
        print("판단: 다회전 엔코더가 있어도 절대 위치 시스템으로 동작하지 않을 수 있습니다.")
        print("다음 단계: P02-01을 1로 설정하는 절차가 필요할 수 있습니다.")
    elif value == 1:
        print("P02-01 = 1")
        print("현재 설정: 절대 시스템")
        print("판단: 절대 위치 시스템은 켜져 있습니다.")
        print("다음 단계: 절대 엔코더 초기화/알람 클리어/위치값 반영 절차를 확인해야 합니다.")
    else:
        print(f"P02-01 = {value}")
        print("예상 범위 0/1이 아닙니다. 매뉴얼 값 확인이 필요합니다.")

    print("========================================")


if __name__ == "__main__":
    main()