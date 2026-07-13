#!/usr/bin/env python3
"""
read_motor_alarm.py

목적:
- 빨간 불/알람 발생 시 고장 코드와 상태값 확인
- 절대 위치 시스템 상태 확인
- P0D-20 절대 엔코더 리셋 명령값 확인
- 읽기만 수행
- 모터를 움직이지 않음
"""

import motor_config as cfg
import motor_driver as drv


# ============================================================
# BASIC / PARAMETER
# ============================================================

ADDR_P00_00 = 0x0000   # 모터 번호
ADDR_P02_01 = 0x0201   # 절대 위치 검출 시스템
ADDR_P02_34 = 0x0222   # 고장 코드

# ============================================================
# STATUS / MONITOR
# ============================================================

ADDR_P0B_03 = 0x0B03   # DI 입력 상태
ADDR_P0B_04 = 0x0B04   # 버스 상태워드
ADDR_P0B_07 = 0x0B07   # 위치 카운터
ADDR_P0B_26 = 0x0B1A   # 모선 전압 후보

ADDR_P0B_70 = 0x0B46   # 절대 엔코더 다회전 데이터
ADDR_P0B_71 = 0x0B47   # 절대 엔코더 단회전 데이터
ADDR_P0B_77 = 0x0B4D   # 절대 엔코더 절대 위치 low
ADDR_P0B_79 = 0x0B4F   # 절대 엔코더 절대 위치 high

# ============================================================
# P0D CONTROL PARAMETER
# ============================================================

ADDR_P0D_20 = 0x0D14   # 절대 엔코더 리셋 enable


def read_u16(motor, addr, label):
    try:
        value = motor.read_register(addr, 0, functioncode=3, signed=False)
        print(f"[OK] {label:<24} addr=0x{addr:04X} -> {value}")
        return value
    except Exception as e:
        print(f"[FAIL] {label:<24} addr=0x{addr:04X} -> {e}")
        return None


def read_i16(motor, addr, label):
    try:
        value = motor.read_register(addr, 0, functioncode=3, signed=True)
        print(f"[OK] {label:<24} addr=0x{addr:04X} -> {value}")
        return value
    except Exception as e:
        print(f"[FAIL] {label:<24} addr=0x{addr:04X} -> {e}")
        return None


def read_i32(motor, addr, label):
    try:
        regs = motor.read_registers(addr, 2, functioncode=3)

        low = regs[0]
        high = regs[1]

        value = (high << 16) | low

        if value >= (1 << 31):
            value -= (1 << 32)

        print(f"[OK] {label:<24} addr=0x{addr:04X} regs={regs} -> {value}")
        return value

    except Exception as e:
        print(f"[FAIL] {label:<24} addr=0x{addr:04X} -> {e}")
        return None


def decode_fault(code):
    print("\n================ 고장 코드 해석 참고 ================")

    if code is None:
        print("고장 코드를 읽지 못했습니다.")
        return

    print(f"P02-34 fault code = {code}")

    if code == 0:
        print("P02-34 = 0: Modbus상 고장 코드는 0입니다.")
        print("DS2 프로그램의 ErrCode도 함께 확인하는 것이 좋습니다.")
    elif code == 122:
        print("Er.122 가능성: 절대 위치 모드 제품 매칭 고장")
        print("절대값 모터 모델 불일치 또는 모터 모델 설정 오류 가능성.")
    elif code == 136:
        print("Er.136 가능성: 모터 ROM 데이터 검증 오류 또는 파라미터 미저장")
        print("모터/드라이버 모델, 엔코더선 연결, 노이즈 확인 필요.")
    elif code in [134, 135]:
        print("Er.A34/Er.A35 계열 가능성: 엔코더 통신/피드백/Z신호 이상")
        print("엔코더 케이블, 커넥터, 배터리 연결을 확인하세요.")
    else:
        print("이 코드의 정확한 의미는 DS2 프로그램의 ErrCode와 대조해야 합니다.")

    print("=====================================================")


def decode_p0d20(value):
    print("\n================ P0D-20 해석 참고 ================")

    if value is None:
        print("P0D-20을 읽지 못했습니다.")
        return

    print(f"P0D-20 absolute encoder reset enable = {value}")

    if value == 0:
        print("P0D-20 = 0: 정상 대기값입니다.")
        print("절대 엔코더 리셋 명령이 현재 걸려 있지 않습니다.")
    elif value == 1:
        print("P0D-20 = 1: 절대 엔코더 리셋 명령값입니다.")
        print("0x733 제거용으로 한 번 사용한 값일 수 있습니다.")
        print("리셋 후에는 가능하면 0으로 돌아가 있는 것이 안전합니다.")
    elif value == 2:
        print("P0D-20 = 2: 절대 엔코더 리셋 명령값입니다.")
        print("1로 해결되지 않을 때 사용하는 리셋 단계일 수 있습니다.")
        print("리셋 후에는 가능하면 0으로 돌아가 있는 것이 안전합니다.")
    else:
        print("P0D-20 값이 예상 범위 0/1/2가 아닙니다.")

    print("=================================================")


def main():
    print("============================================")
    print(" 모터 빨간불 / 알람 / 절대 엔코더 상태 확인")
    print("============================================")
    print(f"PORT     = {cfg.PORT}")
    print(f"SLAVE_ID = {cfg.SLAVE_ID}")
    print("주의: 이 코드는 읽기만 합니다. 모터를 움직이지 않습니다.")
    print("============================================")

    motor = drv.create_motor_connection(cfg.PORT, cfg.SLAVE_ID)

    print("\n================ BASIC ================")
    p0000 = read_u16(motor, ADDR_P00_00, "P00-00 motor no")
    p0201 = read_u16(motor, ADDR_P02_01, "P02-01 abs system")
    fault = read_u16(motor, ADDR_P02_34, "P02-34 fault code")

    print("\n================ P0D RESET PARAMETER ================")
    p0d20 = read_u16(motor, ADDR_P0D_20, "P0D-20 abs reset")

    print("\n================ STATUS ================")
    di = read_u16(motor, ADDR_P0B_03, "P0B-03 DI monitor")
    bus = read_u16(motor, ADDR_P0B_04, "P0B-04 bus status")
    vbus = read_u16(motor, ADDR_P0B_26, "P0B-26 bus voltage")

    print("\n================ POSITION / ENCODER ================")
    pos = read_i32(motor, ADDR_P0B_07, "P0B-07 pos counter")
    mt_i16 = read_i16(motor, ADDR_P0B_70, "P0B-70 multi-turn")
    mt_u16 = read_u16(motor, ADDR_P0B_70, "P0B-70 multi-turn")
    st_u16 = read_u16(motor, ADDR_P0B_71, "P0B-71 single-turn")
    abs_low = read_i32(motor, ADDR_P0B_77, "P0B-77 abs low")
    abs_high = read_i32(motor, ADDR_P0B_79, "P0B-79 abs high")

    decode_fault(fault)
    decode_p0d20(p0d20)

    print("\n================ SUMMARY ================")
    print(f"P00-00 = {p0000}")
    print(f"P02-01 = {p0201}")
    print(f"P02-34 = {fault}")
    print(f"P0D-20 = {p0d20}")
    print(f"P0B-03 = {di}")
    print(f"P0B-04 = {bus}")
    print(f"P0B-26 = {vbus}")
    print(f"P0B-07 = {pos}")
    print(f"P0B-70 I16/U16 = {mt_i16} / {mt_u16}")
    print(f"P0B-71 = {st_u16}")
    print(f"P0B-77 = {abs_low}")
    print(f"P0B-79 = {abs_high}")
    print("=========================================")


if __name__ == "__main__":
    main()