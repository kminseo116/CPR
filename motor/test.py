#!/usr/bin/env python3
"""
check_absolute_position_candidates.py

목적:
- 모터를 움직이지 않고 읽기만 수행
- P0B-07 위치 카운터와 P0B-70/P0B-71 절대 엔코더 후보값 비교
- 전원 OFF/ON 또는 드라이버 재연결 후에도 유지되는 값이 있는지 확인

사용 방법:
1. 모터를 움직이지 않은 상태에서 실행
   python3 check_absolute_position_candidates.py

2. 출력값 기록
   - P0B-07
   - P0B-70
   - P0B-71
   - P0B-70/71 조합값

3. 드라이버 전원 OFF → ON
   축은 움직이지 않게 유지

4. 다시 실행해서 값 비교
"""

import time

import motor_config as cfg
import motor_driver as drv


# ============================================================
# 절대 엔코더 후보 주소
# ============================================================

ADDR_P0B_70 = 0x0B46   # absolute encoder multi-turn 후보
ADDR_P0B_71 = 0x0B47   # absolute encoder single-turn 후보

# 주변 주소 확인 범위
SCAN_START = 0x0B44
SCAN_END   = 0x0B4F


# ============================================================
# 안전 읽기 함수
# ============================================================

def try_read_u16_fc3(motor, addr, label):
    try:
        value = motor.read_register(addr, 0, functioncode=3, signed=False)
        print(f"[OK] {label} U16 FC3 addr=0x{addr:04X} -> {value}")
        return value
    except Exception as e:
        print(f"[FAIL] {label} U16 FC3 addr=0x{addr:04X} -> {e}")
        return None


def try_read_i16_fc3(motor, addr, label):
    try:
        value = motor.read_register(addr, 0, functioncode=3, signed=True)
        print(f"[OK] {label} I16 FC3 addr=0x{addr:04X} -> {value}")
        return value
    except Exception as e:
        print(f"[FAIL] {label} I16 FC3 addr=0x{addr:04X} -> {e}")
        return None


def try_read_u16_fc4(motor, addr, label):
    try:
        value = motor.read_register(addr, 0, functioncode=4, signed=False)
        print(f"[OK] {label} U16 FC4 addr=0x{addr:04X} -> {value}")
        return value
    except Exception as e:
        print(f"[FAIL] {label} U16 FC4 addr=0x{addr:04X} -> {e}")
        return None


def try_read_i16_fc4(motor, addr, label):
    try:
        value = motor.read_register(addr, 0, functioncode=4, signed=True)
        print(f"[OK] {label} I16 FC4 addr=0x{addr:04X} -> {value}")
        return value
    except Exception as e:
        print(f"[FAIL] {label} I16 FC4 addr=0x{addr:04X} -> {e}")
        return None


def try_read_regs_fc3(motor, addr, count, label):
    try:
        regs = motor.read_registers(addr, count, functioncode=3)
        print(f"[OK] {label} REGS FC3 addr=0x{addr:04X}, count={count} -> {regs}")
        return regs
    except Exception as e:
        print(f"[FAIL] {label} REGS FC3 addr=0x{addr:04X}, count={count} -> {e}")
        return None


def try_read_regs_fc4(motor, addr, count, label):
    try:
        regs = motor.read_registers(addr, count, functioncode=4)
        print(f"[OK] {label} REGS FC4 addr=0x{addr:04X}, count={count} -> {regs}")
        return regs
    except Exception as e:
        print(f"[FAIL] {label} REGS FC4 addr=0x{addr:04X}, count={count} -> {e}")
        return None


# ============================================================
# 조합 함수
# ============================================================

def combine_2regs_to_u32(regs, word_order="LOW_HIGH"):
    """
    16bit 레지스터 2개를 32bit unsigned 값으로 조합.
    사용자님 환경은 LOW_HIGH로 확인됨.
    그래도 비교를 위해 HIGH_LOW도 같이 출력함.
    """
    if regs is None or len(regs) < 2:
        return None

    if word_order == "LOW_HIGH":
        low_word = regs[0]
        high_word = regs[1]
    elif word_order == "HIGH_LOW":
        high_word = regs[0]
        low_word = regs[1]
    else:
        raise ValueError("word_order는 LOW_HIGH 또는 HIGH_LOW만 가능합니다.")

    return (high_word << 16) | low_word


def to_i32_from_u32(value):
    """
    unsigned 32bit 값을 signed 32bit 값으로 변환.
    """
    if value is None:
        return None

    if value >= (1 << 31):
        value -= (1 << 32)

    return value


# ============================================================
# 현재 기준 위치 출력
# ============================================================

def print_current_reference(motor):
    print("\n================ CURRENT REFERENCE ================")

    try:
        p0b07 = drv.read_i32(motor, cfg.ADDR_P0B_07)
        print(f"P0B-07 position counter = {p0b07}")
    except Exception as e:
        print(f"P0B-07 read fail = {e}")

    try:
        p0b70_i16 = motor.read_register(ADDR_P0B_70, 0, functioncode=3, signed=True)
        print(f"P0B-70 I16 FC3 = {p0b70_i16}")
    except Exception as e:
        print(f"P0B-70 I16 FC3 fail = {e}")

    try:
        p0b71_u16 = motor.read_register(ADDR_P0B_71, 0, functioncode=3, signed=False)
        print(f"P0B-71 U16 FC3 = {p0b71_u16}")
    except Exception as e:
        print(f"P0B-71 U16 FC3 fail = {e}")

    print("===================================================\n")


# ============================================================
# 1. P0B-71 단일 주소 테스트
# ============================================================

def test_p0b71_single_address(motor):
    print("\n================ P0B-71 SINGLE ADDRESS TEST ================")
    print("목적: P0B-71을 U16/I16, FC3/FC4 방식으로 각각 읽어봅니다.")
    print("=============================================================")

    try_read_u16_fc3(motor, ADDR_P0B_71, "P0B-71")
    time.sleep(0.05)

    try_read_i16_fc3(motor, ADDR_P0B_71, "P0B-71")
    time.sleep(0.05)

    try_read_u16_fc4(motor, ADDR_P0B_71, "P0B-71")
    time.sleep(0.05)

    try_read_i16_fc4(motor, ADDR_P0B_71, "P0B-71")
    time.sleep(0.05)


# ============================================================
# 2. P0B-70/71 pair 테스트
# ============================================================

def test_p0b70_71_pair(motor):
    print("\n================ P0B-70/71 PAIR TEST ================")
    print("목적: P0B-70과 P0B-71이 2워드 데이터처럼 붙어 있는지 확인합니다.")
    print("======================================================")

    regs3 = try_read_regs_fc3(motor, ADDR_P0B_70, 2, "P0B-70/71")

    if regs3 is not None:
        u32_low_high = combine_2regs_to_u32(regs3, "LOW_HIGH")
        u32_high_low = combine_2regs_to_u32(regs3, "HIGH_LOW")

        i32_low_high = to_i32_from_u32(u32_low_high)
        i32_high_low = to_i32_from_u32(u32_high_low)

        print(f"[COMBINE] FC3 LOW_HIGH U32 -> {u32_low_high}")
        print(f"[COMBINE] FC3 LOW_HIGH I32 -> {i32_low_high}")
        print(f"[COMBINE] FC3 HIGH_LOW U32 -> {u32_high_low}")
        print(f"[COMBINE] FC3 HIGH_LOW I32 -> {i32_high_low}")

    time.sleep(0.05)

    regs4 = try_read_regs_fc4(motor, ADDR_P0B_70, 2, "P0B-70/71")

    if regs4 is not None:
        u32_low_high = combine_2regs_to_u32(regs4, "LOW_HIGH")
        u32_high_low = combine_2regs_to_u32(regs4, "HIGH_LOW")

        i32_low_high = to_i32_from_u32(u32_low_high)
        i32_high_low = to_i32_from_u32(u32_high_low)

        print(f"[COMBINE] FC4 LOW_HIGH U32 -> {u32_low_high}")
        print(f"[COMBINE] FC4 LOW_HIGH I32 -> {i32_low_high}")
        print(f"[COMBINE] FC4 HIGH_LOW U32 -> {u32_high_low}")
        print(f"[COMBINE] FC4 HIGH_LOW I32 -> {i32_high_low}")


# ============================================================
# 3. 주변 주소 스캔
# ============================================================

def scan_nearby_registers(motor):
    print("\n================ NEARBY ADDRESS SCAN ================")
    print(f"목적: 0x{SCAN_START:04X} ~ 0x{SCAN_END:04X} 주변 주소 중 읽히는 값을 찾습니다.")
    print("주의: 읽기만 수행하며 모터는 움직이지 않습니다.")
    print("=====================================================")

    for addr in range(SCAN_START, SCAN_END + 1):
        print(f"\n--- addr=0x{addr:04X} ---")
        try_read_u16_fc3(motor, addr, f"0x{addr:04X}")
        time.sleep(0.03)


# ============================================================
# 4. 반복 관찰
# ============================================================

def observe_values(motor, samples=5, interval_s=0.5):
    print("\n================ OBSERVE VALUES ================")
    print("목적: P0B-07, P0B-70, P0B-71 후보값이 시간에 따라 어떻게 보이는지 확인합니다.")
    print("모터를 손대지 않은 상태 / 조금 움직인 상태에서 각각 실행해보세요.")
    print("================================================")

    for i in range(samples):
        print(f"\n[SAMPLE {i + 1}/{samples}]")

        try:
            p0b07 = drv.read_i32(motor, cfg.ADDR_P0B_07)
        except Exception as e:
            p0b07 = None
            print(f"P0B-07 fail: {e}")

        p0b70_i16 = try_read_i16_fc3(motor, ADDR_P0B_70, "P0B-70")
        p0b71_u16 = try_read_u16_fc3(motor, ADDR_P0B_71, "P0B-71")
        p0b71_i16 = try_read_i16_fc3(motor, ADDR_P0B_71, "P0B-71")

        regs3 = try_read_regs_fc3(motor, ADDR_P0B_70, 2, "P0B-70/71")
        if regs3 is not None:
            pair_low_high = combine_2regs_to_u32(regs3, "LOW_HIGH")
            pair_high_low = combine_2regs_to_u32(regs3, "HIGH_LOW")
        else:
            pair_low_high = None
            pair_high_low = None

        print(
            "[SUMMARY] "
            f"P0B-07={p0b07}, "
            f"P0B-70_I16={p0b70_i16}, "
            f"P0B-71_U16={p0b71_u16}, "
            f"P0B-71_I16={p0b71_i16}, "
            f"P0B70/71_LOW_HIGH_U32={pair_low_high}, "
            f"P0B70/71_HIGH_LOW_U32={pair_high_low}"
        )

        time.sleep(interval_s)


# ============================================================
# 메인
# ============================================================

def main():
    print("=== P0B-70 / P0B-71 절대 위치 후보 확인 코드 ===")
    print(f"PORT={cfg.PORT}, SLAVE_ID={cfg.SLAVE_ID}")
    print("이 코드는 모터를 움직이지 않고 읽기만 수행합니다.")

    motor = drv.create_motor_connection(cfg.PORT, cfg.SLAVE_ID)

    print_current_reference(motor)

    test_p0b71_single_address(motor)
    test_p0b70_71_pair(motor)
    scan_nearby_registers(motor)
    observe_values(motor, samples=5, interval_s=0.5)

    print("\n=== 테스트 완료 ===")


if __name__ == "__main__":
    main()