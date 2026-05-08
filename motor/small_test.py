#!/usr/bin/env python3
"""
small_test_v2.py

목적:
- 기존 motor_driver.py의 Servo ON 방식(P0D-18=0x003B, P0D-17=1)을 사용
- 현재 위치 기준으로 +1000만 이동 테스트
- 모터가 안 도는 원인이 Servo ON 실패인지, 브레이크/리미트 문제인지 구분

주의:
- 압박봉이 사람/마네킹/장애물에 닿지 않는 상태에서만 실행하세요.
- 브레이크 타입 모터면 브레이크 전원이 실제로 들어가 있어야 합니다.
"""

import time

import motor_config as cfg
import motor_driver as drv


# ============================================================
# 주소
# ============================================================

ADDR_P02_34 = 0x0222
ADDR_P0B_00 = 0x0B00   # speed monitor 후보
ADDR_P0B_03 = 0x0B03   # DI monitor
ADDR_P0B_04 = 0x0B04   # status word
ADDR_P0B_07 = 0x0B07   # position counter
ADDR_P0B_24 = 0x0B18   # current monitor 후보. 기존 cfg와 다르면 cfg.ADDR_P0B_24 사용 권장

ADDR_P03_06 = 0x0306   # DI3 function
ADDR_P03_07 = 0x0307   # DI3 logic
ADDR_P03_08 = 0x0308   # DI4 function, P-OT
ADDR_P03_10 = 0x030A   # DI5 function, N-OT

ADDR_P0D_08 = 0x0D08
ADDR_P0D_17 = 0x0D11
ADDR_P0D_18 = 0x0D12

ADDR_P10_03 = 0x1003
ADDR_P10_14 = 0x100E
ADDR_P10_25 = 0x1019
ADDR_P10_27 = 0x101B
ADDR_P10_29 = 0x101D


# ============================================================
# 테스트 설정
# ============================================================

TEST_MOVE_UNITS = 1000
TEST_SPEED_CMD = 3000
TEST_ACCEL_CMD = 10000
TEST_DECEL_CMD = 10000

POSITION_TOLERANCE = 100
MOVE_TIMEOUT_S = 5.0

# 리미트 스위치가 실제로 배선되어 있지 않은 테스트 환경이면 True
# 실제 장비 안전 리미트를 쓰는 상태에서는 False 권장
DISABLE_OVERTRAVEL_FOR_TEST = True


# ============================================================
# 기본 Modbus 함수
# ============================================================

def read_u16(motor, addr, label):
    value = motor.read_register(addr, 0, functioncode=3, signed=False)
    print(f"[READ] {label:<22} addr=0x{addr:04X} -> {value}")
    return value


def read_i16(motor, addr, label):
    value = motor.read_register(addr, 0, functioncode=3, signed=True)
    print(f"[READ] {label:<22} addr=0x{addr:04X} -> {value}")
    return value


def write_u16(motor, addr, value, label):
    motor.write_register(addr, value, 0, functioncode=6, signed=False)
    print(f"[WRITE] {label:<21} addr=0x{addr:04X} <- {value}")


def read_i32(motor, addr, label):
    regs = motor.read_registers(addr, 2, functioncode=3)
    low = regs[0]
    high = regs[1]

    value = (high << 16) | low
    if value >= (1 << 31):
        value -= (1 << 32)

    print(f"[READ] {label:<22} addr=0x{addr:04X} regs={regs} -> {value}")
    return value


def write_i32(motor, addr, value, label):
    value_u32 = value & 0xFFFFFFFF
    low = value_u32 & 0xFFFF
    high = (value_u32 >> 16) & 0xFFFF

    motor.write_registers(addr, [low, high])
    print(f"[WRITE] {label:<21} addr=0x{addr:04X} <- {value} regs=[{low}, {high}]")


# ============================================================
# 상태 확인
# ============================================================

def check_fault(motor, where):
    fault = read_u16(motor, ADDR_P02_34, f"P02-34 fault {where}")
    if fault != 0:
        raise RuntimeError(f"고장 코드 발생: P02-34={fault}")
    return fault


def read_current_a_safe(motor):
    try:
        # 기존 motor_config에 주소/스케일이 있으면 그걸 우선 사용
        raw = motor.read_register(cfg.ADDR_P0B_24, 0, functioncode=3, signed=True)
        current = raw * cfg.CURRENT_SCALE
        print(f"[READ] current             raw={raw} -> {current:.3f} A")
        return current
    except Exception as e:
        print(f"[WARN] current read fail -> {e}")
        return None


def print_status(motor, title):
    print(f"\n================ STATUS: {title} ================")

    try:
        pos = read_i32(motor, ADDR_P0B_07, "P0B-07 position")
    except Exception as e:
        pos = None
        print(f"[FAIL] position read -> {e}")

    try:
        speed = read_i16(motor, ADDR_P0B_00, "P0B-00 speed")
    except Exception as e:
        speed = None
        print(f"[WARN] speed read fail -> {e}")

    current = read_current_a_safe(motor)

    try:
        di = read_u16(motor, ADDR_P0B_03, "P0B-03 DI monitor")
    except Exception as e:
        di = None
        print(f"[WARN] DI read fail -> {e}")

    try:
        status = read_u16(motor, ADDR_P0B_04, "P0B-04 status")
    except Exception as e:
        status = None
        print(f"[WARN] status read fail -> {e}")

    try:
        fault = read_u16(motor, ADDR_P02_34, "P02-34 fault")
    except Exception as e:
        fault = None
        print(f"[WARN] fault read fail -> {e}")

    print("\n[SUMMARY]")
    print(f"  pos     = {pos}")
    print(f"  speed   = {speed}")
    print(f"  current = {current}")
    print(f"  DI      = {di}")
    print(f"  status  = {status}")
    print(f"  fault   = {fault}")

    return pos


# ============================================================
# 설정 / Servo ON
# ============================================================

def restore_di3_servo_on_setting(motor):
    print("\n================ DI3 S-ON SETTING ================")

    # DI3 = S-ON
    write_u16(motor, ADDR_P03_06, 1, "P03-06 DI3=S-ON")
    time.sleep(0.1)

    # Low active
    write_u16(motor, ADDR_P03_07, 0, "P03-07 DI3 low active")
    time.sleep(0.1)


def disable_overtravel_switches_for_test(motor):
    print("\n================ DISABLE P-OT / N-OT FOR TEST ================")
    print("[WARN] 실제 리미트 스위치를 안전용으로 쓰는 상태라면 이 기능을 쓰면 안 됩니다.")

    # DI4 P-OT 기능 해제
    write_u16(motor, ADDR_P03_08, 0, "P03-08 DI4 disable")
    time.sleep(0.1)

    # DI5 N-OT 기능 해제
    write_u16(motor, ADDR_P03_10, 0, "P03-10 DI5 disable")
    time.sleep(0.1)


def servo_on_by_forced_di(motor):
    print("\n================ SERVO ON by forced DI ================")

    # 기존 motor_driver.py 방식:
    # DI3 Bit2 = 0 상태를 만들기 위해 P0D-18 = 0x003B 사용
    write_u16(motor, ADDR_P0D_18, 0x003B, "P0D-18 forced DI state")
    time.sleep(0.1)

    write_u16(motor, ADDR_P0D_17, 1, "P0D-17 forced DI enable")
    time.sleep(0.5)

    check_fault(motor, "after servo on")


def servo_off(motor):
    print("\n================ SERVO OFF ================")

    try:
        write_u16(motor, ADDR_P0D_08, 256, "P0D-08 stop")
        time.sleep(0.1)
    except Exception as e:
        print(f"[WARN] stop fail -> {e}")

    try:
        write_u16(motor, ADDR_P0D_17, 0, "P0D-17 forced DI disable")
        time.sleep(0.2)
    except Exception as e:
        print(f"[WARN] forced DI disable fail -> {e}")


def set_pp_mode(motor):
    print("\n================ SET PP MODE ================")

    write_u16(motor, ADDR_P10_03, 1, "P10-03 PP mode")
    time.sleep(0.1)

    write_i32(motor, ADDR_P10_25, TEST_SPEED_CMD, "P10-25 speed")
    time.sleep(0.05)

    write_i32(motor, ADDR_P10_27, TEST_ACCEL_CMD, "P10-27 accel")
    time.sleep(0.05)

    write_i32(motor, ADDR_P10_29, TEST_DECEL_CMD, "P10-29 decel")
    time.sleep(0.05)

    check_fault(motor, "after PP mode")


def start_pp_move(motor, target_pos):
    print("\n================ START PP MOVE ================")

    write_i32(motor, ADDR_P10_14, target_pos, "P10-14 target")
    time.sleep(0.05)

    write_u16(motor, ADDR_P0D_08, 0, "P0D-08 clear")
    time.sleep(0.05)

    write_u16(motor, ADDR_P0D_08, 7, "P0D-08 start")
    time.sleep(0.1)

    check_fault(motor, "after move start")


def wait_until_reached(motor, target_pos, label):
    print(f"\n================ WAIT {label} ================")

    start_time = time.time()

    while True:
        pos = read_i32(motor, ADDR_P0B_07, "P0B-07 current")
        error = target_pos - pos

        try:
            speed = motor.read_register(ADDR_P0B_00, 0, functioncode=3, signed=True)
        except Exception:
            speed = None

        current = read_current_a_safe(motor)

        print(
            f"[MOVE] {label} | "
            f"target={target_pos} | pos={pos} | error={error} | "
            f"speed={speed} | current={current}"
        )

        check_fault(motor, f"during {label}")

        if abs(error) <= POSITION_TOLERANCE:
            print(f"[OK] {label} 도달 완료")
            return True

        if time.time() - start_time > MOVE_TIMEOUT_S:
            print(f"[TIMEOUT] {label} 이동 타임아웃")
            return False

        time.sleep(0.2)


# ============================================================
# main
# ============================================================

def main():
    print("================================================")
    print(" 소량 이동 테스트 V2 - 기존 Servo ON 방식 사용")
    print("================================================")
    print(f"PORT     = {cfg.PORT}")
    print(f"SLAVE_ID = {cfg.SLAVE_ID}")
    print(f"TEST_MOVE_UNITS = {TEST_MOVE_UNITS}")
    print("주의: 압박봉이 사람/마네킹/장애물에 닿지 않는 상태에서만 실행하세요.")
    print("================================================")

    motor = drv.create_motor_connection(cfg.PORT, cfg.SLAVE_ID)

    try:
        check_fault(motor, "before test")

        start_pos = print_status(motor, "before servo on")
        if start_pos is None:
            raise RuntimeError("현재 위치를 읽지 못했습니다.")

        target_pos = start_pos + TEST_MOVE_UNITS

        print("\n[POSITION]")
        print(f"start_pos  = {start_pos}")
        print(f"target_pos = {target_pos}")

        restore_di3_servo_on_setting(motor)

        if DISABLE_OVERTRAVEL_FOR_TEST:
            disable_overtravel_switches_for_test(motor)

        servo_on_by_forced_di(motor)
        print_status(motor, "after servo on")

        set_pp_mode(motor)

        start_pp_move(motor, target_pos)
        ok = wait_until_reached(motor, target_pos, "FORWARD_SMALL")

        print_status(motor, "after move")

        print("\n================ RESULT ================")
        if ok:
            print("[PASS] 모터가 소량 이동했습니다.")
            print("다음 단계: 초기 위치 저장/복귀 코드에 같은 Servo ON 방식을 적용하세요.")
        else:
            print("[FAIL] 명령은 들어갔지만 위치가 변하지 않았습니다.")
            print("")
            print("가능성이 큰 원인:")
            print("1. 브레이크 전원이 안 들어가서 기계적으로 잠김")
            print("2. Servo ON이 실제로 안 걸림")
            print("3. P-OT/N-OT 또는 DI 입력이 운전을 막음")
            print("4. P0D-08 시작 제어워드가 현재 설정과 맞지 않음")
            print("")
            print("확인:")
            print("- 브레이크 전원 연결/극성/전압")
            print("- Servo ON 후 모터가 딱 잡히는 느낌이 있는지")
            print("- 이동 명령 중 current가 증가하는지")
            print("- 이동 명령 중 speed가 0이 아닌지")
        print("========================================")

    except Exception as e:
        print("\n================ ERROR ================")
        print(f"[ERROR] {e}")
        print("기존 CPR 왕복 코드는 아직 실행하지 마세요.")
        print("=======================================")

    finally:
        servo_off(motor)


if __name__ == "__main__":
    main()