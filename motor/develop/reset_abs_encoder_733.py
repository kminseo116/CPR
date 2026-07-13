#!/usr/bin/env python3
import time
import minimalmodbus
import serial

PORT = "/dev/ttyUSB0"
SLAVE_ID = 1
BAUDRATE = 19200

ADDR_P02_01 = 0x0201   # 절대 위치 검출 시스템
ADDR_P02_34 = 0x0222   # fault code
ADDR_P0D_01 = 0x0D01   # fault reset
ADDR_P0D_20 = 0x0D14   # absolute encoder reset
ADDR_P0B_07 = 0x0B07   # absolute position counter
ADDR_P0B_70 = 0x0B46   # multi-turn data


def make_driver():
    inst = minimalmodbus.Instrument(PORT, SLAVE_ID)
    inst.serial.baudrate = BAUDRATE
    inst.serial.bytesize = 8
    inst.serial.parity = serial.PARITY_EVEN
    inst.serial.stopbits = 1
    inst.serial.timeout = 0.5
    inst.mode = minimalmodbus.MODE_RTU
    inst.clear_buffers_before_each_transaction = True
    inst.close_port_after_each_call = True
    return inst


def read_u16(inst, addr, name):
    try:
        value = inst.read_register(addr, 0, functioncode=3, signed=False)
        print(f"[OK] READ  {name:22s} addr=0x{addr:04X} -> {value}")
        return value
    except Exception as e:
        print(f"[FAIL] READ {name:22s} addr=0x{addr:04X} -> {e}")
        return None


def read_i32_low_high(inst, addr, name):
    try:
        regs = inst.read_registers(addr, 2, functioncode=3)
        value = (regs[1] << 16) | regs[0]   # LOW_HIGH
        if value >= 0x80000000:
            value -= 0x100000000
        print(f"[OK] READ  {name:22s} addr=0x{addr:04X} regs={regs} -> {value}")
        return value
    except Exception as e:
        print(f"[FAIL] READ {name:22s} addr=0x{addr:04X} -> {e}")
        return None


def write_u16(inst, addr, value, name):
    try:
        inst.write_register(addr, value, 0, functioncode=6, signed=False)
        print(f"[OK] WRITE {name:22s} addr=0x{addr:04X} <- {value}")
        return True
    except Exception as e:
        print(f"[FAIL] WRITE {name:22s} addr=0x{addr:04X} <- {value} -> {e}")
        return False


def main():
    print("============================================")
    print(" Er.733 / 절대 엔코더 다권 오류 리셋")
    print("============================================")
    print(f"PORT     = {PORT}")
    print(f"SLAVE_ID = {SLAVE_ID}")
    print("주의: 모터 정지 + 서보 OFF 상태에서 실행하세요.")
    print("주의: P0D-20은 다권 데이터 초기화 명령입니다.")
    print("============================================")

    inst = make_driver()

    print("\n================ BEFORE ================")
    p02_01 = read_u16(inst, ADDR_P02_01, "P02-01 abs system")
    fault_before = read_u16(inst, ADDR_P02_34, "P02-34 fault code")
    p0d20_before = read_u16(inst, ADDR_P0D_20, "P0D-20 abs reset")
    read_i32_low_high(inst, ADDR_P0B_07, "P0B-07 pos counter")
    read_u16(inst, ADDR_P0B_70, "P0B-70 multi-turn")

    if p02_01 != 1:
        print("\n[STOP] P02-01이 1이 아닙니다. 절대 위치 시스템이 꺼져 있습니다.")
        return

    print("\n================ RESET P0D-20 ================")
    print("[STEP] P0D-20 = 1 쓰기")
    if not write_u16(inst, ADDR_P0D_20, 1, "P0D-20 abs reset"):
        return

    time.sleep(0.5)

    print("[STEP] P0D-20 = 0 복귀")
    if not write_u16(inst, ADDR_P0D_20, 0, "P0D-20 abs reset"):
        return

    time.sleep(0.2)

    print("\n================ FAULT RESET ================")
    print("[STEP] P0D-01 = 1 고장 복원")
    write_u16(inst, ADDR_P0D_01, 1, "P0D-01 fault reset")

    time.sleep(0.3)

    print("[STEP] P0D-01 = 0 복귀")
    write_u16(inst, ADDR_P0D_01, 0, "P0D-01 fault reset")

    time.sleep(0.5)

    print("\n================ AFTER ================")
    fault_after = read_u16(inst, ADDR_P02_34, "P02-34 fault code")
    p0d20_after = read_u16(inst, ADDR_P0D_20, "P0D-20 abs reset")
    read_i32_low_high(inst, ADDR_P0B_07, "P0B-07 pos counter")
    read_u16(inst, ADDR_P0B_70, "P0B-70 multi-turn")

    print("\n================ RESULT ================")
    if fault_after == 0 and p0d20_after == 0:
        print("[OK] 리셋 명령 완료. 현재 Modbus상 fault code = 0 입니다.")
        print("다음 단계: 드라이버 전원을 완전히 OFF/ON 한 뒤 monitor.py로 다시 확인하세요.")
    else:
        print("[WARN] 아직 fault가 남아 있을 수 있습니다.")
        print(f"P02-34 = {fault_after}, P0D-20 = {p0d20_after}")
        print("매뉴얼 기준 Er.733은 P0D-20=2, P0D-01=1 후 재통전 절차도 필요할 수 있습니다.")

    print("============================================")


if __name__ == "__main__":
    main()