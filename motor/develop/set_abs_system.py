#!/usr/bin/env python3
import minimalmodbus
import serial
import time

PORT = "/dev/ttyUSB0"
SLAVE_ID = 1
BAUDRATE = 19200

ADDR_P02_01 = 0x0201   # P02-01 절대 위치 검출 시스템
ADDR_P02_34 = 0x0222   # P02-34 fault code
ADDR_P0D_20 = 0x0D14   # P0D-20 abs encoder reset
ADDR_P0B_07 = 0x0B07   # P0B-07 position counter


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
        print(f"[OK] READ {name} addr=0x{addr:04X} -> {value}")
        return value
    except Exception as e:
        print(f"[FAIL] READ {name} addr=0x{addr:04X} -> {e}")
        return None


def write_u16(inst, addr, value, name):
    try:
        inst.write_register(addr, value, 0, functioncode=6, signed=False)
        print(f"[OK] WRITE {name} addr=0x{addr:04X} <- {value}")
        return True
    except Exception as e:
        print(f"[FAIL] WRITE {name} addr=0x{addr:04X} <- {value} -> {e}")
        return False


def main():
    print("============================================")
    print(" P02-01 절대 위치 검출 시스템 설정")
    print("============================================")
    print(f"PORT     = {PORT}")
    print(f"SLAVE_ID = {SLAVE_ID}")
    print("주의: 모터 정지 + 서보 OFF 상태에서 실행하세요.")
    print("주의: P0D-20은 건드리지 않습니다.")
    print("============================================")

    inst = make_driver()

    print("\n================ BEFORE ================")
    p02_01_before = read_u16(inst, ADDR_P02_01, "P02-01 abs system")
    fault = read_u16(inst, ADDR_P02_34, "P02-34 fault code")
    p0d_20 = read_u16(inst, ADDR_P0D_20, "P0D-20 abs reset")

    if fault not in (0, None):
        print("\n[STOP] 현재 fault code가 0이 아닙니다. 먼저 알람 원인을 확인하세요.")
        return

    if p0d_20 not in (0, None):
        print("\n[STOP] P0D-20이 0이 아닙니다. 절대 엔코더 초기화 명령 상태를 먼저 확인하세요.")
        return

    if p02_01_before == 1:
        print("\n[INFO] 이미 P02-01 = 1 입니다. 추가 설정이 필요 없습니다.")
        return

    print("\n================ WRITE ================")
    ok = write_u16(inst, ADDR_P02_01, 1, "P02-01 abs system")

    if not ok:
        print("\n[FAIL] P02-01 쓰기 실패")
        return

    time.sleep(0.2)

    print("\n================ VERIFY ================")
    p02_01_after = read_u16(inst, ADDR_P02_01, "P02-01 abs system")

    print("\n================ RESULT ================")
    if p02_01_after == 1:
        print("[OK] P02-01 = 1 설정 완료")
        print("현재 설정: 절대 위치 검출 시스템")
        print("\n다음 단계:")
        print("1) 드라이버 전원을 완전히 OFF")
        print("2) 5초 이상 대기")
        print("3) 다시 ON")
        print("4) monitor.py 또는 test.py로 P02-01이 1로 유지되는지 확인")
        print("5) P0B-07 위치값이 전원 OFF/ON 후 유지되는지 확인")
    else:
        print("[FAIL] P02-01이 1로 바뀌지 않았습니다.")
        print(f"현재 읽힌 값: {p02_01_after}")
        print("서보 OFF 상태인지, 쓰기 권한/통신 설정/EEPROM 저장 설정을 확인하세요.")

    print("============================================")


if __name__ == "__main__":
    main()