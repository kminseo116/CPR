#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import minimalmodbus
import serial
import time

PORT = "/dev/ttyUSB0"
SLAVE_ID = 1

BAUDRATE = 19200
PARITY = serial.PARITY_EVEN
STOPBITS = 1
TIMEOUT = 0.5

ADDR_P02_34 = 0x0222  # fault code

ADDR_P0B_00 = 0x0B00  # speed
ADDR_P0B_03 = 0x0B03  # DI status
ADDR_P0B_04 = 0x0B04  # bus status
ADDR_P0B_07 = 0x0B07  # position
ADDR_P0B_24 = 0x0B18  # current

ADDR_P0D_08 = 0x0D08  # control word
ADDR_P0D_17 = 0x0D11  # forced DI enable
ADDR_P0D_18 = 0x0D12  # forced DI value
ADDR_P0D_20 = 0x0D14  # abs encoder reset

ADDR_P03_06 = 0x0306  # DI3 function
ADDR_P03_07 = 0x0307  # DI3 logic

CURRENT_SCALE = 0.1


def connect_motor():
    m = minimalmodbus.Instrument(PORT, SLAVE_ID)
    m.serial.baudrate = BAUDRATE
    m.serial.bytesize = 8
    m.serial.parity = PARITY
    m.serial.stopbits = STOPBITS
    m.serial.timeout = TIMEOUT
    m.mode = minimalmodbus.MODE_RTU
    m.clear_buffers_before_each_transaction = True
    m.close_port_after_each_call = False
    return m


def read_u16(m, addr):
    return m.read_register(addr, 0, functioncode=3)


def read_i16(m, addr):
    return m.read_register(addr, 0, functioncode=3, signed=True)


def read_i32(m, addr):
    regs = m.read_registers(addr, 2, functioncode=3)
    low = regs[0]
    high = regs[1]
    val = (high << 16) | low
    if val >= (1 << 31):
        val -= (1 << 32)
    return val


def write_u16(m, addr, value):
    m.write_register(addr, int(value), functioncode=6)


def main():
    m = connect_motor()

    print("\n========== RAW STATUS ==========")

    fault = read_u16(m, ADDR_P02_34)
    speed = read_i16(m, ADDR_P0B_00)
    di = read_u16(m, ADDR_P0B_03)
    bus = read_u16(m, ADDR_P0B_04)
    pos = read_i32(m, ADDR_P0B_07)
    current_raw = read_i16(m, ADDR_P0B_24)
    current_a = current_raw * CURRENT_SCALE

    p0d17 = read_u16(m, ADDR_P0D_17)
    p0d18 = read_u16(m, ADDR_P0D_18)
    p0d20 = read_u16(m, ADDR_P0D_20)

    p0306 = read_u16(m, ADDR_P03_06)
    p0307 = read_u16(m, ADDR_P03_07)

    print(f"P02-34 fault        = {fault}")
    print(f"P0B-00 speed        = {speed} rpm")
    print(f"P0B-07 position     = {pos}")
    print(f"P0B-24 current      = {current_a:.3f} A")
    print(f"P0B-03 DI status    = {di} / 0b{di:016b}")
    print(f"P0B-04 BUS status   = {bus} / 0b{bus:016b}")
    print(f"P0D-17 forced DI en = {p0d17}")
    print(f"P0D-18 forced DI    = {p0d18} / 0b{p0d18:016b}")
    print(f"P0D-20 enc reset    = {p0d20}")
    print(f"P03-06 DI3 func     = {p0306}")
    print(f"P03-07 DI3 logic    = {p0307}")

    print("\n========== BIT CHECK ==========")
    for i in range(8):
        print(f"DI{i+1} status bit{i} = {(di >> i) & 1}")

    print("\n========== SIMPLE JUDGEMENT ==========")

    if fault != 0:
        print(f"[FAULT] 고장 코드가 있습니다: {fault}")
    else:
        print("[OK] P02-34=0 → 드라이브 고장 알람은 아님")

    if p0d17 != 1:
        print("[CHECK] P0D-17 != 1 → 강제 DI가 꺼져 있음")
    else:
        print("[OK] P0D-17=1 → 강제 DI enable")

    if p0d18 != 59:
        print("[CHECK] P0D-18 != 59 → 기존 Servo ON 강제 DI 값이 아님")
    else:
        print("[OK] P0D-18=59")

    if p0306 != 1:
        print("[CHECK] P03-06 != 1 → DI3가 S-ON 기능이 아닐 수 있음")
    else:
        print("[OK] P03-06=1 → DI3=S-ON")

    print("\n========== END ==========\n")


if __name__ == "__main__":
    main()