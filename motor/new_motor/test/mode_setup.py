#!/usr/bin/env python3
"""RS485 서보모터 위치 제어 모드 확인 및 설정."""

import minimalmodbus
import serial


# ============================================================
# 통신 설정
# ============================================================

PORT = "/dev/ttyUSB0"
SLAVE_ID = 1
BAUDRATE = 115200

# True:
#   위치 모드가 아니면 위치 모드로 변경하고 저장
#
# False:
#   현재 모드만 읽고 종료
CHANGE_TO_POSITION_MODE = True


# ============================================================
# 레지스터
# ============================================================

REG_CONTROL_MODE = 0x0000
REG_ENABLE_METHOD = 0x0020
REG_INTERNAL_ENABLE = 0x0021
REG_MOTOR_START = 0x0023
REG_SPEED_MODE_COMMAND = 0x002C
REG_SAVE_DATA = 0x0030

POSITION_MODE = 1


def create_motor() -> minimalmodbus.Instrument:
    motor = minimalmodbus.Instrument(
        PORT,
        SLAVE_ID,
        mode=minimalmodbus.MODE_RTU,
    )

    motor.serial.baudrate = BAUDRATE
    motor.serial.bytesize = serial.EIGHTBITS
    motor.serial.parity = serial.PARITY_NONE
    motor.serial.stopbits = serial.STOPBITS_ONE
    motor.serial.timeout = 0.5
    motor.serial.write_timeout = 0.5

    motor.clear_buffers_before_each_transaction = True
    motor.close_port_after_each_call = False

    return motor


def read_register(
    motor: minimalmodbus.Instrument,
    address: int,
) -> int:
    return int(
        motor.read_register(
            registeraddress=address,
            number_of_decimals=0,
            functioncode=3,
            signed=False,
        )
    )


def write_register(
    motor: minimalmodbus.Instrument,
    address: int,
    value: int,
) -> None:
    motor.write_register(
        registeraddress=address,
        value=value,
        number_of_decimals=0,
        functioncode=6,
        signed=False,
    )


def mode_name(mode: int) -> str:
    names = {
        0: "외부 펄스 모드",
        1: "위치 모드",
        2: "RS485 속도 모드",
        3: "외부 단자 속도 모드",
        4: "RS485 토크 모드",
        5: "외부 단자 토크 모드",
    }
    return names.get(mode, "알 수 없는 모드")


def safely_stop_motor(
    motor: minimalmodbus.Instrument,
) -> None:
    """설정 변경 전 모터를 정지하고 Servo OFF 처리."""

    print("\n[안전 정지]")

    # 속도 모드 정지
    write_register(
        motor,
        REG_SPEED_MODE_COMMAND,
        0,
    )
    print("0x002C = 0 : 속도 모드 정지")

    # 위치 모드 모터 기동 해제
    write_register(
        motor,
        REG_MOTOR_START,
        0,
    )
    print("0x0023 = 0 : 모터 기동 해제")

    # 내부 인에이블 해제
    write_register(
        motor,
        REG_INTERNAL_ENABLE,
        0,
    )
    print("0x0021 = 0 : 내부 인에이블 OFF")


def main() -> None:
    print("======================================")
    print("서보모터 제어 모드 확인 및 변경")
    print(f"포트: {PORT}")
    print(f"장치 주소: {SLAVE_ID}")
    print("======================================")

    motor = create_motor()

    try:
        current_mode = read_register(
            motor,
            REG_CONTROL_MODE,
        )

        print("\n[현재 제어 모드]")
        print(f"0x0000 = {current_mode}")
        print(f"모드: {mode_name(current_mode)}")

        if current_mode == POSITION_MODE:
            print("\n이미 위치 제어 모드입니다.")
            print("설정을 변경하지 않고 종료합니다.")
            return

        if not CHANGE_TO_POSITION_MODE:
            print("\nCHANGE_TO_POSITION_MODE=False")
            print("현재 모드만 확인하고 종료합니다.")
            return

        print("\n위치 모드로 변경합니다.")

        safely_stop_motor(motor)

        # 제어 모드를 위치 모드로 설정
        print("\n[위치 모드 설정]")
        write_register(
            motor,
            REG_CONTROL_MODE,
            POSITION_MODE,
        )

        mode_after_write = read_register(
            motor,
            REG_CONTROL_MODE,
        )

        print(
            f"쓰기 직후 0x0000 읽기: "
            f"{mode_after_write} "
            f"({mode_name(mode_after_write)})"
        )

        if mode_after_write != POSITION_MODE:
            raise RuntimeError(
                "위치 모드 쓰기 검증 실패: "
                f"읽은 값={mode_after_write}"
            )

        # 비휘발성 저장
        print("\n[설정 저장]")
        write_register(
            motor,
            REG_SAVE_DATA,
            1,
        )
        print("0x0030 = 1 : 설정 저장 명령 완료")

        print("\n======================================")
        print("위치 모드 설정 및 저장이 완료됐습니다.")
        print("이 설정은 전원을 다시 켜야 적용됩니다.")
        print()
        print("1. 모터 전원 OFF")
        print("2. 약 10초 대기")
        print("3. 모터 전원 ON")
        print("4. 이 코드를 다시 실행")
        print("5. 0x0000 = 1인지 확인")
        print("======================================")

    finally:
        if motor.serial.is_open:
            motor.serial.close()


if __name__ == "__main__":
    try:
        main()

    except FileNotFoundError as error:
        print(f"시리얼 장치가 없습니다: {error}")
        print("확인 명령: ls -l /dev/ttyUSB*")

    except PermissionError as error:
        print(f"시리얼 포트 권한 오류: {error}")

    except minimalmodbus.NoResponseError as error:
        print(f"모터 응답 없음: {error}")
        print("모터 전원과 RS485 배선을 확인하십시오.")

    except minimalmodbus.InvalidResponseError as error:
        print(f"잘못된 Modbus 응답: {error}")

    except serial.SerialException as error:
        print(f"시리얼 통신 오류: {error}")

    except Exception as error:
        print(f"오류: {error}")