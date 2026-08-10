#!/usr/bin/env python3
"""RS485 서보모터 제어 모드 확인 및 대화형 설정."""

import minimalmodbus
import serial


# ============================================================
# 통신 설정
# ============================================================

PORT = "/dev/ttyUSB0"
SLAVE_ID = 1
BAUDRATE = 115200

# ============================================================
# 레지스터
# ============================================================

REG_CONTROL_MODE = 0x0000
REG_ENABLE_METHOD = 0x0020
REG_INTERNAL_ENABLE = 0x0021
REG_MOTOR_START = 0x0023
REG_SPEED_MODE_COMMAND = 0x002C
REG_SAVE_DATA = 0x0030

MODE_NAMES = {
    0: "외부 펄스 모드",
    1: "RS485 위치 모드",
    2: "RS485 속도 모드",
    3: "외부 단자 속도 모드",
    4: "RS485 토크 모드",
    5: "외부 단자 토크 모드",
}

MODE_WARNINGS = {
    0: "외부 펄스 배선과 펄스 입력에 따라 모터가 움직일 수 있습니다.",
    1: "0x0042/0x0044 위치 명령과 0x0023 기동 명령을 사용합니다.",
    2: "0x002C 속도 명령이 1 또는 2이면 연속 회전할 수 있습니다.",
    3: "외부 방향·기동 단자의 전압에 따라 연속 회전할 수 있습니다.",
    4: "RS485 토크 명령이 남아 있으면 큰 토크가 발생할 수 있습니다.",
    5: "외부 토크·기동 단자의 전압에 따라 큰 토크가 발생할 수 있습니다.",
}


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
    return MODE_NAMES.get(mode, "알 수 없는 모드")


def print_mode_menu() -> None:
    """선택 가능한 제어 모드와 주의사항을 출력한다."""
    print("\n[선택 가능한 제어 모드]")
    for mode, name in MODE_NAMES.items():
        print(f"  {mode}: {name}")


def read_target_mode() -> int | None:
    """사용자가 선택한 목표 모드를 반환하고 Enter면 None을 반환한다."""
    print_mode_menu()
    raw_value = input(
        "\n변경할 모드 번호 입력 [0~5] "
        "(Enter=현재 모드만 확인하고 종료): "
    ).strip()

    if not raw_value:
        return None

    try:
        target_mode = int(raw_value)
    except ValueError as exc:
        raise ValueError("모드 번호는 0~5의 정수여야 합니다.") from exc

    if target_mode not in MODE_NAMES:
        raise ValueError(f"지원하지 않는 모드 번호입니다: {target_mode}")

    return target_mode


def confirm_mode_change(current_mode: int, target_mode: int) -> bool:
    """비휘발성 설정 변경 전 정확한 확인 문구를 요구한다."""
    print("\n====================================================")
    print(" 제어 모드 변경 확인")
    print("====================================================")
    print(f"현재: {current_mode} ({mode_name(current_mode)})")
    print(f"변경: {target_mode} ({mode_name(target_mode)})")
    print(f"주의: {MODE_WARNINGS[target_mode]}")
    print()
    print("변경 과정에서 모터 정지 및 내부 Servo OFF 명령을 보냅니다.")
    print("0x0030 저장 후 전원을 껐다 켜야 새 모드가 적용됩니다.")
    print("외부 Enable 입력을 LOW로 만들고 모터 축의 하중을 제거하십시오.")

    expected = f"SET {target_mode}"
    confirmation = input(
        f"변경하려면 '{expected}'을 정확히 입력하십시오: "
    ).strip().upper()
    return confirmation == expected


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

        target_mode = read_target_mode()
        if target_mode is None:
            print("\n[완료] 현재 모드만 확인했고 설정을 변경하지 않았습니다.")
            return

        if current_mode == target_mode:
            print(
                f"\n[변경 없음] 이미 {target_mode} "
                f"({mode_name(target_mode)})입니다."
            )
            return

        if not confirm_mode_change(current_mode, target_mode):
            print("\n[취소] 확인 문구가 일치하지 않아 변경하지 않았습니다.")
            return

        safely_stop_motor(motor)

        print("\n[제어 모드 설정]")
        write_register(
            motor,
            REG_CONTROL_MODE,
            target_mode,
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

        if mode_after_write != target_mode:
            raise RuntimeError(
                "제어 모드 쓰기 검증 실패: "
                f"목표={target_mode}, 읽은 값={mode_after_write}"
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
        print(
            f"{target_mode} ({mode_name(target_mode)}) "
            "설정 및 저장이 완료됐습니다."
        )
        print("이 설정은 전원을 다시 켜야 적용됩니다.")
        print()
        print("1. 모터 전원 OFF")
        print("2. 약 10초 대기")
        print("3. 모터 전원 ON")
        print("4. 외부 Enable을 계속 LOW로 유지")
        print("5. 이 코드를 다시 실행하고 Enter로 읽기만 수행")
        print(
            f"6. 0x0000 = {target_mode} "
            f"({mode_name(target_mode)})인지 확인"
        )
        print(f"주의: {MODE_WARNINGS[target_mode]}")
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
