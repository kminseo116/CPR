#!/usr/bin/env python3
"""
IG42S 계열 서보모터 RS485 검증 테스트

기준:
- Modbus-RTU 직접 구현
- 기본 장치 주소: 1
- 기본 통신:
    115200 bps
    8 data bits
    1 stop bit
    parity 없음

현재 실제 배선 확인 결과:
    USB-RS485 A -> 모터 ALM+
    USB-RS485 B -> 모터 ALM-

시험 내용:
1. 제어 모드 읽기
2. 현재 전류 읽기
3. 내부 명령 방식 설정 및 읽기 검증
4. 내부 인에이블 설정 및 읽기 검증
5. 시험 속도 설정 및 읽기 검증
6. 정회전 명령
7. 회전 중 상태/전류/속도 확인
8. 지정 시간 후 정지
9. 정지 후 상태 확인

주의:
- 모터 축과 기구부를 분리하거나 안전하게 고정하십시오.
- 비상시 즉시 전원을 차단할 수 있도록 준비하십시오.
- 처음 시험할 때 TEST_SPEED와 RUN_SECONDS를 낮게 설정하십시오.
"""

import struct
import time
from typing import List

import serial


# ============================================================
# 사용자 설정
# ============================================================

# Windows 예시
# PORT = "COM5"

# Ubuntu 예시
PORT = "/dev/ttyUSB0"

BAUDRATE = 115200
SLAVE_ID = 1

# False: 읽기 시험만 수행
# True: 실제 저속 회전 시험 수행
RUN_MOTOR = True

# 모터 시험 속도 설정값
TEST_SPEED = 100

# 회전 유지 시간
RUN_SECONDS = 2.0

# 시리얼 응답 대기시간
SERIAL_TIMEOUT = 0.5

# 레지스터 쓰기 후 읽기까지 대기시간
WRITE_VERIFY_DELAY = 0.05

# 회전 명령 후 상태를 읽기 전 대기시간
STARTUP_STATUS_DELAY = 0.3

# 정지 명령 후 상태를 읽기 전 대기시간
STOP_STATUS_DELAY = 0.3


# ============================================================
# 레지스터 주소
# ============================================================

REG_CONTROL_MODE = 0x0000       # 제어 모드

REG_ENABLE_METHOD = 0x0020      # 인에이블 방식
REG_INTERNAL_ENABLE = 0x0021    # 내부 인에이블

REG_SPEED_CONTROL = 0x002C      # 속도 모드 제어
REG_INTERNAL_SPEED = 0x002D     # 내부 속도

REG_SAVE_DATA = 0x0030          # 데이터 저장

REG_ALARM_STATUS = 0x0032       # 알람 상태
REG_OPERATION_STATUS = 0x0033   # 작동 상태

REG_CURRENT = 0x0038            # 현재 전류
REG_CURRENT_SPEED = 0x0039      # 현재 회전속도


# ============================================================
# 제어값
# ============================================================

CONTROL_MODE_RS485_SPEED = 2

ENABLE_METHOD_EXTERNAL = 0
ENABLE_METHOD_INTERNAL = 1

INTERNAL_ENABLE_OFF = 0
INTERNAL_ENABLE_ON = 1

SPEED_STOP = 0
SPEED_FORWARD = 1
SPEED_REVERSE = 2


# ============================================================
# Modbus RTU 기본 함수
# ============================================================

def modbus_crc(data: bytes) -> int:
    """
    Modbus-RTU CRC16을 계산합니다.

    반환값:
        16비트 CRC 정수
    """
    crc = 0xFFFF

    for byte in data:
        crc ^= byte

        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1

    return crc & 0xFFFF


def append_crc(frame: bytes) -> bytes:
    """
    Modbus 요청 프레임 뒤에 CRC를 추가합니다.

    Modbus CRC 순서:
        Low byte
        High byte
    """
    crc = modbus_crc(frame)

    return frame + bytes([
        crc & 0xFF,
        (crc >> 8) & 0xFF,
    ])


def hex_string(data: bytes) -> str:
    """바이트열을 읽기 쉬운 HEX 문자열로 변환합니다."""
    return " ".join(f"{byte:02X}" for byte in data)


def read_exact(
    ser: serial.Serial,
    size: int,
) -> bytes:
    """
    지정한 길이만큼 시리얼 데이터를 읽습니다.

    timeout 전에 충분한 데이터가 들어오지 않으면
    TimeoutError를 발생시킵니다.
    """
    data = ser.read(size)

    if len(data) != size:
        raise TimeoutError(
            f"응답 길이 부족: 예상 {size}바이트, "
            f"실제 {len(data)}바이트 "
            f"[{hex_string(data)}]"
        )

    return data


def validate_crc(frame: bytes) -> None:
    """
    수신 프레임의 CRC를 확인합니다.
    """
    if len(frame) < 3:
        raise ValueError(
            "응답 프레임이 너무 짧습니다."
        )

    received_crc = (
        frame[-2]
        | (frame[-1] << 8)
    )

    calculated_crc = modbus_crc(
        frame[:-2]
    )

    if received_crc != calculated_crc:
        raise ValueError(
            "CRC 오류: "
            f"수신=0x{received_crc:04X}, "
            f"계산=0x{calculated_crc:04X}"
        )


def clear_serial_buffers(
    ser: serial.Serial,
) -> None:
    """
    이전 통신에서 남은 송수신 데이터를 비웁니다.
    """
    ser.reset_input_buffer()
    ser.reset_output_buffer()


# ============================================================
# Modbus 레지스터 읽기
# ============================================================

def read_register(
    ser: serial.Serial,
    register: int,
    count: int = 1,
) -> List[int]:
    """
    기능 코드 0x03으로 Holding Register를 읽습니다.

    매개변수:
        ser:
            열린 serial.Serial 객체

        register:
            시작 레지스터 주소

        count:
            읽을 16비트 레지스터 개수

    반환:
        16비트 unsigned 레지스터 값 목록
    """
    if not 1 <= count <= 125:
        raise ValueError(
            f"잘못된 레지스터 개수: {count}"
        )

    request_without_crc = struct.pack(
        ">BBHH",
        SLAVE_ID,
        0x03,
        register,
        count,
    )

    request = append_crc(
        request_without_crc
    )

    ser.reset_input_buffer()

    written = ser.write(request)
    ser.flush()

    if written != len(request):
        raise IOError(
            f"송신 길이 부족: 예상 {len(request)}바이트, "
            f"실제 {written}바이트"
        )

    print(f"TX: {hex_string(request)}")

    # 주소 1바이트 + 기능 코드 1바이트 + 바이트 수 1바이트
    header = read_exact(
        ser,
        3,
    )

    if header[0] != SLAVE_ID:
        raise ValueError(
            f"장치 주소 불일치: "
            f"예상={SLAVE_ID}, 수신={header[0]}"
        )

    # Modbus 예외 응답
    if header[1] & 0x80:
        # 예외 응답 전체 구조:
        # Slave ID + Exception Function + Exception Code + CRC 2바이트
        exception_crc = read_exact(
            ser,
            2,
        )

        response = (
            header
            + exception_crc
        )

        print(
            f"RX: {hex_string(response)}"
        )

        validate_crc(response)

        raise RuntimeError(
            "Modbus 예외 응답: "
            f"기능=0x{header[1]:02X}, "
            f"예외 코드=0x{header[2]:02X}"
        )

    if header[1] != 0x03:
        raise ValueError(
            "기능 코드 불일치: "
            f"예상=0x03, 수신=0x{header[1]:02X}"
        )

    byte_count = header[2]
    expected_byte_count = count * 2

    if byte_count != expected_byte_count:
        raise ValueError(
            "데이터 길이 불일치: "
            f"예상={expected_byte_count}, "
            f"수신={byte_count}"
        )

    data_and_crc = read_exact(
        ser,
        byte_count + 2,
    )

    response = (
        header
        + data_and_crc
    )

    print(f"RX: {hex_string(response)}")

    validate_crc(response)

    register_data = response[3:-2]

    values = list(
        struct.unpack(
            f">{count}H",
            register_data,
        )
    )

    return values


# ============================================================
# Modbus 레지스터 쓰기
# ============================================================

def write_register(
    ser: serial.Serial,
    register: int,
    value: int,
) -> None:
    """
    기능 코드 0x06으로 단일 Holding Register에 씁니다.

    정상 응답은 요청 프레임과 동일합니다.
    """
    if not 0 <= value <= 0xFFFF:
        raise ValueError(
            f"16비트 범위를 벗어난 값: {value}"
        )

    request_without_crc = struct.pack(
        ">BBHH",
        SLAVE_ID,
        0x06,
        register,
        value,
    )

    request = append_crc(
        request_without_crc
    )

    ser.reset_input_buffer()

    written = ser.write(request)
    ser.flush()

    if written != len(request):
        raise IOError(
            f"송신 길이 부족: 예상 {len(request)}바이트, "
            f"실제 {written}바이트"
        )

    print(f"TX: {hex_string(request)}")

    # 정상 0x06 응답은 항상 8바이트
    response = read_exact(
        ser,
        8,
    )

    print(f"RX: {hex_string(response)}")

    validate_crc(response)

    if response[0] != SLAVE_ID:
        raise ValueError(
            f"장치 주소 불일치: "
            f"예상={SLAVE_ID}, 수신={response[0]}"
        )

    if response[1] & 0x80:
        raise RuntimeError(
            "Modbus 예외 응답: "
            f"기능=0x{response[1]:02X}, "
            f"예외 코드=0x{response[2]:02X}"
        )

    if response[1] != 0x06:
        raise ValueError(
            "기능 코드 불일치: "
            f"예상=0x06, 수신=0x{response[1]:02X}"
        )

    if response != request:
        raise ValueError(
            "쓰기 응답이 요청과 일치하지 않습니다.\n"
            f"요청: {hex_string(request)}\n"
            f"응답: {hex_string(response)}"
        )


def write_and_verify(
    ser: serial.Serial,
    register: int,
    value: int,
    name: str,
) -> int:
    """
    레지스터에 값을 쓴 뒤 동일 레지스터를 다시 읽어
    실제 반영 여부를 확인합니다.
    """
    write_register(
        ser,
        register,
        value,
    )

    time.sleep(
        WRITE_VERIFY_DELAY
    )

    readback = read_register(
        ser,
        register,
        1,
    )[0]

    print(
        f"[검증] {name}: "
        f"설정값={value}, "
        f"읽은 값={readback}"
    )

    if readback != value:
        raise RuntimeError(
            f"{name} 설정 검증 실패: "
            f"설정값={value}, "
            f"읽은 값={readback}"
        )

    return readback


# ============================================================
# 상태 확인 함수
# ============================================================

def get_control_mode(
    ser: serial.Serial,
) -> int:
    """현재 제어 모드를 읽습니다."""
    return read_register(
        ser,
        REG_CONTROL_MODE,
        1,
    )[0]


def get_current_raw(
    ser: serial.Serial,
) -> int:
    """현재 전류 RAW 값을 읽습니다."""
    return read_register(
        ser,
        REG_CURRENT,
        1,
    )[0]


def print_initial_status(
    ser: serial.Serial,
) -> int:
    """
    프로그램 시작 시 제어 모드와 전류를 읽어 출력합니다.
    """
    control_mode = get_control_mode(
        ser
    )

    current_raw = get_current_raw(
        ser
    )

    mode_names = {
        0: "외부 펄스 모드",
        1: "위치 모드",
        2: "RS485 속도 모드",
        3: "외부 단자 속도 모드",
        4: "RS485 토크 모드",
        5: "외부 단자 토크 모드",
    }

    print()
    print("========== 모터 초기 상태 ==========")

    print(
        f"제어 모드: {control_mode} "
        f"({mode_names.get(control_mode, '알 수 없음')})"
    )

    print(
        f"현재 전류 RAW: {current_raw}"
    )

    print("====================================")
    print()

    return control_mode


def print_runtime_status(
    ser: serial.Serial,
    title: str,
) -> None:
    """
    모터의 실시간 상태 레지스터를 읽어 출력합니다.

    매뉴얼에 단위 또는 비트별 의미가 불명확한 항목은
    RAW 값 그대로 출력합니다.
    """
    alarm_status = read_register(
        ser,
        REG_ALARM_STATUS,
        1,
    )[0]

    operation_status = read_register(
        ser,
        REG_OPERATION_STATUS,
        1,
    )[0]

    current_raw = read_register(
        ser,
        REG_CURRENT,
        1,
    )[0]

    current_speed_raw = read_register(
        ser,
        REG_CURRENT_SPEED,
        1,
    )[0]

    print()
    print(
        f"========== {title} =========="
    )

    print(
        f"알람 상태 RAW: "
        f"0x{alarm_status:04X} "
        f"({alarm_status})"
    )

    print(
        f"작동 상태 RAW: "
        f"0x{operation_status:04X} "
        f"({operation_status})"
    )

    print(
        f"현재 전류 RAW: "
        f"{current_raw}"
    )

    print(
        f"현재 회전속도 RAW: "
        f"{current_speed_raw}"
    )

    print(
        "=" * (22 + len(title))
    )
    print()


# ============================================================
# 모터 제어 함수
# ============================================================

def stop_motor(
    ser: serial.Serial,
    verify: bool = True,
) -> None:
    """
    속도 모드 제어 레지스터에 0을 써서 정지합니다.
    """
    print("\n[정지 명령]")

    if verify:
        write_and_verify(
            ser,
            REG_SPEED_CONTROL,
            SPEED_STOP,
            "속도 제어 정지",
        )
    else:
        write_register(
            ser,
            REG_SPEED_CONTROL,
            SPEED_STOP,
        )


def run_test(
    ser: serial.Serial,
) -> None:
    """
    RS485 속도 모드에서 저속 회전 시험을 수행합니다.

    정지 명령은 이 함수 안에서 보내지 않습니다.
    main()의 finally 블록에서 한 번만 전송합니다.
    """

    print("\n[내부 명령 방식 설정]")

    write_and_verify(
        ser,
        REG_ENABLE_METHOD,
        ENABLE_METHOD_INTERNAL,
        "내부 명령 방식",
    )

    print("\n[내부 인에이블 ON]")

    write_and_verify(
        ser,
        REG_INTERNAL_ENABLE,
        INTERNAL_ENABLE_ON,
        "내부 인에이블",
    )

    print(
        f"\n[시험 속도 설정: {TEST_SPEED}]"
    )

    write_and_verify(
        ser,
        REG_INTERNAL_SPEED,
        TEST_SPEED,
        "내부 속도",
    )

    print_runtime_status(
        ser,
        "회전 전 상태",
    )

    print("\n[정회전 시작]")

    write_and_verify(
        ser,
        REG_SPEED_CONTROL,
        SPEED_FORWARD,
        "정회전 명령",
    )

    # 모터가 회전을 시작하고 상태값이 반영될 시간
    time.sleep(
        STARTUP_STATUS_DELAY
    )

    print_runtime_status(
        ser,
        "회전 중 상태",
    )

    remaining_time = max(
        0.0,
        RUN_SECONDS - STARTUP_STATUS_DELAY,
    )

    print(
        f"{remaining_time:.1f}초 동안 "
        "회전을 추가로 유지합니다."
    )

    time.sleep(
        remaining_time
    )


# ============================================================
# 메인 프로그램
# ============================================================

def main() -> None:
    print("RS485 포트를 엽니다.")
    print(f"포트: {PORT}")
    print(f"속도: {BAUDRATE}")
    print(f"장치 주소: {SLAVE_ID}")
    print()

    with serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=SERIAL_TIMEOUT,
        write_timeout=SERIAL_TIMEOUT,
    ) as ser:

        # USB-RS485 변환기와 시리얼 포트 안정화 대기
        time.sleep(0.3)

        clear_serial_buffers(ser)

        control_mode = print_initial_status(
            ser
        )

        if not RUN_MOTOR:
            print(
                "RUN_MOTOR=False이므로 "
                "읽기 시험만 완료했습니다."
            )

            print(
                "모터를 회전시키려면 안전을 확인한 뒤 "
                "코드 위쪽의 RUN_MOTOR를 True로 "
                "변경하십시오."
            )

            return

        if control_mode != CONTROL_MODE_RS485_SPEED:
            print(
                "현재 제어 모드가 2가 아니므로 "
                "회전 시험을 중단합니다."
            )

            print(
                "0x0000 레지스터를 2로 설정하고, "
                "0x0030 저장 명령 후 전원을 "
                "재인가하십시오."
            )

            return

        motor_start_command_sent = False

        try:
            # 정회전 명령까지 정상 처리되면
            # 이후 finally에서 정지를 수행합니다.
            run_test(ser)

            motor_start_command_sent = True

        except KeyboardInterrupt:
            print("\nCtrl+C가 감지되었습니다.")

        finally:
            # RUN_MOTOR=True이면 정지 명령을 시도합니다.
            #
            # 회전 명령 이전에 오류가 발생했어도
            # 정지 명령을 보내는 편이 안전합니다.
            if RUN_MOTOR:
                try:
                    stop_motor(
                        ser,
                        verify=True,
                    )

                    time.sleep(
                        STOP_STATUS_DELAY
                    )

                    print_runtime_status(
                        ser,
                        "정지 후 상태",
                    )

                    if motor_start_command_sent:
                        print(
                            "저속 회전 시험이 완료되었습니다."
                        )
                    else:
                        print(
                            "시험 도중 오류 또는 중단이 "
                            "발생했으며 정지 명령을 "
                            "전송했습니다."
                        )

                except Exception as stop_error:
                    print(
                        "최종 정지 명령 또는 "
                        "정지 후 상태 확인 실패: "
                        f"{stop_error}"
                    )

                    print(
                        "모터가 움직이고 있다면 "
                        "즉시 전원을 차단하십시오."
                    )


# ============================================================
# 프로그램 시작점
# ============================================================

if __name__ == "__main__":
    try:
        main()

    except serial.SerialException as error:
        print(
            f"시리얼 포트 오류: {error}"
        )

    except TimeoutError as error:
        print(
            f"응답 시간 초과: {error}"
        )

    except KeyboardInterrupt:
        print(
            "\n프로그램이 사용자에 의해 종료되었습니다."
        )

    except Exception as error:
        print(
            f"오류: {error}"
        )