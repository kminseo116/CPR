# #!/usr/bin/env python3

# import time
# import minimalmodbus
# import serial


# # 통신 설정
# PORT = "/dev/ttyUSB0"
# SLAVE_ID = 1
# BAUDRATE = 115200

# # 시험 설정
# RUN_MOTOR = True
# TEST_SPEED = 100
# RUN_SECONDS = 2.0


# # 레지스터
# REG_CONTROL_MODE = 0x0000
# REG_ENABLE_METHOD = 0x0020
# REG_INTERNAL_ENABLE = 0x0021
# REG_SPEED_CONTROL = 0x002C
# REG_INTERNAL_SPEED = 0x002D
# REG_ALARM_STATUS = 0x0032
# REG_OPERATION_STATUS = 0x0033
# REG_CURRENT = 0x0038
# REG_CURRENT_SPEED = 0x0039


# def create_motor():
#     motor = minimalmodbus.Instrument(
#         PORT,
#         SLAVE_ID,
#         mode=minimalmodbus.MODE_RTU,
#     )

#     motor.serial.baudrate = BAUDRATE
#     motor.serial.bytesize = 8
#     motor.serial.parity = serial.PARITY_NONE
#     motor.serial.stopbits = 1
#     motor.serial.timeout = 0.5

#     motor.clear_buffers_before_each_transaction = True
#     motor.close_port_after_each_call = False

#     return motor


# def read_reg(motor, address):
#     return int(
#         motor.read_register(
#             registeraddress=address,
#             number_of_decimals=0,
#             functioncode=3,
#             signed=False,
#         )
#     )


# def write_reg(motor, address, value):
#     motor.write_register(
#         registeraddress=address,
#         value=value,
#         number_of_decimals=0,
#         functioncode=6,
#         signed=False,
#     )


# def print_status(motor, title):
#     alarm = read_reg(motor, REG_ALARM_STATUS)
#     operation = read_reg(motor, REG_OPERATION_STATUS)
#     current = read_reg(motor, REG_CURRENT)
#     speed = read_reg(motor, REG_CURRENT_SPEED)

#     print(f"\n========== {title} ==========")
#     print(f"알람 상태 RAW: {alarm}")
#     print(f"작동 상태 RAW: {operation}")
#     print(f"현재 전류 RAW: {current}")
#     print(f"현재 회전속도 RAW: {speed}")
#     print("==============================")


# def stop_motor(motor):
#     write_reg(motor, REG_SPEED_CONTROL, 0)
#     print("\n모터 정지 명령 완료")


# def main():
#     motor = create_motor()

#     try:
#         control_mode = read_reg(
#             motor,
#             REG_CONTROL_MODE,
#         )

#         print(f"제어 모드: {control_mode}")

#         if control_mode != 2:
#             print("RS485 속도 모드가 아닙니다.")
#             return

#         if not RUN_MOTOR:
#             print_status(motor, "현재 상태")
#             return

#         # 내부 명령 방식
#         write_reg(
#             motor,
#             REG_ENABLE_METHOD,
#             1,
#         )

#         # 내부 인에이블
#         write_reg(
#             motor,
#             REG_INTERNAL_ENABLE,
#             1,
#         )

#         # 속도 설정
#         write_reg(
#             motor,
#             REG_INTERNAL_SPEED,
#             TEST_SPEED,
#         )

#         print_status(
#             motor,
#             "회전 전 상태",
#         )

#         # 정회전 시작
#         write_reg(
#             motor,
#             REG_SPEED_CONTROL,
#             1,
#         )

#         time.sleep(0.3)

#         print_status(
#             motor,
#             "회전 중 상태",
#         )

#         time.sleep(
#             max(0.0, RUN_SECONDS - 0.3)
#         )

#     except KeyboardInterrupt:
#         print("\nCtrl+C 감지")

#     finally:
#         try:
#             stop_motor(motor)
#             time.sleep(0.3)
#             print_status(
#                 motor,
#                 "정지 후 상태",
#             )

#         except Exception as error:
#             print(f"정지 실패: {error}")

#         if motor.serial.is_open:
#             motor.serial.close()


# if __name__ == "__main__":
#     try:
#         main()

#     except Exception as error:
#         print(f"오류: {error}")

#!/usr/bin/env python3
"""RS485 서보모터 제어 모드 확인 코드."""

import minimalmodbus
import serial


PORT = "/dev/ttyUSB0"
SLAVE_ID = 1
BAUDRATE = 115200

REG_CONTROL_MODE = 0x0000


def main() -> None:
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

    motor.clear_buffers_before_each_transaction = True
    motor.close_port_after_each_call = False

    try:
        control_mode = int(
            motor.read_register(
                registeraddress=REG_CONTROL_MODE,
                number_of_decimals=0,
                functioncode=3,
                signed=False,
            )
        )

        mode_names = {
            0: "외부 펄스 모드",
            1: "위치 모드",
            2: "RS485 속도 모드",
            3: "외부 단자 속도 모드",
            4: "RS485 토크 모드",
            5: "외부 단자 토크 모드",
        }

        print("========== 제어 모드 확인 ==========")
        print(f"레지스터 주소: 0x{REG_CONTROL_MODE:04X}")
        print(f"읽은 값: {control_mode}")
        print(
            "현재 모드:",
            mode_names.get(control_mode, "알 수 없는 값"),
        )

        if control_mode == 1:
            print("\n결과: 위치 제어 모드입니다.")
            print("위치 레지스터 검증 단계로 진행할 수 있습니다.")

        else:
            print("\n결과: 위치 제어 모드가 아닙니다.")

            if control_mode == 2:
                print(
                    "현재는 이전 시험에서 사용한 "
                    "RS485 속도 모드입니다."
                )

            print(
                "현재 상태에서는 CPR 절대 위치 이동 코드를 "
                "실행하지 마십시오."
            )

        print("====================================")

    finally:
        if motor.serial.is_open:
            motor.serial.close()


if __name__ == "__main__":
    try:
        main()

    except FileNotFoundError as error:
        print(f"시리얼 포트가 없습니다: {error}")
        print("확인: ls -l /dev/ttyUSB*")

    except PermissionError as error:
        print(f"시리얼 포트 권한 오류: {error}")

    except minimalmodbus.NoResponseError as error:
        print(f"모터 응답 없음: {error}")

    except Exception as error:
        print(f"오류: {error}")