#!/usr/bin/env python3
"""수동 위치 확인 전 모터를 내부 제어 방식의 Servo OFF로 준비한다.

이 설정은 런타임에만 적용하며 파라미터 저장 명령은 보내지 않는다.
전원을 다시 인가하면 장비의 저장된 설정으로 돌아갈 수 있다.
"""

import motor_config as cfg
from motor_driver import MotorDriver, MotorError


def main() -> None:
    print("====================================================")
    print(" 수동 위치 확인 준비: 이동 해제 + 내부 방식 + Servo OFF")
    print("====================================================")

    driver = MotorDriver()

    try:
        driver.verify_connection()

        # 내부 인에이블 명령이 남아 있더라도 전환 순간 켜지지 않게 한다.
        driver.write_u16(cfg.REG_INTERNAL_ENABLE, 0)

        # 이전 위치 이동 시작 명령을 해제한다.
        driver.write_u16(cfg.REG_MOTOR_START, 0)

        # Servo ON/OFF를 RS485 내부 명령으로 제어하도록 전환한다.
        driver.write_u16(cfg.REG_ENABLE_METHOD, 1)

        # 내부 제어 방식으로 전환한 뒤 Servo OFF를 다시 명시한다.
        driver.write_u16(cfg.REG_INTERNAL_ENABLE, 0)

        enable_method = driver.read_u16(cfg.REG_ENABLE_METHOD)
        internal_enable = driver.read_u16(cfg.REG_INTERNAL_ENABLE)

        print(f"인에이블 방식 0x0020 = {enable_method}")
        print(f"내부 인에이블 0x0021 = {internal_enable}")

        if enable_method != 1 or internal_enable != 0:
            raise MotorError(
                "Servo OFF 준비 검증 실패: "
                f"method={enable_method}, "
                f"internal_enable={internal_enable}"
            )

        print("[성공] RS485 내부 인에이블 방식으로 전환했습니다.")
        print("[성공] Servo OFF 상태를 확인했습니다.")
        print("전원을 끄지 말고 다음 위치 확인을 진행하십시오.")
        print("주의: 설정 저장 명령은 보내지 않았습니다.")
    finally:
        serial_port = driver.instrument.serial
        if serial_port.is_open:
            serial_port.close()


if __name__ == "__main__":
    main()
