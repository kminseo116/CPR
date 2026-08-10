#!/usr/bin/env python3
"""전원 투입 후 Servo ON 상태를 읽기 전용으로 진단한다.

주의
-----
1. 레지스터 쓰기, Servo ON/OFF 및 모터 기동 명령을 보내지 않는다.
2. 내부 인에이블 방식은 0x0021 값으로 상태를 판정한다.
3. 외부 인에이블 방식은 외부 입력 상태 주소가 확인되지 않았으므로
   자동으로 ON/OFF를 단정하지 않고 관련 상태값을 연속 기록한다.
"""

import time

import motor_config as cfg
from motor_driver import MotorDriver


SAMPLE_DURATION_S = 5.0
SAMPLE_PERIOD_S = 0.25


def read_required(driver: MotorDriver, address: int, label: str) -> int:
    """필수 WORD 레지스터를 읽고 주소와 값을 출력한다."""
    value = driver.read_u16(address)
    print(f"{label:<22} 0x{address:04X} = {value}")
    return value


def main() -> None:
    print("====================================================")
    print(" 전원 투입 후 Servo ON 상태 확인 (읽기 전용)")
    print("====================================================")
    print(f"포트: {cfg.PORT}")
    print(f"장치 주소: {cfg.SLAVE_ID}")
    print("모터 축이나 연결된 기구를 손으로 만지지 마십시오.")
    print("이 코드는 어떤 쓰기 및 모터 제어 명령도 보내지 않습니다.")

    driver = MotorDriver()

    try:
        print("\n================ 인에이블 설정 ================")
        enable_method = read_required(
            driver,
            cfg.REG_ENABLE_METHOD,
            "인에이블 방식",
        )
        internal_enable = read_required(
            driver,
            cfg.REG_INTERNAL_ENABLE,
            "내부 인에이블 명령",
        )

        if enable_method == 1:
            if internal_enable == 1:
                print("[확정] 내부 명령 기준 Servo ON 상태입니다.")
            elif internal_enable == 0:
                print("[확정] 내부 명령 기준 Servo OFF 상태입니다.")
            else:
                print(
                    "[주의] 내부 인에이블 값이 예상 범위 0/1이 아닙니다: "
                    f"{internal_enable}"
                )
        elif enable_method == 0:
            print("[판정 보류] 외부 단자 인에이블 방식입니다.")
            print("0x0021 값은 외부 단자의 실제 Servo 상태를 뜻하지 않습니다.")
            print("외부 입력 상태 레지스터 또는 배선 상태 확인이 필요합니다.")
        else:
            print(f"[판정 보류] 알 수 없는 인에이블 방식: {enable_method}")

        print("\n================ 5초 상태 기록 ================")
        print(" time   run     speed   current   position   error   alarm")

        start_time = time.monotonic()
        positions: list[int] = []
        errors: list[int] = []
        currents: list[int] = []
        run_values: list[int] = []

        while True:
            elapsed = time.monotonic() - start_time
            if elapsed > SAMPLE_DURATION_S:
                break

            try:
                status = driver.read_status()
                positions.append(status.position)
                errors.append(status.position_error)
                currents.append(status.current_raw)
                run_values.append(status.run_status)

                print(
                    f"{elapsed:5.2f}  "
                    f"0x{status.run_status:04X}  "
                    f"{status.speed_rpm:6d}  "
                    f"{status.current_raw:8d}  "
                    f"{status.position:9d}  "
                    f"{status.position_error:6d}  "
                    f"0x{status.alarm:04X}"
                )
            except Exception as error:
                print(f"{elapsed:5.2f}  [READ FAIL] {error}")

            time.sleep(SAMPLE_PERIOD_S)

        print("\n================ 진단 요약 ================")
        if positions:
            print(
                f"position 범위: {min(positions)} ~ {max(positions)} "
                f"(변화량 {max(positions) - min(positions)})"
            )
            print(
                f"error 범위: {min(errors)} ~ {max(errors)}"
            )
            print(
                f"current 범위: {min(currents)} ~ {max(currents)} raw"
            )
            print(
                "run_status 관찰값: "
                + ", ".join(f"0x{value:04X}" for value in sorted(set(run_values)))
            )

        if enable_method == 0:
            print("\n최종 판정: 외부 인에이블 방식이라 코드만으로 확정할 수 없습니다.")
            print("외부 Enable 입력의 전압/배선 또는 입력 모니터 주소를 확인하십시오.")
        elif enable_method == 1 and internal_enable == 1:
            print("\n최종 판정: 전원 투입 후 내부 Servo ON입니다.")
        elif enable_method == 1 and internal_enable == 0:
            print("\n최종 판정: 전원 투입 후 내부 Servo OFF입니다.")
        else:
            print("\n최종 판정: 인에이블 설정값을 확인해야 합니다.")

        print("읽기 전용 검사 완료: 모터 설정을 변경하지 않았습니다.")
    finally:
        serial_port = driver.instrument.serial
        if serial_port.is_open:
            serial_port.close()


if __name__ == "__main__":
    main()
