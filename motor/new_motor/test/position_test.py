#!/usr/bin/env python3
"""RS485 서보모터 소량 절대 위치 이동 시험."""

import time

from motor_driver import MotorDriver


# 시험 조건
TEST_MOVE_UNITS = 150
TEST_SPEED_RPM = 20
MOVE_TIMEOUT_S = 5.0
POSITION_TOLERANCE = 10
STATUS_PERIOD_S = 0.05


def wait_until_reached(
    driver: MotorDriver,
    target_position: int,
) -> None:
    start_time = time.monotonic()

    while True:
        status = driver.read_status()

        print(
            f"target={target_position}, "
            f"position={status.position}, "
            f"error={status.position_error}, "
            f"speed={status.speed_rpm}, "
            f"current={status.current_raw}, "
            f"alarm=0x{status.alarm:04X}, "
            f"run=0x{status.run_status:04X}"
        )

        if status.alarm != 0:
            driver.stop()
            raise RuntimeError(
                f"모터 알람 발생: 0x{status.alarm:04X}"
            )

        position_error = abs(
            status.position - target_position
        )

        if position_error <= POSITION_TOLERANCE:
            print("목표 위치 도달")
            return

        if time.monotonic() - start_time >= MOVE_TIMEOUT_S:
            driver.stop()
            raise TimeoutError(
                f"이동 시간 초과: "
                f"target={target_position}, "
                f"current={status.position}"
            )

        time.sleep(STATUS_PERIOD_S)


def main() -> None:
    driver = MotorDriver()

    initial_position = None

    try:
        print("=== 소량 위치 이동 시험 ===")

        driver.verify_connection()

        # 실행 시 설정
        driver.setup_runtime()
        driver.enable()

        initial_position = driver.read_position()
        target_position = (
            initial_position + TEST_MOVE_UNITS
        )

        print(f"초기 위치: {initial_position}")
        print(f"목표 위치: {target_position}")
        print(f"이동량: {TEST_MOVE_UNITS} unit")
        print(f"시험 속도: {TEST_SPEED_RPM} rpm")

        input(
            "\n주변을 확인한 후 Enter를 누르면 "
            "소량 이동합니다. "
            "취소하려면 Ctrl+C: "
        )

        # 소량 이동
        print("\n[목표 위치로 이동]")
        driver.move_absolute(
            target_position,
            TEST_SPEED_RPM,
        )
        wait_until_reached(
            driver,
            target_position,
        )

        time.sleep(1.0)

        # 초기 위치 복귀
        print("\n[초기 위치로 복귀]")
        driver.move_absolute(
            initial_position,
            TEST_SPEED_RPM,
        )
        wait_until_reached(
            driver,
            initial_position,
        )

        print("\n소량 위치 이동 시험 완료")

    except KeyboardInterrupt:
        print("\nCtrl+C 감지: 모터를 정지합니다.")

    except Exception as error:
        print(f"\n시험 오류: {error}")

    finally:
        try:
            driver.stop()
            print("모터 정지 명령 완료")
        except Exception as stop_error:
            print(f"정지 명령 실패: {stop_error}")


if __name__ == "__main__":
    main()