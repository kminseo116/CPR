#!/usr/bin/env python3
"""RS485 서보모터 소량 증분 위치 이동 및 단위 측정 시험."""

import time

import motor_config as cfg
from motor_driver import MotorDriver


# 시험 조건
TEST_MOVE_DEGREES = 90.0
TEST_MOVE_UNITS = round(
    cfg.COMMAND_UNITS_PER_MOTOR_REV
    * TEST_MOVE_DEGREES
    / 360.0
)
EXPECTED_FEEDBACK_DELTA = round(
    cfg.FEEDBACK_COUNTS_PER_MOTOR_REV
    * TEST_MOVE_DEGREES
    / 360.0
)

# MSTEP 1000에서 실제 검증한 내부 속도 설정값까지만 사용한다.
TEST_SPEED_SETTING = 50
MOVE_TIMEOUT_S = 5.0
FEEDBACK_TOLERANCE = 100
MIN_FEEDBACK_MOVEMENT = 16
MAX_FEEDBACK_OVERSHOOT = 256
STATUS_PERIOD_S = 0.05
REACHED_CONFIRM_COUNT = 3


def wait_until_reached(
    driver: MotorDriver,
    start_position: int,
    command_units: int,
    expected_feedback_delta: int,
) -> int:
    start_time = time.monotonic()
    movement_detected = False
    reached_count = 0
    peak_feedback_delta = 0

    while True:
        status = driver.read_status()
        feedback_delta = status.position - start_position
        if abs(feedback_delta) > abs(peak_feedback_delta):
            peak_feedback_delta = feedback_delta

        print(
            f"command_delta={command_units}, "
            f"position={status.position}, "
            f"feedback_delta={feedback_delta}, "
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

        if (
            abs(feedback_delta) >= MIN_FEEDBACK_MOVEMENT
            or status.speed_rpm != 0
        ):
            movement_detected = True

        if (
            abs(feedback_delta)
            > abs(expected_feedback_delta) + MAX_FEEDBACK_OVERSHOOT
        ):
            driver.stop()
            raise RuntimeError(
                "안전 범위를 초과해 즉시 정지: "
                f"expected={expected_feedback_delta}, "
                f"actual={feedback_delta}"
            )

        if (
            abs(feedback_delta) >= MIN_FEEDBACK_MOVEMENT
            and feedback_delta * expected_feedback_delta < 0
        ):
            driver.stop()
            raise RuntimeError(
                "예상과 반대 방향으로 움직여 즉시 정지: "
                f"expected={expected_feedback_delta}, "
                f"actual={feedback_delta}"
            )

        motor_reached = (
            movement_detected
            and abs(feedback_delta - expected_feedback_delta)
            <= FEEDBACK_TOLERANCE
            and status.speed_rpm == 0
        )

        if motor_reached:
            reached_count += 1
            if reached_count >= REACHED_CONFIRM_COUNT:
                print("모터 내부 목표 위치 도달")
                return status.position
        else:
            reached_count = 0

        if time.monotonic() - start_time >= MOVE_TIMEOUT_S:
            driver.stop()
            raise TimeoutError(
                f"이동 시간 초과: "
                f"command_delta={command_units}, "
                f"feedback_delta={feedback_delta}, "
                f"peak_feedback_delta={peak_feedback_delta}, "
                f"position={status.position}, "
                f"error={status.position_error}"
            )

        time.sleep(STATUS_PERIOD_S)


def main() -> None:
    if not (
        cfg.MSTEP_POSITION_CALIBRATION_COMPLETE
        and cfg.MSTEP_SPEED_CALIBRATION_COMPLETE
    ):
        print("====================================================")
        print(" position_test 실행 차단")
        print("====================================================")
        print("MSTEP 1000 위치·속도 재보정이 아직 완료되지 않았습니다.")
        print("mstep_recalibration.py와 mstep_90deg_calibration.py를 먼저 실행하십시오.")
        return

    driver = MotorDriver()

    initial_position = None

    try:
        print("=== 소량 증분 위치 이동 및 단위 측정 시험 ===")

        driver.verify_connection()

        # 실행 시 설정
        driver.setup_runtime()

        initial_position = driver.read_position()

        print(f"초기 위치: {initial_position}")
        print(f"시험 회전각: {TEST_MOVE_DEGREES:.1f} degree")
        print(f"증분 위치 명령: {TEST_MOVE_UNITS} command unit")
        print(
            "예상 피드백 변화: "
            f"{EXPECTED_FEEDBACK_DELTA} feedback count"
        )
        print(f"시험 내부 속도 설정: {TEST_SPEED_SETTING}")

        input(
            "\n주변을 확인한 후 Enter를 누르면 "
            "Servo ON 후 소량 이동합니다. "
            "취소하려면 Ctrl+C: "
        )

        driver.enable()
        time.sleep(0.2)

        start_position = driver.read_position()
        enable_shift = start_position - initial_position
        print(f"Servo ON 직후 위치: {start_position}")
        print(f"Servo ON 위치 변화: {enable_shift} feedback count")

        # 소량 증분 이동
        print("\n[양의 방향 증분 이동]")
        driver.move_incremental(
            TEST_MOVE_UNITS,
            TEST_SPEED_SETTING,
        )
        moved_position = wait_until_reached(
            driver,
            start_position,
            TEST_MOVE_UNITS,
            EXPECTED_FEEDBACK_DELTA,
        )

        feedback_delta = moved_position - start_position
        angle_degrees = (
            feedback_delta
            * 360.0
            / cfg.FEEDBACK_COUNTS_PER_MOTOR_REV
        )
        feedback_per_command = feedback_delta / TEST_MOVE_UNITS

        print("\n[이동량 측정 결과]")
        print(f"command delta: {TEST_MOVE_UNITS} unit")
        print(f"feedback delta: {feedback_delta} count")
        print(f"축 회전각: {angle_degrees:.3f} degree")
        print(
            "명령 1 unit당 피드백: "
            f"{feedback_per_command:.6f} count"
        )

        time.sleep(1.0)

        # 반대 증분 명령으로 이동 전 위치에 복귀
        print("\n[음의 방향 증분 복귀]")
        driver.move_incremental(
            -TEST_MOVE_UNITS,
            TEST_SPEED_SETTING,
        )
        returned_position = wait_until_reached(
            driver,
            moved_position,
            -TEST_MOVE_UNITS,
            -EXPECTED_FEEDBACK_DELTA,
        )

        return_error = returned_position - start_position
        print(f"복귀 후 위치: {returned_position}")
        print(f"시작 위치 대비 복귀 오차: {return_error} feedback count")

        print("\n소량 증분 위치 이동 시험 완료")

    except KeyboardInterrupt:
        print("\nCtrl+C 감지: 모터를 정지합니다.")

    except Exception as error:
        print(f"\n시험 오류: {error}")

    finally:
        try:
            if cfg.SERVO_OFF_ON_PROGRAM_EXIT:
                driver.disable()
                print("모터 정지 및 Servo OFF 완료")
            else:
                driver.stop()
                print("모터 정지 명령 완료 (Servo ON 유지)")
        except Exception as stop_error:
            print(f"정지 또는 Servo OFF 실패: {stop_error}")


if __name__ == "__main__":
    main()
