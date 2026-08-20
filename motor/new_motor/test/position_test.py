#!/usr/bin/env python3
"""RS485 서보모터 반복 증분 위치 왕복 시험."""

import time

import motor_config as cfg
from motor_driver import MotorDriver


# ============================================================
# 시험 조건
# ============================================================

# 한 방향 회전각
# 1080° = 모터축 3회전
TEST_MOVE_DEGREES = 360*5

# 왕복 반복 횟수
TEST_REPEAT_COUNT = 10

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

# MSTEP 1000에서 사용할 내부 속도 설정값
TEST_SPEED_SETTING = 2500

# 한 방향 이동당 제한 시간
MOVE_TIMEOUT_S = 10.0

# 실제 이동이 시작되었다고 판단할 최소 피드백 변화량
MIN_FEEDBACK_MOVEMENT = 1000

# 예상 피드백보다 이 값 이상 더 움직이면 안전 정지
MAX_FEEDBACK_OVERSHOOT = 1000

# 상태 확인 주기
STATUS_PERIOD_S = 0.05

# 목표 위치 도달 확인 횟수
REACHED_CONFIRM_COUNT = 1

# 목표 위치 도달 판정용 위치 오차 허용값
POSITION_ERROR_TOLERANCE = 50

# 정회전 ↔ 역회전 사이 대기시간
DIRECTION_CHANGE_DELAY_S = 0.1


def wait_until_reached(
    driver: MotorDriver,
    start_position: int,
    command_units: int,
    expected_feedback_delta: int,
) -> int:
    """모터가 증분 위치 명령을 완료할 때까지 상태를 확인한다."""

    start_time = time.monotonic()

    movement_detected = False
    reached_count = 0
    peak_feedback_delta = 0

    while True:
        status = driver.read_status()

        feedback_delta = (
            status.position - start_position
        )

        # 최대 이동량 기록
        if abs(feedback_delta) > abs(
            peak_feedback_delta
        ):
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

        # ====================================================
        # 1. 모터 알람 검사
        # ====================================================

        if status.alarm != 0:
            driver.stop()

            raise RuntimeError(
                f"모터 알람 발생: "
                f"0x{status.alarm:04X}"
            )

        # ====================================================
        # 2. 실제 이동 시작 여부
        # ====================================================

        if (
            abs(feedback_delta)
            >= MIN_FEEDBACK_MOVEMENT
            or status.speed_rpm != 0
        ):
            movement_detected = True

        # ====================================================
        # 3. 예상 이동량 과도 초과 검사
        # ====================================================

        if (
            abs(feedback_delta)
            > abs(expected_feedback_delta)
            + MAX_FEEDBACK_OVERSHOOT
        ):
            driver.stop()

            raise RuntimeError(
                "안전 범위를 초과해 즉시 정지: "
                f"expected="
                f"{expected_feedback_delta}, "
                f"actual={feedback_delta}"
            )

        # ====================================================
        # 4. 실제 운전 중 반대 방향 이동 검사
        #
        # speed=0인 정착 구간의 미세 위치 변화는
        # 반대 방향 오류로 판단하지 않는다.
        # ====================================================

        if (
            status.speed_rpm != 0
            and abs(feedback_delta)
            >= MIN_FEEDBACK_MOVEMENT
            and (
                feedback_delta
                * expected_feedback_delta
                < 0
            )
        ):
            driver.stop()

            raise RuntimeError(
                "예상과 반대 방향으로 움직여 "
                "즉시 정지: "
                f"expected="
                f"{expected_feedback_delta}, "
                f"actual={feedback_delta}"
            )

        # ====================================================
        # 5. 목표 위치 도달 판정
        # ====================================================

        motor_reached = (
            movement_detected
            and status.speed_rpm == 0
            and abs(status.position_error)
            <= POSITION_ERROR_TOLERANCE
        )

        if motor_reached:
            reached_count += 1

            if (
                reached_count
                >= REACHED_CONFIRM_COUNT
            ):
                print(
                    "모터 내부 목표 위치 도달 "
                    f"(feedback_delta="
                    f"{feedback_delta}, "
                    f"expected="
                    f"{expected_feedback_delta}, "
                    f"position_error="
                    f"{status.position_error})"
                )

                return status.position

        else:
            reached_count = 0

        # ====================================================
        # 6. 이동 시간 초과
        # ====================================================

        if (
            time.monotonic() - start_time
            >= MOVE_TIMEOUT_S
        ):
            driver.stop()

            raise TimeoutError(
                "이동 시간 초과: "
                f"command_delta={command_units}, "
                f"feedback_delta="
                f"{feedback_delta}, "
                f"expected_feedback_delta="
                f"{expected_feedback_delta}, "
                f"peak_feedback_delta="
                f"{peak_feedback_delta}, "
                f"position={status.position}, "
                f"error="
                f"{status.position_error}, "
                f"speed={status.speed_rpm}"
            )

        time.sleep(STATUS_PERIOD_S)


def main() -> None:
    """1080° 정·역회전을 지정 횟수만큼 반복한다."""

    # ========================================================
    # MSTEP 위치·속도 보정 확인
    # ========================================================

    if not (
        cfg.MSTEP_POSITION_CALIBRATION_COMPLETE
        and cfg.MSTEP_SPEED_CALIBRATION_COMPLETE
    ):
        print(
            "========================================"
        )
        print(" position_test 실행 차단")
        print(
            "========================================"
        )
        print(
            "MSTEP 위치·속도 재보정이 "
            "완료되지 않았습니다."
        )
        print(
            "mstep_recalibration.py와 "
            "mstep_90deg_calibration.py를 "
            "먼저 실행하십시오."
        )

        return

    driver = MotorDriver()

    initial_position = None

    try:
        print(
            "=== RS485 서보모터 반복 왕복 위치 시험 ==="
        )

        # ====================================================
        # 통신 확인
        # ====================================================

        driver.verify_connection()

        # ====================================================
        # 실행 시 모터 설정
        # ====================================================

        driver.setup_runtime()

        # ====================================================
        # Servo ON 전 초기 위치
        # ====================================================

        initial_position = (
            driver.read_position()
        )

        print(
            f"초기 위치: "
            f"{initial_position}"
        )

        print(
            f"한 방향 회전각: "
            f"{TEST_MOVE_DEGREES:.1f} degree"
        )

        print(
            f"왕복 반복 횟수: "
            f"{TEST_REPEAT_COUNT}"
        )

        print(
            f"증분 위치 명령: "
            f"{TEST_MOVE_UNITS} command unit"
        )

        print(
            "예상 피드백 변화: "
            f"{EXPECTED_FEEDBACK_DELTA} "
            f"feedback count"
        )

        print(
            f"시험 내부 속도 설정: "
            f"{TEST_SPEED_SETTING}"
        )

        print(
            f"위치 오차 허용값: "
            f"{POSITION_ERROR_TOLERANCE}"
        )

        print(
            f"피드백 초과 허용값: "
            f"{MAX_FEEDBACK_OVERSHOOT}"
        )

        print(
            f"이동 제한 시간: "
            f"{MOVE_TIMEOUT_S:.1f} s"
        )

        # ====================================================
        # 사용자 확인
        # ====================================================

        input(
            "\n주변을 확인한 후 Enter를 누르면 "
            f"{TEST_MOVE_DEGREES:.0f}° 정·역회전을 "
            f"{TEST_REPEAT_COUNT}회 반복합니다. "
            "취소하려면 Ctrl+C: "
        )

        # ====================================================
        # Servo ON
        # ====================================================

        driver.enable()

        time.sleep(0.2)

        start_position = (
            driver.read_position()
        )

        enable_shift = (
            start_position
            - initial_position
        )

        print(
            f"Servo ON 직후 위치: "
            f"{start_position}"
        )

        print(
            f"Servo ON 위치 변화: "
            f"{enable_shift} feedback count"
        )

        # 최초 기준 위치
        original_start_position = (
            start_position
        )

        # ====================================================
        # 왕복 반복
        # ====================================================

        for cycle in range(
            1,
            TEST_REPEAT_COUNT + 1,
        ):
            print(
                "\n"
                "========================================"
            )
            print(
                f" 왕복 {cycle} / "
                f"{TEST_REPEAT_COUNT}"
            )
            print(
                "========================================"
            )

            # ------------------------------------------------
            # 양의 방향 1080°
            # ------------------------------------------------

            print(
                f"\n[{cycle}/{TEST_REPEAT_COUNT}] "
                "양의 방향 이동"
            )

            forward_start_position = (
                driver.read_position()
            )

            driver.move_incremental(
                TEST_MOVE_UNITS,
                TEST_SPEED_SETTING,
            )

            moved_position = (
                wait_until_reached(
                    driver,
                    forward_start_position,
                    TEST_MOVE_UNITS,
                    EXPECTED_FEEDBACK_DELTA,
                )
            )

            forward_feedback_delta = (
                moved_position
                - forward_start_position
            )

            forward_angle = (
                forward_feedback_delta
                * 360.0
                / cfg.FEEDBACK_COUNTS_PER_MOTOR_REV
            )

            print(
                f"[{cycle}/{TEST_REPEAT_COUNT}] "
                "양의 방향 완료"
            )

            print(
                f"이동 위치: "
                f"{moved_position}"
            )

            print(
                f"피드백 변화: "
                f"{forward_feedback_delta} count"
            )

            print(
                f"환산 회전각: "
                f"{forward_angle:.3f} degree"
            )

            time.sleep(
                DIRECTION_CHANGE_DELAY_S
            )

            # ------------------------------------------------
            # 음의 방향 1080° 복귀
            # ------------------------------------------------

            print(
                f"\n[{cycle}/{TEST_REPEAT_COUNT}] "
                "음의 방향 복귀"
            )

            reverse_start_position = (
                driver.read_position()
            )

            driver.move_incremental(
                -TEST_MOVE_UNITS,
                TEST_SPEED_SETTING,
            )

            returned_position = (
                wait_until_reached(
                    driver,
                    reverse_start_position,
                    -TEST_MOVE_UNITS,
                    -EXPECTED_FEEDBACK_DELTA,
                )
            )

            reverse_feedback_delta = (
                returned_position
                - reverse_start_position
            )

            reverse_angle = (
                reverse_feedback_delta
                * 360.0
                / cfg.FEEDBACK_COUNTS_PER_MOTOR_REV
            )

            # 최초 시작점 대비 누적 복귀 오차
            cumulative_return_error = (
                returned_position
                - original_start_position
            )

            print(
                f"[{cycle}/{TEST_REPEAT_COUNT}] "
                "음의 방향 완료"
            )

            print(
                f"복귀 위치: "
                f"{returned_position}"
            )

            print(
                f"피드백 변화: "
                f"{reverse_feedback_delta} count"
            )

            print(
                f"환산 회전각: "
                f"{reverse_angle:.3f} degree"
            )

            print(
                "최초 시작 위치 대비 "
                f"누적 복귀 오차: "
                f"{cumulative_return_error} count"
            )

            print(
                f"\n>>> 왕복 "
                f"{cycle}/{TEST_REPEAT_COUNT} 완료"
            )

            time.sleep(
                DIRECTION_CHANGE_DELAY_S
            )

        # ====================================================
        # 전체 시험 결과
        # ====================================================

        final_position = (
            driver.read_position()
        )

        total_return_error = (
            final_position
            - original_start_position
        )

        total_motor_revolutions = (
            TEST_MOVE_DEGREES
            / 360.0
            * 2
            * TEST_REPEAT_COUNT
        )

        print(
            "\n"
            "========================================"
        )
        print(" 전체 왕복 시험 완료")
        print(
            "========================================"
        )

        print(
            f"왕복 횟수: "
            f"{TEST_REPEAT_COUNT}"
        )

        print(
            f"한 방향 회전각: "
            f"{TEST_MOVE_DEGREES:.1f} degree"
        )

        print(
            "총 누적 모터 회전수: "
            f"{total_motor_revolutions:.1f} rev"
        )

        print(
            f"최초 시작 위치: "
            f"{original_start_position}"
        )

        print(
            f"최종 위치: "
            f"{final_position}"
        )

        print(
            "최종 복귀 오차: "
            f"{total_return_error} "
            "feedback count"
        )

    except KeyboardInterrupt:
        print(
            "\nCtrl+C 감지: "
            "모터를 정지합니다."
        )

    except Exception as error:
        print(
            f"\n시험 오류: {error}"
        )

    finally:
        try:
            if (
                cfg.SERVO_OFF_ON_PROGRAM_EXIT
            ):
                driver.disable()

                print(
                    "모터 정지 및 "
                    "Servo OFF 완료"
                )

            else:
                driver.stop()

                print(
                    "모터 정지 명령 완료 "
                    "(Servo ON 유지)"
                )

        except Exception as stop_error:
            print(
                "정지 또는 Servo OFF 실패: "
                f"{stop_error}"
            )


if __name__ == "__main__":
    main()