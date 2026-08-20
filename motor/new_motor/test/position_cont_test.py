#!/usr/bin/env python3
"""RS485 서보모터 지정 위치 이동 후 초기 위치 복귀 시험."""

import time

import motor_config as cfg
from motor_driver import MotorDriver


# ============================================================
# 시험 조건
# ============================================================

# ------------------------------------------------------------
# 사용자가 직접 설정하는 위치값
#
# 현재 MotorDriver의 move_incremental()을 사용하므로
# 아래 값은 "논리적 위치(command unit)"로 사용한다.
#
# 예:
# INITIAL_POSITION = 0
# TARGET_POSITION = 50000
#
# 실제 명령:
# 1) 0 -> 50000 : +50000 command unit
# 2) 50000 -> 0 : -50000 command unit
# ------------------------------------------------------------
INITIAL_POSITION = 0
TARGET_POSITION = 50000

# MSTEP 1000에서 실제 검증한 내부 속도 설정값까지만 사용한다.
TEST_SPEED_SETTING = 50

# 이동 제한 시간
MOVE_TIMEOUT_S = 10.0

# 최소 이동 감지량
MIN_FEEDBACK_MOVEMENT = 1000

# 예상 피드백보다 이 값 이상 더 움직이면 안전 정지
MAX_FEEDBACK_OVERSHOOT = 256

# 상태 확인 주기
STATUS_PERIOD_S = 0.05

# 목표 위치 도달을 연속 몇 번 확인할지
REACHED_CONFIRM_COUNT = 1

# 목표 도달 판정용 position_error 허용값
POSITION_ERROR_TOLERANCE = 50


# ============================================================
# 위치 차이 계산
# ============================================================

FORWARD_COMMAND_UNITS = TARGET_POSITION - INITIAL_POSITION
RETURN_COMMAND_UNITS = INITIAL_POSITION - TARGET_POSITION

# command unit -> 예상 feedback count 환산
FORWARD_EXPECTED_FEEDBACK_DELTA = round(
    cfg.FEEDBACK_COUNTS_PER_MOTOR_REV
    * FORWARD_COMMAND_UNITS
    / cfg.COMMAND_UNITS_PER_MOTOR_REV
)

RETURN_EXPECTED_FEEDBACK_DELTA = round(
    cfg.FEEDBACK_COUNTS_PER_MOTOR_REV
    * RETURN_COMMAND_UNITS
    / cfg.COMMAND_UNITS_PER_MOTOR_REV
)


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

        feedback_delta = status.position - start_position

        # 최대 이동량 기록
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

        # ====================================================
        # 1. 모터 알람 검사
        # ====================================================

        if status.alarm != 0:
            driver.stop()

            raise RuntimeError(
                f"모터 알람 발생: 0x{status.alarm:04X}"
            )

        # ====================================================
        # 2. 실제로 모터가 움직이기 시작했는지 확인
        # ====================================================

        if (
            abs(feedback_delta) >= MIN_FEEDBACK_MOVEMENT
            or status.speed_rpm != 0
        ):
            movement_detected = True

        # ====================================================
        # 3. 예상 이동 범위를 과도하게 초과하면 안전 정지
        # ====================================================

        if (
            abs(feedback_delta)
            > abs(expected_feedback_delta)
            + MAX_FEEDBACK_OVERSHOOT
        ):
            driver.stop()

            raise RuntimeError(
                "안전 범위를 초과해 즉시 정지: "
                f"expected={expected_feedback_delta}, "
                f"actual={feedback_delta}"
            )

        # ====================================================
        # 4. 예상과 반대 방향으로 이동하면 즉시 정지
        # ====================================================

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

            if reached_count >= REACHED_CONFIRM_COUNT:
                print(
                    "모터 내부 목표 위치 도달 "
                    f"(feedback_delta={feedback_delta}, "
                    f"expected={expected_feedback_delta}, "
                    f"position_error={status.position_error})"
                )

                return status.position

        else:
            reached_count = 0

        # ====================================================
        # 6. 이동 시간 초과
        # ====================================================

        if time.monotonic() - start_time >= MOVE_TIMEOUT_S:
            driver.stop()

            raise TimeoutError(
                "이동 시간 초과: "
                f"command_delta={command_units}, "
                f"feedback_delta={feedback_delta}, "
                f"expected_feedback_delta="
                f"{expected_feedback_delta}, "
                f"peak_feedback_delta={peak_feedback_delta}, "
                f"position={status.position}, "
                f"error={status.position_error}, "
                f"speed={status.speed_rpm}"
            )

        time.sleep(STATUS_PERIOD_S)


def main() -> None:
    """지정 위치 이동 후 초기 위치 복귀 시험을 실행한다."""

    # ========================================================
    # MSTEP 위치·속도 보정 확인
    # ========================================================

    if not (
        cfg.MSTEP_POSITION_CALIBRATION_COMPLETE
        and cfg.MSTEP_SPEED_CALIBRATION_COMPLETE
    ):
        print(
            "===================================================="
        )
        print(" position_target_return_test 실행 차단")
        print(
            "===================================================="
        )
        print(
            "MSTEP 1000 위치·속도 재보정이 "
            "아직 완료되지 않았습니다."
        )
        print(
            "mstep_recalibration.py와 "
            "mstep_90deg_calibration.py를 "
            "먼저 실행하십시오."
        )

        return

    if FORWARD_COMMAND_UNITS == 0:
        print(
            "INITIAL_POSITION과 TARGET_POSITION이 같습니다. "
            "이동할 거리가 없습니다."
        )
        return

    driver = MotorDriver()

    servo_off_before_position = None

    try:
        print(
            "=== 지정 위치 이동 후 초기 위치 복귀 시험 ==="
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
        # Servo ON 전 실제 피드백 위치
        # ====================================================

        servo_off_before_position = driver.read_position()

        print(
            f"Servo ON 전 실제 피드백 위치: "
            f"{servo_off_before_position}"
        )

        print("\n[설정 위치]")
        print(
            f"INITIAL_POSITION = "
            f"{INITIAL_POSITION} command unit"
        )
        print(
            f"TARGET_POSITION  = "
            f"{TARGET_POSITION} command unit"
        )

        print("\n[계산된 이동 명령]")
        print(
            f"정방향 이동량 = "
            f"{FORWARD_COMMAND_UNITS} command unit"
        )
        print(
            f"복귀 이동량   = "
            f"{RETURN_COMMAND_UNITS} command unit"
        )

        print(
            f"정방향 예상 feedback 변화 = "
            f"{FORWARD_EXPECTED_FEEDBACK_DELTA} count"
        )
        print(
            f"복귀 예상 feedback 변화   = "
            f"{RETURN_EXPECTED_FEEDBACK_DELTA} count"
        )

        print(
            f"시험 내부 속도 설정 = "
            f"{TEST_SPEED_SETTING}"
        )

        # ====================================================
        # 사용자 확인
        # ====================================================

        input(
            "\n주변을 확인한 후 Enter를 누르면 "
            "Servo ON 후 TARGET_POSITION까지 이동하고 "
            "다시 INITIAL_POSITION으로 복귀합니다. "
            "취소하려면 Ctrl+C: "
        )

        # ====================================================
        # Servo ON
        # ====================================================

        driver.enable()

        time.sleep(0.2)

        actual_start_position = driver.read_position()

        enable_shift = (
            actual_start_position
            - servo_off_before_position
        )

        print(
            f"\nServo ON 직후 실제 피드백 위치: "
            f"{actual_start_position}"
        )

        print(
            f"Servo ON 위치 변화: "
            f"{enable_shift} feedback count"
        )

        # ====================================================
        # 1단계: INITIAL_POSITION -> TARGET_POSITION
        # ====================================================

        print(
            "\n[1단계] "
            f"{INITIAL_POSITION} -> {TARGET_POSITION}"
        )

        driver.move_incremental(
            FORWARD_COMMAND_UNITS,
            TEST_SPEED_SETTING,
        )

        target_feedback_position = wait_until_reached(
            driver,
            actual_start_position,
            FORWARD_COMMAND_UNITS,
            FORWARD_EXPECTED_FEEDBACK_DELTA,
        )

        forward_feedback_delta = (
            target_feedback_position
            - actual_start_position
        )

        print("\n[목표 위치 이동 결과]")
        print(
            f"시작 실제 피드백 위치: "
            f"{actual_start_position}"
        )
        print(
            f"도착 실제 피드백 위치: "
            f"{target_feedback_position}"
        )
        print(
            f"실제 feedback 이동량: "
            f"{forward_feedback_delta}"
        )

        time.sleep(1.0)

        # ====================================================
        # 2단계: TARGET_POSITION -> INITIAL_POSITION
        # ====================================================

        print(
            "\n[2단계] "
            f"{TARGET_POSITION} -> {INITIAL_POSITION}"
        )

        driver.move_incremental(
            RETURN_COMMAND_UNITS,
            TEST_SPEED_SETTING,
        )

        returned_feedback_position = wait_until_reached(
            driver,
            target_feedback_position,
            RETURN_COMMAND_UNITS,
            RETURN_EXPECTED_FEEDBACK_DELTA,
        )

        # ====================================================
        # 복귀 결과
        # ====================================================

        return_error = (
            returned_feedback_position
            - actual_start_position
        )

        print("\n[초기 위치 복귀 결과]")
        print(
            f"복귀 후 실제 피드백 위치: "
            f"{returned_feedback_position}"
        )
        print(
            f"시험 시작 위치 대비 복귀 오차: "
            f"{return_error} feedback count"
        )

        print(
            "\n지정 위치 이동 및 초기 위치 복귀 시험 완료"
        )

    except KeyboardInterrupt:
        print(
            "\nCtrl+C 감지: 모터를 정지합니다."
        )

    except Exception as error:
        print(
            f"\n시험 오류: {error}"
        )

    finally:
        try:
            if cfg.SERVO_OFF_ON_PROGRAM_EXIT:
                driver.disable()

                print(
                    "모터 정지 및 Servo OFF 완료"
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