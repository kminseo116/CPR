#!/usr/bin/env python3
"""MSTEP 1000 적용 후 약 90도 왕복으로 위치 명령 단위를 정밀 측정한다.

이 파일은 측정값 2005 command unit/rev를 기준으로 공용 설정을 바꾸지 않고,
로컬 시험 명령 약 +500/-500만 사용한다. 양방향 피드백 평균으로 최종
command unit/rev를 계산하며 설정 저장 명령은 보내지 않는다.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import motor_config as cfg
from motor_driver import MotorDriver, MotorError


PROVISIONAL_COMMAND_UNITS_PER_REV = 2005
TEST_COMMAND_UNITS = round(PROVISIONAL_COMMAND_UNITS_PER_REV / 4)
TEST_SPEED_SETTING = 10

MOVE_TIMEOUT_S = 20.0
SAMPLE_PERIOD_S = 0.02
LOG_PERIOD_S = 0.50
MIN_MOVEMENT_COUNTS = 5
DONE_CONFIRM_COUNT = 3
MAX_SAFE_ANGLE_DEG = 140.0


@dataclass(frozen=True)
class RuntimeBackup:
    internal_speed: int
    enable_method: int


@dataclass(frozen=True)
class MoveResult:
    start_position: int
    final_position: int
    feedback_delta: int
    angle_deg: float
    active_elapsed_s: float
    average_output_rpm: float
    max_speed_display: int
    max_current_raw: int


def read_checked_status(driver: MotorDriver, current_trip_count: int):
    status = driver.read_status()

    if status.alarm:
        decoded = MotorDriver.decode_alarm(status.alarm)
        raise MotorError(
            "모터 알람 발생: "
            + (", ".join(decoded) or f"0x{status.alarm:04X}")
        )

    if status.current_raw >= cfg.MAX_CURRENT_RAW:
        current_trip_count += 1
    else:
        current_trip_count = 0

    if current_trip_count >= cfg.CURRENT_TRIP_COUNT_LIMIT:
        raise MotorError(
            "전류 상한 연속 초과: "
            f"current={status.current_raw}, limit={cfg.MAX_CURRENT_RAW}"
        )

    return status, current_trip_count


def move_and_measure(
    driver: MotorDriver,
    command_units: int,
    label: str,
) -> MoveResult:
    """한 방향으로 약 90도 이동하고 피드백과 실제 평균속도를 측정한다."""
    start_position = driver.read_position()
    expected_sign = 1 if command_units > 0 else -1
    max_safe_counts = round(
        cfg.FEEDBACK_COUNTS_PER_MOTOR_REV * MAX_SAFE_ANGLE_DEG / 360.0
    )

    print(f"\n================ {label} ================")
    print(f"명령: {command_units:+d} command unit")
    print(f"속도 설정: {TEST_SPEED_SETTING}")
    print(f"시작 위치: {start_position}")

    command_time = time.monotonic()
    driver.move_incremental(command_units, TEST_SPEED_SETTING)

    first_motion_time: float | None = None
    first_motion_position: int | None = None
    last_motion_time: float | None = None
    last_motion_position: int | None = None
    previous_position = start_position
    movement_detected = False
    current_trip_count = 0
    done_count = 0
    max_speed_display = 0
    max_current_raw = 0
    final_position = start_position
    last_log_time = command_time - LOG_PERIOD_S

    while True:
        now = time.monotonic()
        status, current_trip_count = read_checked_status(
            driver,
            current_trip_count,
        )
        final_position = status.position
        delta = final_position - start_position
        max_speed_display = max(max_speed_display, abs(status.speed_rpm))
        max_current_raw = max(max_current_raw, status.current_raw)

        if abs(delta) >= MIN_MOVEMENT_COUNTS or status.speed_rpm != 0:
            movement_detected = True
            if first_motion_time is None:
                first_motion_time = now
                first_motion_position = final_position

        if final_position != previous_position and first_motion_time is not None:
            last_motion_time = now
            last_motion_position = final_position
        previous_position = final_position

        if abs(delta) >= MIN_MOVEMENT_COUNTS and delta * expected_sign < 0:
            raise MotorError(
                "예상과 반대 방향 이동: "
                f"command={command_units}, feedback_delta={delta}"
            )

        if abs(delta) > max_safe_counts:
            raise MotorError(
                f"안전 상한 {MAX_SAFE_ANGLE_DEG:.0f}도를 초과했습니다: "
                f"feedback_delta={delta}"
            )

        if now - last_log_time >= LOG_PERIOD_S:
            angle = delta / cfg.FEEDBACK_COUNTS_PER_MOTOR_REV * 360.0
            print(
                f"[진행] {now - command_time:4.2f}s | "
                f"변화={delta:+d} count | 각도={angle:+.2f}도 | "
                f"표시속도={status.speed_rpm} | 전류={status.current_raw} | "
                f"오차={status.position_error}"
            )
            last_log_time = now

        finished = (
            movement_detected
            and status.speed_rpm == 0
            and abs(status.position_error) <= cfg.POSITION_TOLERANCE_UNITS
        )
        done_count = done_count + 1 if finished else 0
        if done_count >= DONE_CONFIRM_COUNT:
            break

        if now - command_time >= MOVE_TIMEOUT_S:
            raise MotorError(
                "90도 이동 시간 초과: "
                f"command={command_units}, feedback_delta={delta}, "
                f"error={status.position_error}"
            )

        time.sleep(SAMPLE_PERIOD_S)

    driver.stop()

    feedback_delta = final_position - start_position
    angle_deg = feedback_delta / cfg.FEEDBACK_COUNTS_PER_MOTOR_REV * 360.0
    active_elapsed_s = 0.0
    average_output_rpm = 0.0
    if (
        first_motion_time is not None
        and first_motion_position is not None
        and last_motion_time is not None
        and last_motion_position is not None
        and last_motion_time > first_motion_time
    ):
        active_elapsed_s = last_motion_time - first_motion_time
        active_counts = abs(last_motion_position - first_motion_position)
        average_output_rpm = (
            active_counts
            / cfg.FEEDBACK_COUNTS_PER_MOTOR_REV
            * 60.0
            / active_elapsed_s
        )

    print("\n[이동 완료]")
    print(f"최종 위치: {final_position}")
    print(f"피드백 변화: {feedback_delta:+d} count")
    print(f"실제 회전각: {angle_deg:+.3f}도")
    print(f"이동 중 평균 출력축 속도: {average_output_rpm:.2f} rpm")
    print(f"0x0039 최대 표시값: {max_speed_display}")
    print(f"최대 전류: {max_current_raw} raw")

    return MoveResult(
        start_position=start_position,
        final_position=final_position,
        feedback_delta=feedback_delta,
        angle_deg=angle_deg,
        active_elapsed_s=active_elapsed_s,
        average_output_rpm=average_output_rpm,
        max_speed_display=max_speed_display,
        max_current_raw=max_current_raw,
    )


def command_units_per_rev(feedback_delta: int) -> float:
    if feedback_delta == 0:
        raise MotorError("피드백 변화가 0이라 회전 단위를 계산할 수 없습니다.")
    return (
        abs(TEST_COMMAND_UNITS)
        * cfg.FEEDBACK_COUNTS_PER_MOTOR_REV
        / abs(feedback_delta)
    )


def print_final_calibration(
    forward: MoveResult,
    reverse: MoveResult,
) -> None:
    forward_units = command_units_per_rev(forward.feedback_delta)
    reverse_units = command_units_per_rev(reverse.feedback_delta)
    average_feedback = (
        abs(forward.feedback_delta) + abs(reverse.feedback_delta)
    ) / 2.0
    combined_units = (
        TEST_COMMAND_UNITS
        * cfg.FEEDBACK_COUNTS_PER_MOTOR_REV
        / average_feedback
    )
    direction_difference = abs(forward_units - reverse_units)
    direction_difference_percent = (
        direction_difference / combined_units * 100.0
    )

    print("\n================ 최종 재보정 결과 ================")
    print(f"시험 명령: ±{TEST_COMMAND_UNITS} command unit")
    print(f"양의 방향 피드백: {forward.feedback_delta:+d} count")
    print(f"음의 방향 피드백: {reverse.feedback_delta:+d} count")
    print(f"양의 방향 1회전 명령 단위: {forward_units:.1f}")
    print(f"음의 방향 1회전 명령 단위: {reverse_units:.1f}")
    print(f"양방향 평균 1회전 명령 단위: {combined_units:.1f}")
    print(f"양방향 차이: {direction_difference_percent:.2f}%")
    print("\n설정 후보:")
    print(f"COMMAND_UNITS_PER_MOTOR_REV = {round(combined_units)}")


def restore_runtime(driver: MotorDriver, backup: RuntimeBackup) -> None:
    driver.stop()
    driver.write_u16(cfg.REG_INTERNAL_ENABLE, 0)
    driver.write_u16(cfg.REG_INTERNAL_SPEED, backup.internal_speed)
    driver.write_u16(cfg.REG_ENABLE_METHOD, backup.enable_method)


def main() -> None:
    print("====================================================")
    print(" MSTEP 1000 적용 후 약 90도 정밀 왕복 재보정")
    print("====================================================")
    print(f"시험 명령: ±{TEST_COMMAND_UNITS} command unit")
    print(f"시험 속도 설정: {TEST_SPEED_SETTING}")
    print("공용 설정값과 저장 레지스터는 변경하지 않습니다.")

    driver = MotorDriver()
    backup: RuntimeBackup | None = None
    runtime_changed = False
    forward_result: MoveResult | None = None

    try:
        driver.verify_connection()
        control_mode = driver.read_u16(cfg.REG_CONTROL_MODE)
        encoder_mode = driver.read_u16(cfg.REG_ENCODER_DIV_MODE)
        internal_enable = driver.read_u16(cfg.REG_INTERNAL_ENABLE)
        alarm = driver.read_u16(cfg.REG_ALARM_STATUS)

        print("\n[실행 전 확인]")
        print(f"제어 모드 0x0000 = {control_mode}")
        print(f"분주 방식 0x0007 = {encoder_mode}")
        print(f"Servo 명령 0x0021 = {internal_enable}")
        print(f"알람 0x0032 = 0x{alarm:04X}")

        if control_mode != 1:
            raise MotorError("RS485 위치 모드(1)가 아닙니다.")
        if encoder_mode != 0:
            raise MotorError("외부 DIP 분주 방식(0)이 아닙니다.")
        if internal_enable != 0:
            raise MotorError("시험 시작 전 Servo OFF 상태가 아닙니다.")
        if alarm != 0:
            raise MotorError(f"시험 시작 전 알람이 있습니다: 0x{alarm:04X}")

        backup = RuntimeBackup(
            internal_speed=driver.read_u16(cfg.REG_INTERNAL_SPEED),
            enable_method=driver.read_u16(cfg.REG_ENABLE_METHOD),
        )

        print("\n축 주변 양방향 140도 이상의 공간과 무부하 상태를 확인하십시오.")
        answer = input(
            "약 +90도 정밀 시험을 시작하려면 CALIBRATE 입력: "
        ).strip().upper()
        if answer != "CALIBRATE":
            print("[취소] Servo ON 및 이동 명령을 보내지 않았습니다.")
            return

        runtime_changed = True
        driver.stop()
        driver.write_u16(cfg.REG_ENABLE_METHOD, 1)
        driver.write_u16(cfg.REG_INTERNAL_ENABLE, 0)
        driver.write_u16(cfg.REG_POSITION_COMMAND_MODE, 1)
        driver.enable()
        time.sleep(0.2)
        print(f"Servo ON 후 기준 위치: {driver.read_position()}")

        forward_result = move_and_measure(
            driver,
            TEST_COMMAND_UNITS,
            "양의 방향 약 90도",
        )

        answer = input(
            "\n반대 방향으로 복귀하려면 RETURN 입력 "
            "(그 외 입력=Servo OFF 종료): "
        ).strip().upper()
        if answer != "RETURN":
            print("[복귀 생략] 현재 위치에서 Servo OFF 합니다.")
            return

        reverse_result = move_and_measure(
            driver,
            -TEST_COMMAND_UNITS,
            "음의 방향 약 90도 복귀",
        )
        return_error = reverse_result.final_position - forward_result.start_position
        return_error_angle = (
            return_error / cfg.FEEDBACK_COUNTS_PER_MOTOR_REV * 360.0
        )
        print(
            f"\n시작 위치 대비 복귀 오차: {return_error:+d} count "
            f"({return_error_angle:+.3f}도)"
        )
        print_final_calibration(forward_result, reverse_result)

    except KeyboardInterrupt:
        print("\n[중단] Ctrl+C 입력을 감지했습니다.")
    except Exception as exc:
        print(f"\n[오류] {exc}")
    finally:
        if backup is not None and runtime_changed:
            try:
                restore_runtime(driver, backup)
                print("\n[종료 처리]")
                print(f"0x002D = {backup.internal_speed} 복구")
                print("모터 정지 및 Servo OFF 완료")
                print("0x0030 저장 명령은 보내지 않았습니다.")
            except Exception as exc:
                print(f"[중요 경고] 종료 처리 실패: {exc}")


if __name__ == "__main__":
    main()
