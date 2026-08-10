#!/usr/bin/env python3
"""MSTEP 1000 위치모드의 1000/1500/2000 rpm 장거리 무부하 시험.

일반 운전 코드의 검증 속도 제한은 변경하지 않는다. 이 전용 시험에서만
실측 환산비로 내부 속도 설정값을 계산하고, 각 단계에서 양의 방향 9회전 후
사용자 확인을 받아 음의 방향 9회전으로 복귀한다.
"""

from __future__ import annotations

import statistics
import time
from dataclasses import dataclass

import motor_config as cfg
from motor_driver import MotorDriver, MotorError


TEST_OUTPUT_RPMS = (1000.0, 1500.0, 2000.0)
TEST_MOTOR_REVOLUTIONS = 9.0
TEST_COMMAND_UNITS = round(
    cfg.COMMAND_UNITS_PER_MOTOR_REV * TEST_MOTOR_REVOLUTIONS
)
EXPECTED_FEEDBACK_COUNTS = round(
    cfg.FEEDBACK_COUNTS_PER_MOTOR_REV * TEST_MOTOR_REVOLUTIONS
)

SAMPLE_PERIOD_S = 0.02
LOG_PERIOD_S = 0.25
MIN_MOVEMENT_COUNTS = 16
DONE_CONFIRM_COUNT = 3
MAX_SAFE_REVOLUTIONS = 9.5

# 공용 CPR 설정과 독립된 고속 무부하 시험용 임시 안전선이다.
# 500/700 rpm 장거리 시험의 최대 단발 피크는 3192 raw였다.
TEST_MAX_CURRENT_RAW = 10_000
TEST_CURRENT_TRIP_COUNT_LIMIT = 3


@dataclass(frozen=True)
class RuntimeBackup:
    internal_speed: int
    enable_method: int
    accel: int
    decel: int


@dataclass(frozen=True)
class Measurement:
    target_output_rpm: float
    speed_setting: int
    direction: str
    feedback_delta: int
    angle_deg: float
    measured_output_rpm: float
    max_current_raw: int
    max_speed_display: int
    max_position_error: int
    command_to_motion_s: float
    central_80_time_s: float
    estimated_motion_time_s: float
    settle_time_s: float
    total_time_s: float


def diagnostic_speed_setting(output_rpm: float) -> int:
    """일반 운전 제한을 우회하되 장비·정격 상한은 지킨다."""
    output_rpm = float(output_rpm)
    if not 0 < output_rpm <= cfg.MAX_OUTPUT_SPEED_RPM:
        raise MotorError(
            f"시험 속도 범위 초과: {output_rpm}, "
            f"rated_limit={cfg.MAX_OUTPUT_SPEED_RPM}"
        )

    setting = round(output_rpm * cfg.SPEED_SETTING_PER_OUTPUT_RPM)
    if not 1 <= setting <= cfg.MAX_INTERNAL_SPEED_SETTING:
        raise MotorError(
            f"내부 속도 설정 범위 초과: {setting}, "
            f"limit={cfg.MAX_INTERNAL_SPEED_SETTING}"
        )
    return setting


def calculate_timeout_s(target_output_rpm: float) -> float:
    expected_s = TEST_MOTOR_REVOLUTIONS / target_output_rpm * 60.0
    return max(3.0, expected_s * 3.0 + 2.0)


def check_status(driver: MotorDriver, current_trip_count: int):
    status = driver.read_status()

    if status.alarm:
        decoded = MotorDriver.decode_alarm(status.alarm)
        raise MotorError(
            "모터 알람 발생: "
            + (", ".join(decoded) or f"0x{status.alarm:04X}")
        )

    if status.current_raw >= TEST_MAX_CURRENT_RAW:
        current_trip_count += 1
    else:
        current_trip_count = 0

    if current_trip_count >= TEST_CURRENT_TRIP_COUNT_LIMIT:
        raise MotorError(
            "전류 상한 연속 초과: "
            f"current={status.current_raw}, limit={TEST_MAX_CURRENT_RAW}"
        )

    return status, current_trip_count


def move_and_measure(
    driver: MotorDriver,
    target_output_rpm: float,
    command_units: int,
    expected_feedback: int,
    direction: str,
) -> Measurement:
    speed_setting = diagnostic_speed_setting(target_output_rpm)
    start_position = driver.read_position()
    expected_sign = 1 if expected_feedback > 0 else -1
    timeout_s = calculate_timeout_s(target_output_rpm)
    max_safe_counts = round(
        cfg.FEEDBACK_COUNTS_PER_MOTOR_REV * MAX_SAFE_REVOLUTIONS
    )
    overshoot_limit = max(
        cfg.MIN_FEEDBACK_OVERSHOOT_COUNTS,
        round(
            abs(expected_feedback)
            * cfg.MAX_FEEDBACK_OVERSHOOT_RATIO
        ),
    )

    print("\n----------------------------------------------------")
    print(f"[이동 시작] 방향={direction}")
    print(
        f"목표 출력축={target_output_rpm:.0f} rpm | "
        f"내부 설정={speed_setting}"
    )
    print(
        f"명령={command_units:+d} unit | "
        f"예상 피드백={expected_feedback:+d} count | "
        f"예상 기구 이동={TEST_MOTOR_REVOLUTIONS / cfg.GEAR_MOTOR_REV_PER_SCREW_REV * cfg.SCREW_LEAD_MM_PER_REV:.1f} mm"
    )

    command_time = time.monotonic()
    driver.move_incremental(command_units, speed_setting)

    first_motion_time: float | None = None
    movement_detected = False
    current_trip_count = 0
    done_count = 0
    max_current_raw = 0
    max_speed_display = 0
    max_position_error = 0
    final_position = start_position
    last_log_time = command_time - LOG_PERIOD_S
    previous_sample_time = command_time
    previous_abs_delta = 0
    crossing_10_time: float | None = None
    crossing_90_time: float | None = None
    motion_stop_time: float | None = None
    done_time: float | None = None

    while True:
        now = time.monotonic()
        status, current_trip_count = check_status(
            driver,
            current_trip_count,
        )
        final_position = status.position
        feedback_delta = final_position - start_position
        max_current_raw = max(max_current_raw, status.current_raw)
        max_speed_display = max(max_speed_display, abs(status.speed_rpm))
        max_position_error = max(
            max_position_error,
            abs(status.position_error),
        )

        abs_delta = abs(feedback_delta)
        if abs_delta > previous_abs_delta:
            threshold_10 = abs(expected_feedback) * 0.10
            if (
                crossing_10_time is None
                and previous_abs_delta < threshold_10 <= abs_delta
            ):
                ratio = (
                    (threshold_10 - previous_abs_delta)
                    / (abs_delta - previous_abs_delta)
                )
                crossing_10_time = previous_sample_time + ratio * (
                    now - previous_sample_time
                )

            threshold_90 = abs(expected_feedback) * 0.90
            if (
                crossing_90_time is None
                and previous_abs_delta < threshold_90 <= abs_delta
            ):
                ratio = (
                    (threshold_90 - previous_abs_delta)
                    / (abs_delta - previous_abs_delta)
                )
                crossing_90_time = previous_sample_time + ratio * (
                    now - previous_sample_time
                )

        if (
            abs(feedback_delta) >= MIN_MOVEMENT_COUNTS
            or status.speed_rpm != 0
        ):
            movement_detected = True
            if first_motion_time is None:
                first_motion_time = now

        if (
            movement_detected
            and motion_stop_time is None
            and status.speed_rpm == 0
            and abs_delta >= abs(expected_feedback) * 0.90
        ):
            motion_stop_time = now

        if (
            abs(feedback_delta) >= MIN_MOVEMENT_COUNTS
            and feedback_delta * expected_sign < 0
        ):
            raise MotorError(
                "예상과 반대 방향 이동: "
                f"expected={expected_feedback}, actual={feedback_delta}"
            )

        if abs(feedback_delta) > max_safe_counts:
            raise MotorError(
                f"안전 이동량 {MAX_SAFE_REVOLUTIONS:.1f}회전 초과: "
                f"feedback_delta={feedback_delta}"
            )

        if (
            abs(feedback_delta)
            > abs(expected_feedback) + overshoot_limit
        ):
            raise MotorError(
                "목표 피드백 초과: "
                f"expected={expected_feedback}, actual={feedback_delta}, "
                f"allowed={overshoot_limit}"
            )

        if now - last_log_time >= LOG_PERIOD_S:
            progress = min(
                100.0,
                abs(feedback_delta) / abs(expected_feedback) * 100.0,
            )
            print(
                f"[진행] {now - command_time:4.2f}s | {progress:5.1f}% | "
                f"위치={status.position} | 표시속도={status.speed_rpm} | "
                f"전류={status.current_raw} | 오차={status.position_error}"
            )
            last_log_time = now

        finished = (
            movement_detected
            and status.speed_rpm == 0
            and abs(status.position_error) <= cfg.POSITION_TOLERANCE_UNITS
        )
        done_count = done_count + 1 if finished else 0
        if done_count >= DONE_CONFIRM_COUNT:
            done_time = now
            break

        if now - command_time >= timeout_s:
            raise MotorError(
                "고속 위치 이동 시간 초과: "
                f"target={target_output_rpm}, actual={feedback_delta}"
            )

        previous_sample_time = now
        previous_abs_delta = abs_delta
        time.sleep(SAMPLE_PERIOD_S)

    driver.stop()
    feedback_delta = final_position - start_position
    angle_deg = feedback_delta / cfg.FEEDBACK_COUNTS_PER_MOTOR_REV * 360.0

    command_to_motion_s = (
        first_motion_time - command_time
        if first_motion_time is not None
        else 0.0
    )
    central_80_time_s = 0.0
    estimated_motion_time_s = 0.0
    measured_output_rpm = 0.0
    if (
        crossing_10_time is not None
        and crossing_90_time is not None
        and crossing_90_time > crossing_10_time
    ):
        central_80_time_s = crossing_90_time - crossing_10_time
        estimated_motion_time_s = central_80_time_s / 0.80
        measured_output_rpm = (
            TEST_MOTOR_REVOLUTIONS
            * 60.0
            / estimated_motion_time_s
        )

    if done_time is None:
        done_time = time.monotonic()
    settle_reference = motion_stop_time or crossing_90_time or done_time
    settle_time_s = max(0.0, done_time - settle_reference)
    total_time_s = done_time - command_time

    print("[이동 완료]")
    print(f"실제 피드백={feedback_delta:+d} count | 각도={angle_deg:+.2f}도")
    print(
        f"측정 출력축={measured_output_rpm:.1f} rpm | "
        f"최대전류={max_current_raw} | 0x0039 최대={max_speed_display}"
    )
    print(
        f"[시간] 명령→움직임 감지={command_to_motion_s:.3f}s | "
        f"중앙 10~90%={central_80_time_s:.3f}s | "
        f"전체 이동 추정={estimated_motion_time_s:.3f}s"
    )
    print(
        f"[시간] 정지 후 정착={settle_time_s:.3f}s | "
        f"명령→완료 총시간={total_time_s:.3f}s | "
        f"최대 위치오차={max_position_error} count"
    )
    if max_current_raw >= TEST_MAX_CURRENT_RAW:
        print(
            "[전류 주의] 단발성 전류 피크가 시험 상한 이상입니다. "
            "다음 단계 진행 전 결과를 확인하십시오."
        )

    return Measurement(
        target_output_rpm=target_output_rpm,
        speed_setting=speed_setting,
        direction=direction,
        feedback_delta=feedback_delta,
        angle_deg=angle_deg,
        measured_output_rpm=measured_output_rpm,
        max_current_raw=max_current_raw,
        max_speed_display=max_speed_display,
        max_position_error=max_position_error,
        command_to_motion_s=command_to_motion_s,
        central_80_time_s=central_80_time_s,
        estimated_motion_time_s=estimated_motion_time_s,
        settle_time_s=settle_time_s,
        total_time_s=total_time_s,
    )


def print_summary(results: list[Measurement], return_error: int) -> None:
    print("\n================ 고속 단계 시험 결과 ================")
    print(" target  setting  direction          measured   angle   current")
    for result in results:
        print(
            f" {result.target_output_rpm:6.0f}  "
            f"{result.speed_setting:7d}  "
            f"{result.direction:<17} "
            f"{result.measured_output_rpm:8.1f}  "
            f"{result.angle_deg:7.1f}  "
            f"{result.max_current_raw:7d}"
        )

    valid_ratios = [
        result.speed_setting / result.measured_output_rpm
        for result in results
        if result.measured_output_rpm > 0
    ]
    if valid_ratios:
        print(
            "고속 설정값/실제 rpm 중앙값: "
            f"{statistics.median(valid_ratios):.3f}:1"
        )

    print(
        f"\n[실제 왕복 시간과 BPM: {TEST_MOTOR_REVOLUTIONS / cfg.GEAR_MOTOR_REV_PER_SCREW_REV * cfg.SCREW_LEAD_MM_PER_REV:.1f} mm 스트로크]"
    )
    print(
        " target   motion_cycle_s   command_cycle_s   "
        "actual_bpm   vs_target"
    )
    for index in range(0, len(results) - 1, 2):
        forward = results[index]
        reverse = results[index + 1]
        if forward.target_output_rpm != reverse.target_output_rpm:
            continue
        motion_cycle_s = (
            forward.estimated_motion_time_s
            + reverse.estimated_motion_time_s
        )
        command_cycle_s = forward.total_time_s + reverse.total_time_s
        actual_bpm = 60.0 / command_cycle_s if command_cycle_s > 0 else 0.0
        bpm_gap = actual_bpm - cfg.TARGET_BPM
        print(
            f" {forward.target_output_rpm:6.0f}  "
            f"{motion_cycle_s:15.3f}  "
            f"{command_cycle_s:16.3f}  "
            f"{actual_bpm:10.1f}  "
            f"{bpm_gap:+9.1f}"
        )
    print(f"목표 BPM: {cfg.TARGET_BPM:.1f}")
    print(f"전체 왕복 후 시작 위치 오차: {return_error:+d} count")


def restore_runtime(driver: MotorDriver, backup: RuntimeBackup) -> None:
    driver.stop()
    driver.write_u16(cfg.REG_INTERNAL_ENABLE, 0)
    driver.write_u16(cfg.REG_INTERNAL_SPEED, backup.internal_speed)
    # 가감속은 motor_config.py의 요청값을 런타임에 계속 유지한다.
    # 0x0030 저장은 보내지 않으므로 전원 재인가 후에는 저장값으로 돌아간다.
    driver.write_u16(cfg.REG_ACCEL, cfg.ACCEL_FACTOR)
    driver.write_u16(cfg.REG_DECEL, cfg.DECEL_FACTOR)
    driver.write_u16(cfg.REG_ENABLE_METHOD, backup.enable_method)


def main() -> None:
    print("====================================================")
    print(" MSTEP 1000 위치모드 고속 단계 시험")
    print("====================================================")
    stage_text = " → ".join(f"{rpm:.0f}" for rpm in TEST_OUTPUT_RPMS)
    print(f"시험 단계: 실제 출력축 목표 {stage_text} rpm")
    print(
        f"각 방향 이동: {TEST_MOTOR_REVOLUTIONS:.1f} motor rev, "
        f"기구 환산 약 {TEST_MOTOR_REVOLUTIONS / cfg.GEAR_MOTOR_REV_PER_SCREW_REV * cfg.SCREW_LEAD_MM_PER_REV:.1f} mm"
    )
    print("일반 운전의 200 rpm 검증 제한과 저장 설정은 변경하지 않습니다.")
    print(
        f"시험 전류 안전선: {TEST_MAX_CURRENT_RAW} raw 이상 "
        f"{TEST_CURRENT_TRIP_COUNT_LIMIT}회 연속 시 중단"
    )
    print(
        "주의: 고속 단계는 이동 시간이 매우 짧아 통신 감시보다 먼저 "
        "이동이 끝날 수 있습니다. 반드시 무부하로 시험하십시오."
    )
    print(
        "시간 측정은 Modbus 상태 샘플 사이의 위치를 보간하며, "
        "명령→완료 시간에는 통신 및 위치 정착 대기가 포함됩니다."
    )
    print(
        f"요청 가감속 설정: 0x002E={cfg.ACCEL_FACTOR}, "
        f"0x002F={cfg.DECEL_FACTOR}"
    )

    driver = MotorDriver()
    backup: RuntimeBackup | None = None
    runtime_changed = False
    results: list[Measurement] = []
    overall_start_position: int | None = None

    try:
        if not 0 <= cfg.ACCEL_FACTOR <= 100:
            raise MotorError(
                f"가속 설정 범위 초과: {cfg.ACCEL_FACTOR} (허용 0~100)"
            )
        if not 0 <= cfg.DECEL_FACTOR <= 100:
            raise MotorError(
                f"감속 설정 범위 초과: {cfg.DECEL_FACTOR} (허용 0~100)"
            )

        driver.verify_connection()
        control_mode = driver.read_u16(cfg.REG_CONTROL_MODE)
        encoder_mode = driver.read_u16(cfg.REG_ENCODER_DIV_MODE)
        internal_enable = driver.read_u16(cfg.REG_INTERNAL_ENABLE)
        alarm = driver.read_u16(cfg.REG_ALARM_STATUS)

        print("\n[실행 전 확인]")
        print(f"제어 모드={control_mode}, 분주 방식={encoder_mode}")
        print(f"Servo 명령={internal_enable}, 알람=0x{alarm:04X}")
        print(f"명령 단위/rev={cfg.COMMAND_UNITS_PER_MOTOR_REV}")
        print(f"현재 속도 환산={cfg.SPEED_SETTING_PER_OUTPUT_RPM:.3f}:1")

        if not (
            cfg.MSTEP_POSITION_CALIBRATION_COMPLETE
            and cfg.MSTEP_SPEED_CALIBRATION_COMPLETE
        ):
            raise MotorError("MSTEP 1000 위치·속도 보정이 완료되지 않았습니다.")
        if control_mode != 1 or encoder_mode != 0:
            raise MotorError("RS485 위치 모드 또는 외부 DIP 분주 방식이 아닙니다.")
        if internal_enable != 0:
            raise MotorError("시험 시작 전 Servo OFF 상태가 아닙니다.")
        if alarm != 0:
            raise MotorError(f"시험 시작 전 알람: 0x{alarm:04X}")

        backup = RuntimeBackup(
            internal_speed=driver.read_u16(cfg.REG_INTERNAL_SPEED),
            enable_method=driver.read_u16(cfg.REG_ENABLE_METHOD),
            accel=driver.read_u16(cfg.REG_ACCEL),
            decel=driver.read_u16(cfg.REG_DECEL),
        )

        print(
            f"[시험 전 가감속] 0x002E={backup.accel}, "
            f"0x002F={backup.decel}"
        )

        print("\n압박 하중을 제거하고 양의 방향으로 최소 65 mm 공간을 확보하십시오.")
        answer = input("고속 시험 준비가 끝났으면 START 입력: ").strip().upper()
        if answer != "START":
            print("[취소] Servo ON 및 이동 명령을 보내지 않았습니다.")
            return

        runtime_changed = True
        driver.stop()
        driver.write_u16(cfg.REG_ENABLE_METHOD, 1)
        driver.write_u16(cfg.REG_INTERNAL_ENABLE, 0)
        driver.write_u16(cfg.REG_POSITION_COMMAND_MODE, 1)
        driver.write_u16(cfg.REG_ACCEL, cfg.ACCEL_FACTOR)
        driver.write_u16(cfg.REG_DECEL, cfg.DECEL_FACTOR)

        applied_accel = driver.read_u16(cfg.REG_ACCEL)
        applied_decel = driver.read_u16(cfg.REG_DECEL)
        print(
            f"[가감속 적용 확인] 0x002E={applied_accel}, "
            f"0x002F={applied_decel}"
        )
        if applied_accel != cfg.ACCEL_FACTOR:
            raise MotorError(
                "가속 설정 적용 실패: "
                f"requested={cfg.ACCEL_FACTOR}, actual={applied_accel}"
            )
        if applied_decel != cfg.DECEL_FACTOR:
            raise MotorError(
                "감속 설정 적용 실패: "
                f"requested={cfg.DECEL_FACTOR}, actual={applied_decel}"
            )

        driver.enable()
        time.sleep(0.2)
        overall_start_position = driver.read_position()
        print(f"기준 위치: {overall_start_position}")

        for target_rpm in TEST_OUTPUT_RPMS:
            answer = input(
                f"\n{target_rpm:.0f} rpm 단계를 시작하려면 "
                f"RUN {target_rpm:.0f} 입력 (Enter=종료): "
            ).strip().upper()
            if answer != f"RUN {target_rpm:.0f}":
                print(f"[단계 종료] {target_rpm:.0f} rpm부터 실행하지 않습니다.")
                break

            forward = move_and_measure(
                driver,
                target_rpm,
                TEST_COMMAND_UNITS,
                EXPECTED_FEEDBACK_COUNTS,
                cfg.POSITIVE_DIRECTION_NAME,
            )
            results.append(forward)

            answer = input(
                f"{target_rpm:.0f} rpm으로 원위치 복귀하려면 "
                f"RETURN {target_rpm:.0f} 입력: "
            ).strip().upper()
            if answer != f"RETURN {target_rpm:.0f}":
                print("[복귀 생략] 현재 위치에서 Servo OFF 후 종료합니다.")
                break

            reverse = move_and_measure(
                driver,
                target_rpm,
                -TEST_COMMAND_UNITS,
                -EXPECTED_FEEDBACK_COUNTS,
                cfg.NEGATIVE_DIRECTION_NAME,
            )
            results.append(reverse)

        if overall_start_position is not None:
            final_position = driver.read_position()
            return_error = final_position - overall_start_position
            if results:
                print_summary(results, return_error)

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
                print(
                    f"0x002E = {cfg.ACCEL_FACTOR}, "
                    f"0x002F = {cfg.DECEL_FACTOR} 설정값 유지"
                )
                print("모터 정지 및 Servo OFF 완료")
                print("0x0030 저장 명령은 보내지 않았습니다.")
            except Exception as exc:
                print(f"[중요 경고] 종료 처리 실패: {exc}")


if __name__ == "__main__":
    main()
