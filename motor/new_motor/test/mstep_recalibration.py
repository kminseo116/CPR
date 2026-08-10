#!/usr/bin/env python3
"""DIP MSTEP 1000 변경 후 RS485 증분 위치 명령을 소량 재측정한다.

기존 COMMAND_UNITS_PER_MOTOR_REV 값은 사용하지 않는다. 고정된 +100 command
unit만 이동한 뒤 0x003D 피드백 변화량으로 새로운 1회전 명령 단위를 계산하고,
사용자 확인 후 -100 command unit으로 복귀한다.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import motor_config as cfg
from motor_driver import MotorDriver, MotorError


TEST_COMMAND_UNITS = 100
TEST_SPEED_SETTING = 10
MOVE_TIMEOUT_S = 10.0
SAMPLE_PERIOD_S = 0.02
LOG_PERIOD_S = 0.25
MIN_MOVEMENT_COUNTS = 3
DONE_CONFIRM_COUNT = 3


@dataclass(frozen=True)
class RuntimeBackup:
    internal_speed: int
    enable_method: int


@dataclass(frozen=True)
class MoveResult:
    start_position: int
    final_position: int
    feedback_delta: int
    elapsed_s: float
    max_speed_display: int
    max_current_raw: int


def check_status(driver: MotorDriver, current_trip_count: int):
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


def run_small_move(
    driver: MotorDriver,
    command_units: int,
    label: str,
) -> MoveResult:
    """고정된 소량 증분 명령을 실행하고 최종 피드백 변화량을 측정한다."""
    start_position = driver.read_position()
    expected_sign = 1 if command_units > 0 else -1

    print(f"\n================ {label} ================")
    print(f"명령: {command_units:+d} command unit")
    print(f"속도 설정: {TEST_SPEED_SETTING}")
    print(f"시작 위치: {start_position}")

    command_time = time.monotonic()
    driver.move_incremental(command_units, TEST_SPEED_SETTING)

    movement_detected = False
    done_count = 0
    current_trip_count = 0
    max_speed_display = 0
    max_current_raw = 0
    last_log_time = command_time - LOG_PERIOD_S
    final_position = start_position

    while True:
        now = time.monotonic()
        status, current_trip_count = check_status(
            driver,
            current_trip_count,
        )
        final_position = status.position
        delta = final_position - start_position
        max_speed_display = max(max_speed_display, abs(status.speed_rpm))
        max_current_raw = max(max_current_raw, status.current_raw)

        if abs(delta) >= MIN_MOVEMENT_COUNTS or status.speed_rpm != 0:
            movement_detected = True

        if abs(delta) >= MIN_MOVEMENT_COUNTS and delta * expected_sign < 0:
            raise MotorError(
                "예상과 반대 방향 이동: "
                f"command={command_units}, feedback_delta={delta}"
            )

        # +100 명령이 예상보다 훨씬 크게 적용되는 경우 즉시 중단한다.
        if abs(delta) > cfg.FEEDBACK_COUNTS_PER_MOTOR_REV // 4:
            raise MotorError(
                "안전 상한 90도를 초과했습니다: "
                f"feedback_delta={delta}"
            )

        if now - last_log_time >= LOG_PERIOD_S:
            angle = delta / cfg.FEEDBACK_COUNTS_PER_MOTOR_REV * 360.0
            print(
                f"[진행] {now - command_time:4.2f}s | "
                f"위치={status.position} | 변화={delta:+d} count | "
                f"각도={angle:+.2f}도 | 표시속도={status.speed_rpm} | "
                f"전류={status.current_raw}"
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
                "소량 이동 시간 초과: "
                f"command={command_units}, feedback_delta={delta}, "
                f"error={status.position_error}"
            )

        time.sleep(SAMPLE_PERIOD_S)

    driver.stop()
    elapsed_s = time.monotonic() - command_time
    feedback_delta = final_position - start_position
    angle = feedback_delta / cfg.FEEDBACK_COUNTS_PER_MOTOR_REV * 360.0

    print("\n[이동 완료]")
    print(f"최종 위치: {final_position}")
    print(f"피드백 변화: {feedback_delta:+d} count")
    print(f"실제 회전각: {angle:+.3f}도")
    print(f"0x0039 최대 표시값: {max_speed_display}")
    print(f"최대 전류: {max_current_raw} raw")

    return MoveResult(
        start_position=start_position,
        final_position=final_position,
        feedback_delta=feedback_delta,
        elapsed_s=elapsed_s,
        max_speed_display=max_speed_display,
        max_current_raw=max_current_raw,
    )


def print_calibration(result: MoveResult) -> None:
    delta = abs(result.feedback_delta)
    print("\n================ 새 명령 단위 계산 ================")
    if delta == 0:
        print("피드백 변화가 0이어서 계산할 수 없습니다.")
        return

    command_units_per_rev = (
        abs(TEST_COMMAND_UNITS)
        * cfg.FEEDBACK_COUNTS_PER_MOTOR_REV
        / delta
    )
    ratio_from_previous = cfg.COMMAND_UNITS_PER_MOTOR_REV / command_units_per_rev

    print(f"시험 명령: {TEST_COMMAND_UNITS} command unit")
    print(f"측정 피드백: {delta} count")
    print(f"피드백 1회전: {cfg.FEEDBACK_COUNTS_PER_MOTOR_REV} count")
    print(f"새 1회전 명령 단위 추정: {command_units_per_rev:.1f}")
    print(f"기존 대비 이동 배율: {ratio_from_previous:.2f}배")

    if 650 <= delta <= 1000:
        print("[판정] MSTEP 1000 적용 예상 범위와 가깝습니다.")
    elif 300 <= delta <= 550:
        print("[판정] 이전 MSTEP 2000 동작 범위와 가깝습니다.")
    elif 8 <= delta <= 40:
        print("[판정] 기존 MSTEP 40000 동작 범위와 가깝습니다.")
    else:
        print("[판정 보류] 예상 범위와 달라 반복 측정이 필요합니다.")


def restore_runtime(driver: MotorDriver, backup: RuntimeBackup) -> None:
    driver.stop()
    driver.write_u16(cfg.REG_INTERNAL_ENABLE, 0)
    driver.write_u16(cfg.REG_INTERNAL_SPEED, backup.internal_speed)
    driver.write_u16(cfg.REG_ENABLE_METHOD, backup.enable_method)


def main() -> None:
    print("====================================================")
    print(" MSTEP 1000 변경 후 +100/-100 command unit 재보정")
    print("====================================================")
    print("기존 1회전 명령 단위를 사용하지 않는 독립 시험입니다.")
    print("설정 저장 명령은 보내지 않습니다.")

    driver = MotorDriver()
    backup: RuntimeBackup | None = None
    runtime_changed = False
    positive_result: MoveResult | None = None

    try:
        driver.verify_connection()
        control_mode = driver.read_u16(cfg.REG_CONTROL_MODE)
        encoder_mode = driver.read_u16(cfg.REG_ENCODER_DIV_MODE)
        alarm = driver.read_u16(cfg.REG_ALARM_STATUS)
        internal_enable = driver.read_u16(cfg.REG_INTERNAL_ENABLE)

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

        print("\nDIP SW1~SW4가 ON, ON, ON, OFF인지 다시 확인하십시오.")
        print("축 주변 약 25도 공간을 확보하고 하중을 제거하십시오.")
        answer = input("+100 시험을 시작하려면 TEST 입력: ").strip().upper()
        if answer != "TEST":
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

        positive_result = run_small_move(
            driver,
            TEST_COMMAND_UNITS,
            "양의 방향 재보정",
        )
        print_calibration(positive_result)

        answer = input(
            "\n-100으로 원위치 복귀하려면 RETURN 입력 "
            "(그 외 입력=Servo OFF 종료): "
        ).strip().upper()
        if answer == "RETURN":
            negative_result = run_small_move(
                driver,
                -TEST_COMMAND_UNITS,
                "음의 방향 복귀",
            )
            return_error = (
                negative_result.final_position
                - positive_result.start_position
            )
            print(f"\n시작 위치 대비 복귀 오차: {return_error:+d} count")
        else:
            print("[복귀 생략] 현재 위치에서 Servo OFF 합니다.")

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
