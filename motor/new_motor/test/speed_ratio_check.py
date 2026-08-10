#!/usr/bin/env python3
"""현재 MSTEP 재보정 후 속도 설정값과 실제 출력축 속도를 확인하는 시험.

기본 실행은 관련 레지스터만 읽는다. 레지스터 확인 후 사용자가 MOVE를
입력한 경우에만 Servo ON하고, 10/25/50 설정을 단계별 확인 후 측정한다.
"""

from __future__ import annotations

import statistics
import time
from dataclasses import dataclass

import motor_config as cfg
from motor_driver import MotorDriver, MotorError, MotorStatus


TEST_SPEEDS = (10, 25, 50)
TEST_COMMAND_UNITS = round(cfg.COMMAND_UNITS_PER_MOTOR_REV / 4)
EXPECTED_FEEDBACK_COUNTS = round(cfg.FEEDBACK_COUNTS_PER_MOTOR_REV / 4)
MOVE_TIMEOUT_S = 15.0
STATUS_PRINT_PERIOD_S = 1.0
MOVE_CONFIRM_COUNT = 3
MIN_MOVEMENT_COUNTS = 16


REGISTER_ITEMS = (
    ("제어 모드", cfg.REG_CONTROL_MODE),
    ("엔코더 분주 방식", cfg.REG_ENCODER_DIV_MODE),
    ("엔코더 해상도", cfg.REG_ENCODER_RESOLUTION),
    ("하드웨어 최대 전류", cfg.REG_MAX_CURRENT),
    ("속도 적분 포화 계수", cfg.REG_MAX_SPEED),
    ("인에이블 방식", cfg.REG_ENABLE_METHOD),
    ("내부 Servo 인에이블", cfg.REG_INTERNAL_ENABLE),
    ("내부 속도 설정", cfg.REG_INTERNAL_SPEED),
    ("가속 설정", cfg.REG_ACCEL),
    ("감속 설정", cfg.REG_DECEL),
    ("알람 상태", cfg.REG_ALARM_STATUS),
)


@dataclass(frozen=True)
class SpeedMeasurement:
    setting: int
    direction: str
    feedback_delta: int
    angle_deg: float
    elapsed_s: float
    output_rpm: float
    setting_to_output_ratio: float
    max_current_raw: int


def read_register_report(driver: MotorDriver) -> dict[int, int]:
    """속도 비율 진단에 필요한 레지스터를 읽고 출력한다."""
    print("\n================ 레지스터 확인 ================")
    values: dict[int, int] = {}

    for label, address in REGISTER_ITEMS:
        try:
            value = driver.read_u16(address)
        except Exception as exc:
            print(f"[읽기 실패] {label:<22} 0x{address:04X}: {exc}")
            continue

        values[address] = value
        print(f"[확인] {label:<22} 0x{address:04X} = {value}")

    encoder_resolution = values.get(cfg.REG_ENCODER_RESOLUTION)
    if encoder_resolution:
        command_ratio = (
            cfg.COMMAND_UNITS_PER_MOTOR_REV
            / encoder_resolution
        )
        print("\n[위치 단위 비교]")
        print(
            "보정된 출력축 1회전 명령 단위: "
            f"{cfg.COMMAND_UNITS_PER_MOTOR_REV}"
        )
        print(f"엔코더 해상도 설정값: {encoder_resolution}")
        print(f"명령 단위/엔코더 해상도 비율: {command_ratio:.3f}:1")

    return values


def check_motion_safety(status: MotorStatus, current_trip_count: int) -> int:
    """이동 중 알람과 소프트웨어 전류 상한을 확인한다."""
    if status.alarm:
        alarm_names = MotorDriver.decode_alarm(status.alarm)
        alarm_text = ", ".join(alarm_names) or f"0x{status.alarm:04X}"
        raise MotorError(f"모터 알람 발생: {alarm_text}")

    if status.current_raw >= cfg.MAX_CURRENT_RAW:
        current_trip_count += 1
    else:
        current_trip_count = 0

    if current_trip_count >= cfg.CURRENT_TRIP_COUNT_LIMIT:
        raise MotorError(
            "전류 상한 연속 초과: "
            f"current={status.current_raw}, limit={cfg.MAX_CURRENT_RAW}"
        )

    return current_trip_count


def calculate_active_output_rpm(
    active_samples: list[tuple[float, int]],
    feedback_delta: int,
    elapsed_s: float,
) -> float:
    """회전 중 위치 기울기로 출력축 rpm을 계산한다."""
    if len(active_samples) >= 2:
        first_time, first_position = active_samples[0]
        last_time, last_position = active_samples[-1]
        active_elapsed = last_time - first_time
        if active_elapsed > 0:
            counts_per_second = abs(
                last_position - first_position
            ) / active_elapsed
            return (
                counts_per_second
                / cfg.FEEDBACK_COUNTS_PER_MOTOR_REV
                * 60.0
            )

    if elapsed_s <= 0:
        return 0.0
    return (
        abs(feedback_delta)
        / cfg.FEEDBACK_COUNTS_PER_MOTOR_REV
        * 60.0
        / elapsed_s
    )


def move_and_measure(
    driver: MotorDriver,
    command_units: int,
    expected_feedback: int,
    speed_setting: int,
    direction_name: str,
) -> SpeedMeasurement:
    """한 번 이동하고 실제 피드백 변화율을 측정한다."""
    start_position = driver.read_position()
    print(
        f"\n[측정 시작] 설정={speed_setting}, "
        f"방향={direction_name}, 시작 위치={start_position}"
    )

    command_start_time = time.monotonic()
    driver.move_incremental(command_units, speed_setting)

    active_samples: list[tuple[float, int]] = []
    movement_detected = False
    stopped_time: float | None = None
    reached_count = 0
    current_trip_count = 0
    max_current_raw = 0
    last_print_time = -1.0
    last_status: MotorStatus | None = None

    while True:
        now = time.monotonic()
        status = driver.read_status()
        last_status = status
        current_trip_count = check_motion_safety(
            status,
            current_trip_count,
        )
        max_current_raw = max(max_current_raw, status.current_raw)

        feedback_delta = status.position - start_position
        elapsed_s = now - command_start_time

        if status.speed_rpm != 0:
            active_samples.append((now, status.position))

        if (
            abs(feedback_delta) >= MIN_MOVEMENT_COUNTS
            or status.speed_rpm != 0
        ):
            movement_detected = True

        if (
            abs(feedback_delta) >= MIN_MOVEMENT_COUNTS
            and feedback_delta * expected_feedback < 0
        ):
            raise MotorError(
                "예상과 반대 방향 이동: "
                f"expected={expected_feedback}, actual={feedback_delta}"
            )

        if last_print_time < 0 or now - last_print_time >= STATUS_PRINT_PERIOD_S:
            progress = min(
                100.0,
                abs(feedback_delta) / abs(expected_feedback) * 100.0,
            )
            print(
                f"[측정 중] {elapsed_s:4.1f}s | {progress:5.1f}% | "
                f"위치={status.position} | 속도표시={status.speed_rpm} | "
                f"전류={status.current_raw}"
            )
            last_print_time = now

        reached = (
            movement_detected
            and status.speed_rpm == 0
            and abs(status.position_error) <= cfg.POSITION_TOLERANCE_UNITS
        )
        if reached and stopped_time is None:
            stopped_time = now
        reached_count = reached_count + 1 if reached else 0

        if reached_count >= MOVE_CONFIRM_COUNT:
            break

        if elapsed_s >= MOVE_TIMEOUT_S:
            raise MotorError(
                "속도 측정 이동 시간 초과: "
                f"setting={speed_setting}, actual={feedback_delta}"
            )

        time.sleep(cfg.STATUS_PERIOD_S)

    driver.stop()
    if last_status is None or stopped_time is None:
        raise MotorError("속도 측정 상태값이 없습니다.")

    feedback_delta = last_status.position - start_position
    elapsed_s = stopped_time - command_start_time
    output_rpm = calculate_active_output_rpm(
        active_samples,
        feedback_delta,
        elapsed_s,
    )
    ratio = speed_setting / output_rpm if output_rpm > 0 else 0.0
    angle_deg = (
        feedback_delta
        / cfg.FEEDBACK_COUNTS_PER_MOTOR_REV
        * 360.0
    )

    print(
        f"[측정 완료] 설정={speed_setting} | "
        f"실제 출력축={output_rpm:.2f} rpm | "
        f"비율={ratio:.2f}:1 | 각도={angle_deg:.2f}° | "
        f"최대전류={max_current_raw}"
    )

    return SpeedMeasurement(
        setting=speed_setting,
        direction=direction_name,
        feedback_delta=feedback_delta,
        angle_deg=angle_deg,
        elapsed_s=elapsed_s,
        output_rpm=output_rpm,
        setting_to_output_ratio=ratio,
        max_current_raw=max_current_raw,
    )


def print_measurement_summary(results: list[SpeedMeasurement]) -> None:
    """전체 속도 측정 결과를 표 형태로 출력한다."""
    print("\n================ 측정 결과 ================")
    print(" setting  direction          output_rpm   ratio   angle   current")
    for result in results:
        print(
            f" {result.setting:7d}  {result.direction:<17} "
            f"{result.output_rpm:10.2f}  "
            f"{result.setting_to_output_ratio:6.2f}  "
            f"{result.angle_deg:6.2f}  "
            f"{result.max_current_raw:7d}"
        )

    valid_ratios = [
        result.setting_to_output_ratio
        for result in results
        if result.setting_to_output_ratio > 0
    ]
    if valid_ratios:
        median_ratio = statistics.median(valid_ratios)
        print("\n[최종 속도 비율]")
        print(f"설정값/실제 출력축 rpm 중앙값: {median_ratio:.2f}:1")
        print(
            "예상 출력축 rpm = 속도 설정값 / "
            f"{median_ratio:.2f}"
        )


def run_motion_test(driver: MotorDriver) -> None:
    """10, 25, 50 설정을 확인받으며 90도 왕복 속도를 측정한다."""
    driver.setup_runtime()
    print("\n[안전 확인]")
    print("90도 반시계/시계 왕복을 10, 25, 50 순서로 실행합니다.")
    print("압박 하중을 제거하고 모터 주변 공간을 확인하십시오.")
    confirm = input("계속하려면 RUN을 입력하십시오: ").strip().upper()
    if confirm != "RUN":
        print("[취소] Servo ON 및 이동 명령을 보내지 않았습니다.")
        return

    driver.enable()
    time.sleep(0.2)
    initial_position = driver.read_position()
    print(f"[기준 위치] {initial_position}")

    results: list[SpeedMeasurement] = []
    for speed_setting in TEST_SPEEDS:
        stage_confirm = input(
            f"\n속도 설정 {speed_setting} 시험을 시작하려면 "
            f"RUN {speed_setting} 입력 (Enter=여기서 종료): "
        ).strip().upper()
        if stage_confirm != f"RUN {speed_setting}":
            print(f"[단계 종료] 속도 설정 {speed_setting}부터 실행하지 않습니다.")
            break

        results.append(
            move_and_measure(
                driver,
                TEST_COMMAND_UNITS,
                EXPECTED_FEEDBACK_COUNTS,
                speed_setting,
                cfg.POSITIVE_DIRECTION_NAME,
            )
        )
        results.append(
            move_and_measure(
                driver,
                -TEST_COMMAND_UNITS,
                -EXPECTED_FEEDBACK_COUNTS,
                speed_setting,
                cfg.NEGATIVE_DIRECTION_NAME,
            )
        )

    final_position = driver.read_position()
    if results:
        print_measurement_summary(results)
    else:
        print("\n[측정 없음] 90도 이동 시험을 실행하지 않았습니다.")
    print(f"\n시작 위치: {initial_position}")
    print(f"종료 위치: {final_position}")
    print(f"왕복 위치 차이: {final_position - initial_position} count")


def main() -> None:
    print("====================================================")
    print(" 속도 설정값 / 실제 출력축 속도 확인")
    print("====================================================")
    if not cfg.MSTEP_POSITION_CALIBRATION_COMPLETE:
        print("[실행 차단] MSTEP 1000 위치 재보정이 아직 완료되지 않았습니다.")
        print("mstep_recalibration.py와 mstep_90deg_calibration.py를 먼저 실행하십시오.")
        return
    print("1단계는 읽기 전용입니다.")
    print("2단계 이동 시험은 사용자가 MOVE를 입력해야 진행됩니다.")

    driver = MotorDriver()
    movement_requested = False

    try:
        driver.verify_connection()
        register_values = read_register_report(driver)

        control_mode = register_values.get(cfg.REG_CONTROL_MODE)
        if control_mode != 1:
            print(
                "\n[이동 시험 불가] "
                f"현재 제어 모드={control_mode}, 필요한 모드=1"
            )
            return

        choice = input(
            "\n90도 왕복 속도 측정으로 진행하려면 MOVE 입력 "
            "(Enter=읽기만 종료): "
        ).strip().upper()
        if choice != "MOVE":
            print("[완료] 레지스터만 확인했고 설정은 변경하지 않았습니다.")
            return

        movement_requested = True
        run_motion_test(driver)

    except KeyboardInterrupt:
        print("\n[중단] 사용자가 Ctrl+C를 눌렀습니다.")
    except Exception as exc:
        print(f"\n[오류] {exc}")
    finally:
        if movement_requested:
            try:
                driver.stop()
                driver.disable()
                print("[종료] 모터 정지 및 Servo OFF 완료")
            except Exception as exc:
                print(f"[경고] 종료 처리 실패: {exc}")


if __name__ == "__main__":
    main()
