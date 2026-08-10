#!/usr/bin/env python3
"""위치 모드에서 0x0010과 0x002D의 속도 영향을 분리해 확인한다.

시험 A는 내부 속도(0x002D)를 고정하고 최대 속도(0x0010)만 바꾼다.
시험 B는 최대 속도(0x0010)를 고정하고 내부 속도(0x002D)만 바꾼다.
실제 출력축 속도는 0x003D 위치 피드백의 시간 변화로 계산한다.

주의: 설정 저장(0x0030)은 실행하지 않으며, 종료할 때 시험 전 값으로 복구한다.
"""

from __future__ import annotations

import statistics
import time
from dataclasses import dataclass

import motor_config as cfg
from motor_driver import MotorDriver, MotorError


# 한 번에 약 15도만 이동해 예상과 다른 고속 동작의 위험을 줄인다.
TEST_ANGLE_DEG = 15.0
TEST_COMMAND_UNITS = round(
    cfg.COMMAND_UNITS_PER_MOTOR_REV * TEST_ANGLE_DEG / 360.0
)
EXPECTED_FEEDBACK_COUNTS = round(
    cfg.FEEDBACK_COUNTS_PER_MOTOR_REV * TEST_ANGLE_DEG / 360.0
)

# 시험 A: 0x002D=200으로 고정하고, 그보다 작거나 같은 0x0010을 비교한다.
# 0x0010이 실제 속도 상한이라면 10, 25, 50, 100, 200 순서로
# 출력축 속도가 증가해야 한다. 모두 같은 속도면 이 위치 이동에는 영향이 없다.
TEST_A_INTERNAL_SPEED = 200
TEST_A_MAX_SPEEDS = (10, 25, 50, 100, 200)

# 시험 B: 0x0010=시험 전 값(일반적으로 40000), 0x002D만 변경
TEST_B_INTERNAL_SPEEDS = (10, 25, 50)

MOVE_TIMEOUT_S = 20.0
SAMPLE_PERIOD_S = 0.03
LOG_PERIOD_S = 0.5
MIN_MOVEMENT_COUNTS = 5
DONE_CONFIRM_COUNT = 3


@dataclass(frozen=True)
class RegisterBackup:
    max_speed: int
    internal_speed: int
    enable_method: int
    internal_enable: int
    position_mode: int | None


@dataclass(frozen=True)
class Measurement:
    group: str
    max_speed: int
    internal_speed: int
    direction: str
    feedback_delta: int
    elapsed_s: float
    output_rpm: float
    speed_monitor_max: int


def read_backup(driver: MotorDriver) -> RegisterBackup:
    """시험에서 바꿀 수 있는 런타임 값을 보관한다."""
    try:
        position_mode = driver.read_u16(cfg.REG_POSITION_COMMAND_MODE)
    except Exception:
        # 이 장비에서는 0x0041 읽기가 지원되지 않을 수 있다.
        position_mode = None

    return RegisterBackup(
        max_speed=driver.read_u16(cfg.REG_MAX_SPEED),
        internal_speed=driver.read_u16(cfg.REG_INTERNAL_SPEED),
        enable_method=driver.read_u16(cfg.REG_ENABLE_METHOD),
        internal_enable=driver.read_u16(cfg.REG_INTERNAL_ENABLE),
        position_mode=position_mode,
    )


def write_and_verify_u16(
    driver: MotorDriver,
    address: int,
    value: int,
    label: str,
) -> None:
    driver.write_u16(address, value)
    readback = driver.read_u16(address)
    if readback != value:
        raise MotorError(
            f"{label} 쓰기 검증 실패: 요청={value}, 읽기={readback}"
        )


def check_status(driver: MotorDriver, current_trip_count: int):
    status = driver.read_status()
    if status.alarm:
        decoded = MotorDriver.decode_alarm(status.alarm)
        raise MotorError(
            "모터 알람 발생: " + (", ".join(decoded) or f"0x{status.alarm:04X}")
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
    group: str,
    max_speed: int,
    internal_speed: int,
    command_units: int,
    expected_counts: int,
    direction: str,
) -> Measurement:
    """약 15도 이동 중 위치 변화율을 이용해 실제 평균 rpm을 구한다."""
    write_and_verify_u16(
        driver,
        cfg.REG_MAX_SPEED,
        max_speed,
        "0x0010 최대 속도",
    )
    write_and_verify_u16(
        driver,
        cfg.REG_INTERNAL_SPEED,
        internal_speed,
        "0x002D 내부 속도",
    )

    start_position = driver.read_position()
    print(
        f"\n[{group} 이동] 방향={direction} | "
        f"0x0010={max_speed} | 0x002D={internal_speed}"
    )
    print(
        f"  시작 위치={start_position} | "
        f"예상 이동={expected_counts} count ({TEST_ANGLE_DEG:.1f}도)"
    )

    command_time = time.monotonic()
    driver.move_incremental(command_units, internal_speed)

    first_motion_time: float | None = None
    first_motion_position: int | None = None
    measurement_end_time: float | None = None
    measurement_end_position: int | None = None
    final_position = start_position
    speed_monitor_max = 0
    current_trip_count = 0
    done_count = 0
    last_log_time = command_time - LOG_PERIOD_S

    while True:
        now = time.monotonic()
        status, current_trip_count = check_status(driver, current_trip_count)
        final_position = status.position
        delta = final_position - start_position
        speed_monitor_max = max(speed_monitor_max, abs(status.speed_rpm))

        if first_motion_time is None and abs(delta) >= MIN_MOVEMENT_COUNTS:
            first_motion_time = now
            first_motion_position = final_position

        # 목표의 90%에 도달한 시점을 속도 계산의 끝점으로 고정한다.
        if (
            measurement_end_time is None
            and abs(delta) >= abs(expected_counts) * 0.90
        ):
            measurement_end_time = now
            measurement_end_position = final_position

        if abs(delta) >= MIN_MOVEMENT_COUNTS and delta * expected_counts < 0:
            raise MotorError(
                "예상과 반대 방향으로 이동했습니다: "
                f"expected={expected_counts}, actual={delta}"
            )

        if abs(delta) > abs(expected_counts) * 2.0:
            raise MotorError(
                "예상 이동량의 2배를 초과했습니다: "
                f"expected={expected_counts}, actual={delta}"
            )

        if now - last_log_time >= LOG_PERIOD_S:
            progress = min(100.0, abs(delta) / abs(expected_counts) * 100.0)
            print(
                f"  [진행] {now - command_time:4.1f}s | {progress:5.1f}% | "
                f"위치={status.position} | 표시속도={status.speed_rpm} | "
                f"전류={status.current_raw}"
            )
            last_log_time = now

        finished = (
            first_motion_time is not None
            and status.speed_rpm == 0
            and abs(status.position_error) <= cfg.POSITION_TOLERANCE_UNITS
            and abs(delta) >= abs(expected_counts) * 0.80
        )
        done_count = done_count + 1 if finished else 0
        if done_count >= DONE_CONFIRM_COUNT:
            break

        if now - command_time >= MOVE_TIMEOUT_S:
            raise MotorError(
                "이동 시간 초과: "
                f"0x0010={max_speed}, 0x002D={internal_speed}, delta={delta}"
            )

        time.sleep(SAMPLE_PERIOD_S)

    driver.stop()

    if first_motion_time is None or first_motion_position is None:
        raise MotorError("위치 피드백에서 이동을 확인하지 못했습니다.")

    if measurement_end_time is None or measurement_end_position is None:
        measurement_end_time = time.monotonic()
        measurement_end_position = final_position

    elapsed_s = measurement_end_time - first_motion_time
    measured_counts = abs(measurement_end_position - first_motion_position)
    output_rpm = 0.0
    if elapsed_s > 0:
        output_rpm = (
            measured_counts
            / cfg.FEEDBACK_COUNTS_PER_MOTOR_REV
            * 60.0
            / elapsed_s
        )

    feedback_delta = final_position - start_position
    angle = feedback_delta / cfg.FEEDBACK_COUNTS_PER_MOTOR_REV * 360.0
    print(
        f"  [완료] 실제 평균={output_rpm:.2f} rpm | "
        f"이동각={angle:.2f}도 | 0x0039 최대표시={speed_monitor_max}"
    )
    return Measurement(
        group=group,
        max_speed=max_speed,
        internal_speed=internal_speed,
        direction=direction,
        feedback_delta=feedback_delta,
        elapsed_s=elapsed_s,
        output_rpm=output_rpm,
        speed_monitor_max=speed_monitor_max,
    )


def run_pair(
    driver: MotorDriver,
    group: str,
    max_speed: int,
    internal_speed: int,
) -> list[Measurement]:
    """양의 방향으로 시험한 뒤 같은 명령으로 원위치 복귀한다."""
    forward = move_and_measure(
        driver,
        group,
        max_speed,
        internal_speed,
        TEST_COMMAND_UNITS,
        EXPECTED_FEEDBACK_COUNTS,
        cfg.POSITIVE_DIRECTION_NAME,
    )
    backward = move_and_measure(
        driver,
        group,
        max_speed,
        internal_speed,
        -TEST_COMMAND_UNITS,
        -EXPECTED_FEEDBACK_COUNTS,
        cfg.NEGATIVE_DIRECTION_NAME,
    )
    return [forward, backward]


def confirm_stage(title: str, description: str) -> bool:
    print(f"\n================ {title} ================")
    print(description)
    answer = input("이 단계만 실행하려면 RUN 입력 (Enter=전체 종료): ").strip().upper()
    return answer == "RUN"


def print_summary(results: list[Measurement]) -> None:
    print("\n================ 비교 결과 ================")
    print(" group  0x0010  0x002D  direction          actual_rpm  speed_display")
    for item in results:
        print(
            f" {item.group:<5} {item.max_speed:7d} {item.internal_speed:7d}  "
            f"{item.direction:<17} {item.output_rpm:10.2f} "
            f"{item.speed_monitor_max:14d}"
        )

    print("\n[시험값별 양방향 실제 rpm 평균]")
    keys = sorted({(r.group, r.max_speed, r.internal_speed) for r in results})
    for group, max_speed, internal_speed in keys:
        rpms = [
            r.output_rpm
            for r in results
            if (r.group, r.max_speed, r.internal_speed)
            == (group, max_speed, internal_speed)
        ]
        print(
            f"{group}: 0x0010={max_speed}, 0x002D={internal_speed} "
            f"-> {statistics.mean(rpms):.2f} rpm"
        )

    print("\n[해석 기준]")
    print("A에서 rpm이 달라지면 0x0010이 위치모드 속도에 영향을 줍니다.")
    print("B에서 rpm이 달라지면 0x002D가 위치모드 속도에 영향을 줍니다.")
    print("둘 다 달라지면 0x002D=목표, 0x0010=상한일 가능성이 큽니다.")
    print("둘 다 거의 같으면 분주 설정 또는 다른 위치 속도 설정을 확인해야 합니다.")


def restore_registers(driver: MotorDriver, backup: RegisterBackup) -> None:
    """이동을 끄고 시험에서 바꾼 설정을 저장 없이 원복한다."""
    driver.stop()
    driver.write_u16(cfg.REG_INTERNAL_ENABLE, 0)
    write_and_verify_u16(
        driver, cfg.REG_MAX_SPEED, backup.max_speed, "0x0010 원복"
    )
    write_and_verify_u16(
        driver,
        cfg.REG_INTERNAL_SPEED,
        backup.internal_speed,
        "0x002D 원복",
    )
    if backup.position_mode is not None:
        driver.write_u16(cfg.REG_POSITION_COMMAND_MODE, backup.position_mode)
    driver.write_u16(cfg.REG_ENABLE_METHOD, backup.enable_method)


def main() -> None:
    print("====================================================")
    print(" 위치모드 0x0010 비교 시험 (사용 중지)")
    print("====================================================")
    print("이 시험은 0x0010을 속도 상한으로 잘못 해석해 만든 이전 진단입니다.")
    print("0x0010은 공장값 40000을 유지해야 하므로 이동 시험을 실행하지 않습니다.")
    print("새 MSTEP 속도 측정에는 speed_ratio_check.py를 사용하십시오.")
    return

    driver = MotorDriver()
    backup: RegisterBackup | None = None
    runtime_changed = False
    results: list[Measurement] = []

    try:
        driver.verify_connection()
        control_mode = driver.read_u16(cfg.REG_CONTROL_MODE)
        if control_mode != 1:
            raise MotorError(
                f"현재 제어 모드={control_mode}; RS485 위치 모드(1)가 필요합니다."
            )

        backup = read_backup(driver)
        encoder_mode = driver.read_u16(cfg.REG_ENCODER_DIV_MODE)
        encoder_value = driver.read_u16(cfg.REG_ENCODER_RESOLUTION)
        print("\n[시험 전 값]")
        print(f"0x0007 엔코더 분주 방식 = {encoder_mode}")
        print(f"0x0008 엔코더 분주 값   = {encoder_value}")
        print(f"0x0010 최대 속도        = {backup.max_speed}")
        print(f"0x002D 내부 속도        = {backup.internal_speed}")
        if encoder_mode == 0:
            print("[참고] 0x0007=0이므로 외부 DIP 분주 설정이 사용될 수 있습니다.")

        print("\n압박 하중을 제거하고 모터 주변 15도 왕복 공간을 확보하십시오.")
        if input("Servo ON 및 비교 시험 준비: START 입력: ").strip().upper() != "START":
            print("[취소] 설정과 Servo 상태를 변경하지 않았습니다.")
            return

        # 여기부터 런타임 레지스터가 바뀔 수 있으므로, 중간 오류가 나도
        # finally에서 시험 전 0x0010/0x002D 값을 복구한다.
        runtime_changed = True
        driver.stop()
        driver.write_u16(cfg.REG_ENABLE_METHOD, 1)
        driver.write_u16(cfg.REG_INTERNAL_ENABLE, 0)
        driver.write_u16(cfg.REG_POSITION_COMMAND_MODE, 1)
        driver.enable()

        if confirm_stage(
            "시험 A: 0x0010 영향",
            "0x002D=200으로 고정하고 "
            "0x0010=10, 25, 50, 100, 200을 비교합니다.",
        ):
            for max_speed in TEST_A_MAX_SPEEDS:
                results.extend(
                    run_pair(
                        driver,
                        "A",
                        max_speed,
                        TEST_A_INTERNAL_SPEED,
                    )
                )
        else:
            print("[종료] 시험 A 시작 전에 중단했습니다.")
            return

        # 시험 B는 높은 0x0010 값을 사용하므로 별도로 다시 확인받는다.
        if confirm_stage(
            "시험 B: 0x002D 영향",
            f"0x0010={backup.max_speed}으로 고정하고 "
            "0x002D=10, 25, 50을 비교합니다.",
        ):
            for internal_speed in TEST_B_INTERNAL_SPEEDS:
                results.extend(
                    run_pair(
                        driver,
                        "B",
                        backup.max_speed,
                        internal_speed,
                    )
                )
        else:
            print("[종료] 시험 A 결과까지만 출력합니다.")

        if results:
            print_summary(results)

    except KeyboardInterrupt:
        print("\n[중단] Ctrl+C 입력을 감지했습니다.")
    except Exception as exc:
        print(f"\n[오류] {exc}")
    finally:
        if backup is not None and runtime_changed:
            try:
                restore_registers(driver, backup)
                print("\n[원복 완료]")
                print(f"0x0010 = {backup.max_speed}")
                print(f"0x002D = {backup.internal_speed}")
                print("모터 정지 및 Servo OFF 완료")
                print("0x0030 저장 명령은 보내지 않았습니다.")
            except Exception as exc:
                print(f"[중요 경고] 자동 원복 실패: {exc}")
                print("전원을 끄고 레지스터 상태를 다시 확인하십시오.")


if __name__ == "__main__":
    main()
