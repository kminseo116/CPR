#!/usr/bin/env python3
"""검증된 증분 위치 명령을 사용하는 새 RS485 모터 CPR 시험 코드.

중요
----
1. 모든 운동은 0x0044 증분 명령을 사용한다.
2. MSTEP 1000 측정 명령 단위(2005/rev)와 피드백 단위(16384/rev)를 분리한다.
3. 접촉 탐색은 저속, CPR은 깊이와 목표 BPM으로 계산한 속도를 사용한다.
4. 고속 CPR은 반드시 무인 시험 장비에서 1회부터 검증한다.
5. 접촉 탐색과 CPR은 반드시 기구부 안전을 확인한 뒤 실행한다.
"""

from __future__ import annotations

import select
import sys
import time
from pathlib import Path

import motor_config as cfg
from motor_driver import MotorDriver, MotorError, MotorStatus
from motor_ros_node import MotorRosBridge


MOVE_CONFIRM_COUNT = 3
MIN_FEEDBACK_MOVEMENT = 16

# 고속 CPR 명령 후 이 시간 안에 최소 이동량이 확인되지 않으면
# 명령이 실제 운전으로 이어지지 않은 것으로 판단한다.
CPR_COMMAND_START_TIMEOUT_S = 0.5
CPR_COMMAND_START_MIN_COUNTS = 1000

MOVE_STATE_LABELS = {
    "CONTACT_SEARCH_CONTINUOUS": "접촉 탐색",
    "COMPRESSION_DOWN": "압박 하강",
    "COMPRESSION_UP": "압박 복귀",
    "RETURN_TO_INITIAL": "초기 위치 복귀",
}


class UserStopRequested(RuntimeError):
    """사용자 또는 ROS 정지 토픽에서 정상 정지를 요청한 경우."""


class OvercurrentFault(RuntimeError):
    """드라이버 과전류 알람이 발생한 경우."""


class SoftwareCurrentTrip(RuntimeError):
    """소프트웨어 전류 상한이 연속 초과된 경우."""


class ManualContactDetected(RuntimeError):
    """접촉 탐색 이동 중 사용자가 Enter를 누른 경우."""


def terminal_enter_pressed() -> bool:
    """제어 루프를 멈추지 않고 터미널 Enter 입력을 확인한다."""
    readable, _, _ = select.select([sys.stdin], [], [], 0.0)
    if not readable:
        return False

    sys.stdin.readline()
    return True


def move_state_label(state_name: str) -> str:
    """내부 상태 이름을 운전자용 표시 이름으로 변환한다."""
    return MOVE_STATE_LABELS.get(state_name, state_name)


def mm_to_motor_revolutions(distance_mm: float) -> float:
    """직선 이동 거리를 모터 회전수로 변환한다."""
    return (
        float(distance_mm)
        / cfg.SCREW_LEAD_MM_PER_REV
        * cfg.GEAR_MOTOR_REV_PER_SCREW_REV
    )


def mm_to_command_units(distance_mm: float) -> int:
    """직선 거리 [mm]를 증분 위치 명령 단위로 변환한다."""
    return round(
        mm_to_motor_revolutions(distance_mm)
        * cfg.COMMAND_UNITS_PER_MOTOR_REV
    )


def mm_to_feedback_counts(distance_mm: float) -> int:
    """직선 거리 [mm]를 예상 위치 피드백 count로 변환한다."""
    return round(
        mm_to_motor_revolutions(distance_mm)
        * cfg.FEEDBACK_COUNTS_PER_MOTOR_REV
    )


def feedback_counts_to_cm(feedback_counts: int) -> float:
    """위치 피드백 count를 직선 이동 거리 [cm]로 변환한다."""
    motor_revolutions = (
        float(feedback_counts)
        / cfg.FEEDBACK_COUNTS_PER_MOTOR_REV
    )
    screw_revolutions = (
        motor_revolutions
        / cfg.GEAR_MOTOR_REV_PER_SCREW_REV
    )
    return (
        screw_revolutions
        * cfg.SCREW_LEAD_MM_PER_REV
        / 10.0
    )


def feedback_to_command_units(feedback_counts: int) -> int:
    """현재 위치와 기준 위치의 피드백 차이를 증분 명령으로 변환한다."""
    return round(
        int(feedback_counts)
        * cfg.COMMAND_UNITS_PER_MOTOR_REV
        / cfg.FEEDBACK_COUNTS_PER_MOTOR_REV
    )


def calculate_cpr_output_rpm(depth_cm: float) -> float:
    """지정 깊이를 목표 BPM으로 왕복하는 데 필요한 모터 rpm을 계산한다."""
    one_way_motor_revolutions = mm_to_motor_revolutions(depth_cm * 10.0)
    required_rpm = (
        2.0
        * one_way_motor_revolutions
        * cfg.TARGET_BPM
        * cfg.CPR_SPEED_MARGIN
    )
    allowed_output_rpm = min(
        cfg.MAX_OUTPUT_SPEED_RPM,
        cfg.MAX_VERIFIED_OUTPUT_SPEED_RPM,
    )
    if required_rpm > allowed_output_rpm:
        raise ValueError(
            "요청 깊이와 BPM에 필요한 속도가 허용 범위를 초과합니다: "
            f"required={required_rpm:.1f} rpm, "
            f"allowed={allowed_output_rpm:.1f} rpm"
        )
    return required_rpm


def calculate_move_timeout_s(
    expected_feedback_delta: int,
    output_rpm: float,
) -> float:
    """이동거리와 실제 출력축 rpm으로 안전 제한시간을 계산한다."""
    output_revolutions = (
        abs(expected_feedback_delta)
        / cfg.FEEDBACK_COUNTS_PER_MOTOR_REV
    )
    expected_time_s = output_revolutions / output_rpm * 60.0
    return max(
        cfg.MOVE_TIMEOUT_MIN_S,
        expected_time_s * cfg.MOVE_TIMEOUT_FACTOR
        + cfg.MOVE_TIMEOUT_MARGIN_S,
    )


def read_depth_cm() -> float:
    """사용자로부터 압박 깊이를 입력받는다."""
    prompt = (
        f"압박 깊이 입력 [{cfg.MIN_DEPTH_CM:.1f}~"
        f"{cfg.MAX_DEPTH_CM:.1f} cm, "
        f"Enter={cfg.DEFAULT_DEPTH_CM:.1f} cm]: "
    )
    raw_value = input(prompt).strip()
    if not raw_value:
        return cfg.DEFAULT_DEPTH_CM

    depth_cm = float(raw_value)
    if not cfg.MIN_DEPTH_CM <= depth_cm <= cfg.MAX_DEPTH_CM:
        raise ValueError(
            f"압박 깊이는 {cfg.MIN_DEPTH_CM:.1f}~"
            f"{cfg.MAX_DEPTH_CM:.1f} cm 범위여야 합니다."
        )
    return depth_cm


def print_runtime_settings(initial_pos: int) -> None:
    compression_direction = (
        cfg.POSITIVE_DIRECTION_NAME
        if cfg.MOTOR_DIRECTION_SIGN > 0
        else cfg.NEGATIVE_DIRECTION_NAME
    )
    print("\n=== 실행 설정 ===")
    print(f"[POSITION] INITIAL_FEEDBACK={initial_pos}")
    print(
        "[UNIT] COMMAND/REV="
        f"{cfg.COMMAND_UNITS_PER_MOTOR_REV}, "
        "FEEDBACK/REV="
        f"{cfg.FEEDBACK_COUNTS_PER_MOTOR_REV}"
    )
    print(
        f"[SPEED] SEARCH_OUTPUT={cfg.SEARCH_OUTPUT_SPEED_RPM:.2f} rpm"
    )
    print(
        "[SPEED] COMPRESSION=AUTO_BY_DEPTH_AND_BPM "
        f"(margin={cfg.CPR_SPEED_MARGIN:.3f})"
    )
    print(
        f"[SPEED] MAX_OUTPUT={cfg.MAX_OUTPUT_SPEED_RPM:.2f} rpm"
    )
    print(
        "[SPEED] VERIFIED_LIMIT="
        f"{cfg.MAX_VERIFIED_OUTPUT_SPEED_RPM:.2f} rpm"
    )
    print(f"[DIRECTION] COMPRESSION={compression_direction}")
    print(f"[CPR] REPEAT={cfg.REPEAT_COUNT}")
    print(f"[CPR] TARGET_BPM={cfg.TARGET_BPM:.1f}")
    print(f"[CURRENT] TEMP_LIMIT={cfg.MAX_CURRENT_RAW} raw")


def check_alarm_value(alarm: int) -> None:
    """알람 비트값을 검사하고 해당 예외를 발생시킨다."""
    if alarm == 0:
        return

    alarm_names = MotorDriver.decode_alarm(alarm)
    alarm_text = ", ".join(alarm_names) or f"0x{alarm:04X}"
    if alarm & cfg.ALARM_OVERCURRENT:
        raise OvercurrentFault(f"과전류 알람 발생: {alarm_text}")
    raise MotorError(f"모터 알람 발생: {alarm_text}")


def check_alarm_status(status: MotorStatus) -> None:
    check_alarm_value(status.alarm)


def update_software_current_trip(current_raw: int, count: int) -> int:
    """전류 상한이 연속 초과되는지 검사한다."""
    count = count + 1 if current_raw >= cfg.MAX_CURRENT_RAW else 0
    if count >= cfg.CURRENT_TRIP_COUNT_LIMIT:
        raise SoftwareCurrentTrip(
            "전류 raw 상한 연속 초과: "
            f"current={current_raw}, limit={cfg.MAX_CURRENT_RAW}"
        )
    return count


def move_incremental_and_wait(
    driver: MotorDriver,
    ros: MotorRosBridge,
    command_units: int,
    expected_feedback_delta: int,
    output_rpm: float,
    state_name: str,
    stop_on_contact_enter: bool = False,
    timeout_s: float | None = None,
    allow_direction_change_grace: bool = False,
) -> MotorStatus:
    """증분 이동을 실행하고 실제 피드백 변화로 완료와 안전을 검사한다."""
    if command_units == 0 or expected_feedback_delta == 0:
        raise ValueError("이동 명령과 예상 피드백은 0일 수 없습니다.")
    speed_setting = driver.output_rpm_to_speed_setting(output_rpm)
    if timeout_s is None:
        timeout_s = calculate_move_timeout_s(
            expected_feedback_delta,
            output_rpm,
        )
    feedback_overshoot_limit = max(
        cfg.MIN_FEEDBACK_OVERSHOOT_COUNTS,
        round(
            abs(expected_feedback_delta)
            * cfg.MAX_FEEDBACK_OVERSHOOT_RATIO
        ),
    )

    start_position = driver.read_position()
    ros.publish_state(state_name)
    state_label = move_state_label(state_name)
    direction_name = (
        cfg.POSITIVE_DIRECTION_NAME
        if command_units > 0
        else cfg.NEGATIVE_DIRECTION_NAME
    )
    print(f"\n[이동 시작] {state_label}")
    print(
        f"  방향: {direction_name} | "
        f"출력축 목표: {output_rpm:.2f} rpm | "
        f"내부 설정값: {speed_setting}"
    )
    print(
        f"  시작 위치: {start_position} | "
        f"목표 변화량: {expected_feedback_delta} count"
    )
    print(
        f"  피드백 초과 허용: 목표 대비 +{feedback_overshoot_limit} count "
        f"({cfg.MAX_FEEDBACK_OVERSHOOT_RATIO * 100.0:.1f}% 비율 기준)"
    )
    driver.move_incremental(command_units, speed_setting)

    start_time = time.monotonic()
    movement_detected = False
    reached_count = 0
    current_trip_count = 0
    last_status_print_time = -1.0

    while True:
        ros.spin_once()
        if ros.should_stop():
            driver.stop()
            raise UserStopRequested(f"{state_name} 중 정지 요청")

        if stop_on_contact_enter and terminal_enter_pressed():
            driver.stop()
            print(f"[이동 정지] {state_label} | 원인: 접촉 Enter")
            raise ManualContactDetected

        status = driver.read_status()
        check_alarm_status(status)
        current_trip_count = update_software_current_trip(
            status.current_raw,
            current_trip_count,
        )
        ros.publish_motor_status(status.position, status.current_raw)

        feedback_delta = status.position - start_position
        elapsed = time.monotonic() - start_time

        if (
            last_status_print_time < 0
            or elapsed - last_status_print_time
            >= cfg.MOVE_STATUS_LOG_PERIOD_S
        ):
            progress_percent = min(
                100.0,
                abs(feedback_delta)
                / abs(expected_feedback_delta)
                * 100.0,
            )
            print(
                f"[진행] {state_label} | {elapsed:.1f}s | "
                f"{progress_percent:5.1f}% | 위치={status.position} | "
                f"속도설정={status.speed_rpm} | 전류={status.current_raw} | "
                f"오차={status.position_error}"
            )
            last_status_print_time = elapsed
        if (
            abs(feedback_delta) >= MIN_FEEDBACK_MOVEMENT
            or status.speed_rpm != 0
        ):
            movement_detected = True

        wrong_direction = (
            abs(feedback_delta) >= MIN_FEEDBACK_MOVEMENT
            and feedback_delta * expected_feedback_delta < 0
        )
        within_direction_change_grace = (
            allow_direction_change_grace
            and elapsed <= cfg.CPR_DIRECTION_CHANGE_GRACE_S
            and abs(feedback_delta)
            <= cfg.CPR_DIRECTION_CHANGE_GRACE_COUNTS
        )
        if wrong_direction and not within_direction_change_grace:
            driver.stop()
            raise MotorError(
                f"{state_name} 반대 방향 이동: "
                f"expected={expected_feedback_delta}, actual={feedback_delta}"
            )

        if (
            abs(feedback_delta)
            > abs(expected_feedback_delta) + feedback_overshoot_limit
        ):
            driver.stop()
            raise MotorError(
                f"{state_name} 피드백 초과: "
                f"expected={expected_feedback_delta}, actual={feedback_delta}, "
                f"allowed_overshoot={feedback_overshoot_limit}"
            )

        reached = (
            movement_detected
            and abs(status.position_error)
            <= cfg.POSITION_TOLERANCE_UNITS
            and status.speed_rpm == 0
        )
        reached_count = reached_count + 1 if reached else 0

        if reached_count >= MOVE_CONFIRM_COUNT:
            driver.stop()
            print(
                f"[이동 완료] {state_label} | "
                f"실제 변화량={feedback_delta} count | "
                f"최종 오차={status.position_error}"
            )
            return status

        if abs(status.position_error) > cfg.MAX_POSITION_ERROR_UNITS:
            driver.stop()
            raise MotorError(
                f"위치 편차 초과: error={status.position_error}, "
                f"limit={cfg.MAX_POSITION_ERROR_UNITS}"
            )

        if time.monotonic() - start_time >= timeout_s:
            driver.stop()
            raise MotorError(
                f"이동 시간 초과: state={state_name}, "
                f"expected={expected_feedback_delta}, actual={feedback_delta}"
            )

        time.sleep(cfg.STATUS_PERIOD_S)


def move_incremental_cpr_fast(
    driver: MotorDriver,
    ros: MotorRosBridge,
    command_units: int,
    expected_feedback_delta: int,
    output_rpm: float,
    state_name: str,
    start_position: int | None = None,
    completion_tolerance_counts: int | None = None,
) -> int:
    """위치 도달 즉시 반환하여 다음 CPR 방향 명령을 빠르게 보낸다.

    이 빠른 경로에서는 속도 0과 위치 오차를 기다리지 않는다. 알람,
    전류, 위치 오차 등의 상세 상태는 왕복 한 사이클이 끝난 뒤 읽는다.
    """
    if command_units == 0 or expected_feedback_delta == 0:
        raise ValueError("이동 명령과 예상 피드백은 0일 수 없습니다.")
    if completion_tolerance_counts is None:
        completion_tolerance_counts = cfg.POSITION_TOLERANCE_UNITS
    completion_tolerance_counts = max(
        cfg.POSITION_TOLERANCE_UNITS,
        int(completion_tolerance_counts),
    )

    timeout_s = calculate_move_timeout_s(
        expected_feedback_delta,
        output_rpm,
    )
    emergency_overshoot_limit = max(
        cfg.CPR_MIN_EMERGENCY_OVERSHOOT_COUNTS,
        round(
            abs(expected_feedback_delta)
            * cfg.CPR_MAX_EMERGENCY_OVERSHOOT_RATIO
        ),
    )
    if start_position is None:
        start_position = driver.read_position()
    state_label = move_state_label(state_name)
    direction_name = (
        cfg.POSITIVE_DIRECTION_NAME
        if command_units > 0
        else cfg.NEGATIVE_DIRECTION_NAME
    )
    ros.publish_state(state_name)
    print(
        f"\n[이동 시작] {state_label} | 방향={direction_name} | "
        f"속도={output_rpm:.1f} rpm"
    )

    command_time = time.monotonic()
    driver.move_incremental_prepared(command_units)
    movement_detected = False
    direction_change_wait_logged = False
    last_log_time = command_time

    while True:
        ros.spin_once()
        if ros.should_stop():
            driver.stop()
            raise UserStopRequested(f"{state_name} 중 정지 요청")

        position = driver.read_position()
        now = time.monotonic()
        elapsed_s = now - command_time
        feedback_delta = position - start_position
        abs_delta = abs(feedback_delta)
        moving_in_expected_direction = (
            feedback_delta * expected_feedback_delta > 0
        )

        if (
            abs_delta >= MIN_FEEDBACK_MOVEMENT
            and moving_in_expected_direction
        ):
            movement_detected = True

        if (
            elapsed_s >= CPR_COMMAND_START_TIMEOUT_S
            and abs_delta < CPR_COMMAND_START_MIN_COUNTS
        ):
            driver.stop()
            raise MotorError(
                f"{state_name} 이동 명령 미실행: "
                f"elapsed={elapsed_s:.3f}s, "
                f"expected={expected_feedback_delta}, "
                f"actual={feedback_delta}, "
                f"start_min={CPR_COMMAND_START_MIN_COUNTS}"
            )

        wrong_direction = feedback_delta * expected_feedback_delta < 0

        initial_direction_change_inertia = (
            wrong_direction
            and elapsed_s <= cfg.CPR_DIRECTION_CHANGE_GRACE_S
            and abs_delta <= cfg.CPR_DIRECTION_CHANGE_GRACE_COUNTS
        )

        if wrong_direction and not initial_direction_change_inertia:
            driver.stop()
            raise MotorError(
                f"{state_name} 반대 방향 이동: "
                f"expected={expected_feedback_delta}, actual={feedback_delta}"
            )
        if (
            initial_direction_change_inertia
            and not direction_change_wait_logged
        ):
            print(
                f"[방향 전환 대기] {state_label} | "
                f"기존 방향 관성={feedback_delta:+d} count"
            )
            direction_change_wait_logged = True

        if (
            abs_delta
            > abs(expected_feedback_delta) + emergency_overshoot_limit
        ):
            driver.stop()
            raise MotorError(
                f"{state_name} 비상 피드백 초과: "
                f"expected={expected_feedback_delta}, actual={feedback_delta}, "
                f"emergency_overshoot={emergency_overshoot_limit}"
            )

        reached = (
            movement_detected
            and abs_delta
            >= abs(expected_feedback_delta) - completion_tolerance_counts
        )

        if now - last_log_time >= cfg.MOVE_STATUS_LOG_PERIOD_S:
            progress_percent = min(
                100.0,
                abs_delta / abs(expected_feedback_delta) * 100.0,
            )
            print(
                f"[이동 대기] {state_label} | "
                f"{now - command_time:.3f}s | "
                f"{progress_percent:5.1f}% | 위치={position} | "
                f"위치차이={feedback_delta:+d} count"
            )
            last_log_time = now

        if reached:
            target_difference = feedback_delta - expected_feedback_delta
            average_rpm = (
                abs(feedback_delta)
                / cfg.FEEDBACK_COUNTS_PER_MOTOR_REV
                * 60.0
                / elapsed_s
                if elapsed_s > 0.0
                else 0.0
            )
            print(
                f"[이동 완료] {state_label} | 방향={direction_name} | "
                f"평균속도={average_rpm:.1f} rpm | 위치={position} | "
                f"위치차이={feedback_delta:+d} count | "
                f"목표차이={target_difference:+d} count"
            )
            return position

        if now - command_time >= timeout_s:
            driver.stop()
            raise MotorError(
                f"이동 시간 초과: state={state_name}, "
                f"expected={expected_feedback_delta}, "
                f"position={position}"
            )

        time.sleep(cfg.CPR_POSITION_POLL_PERIOD_S)


def search_contact(
    driver: MotorDriver,
    ros: MotorRosBridge,
) -> tuple[int, str]:
    """한 번의 연속 하강 중 사용자의 Enter 입력으로 접촉 위치를 확정한다."""
    print("\n=== 수동 접촉 탐색 시작 ===")
    print("로드셀 접촉 신호 대신 사용자의 Enter 입력을 사용합니다.")
    print("압박봉이 자동으로 내려갑니다. 접촉하면 즉시 Enter를 누르십시오.")
    print("긴급 정지: Ctrl+C 또는 /motor_stop=True")
    ros.publish_state("SEARCHING_CONTACT_MANUAL")

    direction = cfg.MOTOR_DIRECTION_SIGN
    search_command = mm_to_command_units(cfg.CONTACT_SEARCH_MAX_MM) * direction
    search_feedback = mm_to_feedback_counts(cfg.CONTACT_SEARCH_MAX_MM) * direction
    search_origin = driver.read_position()

    print(
        f"[SEARCH START] 최대 {cfg.CONTACT_SEARCH_MAX_MM:.1f} mm 연속 하강, "
        f"출력축 목표={cfg.SEARCH_OUTPUT_SPEED_RPM:.2f} rpm"
    )

    try:
        move_incremental_and_wait(
            driver,
            ros,
            search_command,
            search_feedback,
            cfg.SEARCH_OUTPUT_SPEED_RPM,
            "CONTACT_SEARCH_CONTINUOUS",
            stop_on_contact_enter=True,
            timeout_s=cfg.CONTACT_SEARCH_TIMEOUT_S,
        )
    except ManualContactDetected:
        contact_position = driver.read_position()
        contact_feedback = contact_position - search_origin
        contact_traveled_mm = abs(
            feedback_counts_to_cm(contact_feedback) * 10.0
        )
        ros.publish_contact_source("MANUAL_ENTER")
        print("----------------------------------------------------")
        print("[CONTACT CONFIRMED] source=MANUAL_ENTER")
        print(f"[CONTACT CONFIRMED] position={contact_position}")
        print(
            "[CONTACT CONFIRMED] "
            f"탐색 이동거리={contact_traveled_mm:.3f} mm"
        )
        print("----------------------------------------------------")
        return contact_position, "MANUAL_ENTER"

    driver.stop()
    raise MotorError(
        "Enter 입력 없이 접촉 탐색 최대 거리에 도달: "
        f"limit={cfg.CONTACT_SEARCH_MAX_MM:.1f} mm"
    )


def run_compression(
    driver: MotorDriver,
    ros: MotorRosBridge,
    start_pos: int,
    depth_cm: float,
) -> None:
    """접촉 위치에서 지정 깊이만큼 증분 왕복한다."""
    direction = cfg.MOTOR_DIRECTION_SIGN
    distance_mm = depth_cm * 10.0
    depth_command = mm_to_command_units(distance_mm) * direction
    depth_feedback = mm_to_feedback_counts(distance_mm) * direction
    compression_output_rpm = calculate_cpr_output_rpm(depth_cm)
    compression_speed_setting = driver.output_rpm_to_speed_setting(
        compression_output_rpm
    )

    print("\n=== CPR 압박 시작 ===")
    print(f"[CPR] START_FEEDBACK={start_pos}")
    print(f"[CPR] DEPTH={depth_cm:.2f} cm")
    print(f"[CPR] DOWN_COMMAND={depth_command}")
    print(f"[CPR] EXPECTED_FEEDBACK={depth_feedback}")
    print(
        "[CPR] OUTPUT_SPEED="
        f"{compression_output_rpm:.2f} rpm "
        f"(internal={compression_speed_setting})"
    )
    print(f"[CPR] TARGET_BPM={cfg.TARGET_BPM:.1f}")
    print(f"[CPR] REPEAT={cfg.REPEAT_COUNT}")

    ros.publish_state("Compressing")
    ros.reset_compression()
    driver.prepare_incremental_motion(compression_speed_setting)
    applied_speed_setting = driver.read_u16(cfg.REG_INTERNAL_SPEED)
    print(
        "[속도 레지스터 확인] "
        f"요청={compression_speed_setting} | "
        f"0x002D 읽기={applied_speed_setting}"
    )
    if applied_speed_setting != compression_speed_setting:
        raise MotorError(
            "내부 속도 설정 검증 실패: "
            f"requested={compression_speed_setting}, "
            f"readback={applied_speed_setting}"
        )
    cycle_start_position = start_pos
    cycle_current_trip_count = 0

    for count in range(1, cfg.REPEAT_COUNT + 1):
        down_position = move_incremental_cpr_fast(
            driver,
            ros,
            depth_command,
            depth_feedback,
            compression_output_rpm,
            "COMPRESSION_DOWN",
            start_position=cycle_start_position,
            completion_tolerance_counts=max(
                cfg.POSITION_TOLERANCE_UNITS,
                round(
                    abs(depth_feedback)
                    * cfg.CPR_DOWN_COMPLETION_TOLERANCE_RATIO
                ),
            ),
        )
        actual_depth_cm = feedback_counts_to_cm(
            abs(down_position - cycle_start_position)
        )
        ros.count_compression(actual_depth_cm)

        move_incremental_cpr_fast(
            driver,
            ros,
            -depth_command,
            -depth_feedback,
            compression_output_rpm,
            "COMPRESSION_UP",
            start_position=down_position,
            completion_tolerance_counts=max(
                cfg.POSITION_TOLERANCE_UNITS,
                round(
                    abs(depth_feedback)
                    * cfg.CPR_RETURN_COMPLETION_TOLERANCE_RATIO
                ),
            ),
        )

        # 빠른 완료 판정은 목표 위치를 통과하는 즉시 반환하므로 모터의
        # 내부 이동이 아직 끝나지 않았을 수 있다. 상세 상태 통신이 만들던
        # 우연한 정착시간에 의존하지 않고, 복귀 직후 명시적으로 기동을
        # 해제한 다음 최소 상태를 읽어 다음 사이클을 준비한다.
        driver.stop()

        # 알람+운전상태, 전류, 최신 위치만 세 번의 통신으로 읽는다.
        # 방향·속도·이동량은 명령과 위치 샘플로 계산하므로 다시 읽지 않는다.
        cycle_status = driver.read_cycle_status()
        check_alarm_value(cycle_status.alarm)
        cycle_current_trip_count = update_software_current_trip(
            cycle_status.current_raw,
            cycle_current_trip_count,
        )
        ros.publish_motor_status(
            cycle_status.position,
            cycle_status.current_raw,
        )
        return_position_difference = cycle_status.position - start_pos
        print(
            f"[CPR] {count}/{cfg.REPEAT_COUNT} | "
            f"방향={cfg.POSITIVE_DIRECTION_NAME}→{cfg.NEGATIVE_DIRECTION_NAME} | "
            f"속도={compression_output_rpm:.1f} rpm | "
            f"전류={cycle_status.current_raw} | "
            f"위치={cycle_status.position} | "
            f"위치차이={return_position_difference:+d} count | "
            f"깊이={actual_depth_cm:.3f} cm | "
            f"BPM={ros.compression_bpm:.1f}"
        )
        # read_cycle_status()가 위치를 마지막에 읽으므로 이 값이 다음
        # 사이클의 가장 최신 기준 위치다. 별도 위치 통신은 추가하지 않는다.
        cycle_start_position = cycle_status.position


def return_to_feedback_position(
    driver: MotorDriver,
    ros: MotorRosBridge,
    target_position: int,
) -> None:
    """현재 피드백과 목표 피드백의 차이를 증분 명령으로 복귀한다."""
    current_position = driver.read_position()
    feedback_delta = target_position - current_position
    if abs(feedback_delta) <= cfg.POSITION_TOLERANCE_UNITS:
        return

    command_units = feedback_to_command_units(feedback_delta)
    move_incremental_and_wait(
        driver,
        ros,
        command_units,
        feedback_delta,
        cfg.RETURN_OUTPUT_SPEED_RPM,
        "RETURN_TO_INITIAL",
        allow_direction_change_grace=True,
    )


def prepare_zero_start_position(
    driver: MotorDriver,
    ros: MotorRosBridge,
) -> int:
    """CPR 시작 전에 축을 설정된 피드백 위치 0으로 물리적으로 복귀한다."""
    target_position = cfg.START_FEEDBACK_POSITION

    # 위치 명령 환산 오차가 남으면 한 번 더 보정한다. 반복 횟수를 제한해
    # 잘못된 피드백이나 기구 걸림에서 무한 왕복하지 않도록 한다.
    for attempt in range(1, 3):
        current_position = driver.read_position()
        remaining_counts = target_position - current_position
        if abs(remaining_counts) <= cfg.POSITION_TOLERANCE_UNITS:
            print(
                f"[START POSITION] 준비 완료 | 위치={current_position} | "
                f"목표={target_position}"
            )
            return target_position

        remaining_cm = feedback_counts_to_cm(abs(remaining_counts))
        print(
            f"[START POSITION] 0 위치 복귀 {attempt}/2 | "
            f"현재={current_position} | 목표={target_position} | "
            f"이동거리={remaining_cm:.3f} cm"
        )
        ros.publish_state("RETURNING_TO_START_ZERO")
        return_to_feedback_position(
            driver,
            ros,
            target_position,
        )

    final_position = driver.read_position()
    if (
        abs(final_position - target_position)
        <= cfg.POSITION_TOLERANCE_UNITS
    ):
        print(
            f"[START POSITION] 준비 완료 | 위치={final_position} | "
            f"목표={target_position}"
        )
        return target_position
    raise MotorError(
        "시작 위치 0 복귀 실패: "
        f"target={target_position}, actual={final_position}, "
        f"error={final_position - target_position}"
    )


def normal_stop_and_return(
    driver: MotorDriver,
    ros: MotorRosBridge,
    initial_pos: int,
) -> None:
    print("\n=== 정상 정지 처리 ===")
    driver.stop()
    if cfg.RETURN_TO_INITIAL_ON_STOP:
        ros.publish_state("RETURNING_TO_INITIAL")
        return_to_feedback_position(driver, ros, initial_pos)

    ros.clear_start_stop_flags()
    ros.publish_state("STOPPED_SERVO_ON")
    print("[STOP] INITIAL_POS 복귀 완료, Servo ON 유지")


def fault_stop(driver: MotorDriver, ros: MotorRosBridge, reason: str) -> None:
    try:
        driver.stop()
    except Exception as exc:
        print(f"[WARN] 고장 정지 명령 실패: {exc}")
    ros.publish_state(f"FAULT_LATCHED_{reason}")
    print(f"[FAULT] {reason}: 자동 초기 위치 복귀 안 함")


def run_once(driver: MotorDriver, ros: MotorRosBridge, initial_pos: int) -> None:
    # 직전 실행이 고장 정지되어 자동 복귀하지 않았더라도 새 압박을 시작하기
    # 전에 반드시 물리적 0 위치로 돌아간다.
    initial_pos = prepare_zero_start_position(driver, ros)
    print("\n============================================================")
    print("새 모터 CPR 증분 위치 시퀀스 시작")
    print("============================================================")
    print_runtime_settings(initial_pos)
    depth_cm = read_depth_cm()

    try:
        start_pos, source = search_contact(driver, ros)
        print(f"[CONTACT RESULT] source={source}, position={start_pos}")
        run_compression(driver, ros, start_pos, depth_cm)
        normal_stop_and_return(driver, ros, initial_pos)
    except UserStopRequested as exc:
        print(f"[USER STOP] {exc}")
        normal_stop_and_return(driver, ros, initial_pos)
    except OvercurrentFault as exc:
        fault_stop(driver, ros, "OVERCURRENT")
        raise MotorError(str(exc)) from exc
    except SoftwareCurrentTrip as exc:
        fault_stop(driver, ros, "SOFTWARE_CURRENT_TRIP")
        raise MotorError(str(exc)) from exc
    except Exception:
        fault_stop(driver, ros, "MOTOR_ERROR")
        raise


def main() -> None:
    if not (
        cfg.MSTEP_POSITION_CALIBRATION_COMPLETE
        and cfg.MSTEP_SPEED_CALIBRATION_COMPLETE
    ):
        print("====================================================")
        print(" motor_main 실행 차단")
        print("====================================================")
        print("MSTEP 1000 위치·속도 재보정이 아직 완료되지 않았습니다.")
        print("이전 MSTEP 2000 속도 환산값으로 움직이는 것을 방지합니다.")
        return

    import rclpy
    from motor_ros_node import MotorRosBridge

    rclpy.init()
    ros = MotorRosBridge()
    driver = MotorDriver()

    try:
        ros.publish_state("SETUP")
        driver.verify_connection()
        driver.setup_runtime()

        servo_off_position = driver.read_position()
        print(f"[SETUP] Servo OFF 위치: {servo_off_position}")
        input(
            "안전을 확인한 후 Enter를 누르면 Servo ON 및 위치 0 복귀를 "
            "시작합니다. Ctrl+C=취소: "
        )

        driver.enable()
        time.sleep(0.2)
        servo_on_position = driver.read_position()
        print(f"[INITIAL] Servo ON 직후 위치: {servo_on_position}")
        initial_pos = prepare_zero_start_position(driver, ros)
        print(f"[INITIAL] 압박 시작 기준 위치: {initial_pos}")
        print_runtime_settings(initial_pos)

        print("[READY] /motor_start=True 를 기다립니다.")
        print('실행: ros2 topic pub /motor_start std_msgs/Bool "data: true" --once')
        print('정지: ros2 topic pub /motor_stop std_msgs/Bool "data: true" --once')

        while rclpy.ok():
            ros.spin_once(timeout_sec=0.1)
            if not ros.motor_start:
                continue

            ros.motor_start = False
            ros.motor_stop = False
            try:
                run_once(driver, ros, initial_pos)
            except Exception as exc:
                print(f"[ERROR] {exc}")
            print("\n[READY] 다음 /motor_start=True 명령 대기")

    except KeyboardInterrupt:
        print("\n[CTRL+C] 프로그램 종료")
    finally:
        try:
            driver.stop()
            if cfg.SERVO_OFF_ON_PROGRAM_EXIT:
                driver.disable()
                print("[EXIT] Servo OFF 완료")
        except Exception as exc:
            print(f"[WARN] 종료 처리 실패: {exc}")

        ros.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
