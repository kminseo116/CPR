#!/usr/bin/env python3
"""새 RS485 모터용 CPR 압박 메인 알고리즘.

프로그램 시작 순서
----------------
1. 통신 확인, 리셋, Servo ON
2. 현재 위치를 INITIAL_POS로 1회 저장
3. /motor_start 대기

CPR 실행 순서
-------------
1. 압박 깊이 입력
2. 무부하 전류 기준값 측정
3. 접촉 탐색
   - 1순위: 로드셀 힘
   - 2순위: 로드셀 지연/누락 + 전류 증가 지속
4. START_POS와 END_POS 사이 왕복
5. 일반 STOP 또는 반복 완료 시 INITIAL_POS 복귀
6. 일반 STOP 후 Servo ON 유지
7. 과전류·모터 오류 시 고장 정지
"""

from __future__ import annotations

import statistics
import time

import rclpy

import motor_config as cfg
from motor_driver import MotorDriver, MotorError, MotorStatus
from motor_ros_node import MotorRosBridge


# ============================================================================
# 0. 시퀀스 전용 예외
# ============================================================================

class UserStopRequested(RuntimeError):
    """사용자 또는 로드셀에서 정상 정지를 요청한 경우."""


class OvercurrentFault(RuntimeError):
    """드라이버 과전류 알람이 발생한 경우."""


class SoftwareCurrentTrip(RuntimeError):
    """소프트웨어 전류 상한이 연속 초과된 경우."""

# ============================================================================
# 1. 위치 단위 변환
# ============================================================================

def mm_to_position_units(distance_mm: float) -> int:
    """직선 이동 거리 [mm]를 모터 위치 단위로 변환한다."""
    motor_revolutions = (
        distance_mm
        / cfg.SCREW_LEAD_MM_PER_REV
        * cfg.GEAR_MOTOR_REV_PER_SCREW_REV
    )
    return round(motor_revolutions * cfg.POSITION_UNITS_PER_MOTOR_REV)


def position_units_to_cm(position_units: int) -> float:
    """모터 위치 단위를 직선 이동 거리 [cm]로 변환한다."""
    motor_revolutions = (
        position_units / cfg.POSITION_UNITS_PER_MOTOR_REV
    )
    screw_revolutions = (
        motor_revolutions / cfg.GEAR_MOTOR_REV_PER_SCREW_REV
    )
    distance_mm = screw_revolutions * cfg.SCREW_LEAD_MM_PER_REV
    return distance_mm / 10.0

# ============================================================================
# 2. 사용자 입력 및 설정 출력
# ============================================================================

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

    try:
        depth_cm = float(raw_value)
    except ValueError as exc:
        raise ValueError("압박 깊이는 숫자로 입력해야 합니다.") from exc

    if not cfg.MIN_DEPTH_CM <= depth_cm <= cfg.MAX_DEPTH_CM:
        raise ValueError(
            f"압박 깊이는 {cfg.MIN_DEPTH_CM:.1f}~"
            f"{cfg.MAX_DEPTH_CM:.1f} cm 범위여야 합니다."
        )

    return depth_cm


def print_runtime_settings(initial_pos: int) -> None:
    print("\n=== 실행 설정 ===")
    print(f"[POSITION] INITIAL_POS={initial_pos}")
    print(f"[SPEED] INITIAL={cfg.INITIAL_SPEED_RPM} rpm")
    print(f"[SPEED] SEARCH={cfg.SEARCH_SPEED_RPM} rpm")
    print(f"[SPEED] COMPRESSION={cfg.COMPRESSION_SPEED_RPM} rpm")
    print(f"[CONTACT] LOADCELL={cfg.CONTACT_FORCE_N:.2f} N")
    print(
        "[CONTACT] CURRENT_BACKUP="
        f"delta>={cfg.CONTACT_CURRENT_DELTA_RAW}, "
        f"confirm={cfg.CONTACT_CURRENT_CONFIRM_COUNT}"
    )
    print(f"[LIMIT] POSITION=[{cfg.MIN_POSITION_CMD}, {cfg.MAX_POSITION_CMD}]")
    print(f"[LIMIT] CURRENT_RAW={cfg.MAX_CURRENT_RAW}")

# ============================================================================
# 3. 알람 및 전류 안전 검사
# ============================================================================

def check_alarm_status(status: MotorStatus) -> None:
    """모터 알람 상태를 검사하고 위험 알람은 즉시 예외 처리한다."""
    if status.alarm == 0:
        return

    alarm_names = MotorDriver.decode_alarm(status.alarm)
    alarm_text = ", ".join(alarm_names) or f"0x{status.alarm:04X}"

    if status.alarm & cfg.ALARM_OVERCURRENT:
        raise OvercurrentFault(f"과전류 알람 발생: {alarm_text}")

    raise MotorError(f"모터 알람 발생: {alarm_text}")


def update_software_current_trip(
    current_raw: int,
    consecutive_count: int,
) -> int:
    """전류 상한 연속 초과 횟수를 갱신한다."""
    if current_raw >= cfg.MAX_CURRENT_RAW:
        consecutive_count += 1
    else:
        consecutive_count = 0

    if consecutive_count >= cfg.CURRENT_TRIP_COUNT_LIMIT:
        raise SoftwareCurrentTrip(
            f"전류 raw 상한 연속 초과: "
            f"current={current_raw}, limit={cfg.MAX_CURRENT_RAW}"
        )

    return consecutive_count

# ============================================================================
# 4. 단일 목표 위치 이동 후 도달 대기
# ============================================================================

def move_and_wait(
    driver: MotorDriver,
    ros: MotorRosBridge,
    target_position: int,
    speed_rpm: int,
    state_name: str,
) -> MotorStatus:
    """한 개의 목표 위치로 이동하고 도달할 때까지 기다린다.

    이 함수 자체는 왕복하지 않는다. CPR 왕복은 run_compression()에서
    END_POS 이동과 START_POS 복귀를 각각 호출하여 구성한다.
    """
    driver.check_position_limit(target_position)
    ros.publish_state(state_name)
    driver.move_absolute(target_position, speed_rpm)

    move_start_time = time.monotonic()
    current_trip_count = 0

    while True:
        ros.spin_once()

        if ros.should_stop():
            driver.stop()
            raise UserStopRequested(f"{state_name} 중 정지 요청")

        status = driver.read_status()
        check_alarm_status(status)
        current_trip_count = update_software_current_trip(
            status.current_raw,
            current_trip_count,
        )

        ros.publish_motor_status(
            status.position,
            status.current_raw,
        )

        if driver.target_reached(status, target_position):
            return status

        if abs(status.position_error) > cfg.MAX_POSITION_ERROR_UNITS:
            raise MotorError(
                f"위치 편차 초과: error={status.position_error}, "
                f"limit={cfg.MAX_POSITION_ERROR_UNITS}"
            )

        elapsed_s = time.monotonic() - move_start_time
        if elapsed_s >= cfg.MOVE_TIMEOUT_S:
            raise MotorError(
                f"이동 시간 초과: state={state_name}, "
                f"target={target_position}, current={status.position}"
            )

        time.sleep(cfg.STATUS_PERIOD_S)

# ============================================================================
# 5. 무부하 전류 기준값 측정
# ============================================================================

def measure_current_baseline(driver: MotorDriver) -> float:
    """접촉 전 정지 상태의 전류 raw 중앙값을 측정한다."""
    print("\n=== 무부하 전류 기준값 측정 ===")
    samples: list[int] = []

    for _ in range(cfg.CURRENT_BASELINE_SAMPLE_COUNT):
        status = driver.read_status()
        check_alarm_status(status)
        samples.append(status.current_raw)
        time.sleep(cfg.CURRENT_BASELINE_SAMPLE_PERIOD_S)

    baseline = float(statistics.median(samples))
    print(
        f"[CURRENT BASELINE] median={baseline:.1f}, "
        f"min={min(samples)}, max={max(samples)}"
    )
    return baseline

# ============================================================================
# 6. 접촉 탐색
# ============================================================================

def search_contact(
    driver: MotorDriver,
    ros: MotorRosBridge,
    initial_pos: int,
    baseline_current_raw: float,
) -> tuple[int, str]:
    """로드셀을 우선하고, 로드셀 이상 시 전류를 백업으로 사용한다."""
    print("\n=== 접촉 탐색 시작 ===")
    ros.publish_state("SEARCHING_CONTACT")

    step_units = (
        mm_to_position_units(cfg.CONTACT_SEARCH_STEP_MM)
        * cfg.MOTOR_DIRECTION_SIGN
    )
    max_units = abs(mm_to_position_units(cfg.CONTACT_SEARCH_MAX_MM))

    target_position = initial_pos
    search_start_time = time.monotonic()
    current_confirm_count = 0

    while abs(target_position - initial_pos) <= max_units:
        ros.spin_once()

        if ros.should_stop():
            driver.stop()
            raise UserStopRequested("접촉 탐색 중 정지 요청")

        status = driver.read_status()
        check_alarm_status(status)
        ros.publish_motor_status(status.position, status.current_raw)

        # 6-1. 주 접촉 조건: 로드셀 힘
        if ros.loadcell_total >= cfg.CONTACT_FORCE_N:
            driver.stop()
            contact_position = driver.read_position()
            contact_source = "LOADCELL"
            ros.publish_contact_source(contact_source)

            print(
                f"[CONTACT:LOADCELL] force={ros.loadcell_total:.2f} N, "
                f"position={contact_position}"
            )
            return contact_position, contact_source

        # 6-2. 백업 접촉 조건: 로드셀 stale + 전류 증가 지속
        loadcell_stale = ros.is_loadcell_stale(
            cfg.LOADCELL_STALE_TIMEOUT_S
        )
        blanking_done = (
            time.monotonic() - search_start_time
            >= cfg.CURRENT_CONTACT_BLANKING_S
        )
        current_delta = status.current_raw - baseline_current_raw

        current_contact_candidate = (
            loadcell_stale
            and blanking_done
            and current_delta >= cfg.CONTACT_CURRENT_DELTA_RAW
        )

        if current_contact_candidate:
            current_confirm_count += 1
        else:
            current_confirm_count = 0

        if current_confirm_count >= cfg.CONTACT_CURRENT_CONFIRM_COUNT:
            driver.stop()
            contact_position = driver.read_position()
            contact_source = "CURRENT_BACKUP"
            ros.publish_contact_source(contact_source)

            print(
                "[CONTACT:CURRENT_BACKUP] "
                f"baseline={baseline_current_raw:.1f}, "
                f"current={status.current_raw}, "
                f"delta={current_delta:.1f}, "
                f"position={contact_position}"
            )
            return contact_position, contact_source

        # 6-3. 접촉하지 않았으면 다음 위치로 한 단계 하강
        target_position += step_units
        driver.check_position_limit(target_position)

        print(
            f"[SEARCH] target={target_position}, "
            f"force={ros.loadcell_total:.2f} N, "
            f"stale={loadcell_stale}, "
            f"current={status.current_raw}, "
            f"delta={current_delta:.1f}, "
            f"confirm={current_confirm_count}/"
            f"{cfg.CONTACT_CURRENT_CONFIRM_COUNT}"
        )

        move_and_wait(
            driver=driver,
            ros=ros,
            target_position=target_position,
            speed_rpm=cfg.SEARCH_SPEED_RPM,
            state_name="CONTACT_SEARCH_STEP",
        )
        time.sleep(cfg.CONTACT_SETTLE_S)

    driver.stop()
    raise MotorError("접촉 탐색 최대 거리 초과")

# ============================================================================
# 7. CPR 왕복 압박
# ============================================================================

def run_compression(
    driver: MotorDriver,
    ros: MotorRosBridge,
    start_pos: int,
    depth_cm: float,
) -> None:
    """
    START_POS와 END_POS 사이를 반복 왕복한다.
    START_POS: 접촉이 검출된 압박 시작 위치
    END_POS: START_POS + 압박 깊이
    """
    depth_units = (
        mm_to_position_units(depth_cm * 10.0)
        * cfg.MOTOR_DIRECTION_SIGN
    )
    end_pos = start_pos + depth_units
    driver.check_position_limit(end_pos)

    print("\n=== CPR 압박 시작 ===")
    print(f"[CPR] START_POS={start_pos}")
    print(f"[CPR] END_POS={end_pos}")
    print(f"[CPR] DEPTH={depth_cm:.2f} cm")
    print(f"[CPR] SPEED={cfg.COMPRESSION_SPEED_RPM} rpm")
    print(f"[CPR] REPEAT={cfg.REPEAT_COUNT}")

    ros.publish_state("Compressing")
    ros.reset_compression()

    for count in range(1, cfg.REPEAT_COUNT + 1):
        # 7-1. START_POS -> END_POS: 아래로 압박
        move_and_wait(
            driver=driver,
            ros=ros,
            target_position=end_pos,
            speed_rpm=cfg.COMPRESSION_SPEED_RPM,
            state_name="COMPRESSION_DOWN",
        )

        actual_position = driver.read_position()
        actual_depth_cm = position_units_to_cm(
            abs(actual_position - start_pos)
        )
        ros.count_compression(actual_depth_cm)

        print(
            f"[CPR] count={count}, "
            f"depth={actual_depth_cm:.2f} cm, "
            f"bpm={ros.compression_bpm:.1f}"
        )

        # 7-2. END_POS -> START_POS: 위로 복귀 및 완전 이완
        move_and_wait(
            driver=driver,
            ros=ros,
            target_position=start_pos,
            speed_rpm=cfg.COMPRESSION_SPEED_RPM,
            state_name="COMPRESSION_UP",
        )

# ============================================================================
# 8. 정상 정지 및 초기 위치 복귀
# ============================================================================

def normal_stop_and_return(
    driver: MotorDriver,
    ros: MotorRosBridge,
    initial_pos: int,
) -> None:
    """정상 정지 후 INITIAL_POS로 복귀하고 Servo ON을 유지한다."""
    print("\n=== 정상 정지 처리 ===")
    driver.stop()

    if cfg.RETURN_TO_INITIAL_ON_STOP:
        ros.publish_state("RETURNING_TO_INITIAL")
        move_and_wait(
            driver=driver,
            ros=ros,
            target_position=initial_pos,
            speed_rpm=cfg.INITIAL_SPEED_RPM,
            state_name="RETURN_TO_INITIAL",
        )

    ros.clear_start_stop_flags()
    ros.publish_state("STOPPED_SERVO_ON")
    print("[STOP] INITIAL_POS 복귀 완료, Servo ON 유지")

# ============================================================================
# 9. 고장 정지
# ============================================================================

def fault_stop(
    driver: MotorDriver,
    ros: MotorRosBridge,
    reason: str,
) -> None:
    """고장 발생 시 추가 움직임 없이 정지하고 상태를 고정한다."""
    try:
        driver.stop()
    except Exception as exc:
        print(f"[WARN] 고장 정지 명령 실패: {exc}")

    ros.publish_state(f"FAULT_LATCHED_{reason}")
    print(f"[FAULT] {reason}: 자동 초기 위치 복귀 안 함")

# ============================================================================
# 10. 시작 명령 1회에 대한 전체 시퀀스
# ============================================================================

def run_once(driver: MotorDriver, ros: MotorRosBridge, initial_pos: int) -> None:
    """한 번의 /motor_start 명령에 대한 전체 CPR 시퀀스.

    initial_pos는 프로그램 시작 시 한 번 저장된 값을 전달받아 사용한다.
    /motor_start를 다시 받아도 initial_pos를 다시 읽지 않는다.
    """
    print("\n============================================================")
    print("새 모터 CPR 시퀀스 시작")
    print("============================================================")

    # 10-1. 프로그램 시작 시 저장된 INITIAL_POS 확인(main에서만 현재 위치를 읽어옴)
    print_runtime_settings(initial_pos)

    # 10-2. 압박 깊이 입력
    depth_cm = read_depth_cm()

    try:
        # 10-3. 무부하 전류 기준 측정
        baseline_current_raw = measure_current_baseline(driver)

        # 10-4. 접촉 탐색
        start_pos, contact_source = search_contact(
            driver=driver,
            ros=ros,
            initial_pos=initial_pos,
            baseline_current_raw=baseline_current_raw,
        )
        print(
            f"[CONTACT RESULT] source={contact_source}, "
            f"position={start_pos}"
        )

        # 10-5. 왕복 압박
        run_compression(
            driver=driver,
            ros=ros,
            start_pos=start_pos,
            depth_cm=depth_cm,
        )

        # 10-6. 반복 완료 후 정상 복귀
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


# ============================================================================
# 11. 프로그램 진입점
# ============================================================================

def main() -> None:
    rclpy.init()
    ros = MotorRosBridge()
    driver = MotorDriver()

    try:
        # 11-1. 프로그램 시작 시 모터를 한 번만 설정한다.
        ros.publish_state("SETUP")
        driver.verify_connection()
        driver.reset()
        driver.setup_runtime()
        driver.enable()

        # 11-2. 현재 위치를 이번 프로그램 실행의 INITIAL_POS로 한 번만 저장한다.
        initial_pos = driver.read_position()
        print(f"[INITIAL] 프로그램 시작 기준 위치 저장: {initial_pos}")
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
                run_once(driver=driver, ros=ros, initial_pos=initial_pos)
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