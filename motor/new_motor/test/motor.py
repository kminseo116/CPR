#!/usr/bin/env python3

"""검증된 증분 위치 명령을 사용하는 새 RS485 모터 CPR 시험 코드.



중요

----

1. 모든 운동은 0x0044 증분 명령을 사용한다.

2. MSTEP 1000 측정 명령 단위(2005/rev)와 피드백 단위(16384/rev)를 분리한다.

3. 저속 이동 중 /loadcell_total이 CONTACT_FORCE_N에 도달할 때까지 접촉을 탐색한다.

4. 접촉 감지 위치를 CPR 시작 기준점으로 사용한다.

5. CPR은 입력 깊이와 목표 BPM으로 계산한 속도를 사용한다.

6. 고속 CPR은 반드시 무인 시험 장비에서 1회부터 검증한다.

"""



from __future__ import annotations



import time

import traceback



import motor_config as cfg

from motor_driver import MotorDriver, MotorError, MotorStatus

from motor_ros_node import MotorRosBridge





MOVE_CONFIRM_COUNT = 5

MIN_FEEDBACK_MOVEMENT = 16



# 고속 CPR 명령 후 이 시간 안에 최소 이동량이 확인되지 않으면

# 명령이 실제 운전으로 이어지지 않은 것으로 판단한다.

CPR_COMMAND_START_TIMEOUT_S = 1.5

CPR_COMMAND_START_MIN_COUNTS = 1000



MOVE_STATE_LABELS = {

    "CONTACT_SEARCH": "로드셀 접촉 탐색",

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





class _RecoveryWithoutRos:

    """Ctrl+C 후 rclpy 컨텍스트가 종료되어도 RS485 복귀 루프를 독립 실행한다.



    ROS 상태 발행 실패로 정지/복귀 명령이 중단되는 것을 막는다.

    CPR 운전에는 사용하지 않으며 사용자가 Ctrl+C를 누른 복귀 경로에만 쓴다.

    """



    def publish_state(self, state: str) -> None:

        print(f"[RECOVERY STATE] {state} (ROS 발행 생략)", flush=True)



    def spin_once(self, *args, **kwargs) -> None:

        pass



    def should_stop(self) -> bool:

        return False  # 첫 Ctrl+C는 이미 처리됨. 복귀 중 두 번째 Ctrl+C는 중단.



    def clear_start_stop_flags(self) -> None:

        pass



    def publish_motor_status(self, position: int, current_raw: int) -> None:

        pass





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





# def calculate_cpr_output_rpm(depth_cm: float) -> float:

#     """지정 깊이를 목표 BPM으로 왕복하는 데 필요한 모터 rpm을 계산한다."""

#     one_way_motor_revolutions = mm_to_motor_revolutions(

#         depth_cm * 10.0

#     )

#     required_rpm = (

#         2.0

#         * one_way_motor_revolutions

#         * cfg.TARGET_BPM

#         * cfg.CPR_SPEED_MARGIN

#     )

#     allowed_output_rpm = min(

#         cfg.MAX_OUTPUT_SPEED_RPM,

#         cfg.MAX_VERIFIED_OUTPUT_SPEED_RPM,

#     )

#     if required_rpm > allowed_output_rpm:

#         raise ValueError(

#             "요청 깊이와 BPM에 필요한 속도가 허용 범위를 초과합니다: "

#             f"required={required_rpm:.1f} rpm, "

#             f"allowed={allowed_output_rpm:.1f} rpm"

#         )

#     return required_rpm



def calculate_cpr_output_rpm(depth_cm: float) -> float:

    """압박 깊이와 관계없이 고정 RPM 사용 (원인 분석용)."""



    fixed_rpm = 3000.0



    allowed_output_rpm = min(

        cfg.MAX_OUTPUT_SPEED_RPM,

        cfg.MAX_VERIFIED_OUTPUT_SPEED_RPM,

    )



    if not 0 < fixed_rpm <= allowed_output_rpm:

        raise ValueError(

            f"고정 RPM이 허용 범위를 벗어났습니다: "

            f"{fixed_rpm:.1f} / {allowed_output_rpm:.1f}"

        )



    print(

        f"[SPEED MODE] FIXED | "

        f"depth={depth_cm:.2f} cm | "

        f"rpm={fixed_rpm:.1f}"

    )



    return fixed_rpm





def calculate_move_timeout_s(

    expected_feedback_delta: int,

    output_rpm: float,

) -> float:

    """이동거리와 실제 출력축 rpm으로 안전 제한시간을 계산한다."""

    output_revolutions = (

        abs(expected_feedback_delta)

        / cfg.FEEDBACK_COUNTS_PER_MOTOR_REV

    )

    expected_time_s = (

        output_revolutions

        / output_rpm

        * 60.0

    )

    return max(

        cfg.MOVE_TIMEOUT_MIN_S,

        expected_time_s

        * cfg.MOVE_TIMEOUT_FACTOR

        + cfg.MOVE_TIMEOUT_MARGIN_S,

    )



# 압박 깊이 키보드로 입력

# def read_depth_cm() -> float:

#     """사용자로부터 압박 깊이를 입력받는다."""

#     prompt = (

#         f"압박 깊이 입력 [{cfg.MIN_DEPTH_CM:.1f}~"

#         f"{cfg.MAX_DEPTH_CM:.1f} cm, "

#         f"Enter={cfg.DEFAULT_DEPTH_CM:.1f} cm]: "

#     )

#     raw_value = input(prompt).strip()



#     if not raw_value:

#         return cfg.DEFAULT_DEPTH_CM



#     depth_cm = float(raw_value)



#     if not (

#         cfg.MIN_DEPTH_CM

#         <= depth_cm

#         <= cfg.MAX_DEPTH_CM

#     ):

#         raise ValueError(

#             f"압박 깊이는 {cfg.MIN_DEPTH_CM:.1f}~"

#             f"{cfg.MAX_DEPTH_CM:.1f} cm 범위여야 합니다."

#         )



#     return depth_cm





def read_depth_cm() -> float:

    """압박 깊이를 3.3cm로 고정한다."""

    depth_cm = 3.3

    print(f"[CPR] 고정 압박 깊이: {depth_cm:.2f} cm")

    return depth_cm





def print_runtime_settings(

    initial_pos: int,

) -> None:

    compression_direction = (

        cfg.POSITIVE_DIRECTION_NAME

        if cfg.MOTOR_DIRECTION_SIGN > 0

        else cfg.NEGATIVE_DIRECTION_NAME

    )



    print("\n=== 실행 설정 ===")



    print(

        f"[POSITION] INITIAL_FEEDBACK="

        f"{initial_pos}"

    )



    print(

        "[UNIT] COMMAND/REV="

        f"{cfg.COMMAND_UNITS_PER_MOTOR_REV}, "

        "FEEDBACK/REV="

        f"{cfg.FEEDBACK_COUNTS_PER_MOTOR_REV}"

    )



    print(

        f"[SPEED] SEARCH_OUTPUT="

        f"{cfg.SEARCH_OUTPUT_SPEED_RPM:.2f} rpm"

    )



    print(

        "[SPEED] COMPRESSION=AUTO_BY_DEPTH_AND_BPM "

        f"(margin={cfg.CPR_SPEED_MARGIN:.3f})"

    )



    print(

        f"[SPEED] MAX_OUTPUT="

        f"{cfg.MAX_OUTPUT_SPEED_RPM:.2f} rpm"

    )



    print(

        "[SPEED] VERIFIED_LIMIT="

        f"{cfg.MAX_VERIFIED_OUTPUT_SPEED_RPM:.2f} rpm"

    )



    print(

        f"[DIRECTION] COMPRESSION="

        f"{compression_direction}"

    )



    print(

        f"[CPR] REPEAT="

        f"{cfg.REPEAT_COUNT}"

    )



    print(

        f"[CPR] TARGET_BPM="

        f"{cfg.TARGET_BPM:.1f}"

    )



    print(

        f"[CURRENT] TEMP_LIMIT="

        f"{cfg.MAX_CURRENT_RAW} raw"

    )





def check_alarm_value(alarm: int) -> None:

    """알람 비트값을 검사하고 해당 예외를 발생시킨다."""

    if alarm == 0:

        return



    alarm_names = MotorDriver.decode_alarm(alarm)

    alarm_text = (

        ", ".join(alarm_names)

        or f"0x{alarm:04X}"

    )



    if alarm & cfg.ALARM_OVERCURRENT:

        raise OvercurrentFault(

            f"과전류 알람 발생: {alarm_text}"

        )



    raise MotorError(

        f"모터 알람 발생: {alarm_text}"

    )





def check_alarm_status(

    status: MotorStatus,

) -> None:

    check_alarm_value(status.alarm)





def update_software_current_trip(

    current_raw: int,

    count: int,

) -> int:

    """전류 상한이 연속 초과되는지 검사한다."""

    count = (

        count + 1

        if current_raw >= cfg.MAX_CURRENT_RAW

        else 0

    )



    if (

        count

        >= cfg.CURRENT_TRIP_COUNT_LIMIT

    ):

        raise SoftwareCurrentTrip(

            "전류 raw 상한 연속 초과: "

            f"current={current_raw}, "

            f"limit={cfg.MAX_CURRENT_RAW}"

        )



    return count





def move_incremental_and_wait(

    driver: MotorDriver,

    ros: MotorRosBridge,

    command_units: int,

    expected_feedback_delta: int,

    output_rpm: float,

    state_name: str,

    timeout_s: float | None = None,

    allow_direction_change_grace: bool = False,

) -> MotorStatus:

    """증분 이동을 실행하고 실제 피드백 변화로 완료와 안전을 검사한다."""



    if (

        command_units == 0

        or expected_feedback_delta == 0

    ):

        raise ValueError(

            "이동 명령과 예상 피드백은 0일 수 없습니다."

        )



    speed_setting = (

        driver.output_rpm_to_speed_setting(

            output_rpm

        )

    )



    if timeout_s is None:

        timeout_s = (

            calculate_move_timeout_s(

                expected_feedback_delta,

                output_rpm,

            )

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



    state_label = move_state_label(

        state_name

    )



    direction_name = (

        cfg.POSITIVE_DIRECTION_NAME

        if command_units > 0

        else cfg.NEGATIVE_DIRECTION_NAME

    )



    print(

        f"\n[이동 시작] {state_label}"

    )



    print(

        f"방향: {direction_name} | "

        f"출력축 목표: {output_rpm:.2f} rpm | "

        f"내부 설정값: {speed_setting}"

    )



    print(

        f"시작 위치: {start_position} | "

        f"목표 변화량: "

        f"{expected_feedback_delta} count"

    )



    print(

        f"피드백 초과 허용: 목표 대비 "

        f"+{feedback_overshoot_limit} count "

        f"({cfg.MAX_FEEDBACK_OVERSHOOT_RATIO * 100.0:.1f}% "

        "비율 기준)"

    )



    driver.move_incremental(

        command_units,

        speed_setting,

    )



    start_time = time.monotonic()



    movement_detected = False

    reached_count = 0

    current_trip_count = 0

    last_status_print_time = -1.0



    while True:

        ros.spin_once()



        if ros.should_stop():

            driver.stop()



            raise UserStopRequested(

                f"{state_name} 중 정지 요청"

            )



        status = driver.read_status()



        check_alarm_status(status)



        current_trip_count = (

            update_software_current_trip(

                status.current_raw,

                current_trip_count,

            )

        )



        ros.publish_motor_status(

            status.position,

            status.current_raw,

        )



        feedback_delta = (

            status.position

            - start_position

        )



        elapsed = (

            time.monotonic()

            - start_time

        )



        if (

            last_status_print_time < 0

            or elapsed

            - last_status_print_time

            >= cfg.MOVE_STATUS_LOG_PERIOD_S

        ):

            progress_percent = min(

                100.0,

                abs(feedback_delta)

                / abs(expected_feedback_delta)

                * 100.0,

            )



            print(

                f"[진행] {state_label} | "

                f"{elapsed:.1f}s | "

                f"{progress_percent:5.1f}% | "

                f"위치={status.position} | "

                f"속도설정={status.speed_rpm} | "

                f"전류={status.current_raw} | "

                f"오차={status.position_error}"

            )



            last_status_print_time = elapsed



        if (

            abs(feedback_delta)

            >= MIN_FEEDBACK_MOVEMENT

            or status.speed_rpm != 0

        ):

            movement_detected = True



        # ====================================================

        # 반대 방향 판정

        #

        # 기존:

        # MIN_FEEDBACK_MOVEMENT(16 count)만 넘어도

        # 부호가 반대이면 즉시 MOTOR_ERROR 처리했다.

        #

        # 수정:

        # DIRECTION_ERROR_MIN_COUNTS 이상 실제로

        # 반대 방향으로 움직였을 때만 오류 처리한다.

        #

        # 따라서 +22, -156 같은 방향 전환 직후

        # 미세한 위치 흔들림은 오류로 판단하지 않는다.

        # ====================================================



        wrong_direction = (

            abs(feedback_delta)

            >= cfg.DIRECTION_ERROR_MIN_COUNTS

            and (

                feedback_delta

                * expected_feedback_delta

                < 0

            )

        )



        within_direction_change_grace = (

            allow_direction_change_grace

            and elapsed

            <= cfg.CPR_DIRECTION_CHANGE_GRACE_S

            and abs(feedback_delta)

            <= cfg.CPR_DIRECTION_CHANGE_GRACE_COUNTS

        )



        if (

            wrong_direction

            and not within_direction_change_grace

        ):

            driver.stop()



            raise MotorError(

                f"{state_name} 반대 방향 이동: "

                f"expected="

                f"{expected_feedback_delta}, "

                f"actual={feedback_delta}, "

                f"direction_limit="

                f"{cfg.DIRECTION_ERROR_MIN_COUNTS}"

            )



        # ====================================================

        # 목표 위치 초과 판정

        #

        # 반대 방향 판정과 별도로 유지한다.

        # ====================================================



        if (

            abs(feedback_delta)

            > abs(expected_feedback_delta)

            + feedback_overshoot_limit

        ):

            driver.stop()



            raise MotorError(

                f"{state_name} 피드백 초과: "

                f"expected="

                f"{expected_feedback_delta}, "

                f"actual={feedback_delta}, "

                f"allowed_overshoot="

                f"{feedback_overshoot_limit}"

            )



        # ====================================================

        # 정상 목표 도달 판정

        #

        # 방향/초과 판정과 별도로 유지한다.

        # ====================================================



        reached = (

            movement_detected

            and abs(status.position_error)

            <= cfg.POSITION_TOLERANCE_UNITS

            and status.speed_rpm == 0

        )



        reached_count = (

            reached_count + 1

            if reached

            else 0

        )



        if (

            reached_count

            >= MOVE_CONFIRM_COUNT

        ):

            driver.stop()



            print(

                f"[이동 완료] {state_label} | "

                f"실제 변화량="

                f"{feedback_delta} count | "

                f"최종 오차="

                f"{status.position_error}"

            )



            return status



        if (

            abs(status.position_error)

            > cfg.MAX_POSITION_ERROR_UNITS

        ):

            driver.stop()



            raise MotorError(

                "위치 편차 초과: "

                f"error={status.position_error}, "

                f"limit="

                f"{cfg.MAX_POSITION_ERROR_UNITS}"

            )



        if (

            time.monotonic()

            - start_time

            >= timeout_s

        ):

            driver.stop()



            raise MotorError(

                f"이동 시간 초과: "

                f"state={state_name}, "

                f"expected="

                f"{expected_feedback_delta}, "

                f"actual={feedback_delta}"

            )



        time.sleep(

            cfg.STATUS_PERIOD_S

        )





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



    if (

        command_units == 0

        or expected_feedback_delta == 0

    ):

        raise ValueError(

            "이동 명령과 예상 피드백은 0일 수 없습니다."

        )



    if (

        completion_tolerance_counts

        is None

    ):

        completion_tolerance_counts = (

            cfg.POSITION_TOLERANCE_UNITS

        )



    completion_tolerance_counts = max(

        cfg.POSITION_TOLERANCE_UNITS,

        int(

            completion_tolerance_counts

        ),

    )



    timeout_s = (

        calculate_move_timeout_s(

            expected_feedback_delta,

            output_rpm,

        )

    )



    emergency_overshoot_limit = max(

        cfg.CPR_MIN_EMERGENCY_OVERSHOOT_COUNTS,

        round(

            abs(expected_feedback_delta)

            * cfg.CPR_MAX_EMERGENCY_OVERSHOOT_RATIO

        ),

    )



    if start_position is None:

        start_position = (

            driver.read_position()

        )



    state_label = move_state_label(

        state_name

    )



    direction_name = (

        cfg.POSITIVE_DIRECTION_NAME

        if command_units > 0

        else cfg.NEGATIVE_DIRECTION_NAME

    )



    ros.publish_state(

        state_name

    )



    print(

        f"\n[이동 시작] {state_label} | "

        f"방향={direction_name} | "

        f"속도={output_rpm:.1f} rpm"

    )



    driver.record_event("CPR 이동 요청", f"state={state_name}, command={command_units}, "

                        f"expected={expected_feedback_delta}, rpm={output_rpm:.1f}, start={start_position}")

    command_time = time.monotonic()



    driver.move_incremental_prepared(

        command_units

    )



    movement_detected = False



    direction_change_wait_logged = False



    last_log_time = command_time



    while True:

        ros.spin_once()



        if ros.should_stop():

            driver.stop()



            raise UserStopRequested(

                f"{state_name} 중 정지 요청"

            )



        position = driver.read_position()



        now = time.monotonic()



        elapsed_s = (

            now - command_time

        )



        feedback_delta = (

            position

            - start_position

        )



        abs_delta = abs(

            feedback_delta

        )



        moving_in_expected_direction = (

            feedback_delta

            * expected_feedback_delta

            > 0

        )



        if (

            abs_delta

            >= MIN_FEEDBACK_MOVEMENT

            and moving_in_expected_direction

        ):

            movement_detected = True



        if (

            elapsed_s

            >= CPR_COMMAND_START_TIMEOUT_S

            and abs_delta

            < CPR_COMMAND_START_MIN_COUNTS

        ):

            driver.record_event("CPR 시작 감시 실패", f"state={state_name}, elapsed={elapsed_s:.3f}s, "

                                f"delta={feedback_delta}, min={CPR_COMMAND_START_MIN_COUNTS}")

            # 정지 명령까지 실패하더라도 '이동 명령 미실행'이라는 최초 오류를 보존한다.

            try:

                driver.stop()

            except Exception as stop_exc:

                driver.record_event("정지 명령 실패", f"{type(stop_exc).__name__}: {stop_exc}")

                print(f"[정지 요청 실패] {type(stop_exc).__name__}: {stop_exc}", flush=True)



            raise MotorError(

                f"{state_name} 이동 명령 미실행: "

                f"elapsed={elapsed_s:.3f}s, "

                f"expected="

                f"{expected_feedback_delta}, "

                f"actual={feedback_delta}, "

                f"start_min="

                f"{CPR_COMMAND_START_MIN_COUNTS}"

            )



        # ====================================================

        # CPR 고속 왕복 반대 방향 판정

        #

        # 핵심 수정 부분.

        #

        # 기존에는 +22 count 같이 작은 위치 흔들림도

        # CPR_DIRECTION_CHANGE_GRACE_S 시간이 지나면

        # 반대 방향 오류가 될 수 있었다.

        #

        # 이제 DIRECTION_ERROR_MIN_COUNTS보다 작은

        # 반대 방향 변화는 시간과 관계없이 무시한다.

        # ====================================================



        wrong_direction = (

            abs_delta

            >= cfg.DIRECTION_ERROR_MIN_COUNTS

            and (

                feedback_delta

                * expected_feedback_delta

                < 0

            )

        )



        # 실제 반대 방향 이동량이 임계값을 넘었더라도,

        # 방향 전환 직후 정해진 시간/거리 안이면

        # 기존 방향의 관성으로 허용한다.

        initial_direction_change_inertia = (

            wrong_direction

            and elapsed_s

            <= cfg.CPR_DIRECTION_CHANGE_GRACE_S

            and abs_delta

            <= cfg.CPR_DIRECTION_CHANGE_GRACE_COUNTS

        )



        if (

            wrong_direction

            and not initial_direction_change_inertia

        ):

            driver.stop()



            raise MotorError(

                f"{state_name} 반대 방향 이동: "

                f"expected="

                f"{expected_feedback_delta}, "

                f"actual={feedback_delta}, "

                f"direction_limit="

                f"{cfg.DIRECTION_ERROR_MIN_COUNTS}"

            )



        if (

            initial_direction_change_inertia

            and not direction_change_wait_logged

        ):

            print(

                f"[방향 전환 대기] "

                f"{state_label} | "

                f"기존 방향 관성="

                f"{feedback_delta:+d} count"

            )



            direction_change_wait_logged = True



        # ====================================================

        # 비상 피드백 초과 판정

        #

        # 방향 판정과 별도로 기존 기준을 유지한다.

        # ====================================================



        if (

            abs_delta

            > abs(expected_feedback_delta)

            + emergency_overshoot_limit

        ):

            driver.stop()



            raise MotorError(

                f"{state_name} 비상 피드백 초과: "

                f"expected="

                f"{expected_feedback_delta}, "

                f"actual={feedback_delta}, "

                f"emergency_overshoot="

                f"{emergency_overshoot_limit}"

            )



        # ====================================================

        # CPR 위치 도달 판정

        #

        # 방향/초과 판정과 별도로 기존 기준을 유지한다.

        # ====================================================



        reached = (

            movement_detected

            and abs_delta

            >= (

                abs(expected_feedback_delta)

                - completion_tolerance_counts

            )

        )



        if (

            now - last_log_time

            >= cfg.MOVE_STATUS_LOG_PERIOD_S

        ):

            progress_percent = min(

                100.0,

                abs_delta

                / abs(expected_feedback_delta)

                * 100.0,

            )



            print(

                f"[이동 대기] {state_label} | "

                f"{now - command_time:.3f}s | "

                f"{progress_percent:5.1f}% | "

                f"위치={position} | "

                f"위치차이="

                f"{feedback_delta:+d} count"

            )



            last_log_time = now



        if reached:

            target_difference = (

                feedback_delta

                - expected_feedback_delta

            )



            average_rpm = (

                abs(feedback_delta)

                / cfg.FEEDBACK_COUNTS_PER_MOTOR_REV

                * 60.0

                / elapsed_s

                if elapsed_s > 0.0

                else 0.0

            )



            print(

                f"[이동 완료] {state_label} | "

                f"방향={direction_name} | "

                f"평균속도={average_rpm:.1f} rpm | "

                f"위치={position} | "

                f"위치차이="

                f"{feedback_delta:+d} count | "

                f"목표차이="

                f"{target_difference:+d} count"

            )



            return position



        if (

            now - command_time

            >= timeout_s

        ):

            driver.stop()



            raise MotorError(

                f"이동 시간 초과: "

                f"state={state_name}, "

                f"expected="

                f"{expected_feedback_delta}, "

                f"position={position}"

            )



        time.sleep(

            cfg.CPR_POSITION_POLL_PERIOD_S

        )





def search_contact_with_loadcell(

    driver: MotorDriver,

    ros: MotorRosBridge,

) -> int:

    """저속 이동 중 로드셀 힘이 기준값에 도달하면 그 위치를 반환한다."""



    distance_mm = cfg.CONTACT_SEARCH_MAX_MM

    direction = cfg.MOTOR_DIRECTION_SIGN

    command_units = (

        mm_to_command_units(distance_mm)

        * direction

    )

    expected_feedback_delta = (

        mm_to_feedback_counts(distance_mm)

        * direction

    )

    speed_setting = driver.output_rpm_to_speed_setting(

        cfg.SEARCH_OUTPUT_SPEED_RPM

    )

    search_origin = driver.read_position()

    timeout_s = cfg.CONTACT_SEARCH_TIMEOUT_S



    # 이전 실행에서 받은 오래된 힘 값으로 즉시 접촉 판정하지 않는다.

    ros.spin_once()

    if ros.is_loadcell_stale(cfg.LOADCELL_STALE_TIMEOUT_S):

        raise MotorError(

            "/loadcell_total 데이터가 없거나 오래되었습니다. "

            "로드셀 노드를 확인하십시오."

        )



    ros.publish_state("CONTACT_SEARCH")

    print("\n=== 로드셀 접촉 탐색 시작 ===")

    print(

        f"[CONTACT] 기준={cfg.CONTACT_FORCE_N:.2f} N | "

        f"최대 이동={distance_mm:.1f} mm | "

        f"속도={cfg.SEARCH_OUTPUT_SPEED_RPM:.2f} rpm"

    )



    if ros.loadcell_total >= cfg.CONTACT_FORCE_N:

        ros.publish_contact_source("LOADCELL")

        print(

            f"[CONTACT] 이동 전 이미 힘 기준 도달 | "

            f"힘={ros.loadcell_total:.2f} N | "

            f"CPR 기준 위치={search_origin}"

        )

        return search_origin



    driver.move_incremental(command_units, speed_setting)

    start_time = time.monotonic()

    current_trip_count = 0



    try:

        while True:

            ros.spin_once()

            if ros.should_stop():

                raise UserStopRequested("접촉 탐색 중 정지 요청")



            if ros.is_loadcell_stale(cfg.LOADCELL_STALE_TIMEOUT_S):

                raise MotorError(

                    "접촉 탐색 중 /loadcell_total 데이터가 끊겼습니다."

                )



            status = driver.read_status()

            check_alarm_status(status)

            current_trip_count = update_software_current_trip(

                status.current_raw,

                current_trip_count,

            )

            ros.publish_motor_status(status.position, status.current_raw)



            feedback_delta = status.position - search_origin

            if (

                abs(feedback_delta) >= cfg.DIRECTION_ERROR_MIN_COUNTS

                and feedback_delta * expected_feedback_delta < 0

            ):

                raise MotorError(

                    "접촉 탐색 중 반대 방향 이동: "

                    f"expected={expected_feedback_delta}, actual={feedback_delta}"

                )



            if ros.loadcell_total >= cfg.CONTACT_FORCE_N:

                driver.stop()

                contact_position = driver.read_position()

                ros.publish_contact_source("LOADCELL")

                print(

                    f"[CONTACT] 힘={ros.loadcell_total:.2f} N | "

                    f"CPR 기준 위치={contact_position}"

                )

                return contact_position



            if abs(feedback_delta) >= abs(expected_feedback_delta):

                raise MotorError(

                    "최대 접촉 탐색거리 안에서 힘 기준에 도달하지 못했습니다: "

                    f"force={ros.loadcell_total:.2f} N, "

                    f"limit={cfg.CONTACT_FORCE_N:.2f} N"

                )



            if time.monotonic() - start_time >= timeout_s:

                raise MotorError(

                    "로드셀 접촉 탐색 시간 초과: "

                    f"force={ros.loadcell_total:.2f} N"

                )



            time.sleep(cfg.STATUS_PERIOD_S)

    except BaseException:

        # 정지 명령 실패 때문에 원래 KeyboardInterrupt/오류가 가려지지 않게 한다.

        try:

            driver.stop()

        except Exception as stop_exc:

            print(f"[SEARCH STOP FAIL] {type(stop_exc).__name__}: {stop_exc}", flush=True)

        raise



def run_compression(

    driver: MotorDriver,

    ros: MotorRosBridge,

    start_pos: int,

    depth_cm: float,

) -> None:

    """로드셀 값을 받고 기준 위치에서 지정 깊이만큼 증분 왕복한다."""



    direction = (

        cfg.MOTOR_DIRECTION_SIGN

    )



    distance_mm = (

        depth_cm * 10.0

    )



    depth_command = (

        mm_to_command_units(

            distance_mm

        )

        * direction

    )



    depth_feedback = (

        mm_to_feedback_counts(

            distance_mm

        )

        * direction

    )



    compression_output_rpm = (

        calculate_cpr_output_rpm(

            depth_cm

        )

    )



    compression_speed_setting = (

        driver.output_rpm_to_speed_setting(

            compression_output_rpm

        )

    )



    print(

        "\n=== CPR 압박 시작 ==="

    )



    print(

        f"[CPR] START_FEEDBACK="

        f"{start_pos}"

    )



    print(

        f"[CPR] DEPTH="

        f"{depth_cm:.2f} cm"

    )



    print(

        f"[CPR] DOWN_COMMAND="

        f"{depth_command}"

    )



    print(

        f"[CPR] EXPECTED_FEEDBACK="

        f"{depth_feedback}"

    )



    print(

        "[CPR] OUTPUT_SPEED="

        f"{compression_output_rpm:.2f} rpm "

        f"(internal="

        f"{compression_speed_setting})"

    )



    print(

        f"[CPR] TARGET_BPM="

        f"{cfg.TARGET_BPM:.1f}"

    )



    print(

        f"[CPR] REPEAT="

        f"{cfg.REPEAT_COUNT}"

    )



    ros.publish_state(

        "Compressing"

    )



    ros.reset_compression()



    driver.prepare_incremental_motion(

        compression_speed_setting

    )



    applied_speed_setting = (

        driver.read_u16(

            cfg.REG_INTERNAL_SPEED

        )

    )



    print(

        "[속도 레지스터 확인] "

        f"요청="

        f"{compression_speed_setting} | "

        f"0x002D 읽기="

        f"{applied_speed_setting}"

    )



    if (

        applied_speed_setting

        != compression_speed_setting

    ):

        raise MotorError(

            "내부 속도 설정 검증 실패: "

            f"requested="

            f"{compression_speed_setting}, "

            f"readback="

            f"{applied_speed_setting}"

        )



    cycle_start_position = (

        start_pos

    )



    cycle_current_trip_count = 0



    for count in range(

        1,

        cfg.REPEAT_COUNT + 1,

    ):

        down_position = (

            move_incremental_cpr_fast(

                driver,

                ros,

                depth_command,

                depth_feedback,

                compression_output_rpm,

                "COMPRESSION_DOWN",

                start_position=(

                    cycle_start_position

                ),

                completion_tolerance_counts=max(

                    cfg.POSITION_TOLERANCE_UNITS,

                    round(

                        abs(depth_feedback)

                        * cfg.CPR_DOWN_COMPLETION_TOLERANCE_RATIO

                    ),

                ),

            )

        )



        actual_depth_cm = (

            feedback_counts_to_cm(

                abs(

                    down_position

                    - cycle_start_position

                )

            )

        )



        ros.count_compression(

            actual_depth_cm

        )



        move_incremental_cpr_fast(

            driver,

            ros,

            -depth_command,

            -depth_feedback,

            compression_output_rpm,

            "COMPRESSION_UP",

            start_position=(

                down_position

            ),

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

        cycle_status = (

            driver.read_cycle_status()

        )

        driver.record_event("정상 왕복 후 상태 확보", f"cycle={count}, alarm=0x{cycle_status.alarm:04X}, "

                            f"current={cycle_status.current_raw}, position={cycle_status.position}")



        check_alarm_value(

            cycle_status.alarm

        )



        cycle_current_trip_count = (

            update_software_current_trip(

                cycle_status.current_raw,

                cycle_current_trip_count,

            )

        )



        ros.publish_motor_status(

            cycle_status.position,

            cycle_status.current_raw,

        )



        return_position_difference = (

            cycle_status.position

            - start_pos

        )



        print(

            f"[CPR] "

            f"{count}/{cfg.REPEAT_COUNT} | "

            f"방향="

            f"{cfg.POSITIVE_DIRECTION_NAME}"

            "→"

            f"{cfg.NEGATIVE_DIRECTION_NAME} | "

            f"속도="

            f"{compression_output_rpm:.1f} rpm | "

            f"전류="

            f"{cycle_status.current_raw} | "

            f"위치="

            f"{cycle_status.position} | "

            f"위치차이="

            f"{return_position_difference:+d} count | "

            f"깊이="

            f"{actual_depth_cm:.3f} cm | "

            f"BPM="

            f"{ros.compression_bpm:.1f}"

        )



        # read_cycle_status()가 위치를 마지막에 읽으므로 이 값이 다음

        # 사이클의 가장 최신 기준 위치다. 별도 위치 통신은 추가하지 않는다.

        cycle_start_position = (

            cycle_status.position

        )





def return_to_feedback_position(

    driver: MotorDriver,

    ros: MotorRosBridge,

    target_position: int,

) -> None:

    """현재 피드백과 목표 피드백의 차이를 증분 명령으로 복귀한다."""



    current_position = (

        driver.read_position()

    )



    feedback_delta = (

        target_position

        - current_position

    )



    if (

        abs(feedback_delta)

        <= cfg.POSITION_TOLERANCE_UNITS

    ):

        return



    command_units = (

        feedback_to_command_units(

            feedback_delta

        )

    )



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



    target_position = (

        cfg.START_FEEDBACK_POSITION

    )



    # 위치 명령 환산 오차가 남으면 한 번 더 보정한다. 반복 횟수를 제한해

    # 잘못된 피드백이나 기구 걸림에서 무한 왕복하지 않도록 한다.

    for attempt in range(

        1,

        3,

    ):

        current_position = (

            driver.read_position()

        )



        remaining_counts = (

            target_position

            - current_position

        )



        if (

            abs(remaining_counts)

            <= cfg.POSITION_TOLERANCE_UNITS

        ):

            print(

                f"[START POSITION] "

                f"준비 완료 | "

                f"위치={current_position} | "

                f"목표={target_position}"

            )



            return target_position



        remaining_cm = (

            feedback_counts_to_cm(

                abs(remaining_counts)

            )

        )



        print(

            f"[START POSITION] "

            f"0 위치 복귀 {attempt}/2 | "

            f"현재={current_position} | "

            f"목표={target_position} | "

            f"이동거리="

            f"{remaining_cm:.3f} cm"

        )



        ros.publish_state(

            "RETURNING_TO_START_ZERO"

        )



        return_to_feedback_position(

            driver,

            ros,

            target_position,

        )



    final_position = (

        driver.read_position()

    )



    if (

        abs(

            final_position

            - target_position

        )

        <= cfg.POSITION_TOLERANCE_UNITS

    ):

        print(

            f"[START POSITION] "

            f"준비 완료 | "

            f"위치={final_position} | "

            f"목표={target_position}"

        )



        return target_position



    raise MotorError(

        "시작 위치 0 복귀 실패: "

        f"target={target_position}, "

        f"actual={final_position}, "

        f"error="

        f"{final_position - target_position}"

    )





def normal_stop_and_return(

    driver: MotorDriver,

    ros: MotorRosBridge,

    initial_pos: int,

) -> None:

    print(

        "\n=== 정상 정지 처리 ==="

    )



    driver.stop()

    # 현재 정지 요청을 소비한 뒤 복귀한다. 플래그가 남아 있으면 복귀

    # 감시 루프가 같은 요청을 다시 감지해 즉시 중단한다.

    ros.clear_start_stop_flags()



    if (

        cfg.RETURN_TO_INITIAL_ON_STOP

    ):

        ros.publish_state(

            "RETURNING_TO_INITIAL"

        )



        return_to_feedback_position(

            driver,

            ros,

            initial_pos,

        )



    ros.publish_state(

        "STOPPED_SERVO_ON"

    )

    if cfg.RETURN_TO_INITIAL_ON_STOP:

        print("[STOP] 초기 위치 복귀 완료, Servo ON 유지", flush=True)

    else:

        print("[STOP] 설정에 따라 초기 복귀 생략, Servo ON 유지", flush=True)





def requested_stop_and_return(

    driver: MotorDriver,

    ros: MotorRosBridge,

    initial_pos: int,

) -> None:

    """사용자 정지 요청: 강제 정지 → 실제 초기 위치 확인/복귀 → 종료 준비.



    정상 정지 요청에 한해 복귀하며 RETURN_TO_INITIAL_ON_STOP 값과 무관하다.

    오류나 과전류의 fault_stop에서는 이 함수를 호출하지 않는다.

    """

    print("[USER STOP] 정지 명령 → 초기 위치 복귀", flush=True)

    driver.stop()

    ros.clear_start_stop_flags()



    # 고장 상태에서 사용자 정지 경로가 우연히 복귀 동작을 유발하지 않도록

    # 정지 뒤 드라이버 알람/전류를 확인한다. 읽기에 실패하면 복귀하지 않는다.

    recovery_status = driver.read_status()

    check_alarm_status(recovery_status)

    if recovery_status.current_raw >= cfg.MAX_CURRENT_RAW:

        raise MotorError(

            "사용자 정지 후 전류가 제한 이상이므로 자동 복귀 금지: "

            f"current={recovery_status.current_raw}, limit={cfg.MAX_CURRENT_RAW}"

        )



    ros.publish_state("RETURNING_TO_INITIAL")

    # 단순 증분 복귀만으로 끝내지 않고 실제 위치를 확인하며 최대 2회 보정한다.

    prepare_zero_start_position(driver, ros)

    final_pos = driver.read_position()

    error = final_pos - initial_pos

    if abs(error) > cfg.POSITION_TOLERANCE_UNITS:

        raise MotorError(

            f"사용자 정지 후 초기 위치 복귀 확인 실패: "

            f"target={initial_pos}, actual={final_pos}, error={error}"

        )

    ros.publish_state("STOPPED_SERVO_ON")

    print(

        f"[USER STOP] 초기 위치 복귀 확인 완료: "

        f"target={initial_pos}, actual={final_pos}, error={error}",

        flush=True,

    )





def interrupt_stop_and_return(

    driver: MotorDriver,

    ros: MotorRosBridge,

    initial_pos: int,

) -> None:

    """Ctrl+C 종료 전에 진행 중인 운동을 멈추고 초기 위치로 복귀한다."""

    requested_stop_and_return(driver, ros, initial_pos)





def print_motor_fault_diagnostics(

    driver: MotorDriver,

    cause: BaseException | None,

    stop_command_ack: bool,

) -> None:

    """고장 후 읽기 전용 진단. 실패 결과를 성공/정상으로 오인하지 않는다."""

    print("\n========== CPR 모터 상세 고장 보고서 ==========", flush=True)

    if cause is not None:

        message = str(cause)

        print(f"[최초 예외] {type(cause).__name__}: {message}", flush=True)

        if "이동 명령 미실행" in message:

            cause_class = "시작 감시시간 안에 최소 위치 변화 미달 (과부하 확정 아님)"

        elif any(word in message.lower() for word in

                 ("no answer", "no communication", "timeout", "timed out", "에코 불일치")):

            cause_class = "통신 무응답/응답 불일치 (모터 내부 알람은 별개)"

        elif "반대 방향" in message:

            cause_class = "이동 방향 감시 오류"

        elif "피드백 초과" in message or "위치 편차" in message:

            cause_class = "위치 감시 오류"

        else:

            cause_class = "기타 제어 예외: 아래 데이터 확인"

        print(f"[소프트웨어 판정] {cause_class}", flush=True)

    print(f"[정지 명령 응답] {'통신 응답 수신 - 실제 정지 미확인' if stop_command_ack else '전송/응답 실패 - 실제 정지 미확인'}", flush=True)

    print("[자동 초기 복귀] 금지 / [재시작] 프로그램 종료 후 수동 점검 필요", flush=True)



    print("\n--- 마지막 정상 측정값 (고장 순간 값 아님) ---", flush=True)

    if driver.last_cycle_status is not None:

        ts, st = driver.last_cycle_status

        print(f"[마지막 정상 사이클] {time.monotonic()-ts:.3f}초 전 | 위치={st.position} | "

              f"전류={st.current_raw} raw | 알람=0x{st.alarm:04X} | 운전=0x{st.run_status:04X}", flush=True)

    else:

        print("[마지막 정상 사이클] 데이터 없음", flush=True)

    if driver.last_position is not None:

        ts, pos = driver.last_position

        print(f"[마지막 정상 위치] {pos} count | 기록 시점 기준 {time.monotonic()-ts:.3f}초 전", flush=True)

    if driver.last_communication_error:

        print(f"[마지막 통신 오류] {driver.last_communication_error}", flush=True)



    print("\n--- 최근 명령 및 통신 이력 (오래된 순서) ---", flush=True)

    for event in list(driver.events)[-18:]:

        print(f"[이력] {event}", flush=True)



    print("\n--- 정지 후 드라이버 읽기 전용 스냅샷 ---", flush=True)

    if not stop_command_ack:

        print("[진단 생략] 정지 명령 응답 실패: 추가 통신보다 하드웨어 안전 확보 우선", flush=True)

        print("[최종 결론] 드라이버 보호 원인 미확인. 물리 정지와 전원/배선 점검 필요", flush=True)

        print("=============================================\n", flush=True)

        return



    def read(label, operation):

        try:

            value = operation()

            print(f"[{label}] {value}", flush=True)

            return value

        except Exception as exc:

            print(f"[{label}] 읽기 실패 / {type(exc).__name__}: {exc}", flush=True)

            return None



    alarm = read("드라이버 보호 알람", lambda: driver.read_u16(cfg.REG_ALARM_STATUS))

    if alarm is None:

        print("[알람 판정] 통신 실패로 확인 불가", flush=True)

    else:

        decoded = MotorDriver.decode_alarm(alarm)

        known = (cfg.ALARM_OVERVOLTAGE | cfg.ALARM_UNDERVOLTAGE |

                 cfg.ALARM_OVERCURRENT | cfg.ALARM_POSITION_DEVIATION)

        print(f"[알람 원본] 0x{alarm:04X} | 확인된 비트={decoded or '정의된 비트 없음'}", flush=True)

        if alarm & ~known:

            print(f"[미정의 알람] 0x{alarm & ~known:04X} (매뉴얼 확인 필요)", flush=True)

        if not alarm:

            print("[주의] 읽은 시점 알람 0: 과거 알람이나 과부하 부재를 증명하지 않음", flush=True)

    current = read("정지 후 전류(raw)", lambda: driver.read_u16(cfg.REG_CURRENT_CURRENT))

    if current is not None:

        print(f"[전류 비교] {current}/{cfg.MAX_CURRENT_RAW} raw | "

              "정지 후 값이므로 고장 순간의 전류 피크 아님", flush=True)

    voltage = read("정지 후 전압(raw)", lambda: driver.read_u16(cfg.REG_CURRENT_VOLTAGE))

    if voltage is not None:

        print("[전압 주의] raw→V 환산 계수 미확인: 실제 전압(V)으로 해석하지 않음", flush=True)

    error = read("위치 편차(count)", lambda: driver.read_i32(cfg.REG_POSITION_ERROR))

    if error is not None:

        print(f"[위치 편차 비교] abs={abs(error)} / 한도={cfg.MAX_POSITION_ERROR_UNITS}", flush=True)

    run = read("정지 후 운전 상태", lambda: driver.read_u16(cfg.REG_RUN_STATUS))

    if run is not None:

        print(f"[운전 비트] servo_enabled={bool(run & cfg.RUN_ENABLED)}, "

              f"running={bool(run & cfg.RUN_RUNNING)}, "

              f"target_reached={bool(run & cfg.RUN_TARGET_REACHED)}", flush=True)

    read("정지 후 위치(count)", driver.read_position)

    print("[과부하 판정] 별도 OVERLOAD 알람 비트 미정의: 과전류/미동작만으로 확정 불가", flush=True)

    print("[최종 주의] 정지 후 순차 측정 스냅샷이며 고장 순간의 동시 측정값이 아님", flush=True)

    print("=============================================\n", flush=True)





def fault_stop(

    driver: MotorDriver,

    ros: MotorRosBridge,

    reason: str,

    cause: BaseException | None = None,

) -> None:

    # 기존 안전 정책: 자동 초기 복귀하지 않고, 다른 통신보다 정지 명령 우선.

    print(f"[최초 오류] {type(cause).__name__}: {cause}" if cause else

          f"[최초 오류] {reason}", flush=True)

    stopped_ack = False

    try:

        driver.stop()

        stopped_ack = True

    except Exception as exc:

        print(f"[정지 명령 실패] {type(exc).__name__}: {exc}", flush=True)

        print("[긴급] 실제 정지 불명: 하드웨어 비상정지 절차로 안전 확보", flush=True)

    ros.publish_state(f"FAULT_LATCHED_{reason}")

    print(f"[FAULT] {reason}: 자동 초기 위치 복귀 안 함", flush=True)

    print_motor_fault_diagnostics(driver, cause, stopped_ack)





def run_once(

    driver: MotorDriver,

    ros: MotorRosBridge,

    initial_pos: int,

) -> bool:

    # True: 사용자 정지 요청으로 복귀 완료, 메인 루프 종료 필요.

    # False: 정상 반복 종료.

    # 직전 실행이 고장 정지되어 자동 복귀하지 않았더라도 새 압박을 시작하기

    # 전에 반드시 물리적 0 위치로 돌아간다.

    initial_pos = (

        prepare_zero_start_position(

            driver,

            ros,

        )

    )



    print(

        "\n============================================================"

    )



    print(

        "새 모터 CPR 증분 위치 시퀀스 시작"

    )



    print(

        "============================================================"

    )



    print_runtime_settings(

        initial_pos

    )



    depth_cm = (

        read_depth_cm()

    )



    try:

        start_pos = (

            search_contact_with_loadcell(

                driver,

                ros,

            )

        )



        print(

            f"[CONTACT RESULT] "

            f"position={start_pos}"

        )



        run_compression(

            driver,

            ros,

            start_pos,

            depth_cm,

        )



        normal_stop_and_return(

            driver,

            ros,

            initial_pos,

        )

        return False



    except UserStopRequested as exc:

        print(f"[USER STOP] {exc}", flush=True)

        requested_stop_and_return(driver, ros, initial_pos)

        return True



    except OvercurrentFault as exc:

        fault_stop(

            driver,

            ros,

            "OVERCURRENT",

            exc,

        )



        raise MotorError(

            str(exc)

        ) from exc



    except SoftwareCurrentTrip as exc:

        fault_stop(

            driver,

            ros,

            "SOFTWARE_CURRENT_TRIP",

            exc,

        )



        raise MotorError(

            str(exc)

        ) from exc



    except Exception as exc:

        fault_stop(

            driver,

            ros,

            "MOTOR_ERROR",

            exc,

        )



        raise





def main() -> None:

    if not (

        cfg.MSTEP_POSITION_CALIBRATION_COMPLETE

        and cfg.MSTEP_SPEED_CALIBRATION_COMPLETE

    ):

        print(

            "===================================================="

        )



        print(

            " motor_main 실행 차단"

        )



        print(

            "===================================================="

        )



        print(

            "MSTEP 1000 위치·속도 재보정이 "

            "아직 완료되지 않았습니다."

        )



        print(

            "이전 MSTEP 2000 속도 환산값으로 "

            "움직이는 것을 방지합니다."

        )



        return



    import rclpy

    from motor_ros_node import (

        MotorRosBridge,

    )



    rclpy.init()



    ros = (

        MotorRosBridge()

    )



    driver = (

        MotorDriver()

    )



    servo_enabled = False

    initial_pos = cfg.START_FEEDBACK_POSITION



    try:

        ros.publish_state(

            "SETUP"

        )



        driver.verify_connection()



        driver.setup_runtime()



        servo_off_position = (

            driver.read_position()

        )



        print(

            f"[SETUP] Servo OFF 위치: "

            f"{servo_off_position}"

        )



        input(

            "안전을 확인한 후 Enter를 누르면 "

            "Servo ON 및 위치 0 복귀를 "

            "시작합니다. Ctrl+C=취소: "

        )



        driver.enable()

        servo_enabled = True



        time.sleep(

            0.2

        )



        servo_on_position = (

            driver.read_position()

        )



        print(

            "[INITIAL] Servo ON 직후 위치: "

            f"{servo_on_position}"

        )



        initial_pos = (

            prepare_zero_start_position(

                driver,

                ros,

            )

        )



        print(

            "[INITIAL] 압박 시작 기준 위치: "

            f"{initial_pos}"

        )



        print_runtime_settings(

            initial_pos

        )



        # 키보드 입력 제어: r=시작, q=정지/종료
        # 모터/ROS 제어 로직은 그대로 두고 입력 방식만 키보드로 변경한다.
        import atexit
        import select
        import sys
        import termios
        import threading
        import tty

        keyboard_fd = sys.stdin.fileno()
        keyboard_old_settings = termios.tcgetattr(keyboard_fd)
        keyboard_stop_event = threading.Event()
        cpr_running_event = threading.Event()
        keyboard_restored = False

        def restore_keyboard_terminal():
            nonlocal keyboard_restored
            if keyboard_restored:
                return
            termios.tcsetattr(
                keyboard_fd,
                termios.TCSADRAIN,
                keyboard_old_settings,
            )
            keyboard_restored = True

        atexit.register(restore_keyboard_terminal)
        tty.setcbreak(keyboard_fd)

        def keyboard_input_loop():
            try:
                while not keyboard_stop_event.is_set():
                    readable, _, _ = select.select(
                        [sys.stdin],
                        [],
                        [],
                        0.1,
                    )

                    if not readable:
                        continue

                    key = sys.stdin.read(1).lower()

                    if key == "r":
                        if cpr_running_event.is_set():
                            continue

                        ros.motor_stop = False
                        ros.motor_start = True
                        print(
                            "\n[KEYBOARD] r 입력 → CPR 시작 요청",
                            flush=True,
                        )

                    elif key == "q":
                        ros.motor_start = False
                        ros.motor_stop = True
                        print(
                            "\n[KEYBOARD] q 입력 → 정지 및 종료 요청",
                            flush=True,
                        )
                        keyboard_stop_event.set()
                        break

            finally:
                restore_keyboard_terminal()

        keyboard_thread = threading.Thread(
            target=keyboard_input_loop,
            daemon=True,
        )
        keyboard_thread.start()

        print(
            "\n[READY] r=CPR 시작 | q=정지 및 종료"
        )

        while rclpy.ok():
            ros.spin_once(
                timeout_sec=0.1
            )

            if ros.should_stop():
                print("[STOP] 대기 중 정지 요청: 초기 위치 복귀 후 종료", flush=True)
                requested_stop_and_return(driver, ros, initial_pos)
                break

            if not ros.motor_start:
                continue

            ros.motor_start = False
            ros.motor_stop = False
            cpr_running_event.set()

            try:
                user_stop_completed = run_once(
                    driver,
                    ros,
                    initial_pos,
                )
                if user_stop_completed:
                    print("[STOP] 초기 위치 복귀 확인 후 프로그램 종료", flush=True)
                    break

            except Exception as exc:
                print(f"[ERROR] {type(exc).__name__}: {exc}", flush=True)
                print("[FAULT LOCKOUT] 재시작 금지: 프로그램 종료 및 수동 점검", flush=True)
                break

            finally:
                cpr_running_event.clear()

            print(
                "\n[READY] r=다시 시작 | q=정지 및 종료"
            )

        keyboard_stop_event.set()
        keyboard_thread.join(timeout=0.5)
        restore_keyboard_terminal()




    except KeyboardInterrupt:

        print(

            "\n[CTRL+C] 초기 위치 복귀 후 프로그램을 종료합니다."

        )



        if servo_enabled:

            try:

                # Ctrl+C 수신 시 rclpy 기본 SIGINT 처리로 컨텍스트가 이미

                # 종료됐을 수 있다. ROS publish/spin을 사용하는 원래 객체 대신

                # 독립 복귀 어댑터를 전달해 RS485 명령이 막히지 않도록 한다.

                interrupt_stop_and_return(

                    driver,

                    _RecoveryWithoutRos(),

                    initial_pos,

                )

                print("[CTRL+C] 초기 위치 복귀 확인 완료, 종료", flush=True)

            except Exception as exc:

                print(

                    "[CTRL+C] 초기 위치 복귀 미완료(자동 복귀 중단): "

                    f"{type(exc).__name__}: {exc}"

                )



    finally:

        try:

            driver.stop()



            if (

                cfg.SERVO_OFF_ON_PROGRAM_EXIT

            ):

                driver.disable()



                print(

                    "[EXIT] Servo OFF 완료"

                )



        except Exception as exc:

            print(

                f"[WARN] 종료 처리 실패: "

                f"{exc}"

            )



        # Ctrl+C로 rclpy 컨텍스트가 먼저 무효화됐을 가능성이 있다.

        # ROS 정리 실패가 모터 정지/서보 OFF 처리를 가리지 않게 분리한다.

        try:

            ros.destroy_node()

        except Exception as exc:

            print(f"[WARN] ROS 노드 정리 실패: {exc}", flush=True)



        try:

            if rclpy.ok():

                rclpy.shutdown()

        except Exception as exc:

            print(f"[WARN] ROS 컨텍스트 정리 실패: {exc}", flush=True)





if __name__ == "__main__":

    main()
