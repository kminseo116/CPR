"""
motor_sequence.py

CPR 로봇 모터 동작의 전체 순서를 담당한다.

전체 흐름:
1. 초기 위치 이동
2. 로드셀 접촉 위치 탐색
3. 접촉 위치 기반 흉부 높이 추정
4. 기본 50 mm + 흉부 높이 보정으로 초기 압박 깊이 계산
5. 왕복 압박 수행
6. peak force로 다음 깊이 보정
7. recoil 확인
8. 정지 요청 시 초기 위치 복귀 후 Servo OFF
"""

import time
import rclpy

import motor_config as cfg
import motor_driver as drv

from motor_depth_control import (
    attach_peak_force_subscriber,
    calculate_initial_depth_mm,
    calculate_initial_to_contact_mm,
    check_recoil_force,
    consume_latest_peak_force,
    estimate_chest_height_mm,
    get_forward_target,
    get_manual_depth_mm,
    position_units_to_cm,
    update_depth_by_peak_force,
)


CONTACT_FORCE_N = cfg.CONTACT_FORCE_N


class UserStopRequested(Exception):
    """사용자 또는 안전 조건에 의한 정지 요청 예외."""
    pass


def spin_ros_once(ros):
    """제어 루프 내부에서도 ROS subscriber callback이 처리되도록 한다."""
    try:
        rclpy.spin_once(ros, timeout_sec=0.0)
    except Exception:
        pass


def return_to_initial_and_shutdown(motor, ros=None, reason="USER_STOP"):
    """
    정지 요청 발생 시 INITIAL_POS로 복귀 후 Servo OFF한다.
    """
    print("\n[USER STOP] 정지 요청:", reason)
    print(f"[USER STOP] INITIAL_POS={cfg.INITIAL_POS}로 복귀 후 Servo OFF")

    if ros is not None:
        ros.motor_stop = False
        ros.loadcell_stop_request = False

    try:
        drv.stop_motion(motor)
        time.sleep(0.3)
    except Exception as e:
        print(f"[WARN] stop_motion 실패: {e}")

    if ros is not None:
        try:
            ros.publish_state("RETURNING_INITIAL")
        except Exception as e:
            print(f"[WARN] ROS publish 실패: {e}")

    try:
        drv.check_position_safety(cfg.INITIAL_POS, "INITIAL_POS return")

        drv.setup_position_mode(motor, cfg.INITIAL_MOVE_RPM)
        drv.servo_on_by_forced_di(motor)
        time.sleep(0.2)

        print(f"[USER STOP] 초기 위치 복귀 시작: {cfg.INITIAL_POS}")
        drv.move_absolute_position(motor, cfg.INITIAL_POS)

        ok = drv.wait_until_position_reached(
            motor,
            cfg.INITIAL_POS,
            ros=None,
        )

        drv.stop_motion(motor)
        time.sleep(0.3)

        drv.servo_off_by_forced_di(motor)
        time.sleep(0.3)

        if ok:
            print("[USER STOP] 초기 위치 복귀 후 Servo OFF 완료")
            if ros is not None:
                ros.publish_state("STOPPED")
        else:
            print("[USER STOP] 초기 위치 복귀 실패")
            if ros is not None:
                ros.publish_state("SAFE_STOP")

        return ok

    except Exception as e:
        print(f"[ERROR] 초기 위치 복귀 중 오류: {e}")
        print("[ERROR] safe_stop 실행")

        try:
            drv.safe_stop(motor)
        except Exception as stop_e:
            print(f"[ERROR] safe_stop 실패: {stop_e}")

        if ros is not None:
            try:
                ros.publish_state("SAFE_STOP")
            except Exception:
                pass

        return False


def move_to_initial_position(motor, ros):
    """
    1단계: 압박봉을 초기 위치 INITIAL_POS로 이동한다.
    """
    print("\n=== 1단계: 지정 초기 위치로 이동 ===")
    print(f"[INITIAL] 목표 초기 위치 INITIAL_POS = {cfg.INITIAL_POS}")

    ros.publish_state("INITIAL_MOVING")

    drv.check_position_safety(cfg.INITIAL_POS, "INITIAL_POS")
    drv.setup_position_mode(motor, cfg.INITIAL_MOVE_RPM)

    drv.servo_on_by_forced_di(motor)
    drv.check_status(motor, "초기 위치 이동 전 상태")

    drv.move_absolute_position(motor, cfg.INITIAL_POS)

    ok = drv.wait_until_position_reached(
        motor,
        cfg.INITIAL_POS,
        ros=ros,
    )

    if not ok:
        raise RuntimeError("[ERROR] 초기 위치 이동 실패")

    print("[INITIAL] 초기 위치 도달 완료")

    drv.stop_motion(motor)
    time.sleep(0.3)

    drv.servo_off_by_forced_di(motor)
    time.sleep(0.5)


def search_until_loadcell_contact(motor, ros):
    """
    2단계: 로드셀 합산 힘을 이용해 흉부 접촉 위치를 찾는다.

    /loadcell_total >= CONTACT_FORCE_N 이 되면 현재 위치를 contact_pos로 저장한다.
    """
    print("\n=== 2단계: 정회전 탐색 시작 ===")
    print(f"[GUIDE] /loadcell_total >= {CONTACT_FORCE_N:.1f}N 이면 현재 위치를 접촉 위치로 저장합니다.")
    print("[GUIDE] /motor_stop=True 또는 /loadcell_stop_request=True 이면 초기 위치 복귀 후 종료합니다.")

    ros.publish_state("SEARCHING")

    start_pos = drv.read_current_position(motor)
    print(f"[SEARCH] 탐색 시작 위치 = {start_pos}")
    ros.publish_absolute_position(start_pos)

    drv.start_forward_speed_run(motor)

    while True:
        spin_ros_once(ros)

        if ros.should_stop():
            raise UserStopRequested("[STOP] 탐색 중 사용자 정지 요청")

        drv.check_fault(motor)
        drv.check_current_safety(motor)

        current_pos = drv.read_current_position(motor)
        current_a = drv.read_current_a(motor)

        drv.check_position_safety(current_pos, "search current_pos")

        ros.publish_absolute_position(current_pos)
        ros.publish_current_a(current_a)

        if ros.loadcell_total >= CONTACT_FORCE_N:
            captured_pos = drv.read_current_position(motor)

            print(
                f"[CONTACT] loadcell_total={ros.loadcell_total:.1f}N | "
                f"저장 위치 = {captured_pos}"
            )

            ros.publish_state("CONTACT_DETECTED")

            drv.stop_motion(motor)
            time.sleep(0.3)

            return int(captured_pos)

        time.sleep(0.001)


def reciprocating_motion(motor, ros, start_pos, initial_depth_mm):
    """
    3단계: 접촉 위치를 기준으로 왕복 압박을 수행한다.
    """
    current_depth_mm = float(initial_depth_mm)
    end_pos = get_forward_target(start_pos, current_depth_mm)

    print(f"\n=== 3단계: 자동 깊이 {current_depth_mm / 10.0:.1f} cm 왕복 운동 시작 ===")
    print("[GUIDE] /motor_stop=True 또는 로드셀 정지 요청 시 즉시 정지합니다.")
    print(f"[RECIP] START_POS = {start_pos}")
    print(f"[RECIP] INIT_END_POS = {end_pos}")
    print(f"[RECIP] 반복 횟수 = {cfg.REPEAT_COUNT}")
    print(f"[RECIP] 왕복 속도 = {cfg.RECIP_RPM} rpm")
    print(f"[RECIP] 위치 도달 허용 오차 = {cfg.POSITION_TOLERANCE} unit")

    ros.publish_state("Compressing")

    drv.check_position_safety(start_pos, "recip start_pos")
    drv.check_position_safety(end_pos, "recip end_pos")

    ros.reset_compression_status()
    cpr_start_time = ros.cpr_start_time

    LOOP_SLEEP_S = 0.001
    SAFETY_CHECK_PERIOD_S = 0.5
    CURRENT_READ_PERIOD_S = 1.0

    last_safety_check_time = time.time()
    last_current_read_time = 0.0
    current_a = 0.0

    target_pos = end_pos
    direction = "FORWARD"
    max_depth_cm_in_cycle = 0.0

    print(f"\n========== 왕복 {ros.compression_count + 1}/{cfg.REPEAT_COUNT} ==========")

    drv.move_absolute_position(motor, target_pos)
    forward_start_time = time.time()

    while ros.compression_count < cfg.REPEAT_COUNT:
        spin_ros_once(ros)

        if ros.should_stop():
            raise UserStopRequested("[STOP] 왕복 운동 중 사용자 정지 요청")

        now_loop = time.time()

        if now_loop - last_safety_check_time >= SAFETY_CHECK_PERIOD_S:
            drv.check_fault(motor)
            drv.check_current_safety(motor)
            last_safety_check_time = now_loop

        current_pos = drv.read_current_position(motor)
        drv.check_position_safety(current_pos, "recip current_pos")

        ros.publish_absolute_position(current_pos)

        depth_now_cm = position_units_to_cm(abs(current_pos - start_pos))

        if direction == "FORWARD":
            if depth_now_cm > max_depth_cm_in_cycle:
                max_depth_cm_in_cycle = depth_now_cm

        error = abs(target_pos - current_pos)

        forward_elapsed = now_loop - forward_start_time if direction == "FORWARD" else 0.0

        if error <= cfg.POSITION_TOLERANCE or (
            direction == "FORWARD" and forward_elapsed >= cfg.MAX_ALLOWED_CYCLE_DT
        ):
            if direction == "FORWARD":
                now = time.time()

                prev_time = ros.last_compression_time
                next_count = ros.compression_count + 1

                if prev_time is None:
                    cycle_dt = 0.0
                    instant_bpm = 0.0
                else:
                    cycle_dt = now - prev_time
                    instant_bpm = 60.0 / cycle_dt if cycle_dt > 0 else 0.0

                elapsed = now - cpr_start_time
                avg_bpm = next_count * 60.0 / elapsed if elapsed > 0 else 0.0

                ros.update_compression_count(
                    bpm=avg_bpm,
                    depth_cm=max_depth_cm_in_cycle,
                )

                if now - last_current_read_time >= CURRENT_READ_PERIOD_S:
                    try:
                        current_a = drv.read_current_a(motor)
                    except Exception as e:
                        print(f"[WARN] current read failed: {e}")
                    last_current_read_time = now

                drv.print_motion_status(
                    cycle=ros.compression_count,
                    direction=direction,
                    target_pos=target_pos,
                    current_pos=current_pos,
                    error=error,
                    current_a=current_a,
                    speed_rpm=cfg.RECIP_RPM,
                    reached=True,
                    cpr_start_time=cpr_start_time,
                    instant_bpm=instant_bpm,
                    avg_bpm=avg_bpm,
                    cycle_dt=cycle_dt,
                )

                if ros.compression_count >= cfg.REPEAT_COUNT:
                    break

                target_pos = start_pos
                direction = "BACKWARD"
                drv.move_absolute_position(motor, target_pos)

            else:
                # BACKWARD 완료 후 recoil 확인
                recoil_ok, recoil_reason = check_recoil_force(ros.loadcell_total)

                if not recoil_ok:
                    print(
                        f"[RECOIL WARN] {recoil_reason} | "
                        f"loadcell_total={ros.loadcell_total:.1f} N"
                    )

                # 최신 peak force를 가져와 다음 깊이 보정
                peak_force_n = consume_latest_peak_force(ros)

                new_depth_mm, should_stop, reason = update_depth_by_peak_force(
                    current_depth_mm,
                    peak_force_n,
                )

                print(
                    f"[DEPTH UPDATE] peak={peak_force_n} N | "
                    f"depth={current_depth_mm:.1f}->{new_depth_mm:.1f} mm | "
                    f"reason={reason}"
                )

                current_depth_mm = new_depth_mm

                if should_stop:
                    raise UserStopRequested(f"[STOP] {reason}")

                target_pos = get_forward_target(start_pos, current_depth_mm)
                drv.check_position_safety(target_pos, "updated recip end_pos")

                direction = "FORWARD"
                max_depth_cm_in_cycle = 0.0

                next_cycle = ros.compression_count + 1
                print(f"\n========== 왕복 {next_cycle}/{cfg.REPEAT_COUNT} ==========")

                drv.move_absolute_position(motor, target_pos)
                forward_start_time = time.time()

        time.sleep(LOOP_SLEEP_S)

    print("\n=== 왕복 운동 종료 ===")


def run_sequence(motor, ros):
    """
    전체 CPR sequence 실행 함수.
    motor_main.py에서 이 함수만 호출하면 전체 동작이 시작된다.
    """
    print("=== 안전 제한 기반 자동 깊이 왕복 알고리즘 시작 ===")
    ros.publish_state("SETUP")

    attach_peak_force_subscriber(ros)

    print(f"[SETTING] INITIAL_POS={cfg.INITIAL_POS}")
    print(f"[SETTING] INITIAL_MOVE_RPM={cfg.INITIAL_MOVE_RPM}")
    print(f"[SETTING] SEARCH_RPM={cfg.SEARCH_RPM}")
    print(f"[SETTING] RECIP_RPM={cfg.RECIP_RPM}")
    print(f"[SETTING] MOTOR_UNITS_PER_MM={cfg.MOTOR_UNITS_PER_MM:.1f}")
    print(f"[SETTING] AUTO_DEPTH={cfg.USE_AUTO_COMPRESSION_DEPTH}")
    print(f"[LIMIT] MAX_CURRENT_A={cfg.MAX_CURRENT_A} A")
    print(f"[LIMIT] position=[{cfg.MIN_POSITION_CMD}, {cfg.MAX_POSITION_CMD}]")

    print("[SETUP] P02-00=9 Modbus 버스 모드")
    drv.write_u16(motor, cfg.ADDR_P02_00, 9)
    time.sleep(0.1)

    if cfg.KEEP_HARDWARE_LIMITS:
        print("[SETUP] 하드웨어 리미트 DI4/DI5 유지")
    else:
        drv.disable_overtravel_switches(motor)

    drv.restore_di3_setting(motor)

    if cfg.MOVE_TO_INITIAL_ON_START:
        move_to_initial_position(motor, ros)

    print("\n=== 2단계 준비: PV 속도모드 전환 ===")
    drv.setup_speed_search_mode(motor)
    drv.servo_on_by_forced_di(motor)
    drv.check_status(motor, "PV 탐색 Servo ON 후 상태")

    contact_pos = search_until_loadcell_contact(motor, ros)

    drv.servo_off_by_forced_di(motor)
    time.sleep(0.3)

    start_pos = int(contact_pos)

    if cfg.USE_AUTO_COMPRESSION_DEPTH:
        initial_to_contact_mm = calculate_initial_to_contact_mm(
            cfg.INITIAL_POS,
            contact_pos
        )

        chest_height_mm = estimate_chest_height_mm(
            cfg.INITIAL_POS,
            contact_pos
        )

        initial_depth_mm = calculate_initial_depth_mm(chest_height_mm)

    else:
        initial_to_contact_mm = 0.0
        chest_height_mm = 0.0
        initial_depth_mm = get_manual_depth_mm()

    initial_end_pos = get_forward_target(start_pos, initial_depth_mm)
    travel_units = int(abs(initial_end_pos - start_pos))

    print("\n=== 3단계 준비: 자동 압박 깊이 계산 ===")
    print(f"[CONTACT_POS]              {contact_pos}")
    print(f"[INITIAL_POS]              {cfg.INITIAL_POS}")
    print(f"[INITIAL_PLATE_HEIGHT]     {cfg.INITIAL_PLATE_HEIGHT_FROM_SUPPORT_MM:.1f} mm")
    print(f"[INITIAL_TO_CONTACT]       {initial_to_contact_mm:.1f} mm")
    print(f"[ESTIMATED_CHEST_HEIGHT]   {chest_height_mm:.1f} mm")
    print(f"[TARGET_DEPTH]             {initial_depth_mm:.1f} mm ({initial_depth_mm / 10.0:.1f} cm)")
    print(f"[TRAVEL_UNITS]             {travel_units}")
    print(f"[START_POS]                {start_pos}")
    print(f"[INIT_END_POS]             {initial_end_pos}")

    drv.check_position_safety(start_pos, "start_pos")
    drv.check_position_safety(initial_end_pos, "initial_end_pos")

    drv.setup_position_mode(motor, cfg.RECIP_RPM)
    drv.servo_on_by_forced_di(motor)
    drv.check_status(motor, "PP 왕복 Servo ON 후 상태")

    reciprocating_motion(motor, ros, start_pos, initial_depth_mm)

    drv.stop_motion(motor)
    time.sleep(0.3)

    drv.servo_off_by_forced_di(motor)

    drv.check_status(motor, "전체 동작 종료 후 상태")
    ros.publish_state("STOPPED")

    print("\n=== 전체 동작 완료 ===")