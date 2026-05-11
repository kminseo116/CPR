import time

import rclpy
from motor_ros_node_02 import MotorRosBridge

import motor_config as cfg
import motor_driver as drv
import motor_diagnostics as diag

# 접촉 힘 기준
CONTACT_FORCE_N = cfg.CONTACT_FORCE_N

# ============================================================
# 사용자 정지 요청 예외
# ============================================================

class UserStopRequested(Exception):
    """
    사용자의 정지 요청 경우. 바로 Servo OFF 하지 않고 초기 위치로 복귀한 뒤 종료한다.
    """
    pass


def return_to_initial_and_shutdown(motor, ros=None, reason="USER_STOP"):
    print("\n[USER STOP] 정지 요청:", reason)
    print(f"[USER STOP] INITIAL_POS={cfg.INITIAL_POS}로 복귀 후 Servo OFF")

    # 복귀 중 다시 stop으로 판단되지 않도록 플래그 초기화
    if ros is not None:
        ros.motor_stop = False
        ros.loadcell_stop_request = False

    # 1. 현재 동작 먼저 정지
    try:
        drv.stop_motion(motor)
        time.sleep(0.3)
    except Exception as e:
        print(f"[WARN] stop_motion 실패: {e}")

    # 2. ROS 상태 publish는 실패해도 무시
    if ros is not None:
        try:
            ros.publish_state("RETURNING_INITIAL")
            ros.publish_target_position(cfg.INITIAL_POS)
        except Exception as e:
            print(f"[WARN] ROS publish 실패: {e}")

    try:
        # 3. 초기 위치로 복귀
        drv.check_position_safety(cfg.INITIAL_POS, "INITIAL_POS return")

        drv.setup_position_mode(motor, cfg.INITIAL_MOVE_RPM)
        drv.servo_on_by_forced_di(motor)
        time.sleep(0.2)

        print(f"[USER STOP] 초기 위치 복귀 시작: {cfg.INITIAL_POS}")
        drv.move_absolute_position(motor, cfg.INITIAL_POS)

        # 중요: 복귀 중에는 ros=None
        # stop flag 때문에 다시 emergency_stop 걸리는 것 방지
        ok = drv.wait_until_position_reached(
            motor,
            cfg.INITIAL_POS,
            ros=None
        )

        # 4. 복귀 후 정지 + Servo OFF
        drv.stop_motion(motor)
        time.sleep(0.3)
        drv.servo_off_by_forced_di(motor)
        time.sleep(0.3)

        if ok:
            print("[USER STOP] 초기 위치 복귀 후 Servo OFF 완료")
            if ros is not None:
                try:
                    ros.publish_state("STOPPED")
                except Exception:
                    pass
        else:
            print("[USER STOP] 초기 위치 복귀 실패")
            if ros is not None:
                try:
                    ros.publish_state("SAFE_STOP")
                except Exception:
                    pass

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

# ============================================================
# 1단계: 초기 위치 이동
# ============================================================

def move_to_initial_position(motor, ros):
    print("\n=== 1단계: 지정 초기 위치로 이동 ===")
    print(f"[INITIAL] 목표 초기 위치 INITIAL_POS = {cfg.INITIAL_POS}")

    ros.publish_state("INITIAL_MOVING")

    drv.check_position_safety(cfg.INITIAL_POS, "INITIAL_POS")
    drv.setup_position_mode(motor, cfg.INITIAL_MOVE_RPM)

    drv.servo_on_by_forced_di(motor)
    drv.check_status(motor, "초기 위치 이동 전 상태")

    ros.publish_target_position(cfg.INITIAL_POS)
    drv.move_absolute_position(motor, cfg.INITIAL_POS)

    ok = drv.wait_until_position_reached(motor,cfg.INITIAL_POS,ros=ros)
    if not ok:
        raise RuntimeError("[ERROR] 초기 위치 이동 실패")

    print("[INITIAL] 초기 위치 도달 완료")

    drv.stop_motion(motor)
    time.sleep(0.3)

    drv.servo_off_by_forced_di(motor)
    time.sleep(0.5)


# ============================================================
# 2단계: 접촉 위치 탐색
# ============================================================

def search_until_loadcell_contact(motor, ros):
    print("\n=== 2단계: 정회전 탐색 시작 ===")
    print(f"[GUIDE] /loadcell_total >= {CONTACT_FORCE_N:.1f}N 이면 현재 위치를 접촉 위치로 저장합니다.")
    print("[GUIDE] /motor_stop=True 또는 /loadcell_stop_request=True 이면 초기 위치 복귀 후 종료합니다.")

    ros.publish_state("SEARCHING")

    start_pos = drv.read_current_position(motor)
    print(f"[SEARCH] 탐색 시작 위치 = {start_pos}")
    ros.publish_absolute_position(start_pos)

    drv.start_forward_speed_run(motor)

    while True:
        if ros.should_stop():
            raise UserStopRequested("[STOP] 탐색 중 사용자 정지 요청")

        drv.check_fault(motor)
        drv.check_current_safety(motor)

        current_pos = drv.read_current_position(motor)
        current_a = drv.read_current_a(motor)

        drv.check_position_safety(current_pos, "search current_pos")

        ros.publish_absolute_position(current_pos)
        ros.publish_current_a(current_a)

        # 기존 Enter 역할
        if ros.loadcell_total >= CONTACT_FORCE_N:
            captured_pos = drv.read_current_position(motor)

            print(
                f"[CONTACT] loadcell_total={ros.loadcell_total:.1f}N | "
                f"저장 위치 = {captured_pos}"
            )

            ros.publish_state("CONTACT_DETECTED")
            ros.publish_contact_position(captured_pos)

            drv.stop_motion(motor)
            time.sleep(0.3)

            return int(captured_pos)

        time.sleep(0.001)


# ============================================================
# 3단계: 왕복 운동
# ============================================================

def reciprocating_motion(motor, ros, start_pos, end_pos, depth_cm):
    print(f"\n=== 3단계: {depth_cm:.1f} cm 왕복 운동 시작 ===")
    print("[GUIDE] /motor_stop=True 또는 로드셀 정지 요청 시 즉시 정지합니다.")
    print(f"[RECIP] START_POS = {start_pos}")
    print(f"[RECIP] END_POS   = {end_pos}")
    print(f"[RECIP] 반복 횟수 = {cfg.REPEAT_COUNT}")
    print(f"[RECIP] 왕복 속도 = {cfg.RECIP_RPM} rpm")
    print(f"[RECIP] 위치 도달 허용 오차 = {cfg.POSITION_TOLERANCE} unit")

    ros.publish_state("RECIPROCATING")

    drv.check_position_safety(start_pos, "recip start_pos")
    drv.check_position_safety(end_pos, "recip end_pos")

    cpr_start_time = time.time()

    ros.cpr_start_time = cpr_start_time
    ros.compression_count = 0
    ros.compression_bpm = 0.0
    ros.last_compression_time = None

    # 왕복 루프 주기
    # 값이 너무 크면 방향 전환이 늦어져 bpm이 떨어짐
    LOOP_SLEEP_S = 0.001

    # 처음에는 압박 방향, 즉 end_pos 방향으로 이동
    target_pos = end_pos
    direction = "FORWARD"

    # 첫 번째 왕복 로그 출력
    print(f"\n========== 왕복 {ros.compression_count + 1}/{cfg.REPEAT_COUNT} ==========")
    print(f"[RECIP {ros.compression_count + 1}] FORWARD 시작")

    ros.publish_target_position(target_pos)
    drv.move_absolute_position(motor, target_pos)

    loop_count = 0

    while ros.compression_count < cfg.REPEAT_COUNT:
        loop_count += 1

        if ros.should_stop():
            raise UserStopRequested("[STOP] 왕복 운동 중 사용자 정지 요청")

        # 매 루프마다 하지 말고 10번에 한 번만 확인
        if loop_count % 10 == 0:
            drv.check_fault(motor)
            drv.check_current_safety(motor)

        current_pos = drv.read_current_position(motor)

        drv.check_position_safety(current_pos, "recip current_pos")

        ros.publish_absolute_position(current_pos)

        error = abs(target_pos - current_pos)

        # 목표 위치 근처에 들어오면 완전 정지까지 기다리지 않고 바로 반대 목표 지령
        if error <= cfg.POSITION_TOLERANCE:

            if direction == "FORWARD":
                # 압박 끝 지점에 도달한 것으로 보고 압박 1회 카운트
                ros.update_compression_count()

                drv.print_motion_status(
                    cycle=ros.compression_count,
                    direction=direction,
                    target_pos=target_pos,
                    current_pos=current_pos,
                    error=error,
                    current_a=0.0,
                    speed_rpm=cfg.RECIP_RPM,
                    reached=True,
                    cpr_start_time=cpr_start_time
                )

                # 목표 횟수 완료 시 종료
                if ros.compression_count >= cfg.REPEAT_COUNT:
                    break

                # 복귀 방향으로 전환
                target_pos = start_pos
                direction = "BACKWARD"

                print(f"[RECIP {ros.compression_count}] BACKWARD 시작")

                ros.publish_target_position(target_pos)
                drv.move_absolute_position(motor, target_pos)

            else:
                # 복귀 완료 후 다음 압박 사이클 시작
                next_cycle = ros.compression_count + 1

                target_pos = end_pos
                direction = "FORWARD"

                print(f"\n========== 왕복 {next_cycle}/{cfg.REPEAT_COUNT} ==========")
                print(f"[RECIP {next_cycle}] FORWARD 시작")

                ros.publish_target_position(target_pos)
                drv.move_absolute_position(motor, target_pos)

        time.sleep(LOOP_SLEEP_S)

    print("\n=== 왕복 운동 종료 ===")


# ============================================================
# 압박 깊이 입력 / 변환
# ============================================================
# 추후 UI에서 /motor_set_depth_cm 토픽 수신하는 것으로 변경해야함.
def get_travel_units():
    if not cfg.USE_INPUT_COMPRESSION_DEPTH:
        return cfg.DEFAULT_TRAVEL_MM / 10.0, cfg.DEFAULT_TRAVEL_MM, cfg.DEFAULT_TRAVEL_UNITS

    while True:
        try:
            # 터미널에서 숫자를 입력하고 Enter를 누르면 그 값을 압박 깊이(cm)로 받는 부분
            depth_cm = round(float(input("\n[INPUT] 압박 깊이(cm) 입력 예: 6.0 > ")), 1)
        except ValueError:
            print("[ERROR] 숫자로 입력하세요.")
            continue

        if depth_cm <= 0:
            print("[ERROR] 0보다 큰 값을 입력하세요.")
            continue

        if depth_cm > cfg.MAX_COMPRESSION_DEPTH_CM:
            print(f"[ERROR] 최대 {cfg.MAX_COMPRESSION_DEPTH_CM:.1f}cm 이하로 입력하세요.")
            continue

        travel_mm = depth_cm * 10.0
        screw_rev = travel_mm / cfg.SCREW_LEAD_MM_PER_REV
        motor_rev = screw_rev * cfg.GEAR_MOTOR_REV_PER_SCREW_REV
        travel_units = int(motor_rev * cfg.COMMAND_UNITS_PER_MOTOR_REV)

        return depth_cm, travel_mm, travel_units


# ============================================================
# 전체 실행 순서
# ============================================================

def run_sequence(motor, ros):
    print(f"=== 안전 제한 기반 왕복 알고리즘 시작 ===")
    ros.publish_state("SETUP")

    print(f"[SETTING] INITIAL_POS={cfg.INITIAL_POS}")
    print(f"[SETTING] INITIAL_MOVE_RPM={cfg.INITIAL_MOVE_RPM}")
    print(f"[SETTING] SEARCH_RPM={cfg.SEARCH_RPM}")
    print(f"[SETTING] RECIP_RPM={cfg.RECIP_RPM}")
    print(f"[SETTING] SCREW_LEAD_MM_PER_REV={cfg.SCREW_LEAD_MM_PER_REV}")
    print(f"[SETTING] GEAR_MOTOR_REV_PER_SCREW_REV={cfg.GEAR_MOTOR_REV_PER_SCREW_REV}")
    if not cfg.USE_INPUT_COMPRESSION_DEPTH:
        print(f"[SETTING] DEFAULT_TRAVEL_MM={cfg.DEFAULT_TRAVEL_MM}")
        print(f"[SETTING] DEFAULT_TRAVEL_SCREW_REV={cfg.DEFAULT_TRAVEL_SCREW_REV}")
        print(f"[SETTING] DEFAULT_TRAVEL_MOTOR_REV={cfg.DEFAULT_TRAVEL_MOTOR_REV}")
        print(f"[SETTING] DEFAULT_TRAVEL_UNITS={cfg.DEFAULT_TRAVEL_UNITS}")
    else:
        print("[SETTING] 압박 깊이 입력 모드 사용")
    print(f"[LIMIT] MAX_CURRENT_A={cfg.MAX_CURRENT_A} A")
    print(f"[LIMIT] position=[{cfg.MIN_POSITION_CMD}, {cfg.MAX_POSITION_CMD}]")
    print("[LIMIT] 사용자 정지 요청시 초기 위치 복귀 후 Servo OFF")

    # Modbus 버스 모드 설정
    print("[SETUP] P02-00=9 Modbus 버스 모드")
    drv.write_u16(motor, cfg.ADDR_P02_00, 9)
    time.sleep(0.1)

    # 하드웨어 리미트 설정
    if cfg.KEEP_HARDWARE_LIMITS:
        print("[SETUP] 하드웨어 리미트 DI4/DI5 유지")
    else:
        drv.disable_overtravel_switches(motor)

    drv.restore_di3_setting(motor)

    # --- 1단계: 초기 위치 이동 ---
    if cfg.MOVE_TO_INITIAL_ON_START:
        move_to_initial_position(motor, ros)

    depth_cm, travel_mm, travel_units = get_travel_units()

    # --- 2단계: PV 속도모드로 접촉 위치 탐색 ---
    print("\n=== 2단계 준비: PV 속도모드 전환 ===")

    drv.setup_speed_search_mode(motor)
    drv.servo_on_by_forced_di(motor)
    drv.check_status(motor, "PV 탐색 Servo ON 후 상태")

    contact_pos = search_until_loadcell_contact(motor, ros)

    drv.servo_off_by_forced_di(motor)
    time.sleep(0.3)

    # --- 3단계: 왕복 위치 계산 및 PP 위치모드 왕복 ---
    start_pos = int(contact_pos)
    end_pos = int(contact_pos + travel_units)

    ros.publish_contact_position(contact_pos)
    ros.publish_target_position(end_pos)

    print(f"\n=== 3단계 준비: {depth_cm:.1f} cm 왕복 위치 계산 ===")
    print(f"[CONTACT_POS]  {contact_pos}")
    print(f"[DEPTH_CM]     {depth_cm:.1f}")
    print(f"[TRAVEL_MM]    {travel_mm:.1f}")
    print(f"[TRAVEL_UNITS] {travel_units}")
    print(f"[START_POS]    {start_pos}")
    print(f"[END_POS]      {end_pos}")

    drv.check_position_safety(start_pos, "start_pos")
    drv.check_position_safety(end_pos,   "end_pos")

    drv.setup_position_mode(motor, cfg.RECIP_RPM)
    drv.servo_on_by_forced_di(motor)
    drv.check_status(motor, "PP 왕복 Servo ON 후 상태")

    reciprocating_motion(motor, ros, start_pos, end_pos, depth_cm)

    drv.stop_motion(motor)
    time.sleep(0.3)
    drv.servo_off_by_forced_di(motor)

    drv.check_status(motor, "전체 동작 종료 후 상태")
    ros.publish_state("STOPPED")
    print("\n=== 전체 동작 완료 ===")


# ============================================================
# 메인
# ============================================================

def main():
    rclpy.init()

    ros = MotorRosBridge()
    motor = drv.create_motor_connection(cfg.PORT, cfg.SLAVE_ID)

    try:
        print("\n[READY] /motor_start=True 를 기다립니다.")
        print('실행: ros2 topic pub /motor_start std_msgs/Bool "data: true" --once')
        print('정지: ros2 topic pub /motor_stop std_msgs/Bool "data: true" --once\n')

        while rclpy.ok():
            rclpy.spin_once(ros, timeout_sec=0.1)

            if ros.motor_start:
                ros.motor_start = False
                ros.motor_stop = False

                try:
                    run_sequence(motor, ros)

                except UserStopRequested as e:
                    print("\n[USER STOP]", e)
                    return_to_initial_and_shutdown(motor, ros, reason=str(e))

                print("\n[READY] 다시 /motor_start=True 를 기다립니다.")

    except KeyboardInterrupt:
        print("\n[KEYBOARD INTERRUPT] 사용자 Ctrl+C 중단")
        drv.safe_stop(motor)

    except UserStopRequested as e:
        print("\n[USER STOP]", e)
        return_to_initial_and_shutdown(motor, ros, reason=str(e))

    except Exception as e:
        print("\n[ERROR] 오류 발생:", e)

        try:
            diag.diagnose_motor(motor)
        except Exception as diag_e:
            print("[DIAG ERROR]", diag_e)

        drv.safe_stop(motor)
        ros.publish_state("SAFE_STOP")

    finally:
        ros.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
