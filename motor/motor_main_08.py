import time

import rclpy
from motor_ros_node_02 import MotorRosBridge

import motor_config as cfg
import motor_driver as drv
import motor_diagnostics as diag

# 접촉 힘 기준
CONTACT_FORCE_N = cfg.CONTACT_FORCE_N

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
    print("[GUIDE] /motor_stop=True 또는 /loadcell_stop_request=True 이면 즉시 정지합니다.")

    ros.publish_state("SEARCHING")

    start_pos = drv.read_current_position(motor)
    print(f"[SEARCH] 탐색 시작 위치 = {start_pos}")
    ros.publish_absolute_position(start_pos)

    drv.start_forward_speed_run(motor)

    while True:
        if ros.should_stop():
            drv.emergency_stop(motor)
            raise RuntimeError("[STOP] 탐색 중 ROS 정지 요청으로 중단")

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

        time.sleep(0.02)


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

    ros.publish_state("RECIPROCATING")

    drv.check_position_safety(start_pos, "recip start_pos")
    drv.check_position_safety(end_pos,   "recip end_pos")

    cpr_start_time = time.time()

    ros.cpr_start_time = cpr_start_time
    ros.compression_count = 0
    ros.compression_bpm = 0.0
    ros.last_compression_time = None

    for i in range(cfg.REPEAT_COUNT):
        print(f"\n========== 왕복 {i + 1}/{cfg.REPEAT_COUNT} ==========")

        if ros.should_stop():
            drv.emergency_stop(motor)
            break

        print(f"\n[RECIP {i + 1}] BACKWARD 시작")
        ros.publish_target_position(start_pos)
        drv.move_absolute_position(motor, start_pos)

        if not drv.wait_until_position_reached(
            motor, start_pos,
            cycle=i + 1, direction="BACKWARD",
            speed_rpm=cfg.RECIP_RPM, cpr_start_time=cpr_start_time,
            ros=ros
        ):
            break

        time.sleep(0.005)

        if ros.should_stop():
            drv.emergency_stop(motor)
            break

        print(f"\n[RECIP {i + 1}] FORWARD 시작")
        ros.publish_target_position(end_pos)
        drv.move_absolute_position(motor, end_pos)

        if not drv.wait_until_position_reached(
            motor, end_pos,
            cycle=i + 1, direction="FORWARD",
            speed_rpm=cfg.RECIP_RPM, cpr_start_time=cpr_start_time,
            ros=ros
        ):
            break

        # 왕복 1회 = 압박 1회
        ros.update_compression_count()

        time.sleep(0.005)

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
    print("[LIMIT] 모터 정지 요청시 긴급정지")

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

                run_sequence(motor, ros)

                print("\n[READY] 다시 /motor_start=True 를 기다립니다.")

    except KeyboardInterrupt:
        print("\n[KEYBOARD INTERRUPT] 사용자 Ctrl+C 중단")
        drv.safe_stop(motor)

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
