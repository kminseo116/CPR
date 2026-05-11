import time
import minimalmodbus

import motor_config as cfg


# ============================================================
# 전역 상태
# ============================================================

current_trip_count = 0

# ============================================================
# Modbus 연결 생성 - RS485 Modbus 장치를 생성
# ============================================================

def create_motor_connection(port, slave_id):
    motor = minimalmodbus.Instrument(port, slave_id)

    motor.serial.baudrate = cfg.BAUDRATE
    motor.serial.bytesize = cfg.BYTESIZE
    motor.serial.parity   = cfg.PARITY
    motor.serial.stopbits = cfg.STOPBITS
    motor.serial.timeout  = cfg.TIMEOUT

    motor.mode = minimalmodbus.MODE_RTU
    motor.clear_buffers_before_each_transaction = True
    motor.close_port_after_each_call = False

    return motor


# ============================================================
# 기본 읽기 / 쓰기
# ============================================================

def write_u16(motor, addr, value):  #16비트 unsigned 값을 Modbus 레지스터에 쓰기
    motor.write_register(addr, value, functioncode=6)


def write_i32(motor, addr, value):  # 32비트 정수 값을 쓰는 함수/ 목표 위치, 속도지령, 가감속이 32비트
    if value < 0:
        value = (1 << 32) + int(value)

    low_word  = int(value) & 0xFFFF
    high_word = (int(value) >> 16) & 0xFFFF

    # ISL60-PR 현재 사용 환경에서 32bit word order는 LOW_HIGH로 확인됨
    words = [low_word, high_word]

    motor.write_registers(addr, words)


def read_u16(motor, addr):  # 16비트 unsigned 값 읽음
    return motor.read_register(addr, 0, functioncode=3)


def read_i16(motor, addr):  # 16비트 signed 값 읽음
    return motor.read_register(addr, 0, functioncode=3, signed=True)


def read_i32(motor, addr):  # 32비트 signed 값 읽음/ 절대값 위치
    regs = motor.read_registers(addr, 2, functioncode=3)

    low_word  = regs[0]
    high_word = regs[1]
   
    value = (high_word << 16) | low_word

    if value >= (1 << 31):
        value -= (1 << 32)

    return value


def rpm_to_command_speed(rpm):
    """모터 rpm → 지령단위/s 변환 (P10-25, P10-42 등에 사용)"""
    return int(float(rpm) * cfg.COMMAND_UNITS_PER_MOTOR_REV / 60.0)


# ============================================================
# 상태 읽기
# ============================================================

def read_current_position(motor):
    """P0B-07 Absolute position counter 읽기"""
    return read_i32(motor, cfg.ADDR_P0B_07)


def read_current_a(motor):
    """P0B-24 상전류 유효값 읽기"""
    return read_i16(motor, cfg.ADDR_P0B_24) * cfg.CURRENT_SCALE


# def read_bus_voltage(motor):
#     """P0B-26 DC bus voltage 읽기"""
#     return read_u16(motor, cfg.ADDR_P0B_26)


def is_position_in_limit(pos):
    return cfg.MIN_POSITION_CMD <= int(pos) <= cfg.MAX_POSITION_CMD


# ============================================================
# 안전 확인
# ============================================================

def check_fault(motor):
    fault_code = motor.read_register(cfg.ADDR_P02_34, 0, functioncode=3)
    if fault_code != 0:
        raise RuntimeError(f"[FAULT] 드라이버 고장 코드 발생: {fault_code}")


def check_current_safety(motor):        # 현재 전류를 읽고, 제한 전류보다 큰지 확인
    global current_trip_count

    current_a = read_current_a(motor)

    if current_a > cfg.MAX_CURRENT_A:
        current_trip_count += 1
        print(
            f"[WARN] 전류 초과 {current_trip_count}/{cfg.CURRENT_TRIP_COUNT_LIMIT}: "
            f"{current_a:.3f} A > {cfg.MAX_CURRENT_A:.3f} A"
        )
    else:
        current_trip_count = 0

    if current_trip_count >= cfg.CURRENT_TRIP_COUNT_LIMIT:
        raise RuntimeError(
            f"[SAFETY] 과전류 지속 감지: {current_a:.3f} A > {cfg.MAX_CURRENT_A:.3f} A"
        )


def check_position_safety(pos, label="position"):   # 현재 위치나 목표 위치가 설정 범위를 벗어나면 에러를 발생
    if not is_position_in_limit(pos):
        raise RuntimeError(
            f"[SAFETY] {label} 위치 제한 초과: {pos}, "
            f"limit=[{cfg.MIN_POSITION_CMD}, {cfg.MAX_POSITION_CMD}]"
        )


def check_status(motor, title="상태 확인"):
    pos       = read_current_position(motor)
    current_a = read_current_a(motor)
    speed     = read_i16(motor, cfg.ADDR_P0B_00)
    fault     = read_u16(motor, cfg.ADDR_P02_34)
    # bus_v     = read_bus_voltage(motor)

    print(
        f"[STATUS] {title} | "
        f"pos={pos} | speed={speed}rpm | "
        f"current={current_a:.3f}A | fault={fault}"
    )


# ============================================================
# 서보 ON / OFF
# ============================================================

def disable_overtravel_switches(motor):
    """테스트용 과행정 리미트 해제 (실제 볼스크류 연결 시 비권장)"""
    print("[SETUP] P-OT/N-OT 기능 해제")
    write_u16(motor, cfg.ADDR_P03_08, 0)
    time.sleep(0.1)
    write_u16(motor, cfg.ADDR_P03_10, 0)
    time.sleep(0.1)


def restore_di3_setting(motor):
    """DI3 = S-ON, Low 유효 설정"""
    print("[SETUP] DI3 = S-ON, Low 유효")
    write_u16(motor, cfg.ADDR_P03_06, 1)
    time.sleep(0.1)
    write_u16(motor, cfg.ADDR_P03_07, 0)
    time.sleep(0.1)


def servo_on_by_forced_di(motor):
    """강제 DI 방식 Servo ON (DI3 Bit2 = 0 → Low 유효 S-ON)"""
    print("[SERVO ON]")
    write_u16(motor, cfg.ADDR_P0D_18, 0x003B)
    time.sleep(0.1)
    write_u16(motor, cfg.ADDR_P0D_17, 1)
    time.sleep(0.5)


def servo_off_by_forced_di(motor):
    """Servo OFF"""
    print("[SERVO OFF]")
    write_u16(motor, cfg.ADDR_P0D_08, 256)
    time.sleep(0.1)
    write_u16(motor, cfg.ADDR_P0D_17, 0)
    time.sleep(0.3)


# ============================================================
# 정지
# ============================================================

def clear_serial_buffers(motor):
    """
    Ctrl+C나 통신 오류 후 시리얼 버퍼에 남은 응답 데이터를 정리합니다.
    """
    try:
        if motor.serial is not None and motor.serial.is_open:
            motor.serial.reset_input_buffer()
            motor.serial.reset_output_buffer()
    except Exception:
        pass

def stop_motion(motor):
    """일반 운전 정지 (P0D-08 = 256)"""
    print("[STOP]")
    write_u16(motor, cfg.ADDR_P0D_08, 256)
    time.sleep(0.1)

    try:
        write_i32(motor, cfg.ADDR_P10_42, 0)
    except Exception as e:
        print("[STOP] P10-42=0 쓰기 실패:", e)


def emergency_stop(motor):
    """즉시 정지 (P0D-08 = 512)"""
    print("[EMERGENCY STOP]")

    clear_serial_buffers(motor)
    time.sleep(0.02)

    write_u16(motor, cfg.ADDR_P0D_08, 512)
    time.sleep(0.1)

    try:
        write_i32(motor, cfg.ADDR_P10_42, 0)
    except Exception as e:
        print("[EMERGENCY STOP] P10-42=0 쓰기 실패:", e)


def safe_stop(motor):
    """긴급정지 + Servo OFF"""
    print("[SAFE STOP]")

    clear_serial_buffers(motor)
    time.sleep(0.05)

    try:
        emergency_stop(motor)
    except Exception as e:
        print("[SAFE STOP] 1차 긴급정지 실패:", e)

        clear_serial_buffers(motor)
        time.sleep(0.1)

        try:
            print("[SAFE STOP] 긴급정지 재시도")
            emergency_stop(motor)
        except Exception as e2:
            print("[SAFE STOP] 2차 긴급정지 실패:", e2)

    clear_serial_buffers(motor)
    time.sleep(0.05)

    try:
        servo_off_by_forced_di(motor)
    except Exception as e:
        print("[SAFE STOP] Servo OFF 실패:", e)


# ============================================================
# 위치모드 PP 설정 / 이동
# ============================================================

def setup_position_mode(motor, move_rpm):
    """PP 위치모드 설정. move_rpm: P10-25에 들어갈 이동 속도(rpm)"""
    print(f"[SETUP] PP 위치 모드 | {move_rpm} rpm")
    write_u16(motor, cfg.ADDR_P10_03, 1)    # 위치모드 설정
    time.sleep(0.01)

    write_i32(motor, cfg.ADDR_P10_23, rpm_to_command_speed(cfg.MAX_RPM_LIMIT))  # 최대 속도 설정
    time.sleep(0.01)

    position_speed = rpm_to_command_speed(move_rpm)
    accel = max(rpm_to_command_speed(move_rpm) * cfg.ACCEL_FACTOR, 10000)
    decel = max(rpm_to_command_speed(move_rpm) * cfg.DECEL_FACTOR, 10000)

    write_i32(motor, cfg.ADDR_P10_07, cfg.POSITION_TOLERANCE)
    time.sleep(0.01)
    write_i32(motor, cfg.ADDR_P10_25, position_speed)
    time.sleep(0.01)
    write_i32(motor, cfg.ADDR_P10_27, accel)
    time.sleep(0.01)
    write_i32(motor, cfg.ADDR_P10_29, decel)
    time.sleep(0.01)

    print(f"[PP PARAM] speed={position_speed}, accel={accel}, decel={decel}")


def move_absolute_position(motor, target_pos):
    """절대 위치 이동 (P10-14 설정 → P0D-08 = 7 실행)"""
    check_position_safety(target_pos, "target_pos")

    write_i32(motor, cfg.ADDR_P10_14, int(target_pos))
    time.sleep(0.001)
    write_u16(motor, cfg.ADDR_P0D_08, 0)
    time.sleep(0.001)
    write_u16(motor, cfg.ADDR_P0D_08, 7)
    time.sleep(0.001)


# ============================================================
# 속도모드 PV 설정
# ============================================================

def setup_speed_search_mode(motor):
    """PV 속도모드 설정 (접촉 위치 탐색용)"""
    print(f"[SETUP] PV 속도 탐색 모드 | {cfg.SEARCH_RPM} rpm")
    write_u16(motor, cfg.ADDR_P10_03, 3)
    time.sleep(0.01)
    write_i32(motor, cfg.ADDR_P10_23, rpm_to_command_speed(cfg.MAX_RPM_LIMIT))
    time.sleep(0.01)
    write_i32(motor, cfg.ADDR_P10_42, rpm_to_command_speed(cfg.SEARCH_RPM))
    time.sleep(0.01)


def start_forward_speed_run(motor):
    """속도 운전 시작 (P0D-08 = 8)"""
    print("[RUN] 정회전 속도 운전 시작")
    write_u16(motor, cfg.ADDR_P0D_08, 8)
    time.sleep(0.01)


# ============================================================
# 로그 출력
# ============================================================

def format_elapsed_time(seconds):
    """초 → mm:ss 형식 변환"""
    seconds = int(seconds)
    return f"{seconds // 60:02d}:{seconds % 60:02d}"


def print_motion_status(
    cycle,
    direction,
    target_pos,
    current_pos,
    error,
    current_a,
    speed_rpm,
    reached=False,
    cpr_start_time=None,
    instant_bpm=None,
    avg_bpm=None,
    cycle_dt=None,
):
    """모터 상태 출력. BPM 값은 motor_main.py에서 계산해 전달한다."""
    status = "REACHED" if reached else "MOVING"

    if cpr_start_time is not None:
        elapsed_sec = time.time() - cpr_start_time
        elapsed = format_elapsed_time(elapsed_sec)
    else:
        elapsed = "00:00"

    bpm_str = ""
    if instant_bpm is not None:
        bpm_error = instant_bpm - cfg.TARGET_BPM
        bpm_str += f"bpm={instant_bpm:.1f}({bpm_error:+.1f}) | "
    if avg_bpm is not None:
        bpm_str += f"avg={avg_bpm:.1f} | "
    if cycle_dt is not None:
        bpm_str += f"dt={cycle_dt:.3f}s | "

    print(
        f"[CPR {elapsed}] "
        f"count={cycle} | "
        f"{bpm_str}"
        f"target={target_pos} | "
        f"pos={current_pos} | "
        f"error={error} | "
        f"current={current_a:.3f}A | "
    )


# ============================================================
# 위치 도달 대기
# ============================================================

def wait_until_position_reached(
    motor,
    target_pos,
    timeout=None,
    cycle=0,
    direction="MOVE",
    speed_rpm=0,
    cpr_start_time=None,
    ros=None
):
    """
    목표 위치 도달까지 대기.
    - 로드셀에서 정지 요청 받으면 긴급정지 후 False 반환
    - LOG_PERIOD_S마다 핵심값 출력
    """
    if timeout is None:
        timeout = cfg.MOVE_TIMEOUT

    start_time    = time.time()
    last_log_time = 0.0

    while True:
        if ros is not None and ros.should_stop():
            print("[USER STOP] 위치 이동 중 사용자 정지 요청 감지")
            return False

        check_fault(motor)
        check_current_safety(motor)

        current_pos = read_current_position(motor)
        check_position_safety(current_pos, "current_pos")

        current_a = read_current_a(motor)

        if ros is not None:
            ros.publish_absolute_position(current_pos)
            ros.publish_current_a(current_a)
            ros.publish_compression_time()

        error = abs(int(target_pos) - int(current_pos))
        now       = time.time()

        if error <= cfg.POSITION_TOLERANCE:
            print_motion_status(
                cycle=cycle, direction=direction,
                target_pos=target_pos, current_pos=current_pos,
                error=error, current_a=current_a, speed_rpm=speed_rpm,
                reached=True, cpr_start_time=cpr_start_time
            )
            return True

        if now - last_log_time >= cfg.LOG_PERIOD_S:
            print_motion_status(
                cycle=cycle, direction=direction,
                target_pos=target_pos, current_pos=current_pos,
                error=error, current_a=current_a, speed_rpm=speed_rpm,
                reached=False, cpr_start_time=cpr_start_time
            )
            last_log_time = now

        if now - start_time > timeout:
            print(
                f"[TIMEOUT] {direction} | "
                f"target={target_pos} | pos={current_pos} | "
                f"error={error} | timeout={timeout}s"
            )
            return False

        time.sleep(0.005)
