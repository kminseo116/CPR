"""
motor_driver.py

ISL60-PR 모터 드라이버와 직접 통신하는 저수준 제어 함수 모음이다.

담당:
- Modbus 연결
- 레지스터 읽기/쓰기
- Servo ON/OFF
- 위치 모드 설정
- 속도 탐색 모드 설정
- 위치 이동
- 전류, 위치, fault 안전 확인
"""

import time
import minimalmodbus

import motor_config as cfg


current_trip_count = 0


def create_motor_connection(port, slave_id):
    """RS485 Modbus RTU 모터 연결 객체 생성."""
    motor = minimalmodbus.Instrument(port, slave_id)

    motor.serial.baudrate = cfg.BAUDRATE
    motor.serial.bytesize = cfg.BYTESIZE
    motor.serial.parity = cfg.PARITY
    motor.serial.stopbits = cfg.STOPBITS
    motor.serial.timeout = cfg.TIMEOUT

    motor.mode = minimalmodbus.MODE_RTU
    motor.clear_buffers_before_each_transaction = True
    motor.close_port_after_each_call = False

    return motor


def write_u16(motor, addr, value):
    """16bit unsigned register 쓰기."""
    motor.write_register(addr, value, functioncode=6)


def write_i32(motor, addr, value):
    """32bit signed integer register 쓰기."""
    if value < 0:
        value = (1 << 32) + int(value)

    low_word = int(value) & 0xFFFF
    high_word = (int(value) >> 16) & 0xFFFF

    words = [low_word, high_word]
    motor.write_registers(addr, words)


def read_u16(motor, addr):
    """16bit unsigned register 읽기."""
    return motor.read_register(addr, 0, functioncode=3)


def read_i16(motor, addr):
    """16bit signed register 읽기."""
    return motor.read_register(addr, 0, functioncode=3, signed=True)


def read_i32(motor, addr):
    """32bit signed integer register 읽기."""
    regs = motor.read_registers(addr, 2, functioncode=3)

    low_word = regs[0]
    high_word = regs[1]

    value = (high_word << 16) | low_word

    if value >= (1 << 31):
        value -= (1 << 32)

    return value


def rpm_to_command_speed(rpm):
    """rpm을 지령단위/s로 변환."""
    return int(float(rpm) * cfg.COMMAND_UNITS_PER_MOTOR_REV / 60.0)


def read_current_position(motor):
    """현재 모터 절대 위치 읽기."""
    return read_i32(motor, cfg.ADDR_P0B_07)


def read_current_a(motor):
    """현재 모터 전류 읽기."""
    return read_i16(motor, cfg.ADDR_P0B_24) * cfg.CURRENT_SCALE


def is_position_in_limit(pos):
    """위치가 안전 제한 안에 있는지 확인."""
    return cfg.MIN_POSITION_CMD <= int(pos) <= cfg.MAX_POSITION_CMD


def check_fault(motor):
    """드라이버 fault code 확인."""
    fault_code = motor.read_register(cfg.ADDR_P02_34, 0, functioncode=3)
    if fault_code != 0:
        raise RuntimeError(f"[FAULT] 드라이버 고장 코드 발생: {fault_code}")


def check_current_safety(motor):
    """전류가 제한값을 연속 초과하는지 확인."""
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


def check_position_safety(pos, label="position"):
    """현재 위치 또는 목표 위치가 안전 범위 안인지 확인."""
    if not is_position_in_limit(pos):
        raise RuntimeError(
            f"[SAFETY] {label} 위치 제한 초과: {pos}, "
            f"limit=[{cfg.MIN_POSITION_CMD}, {cfg.MAX_POSITION_CMD}]"
        )


def check_status(motor, title="상태 확인"):
    """현재 위치, 속도, 전류, fault 상태 출력."""
    pos = read_current_position(motor)
    current_a = read_current_a(motor)
    speed = read_i16(motor, cfg.ADDR_P0B_00)
    fault = read_u16(motor, cfg.ADDR_P02_34)

    print(
        f"[STATUS] {title} | "
        f"pos={pos} | speed={speed}rpm | "
        f"current={current_a:.3f}A | fault={fault}"
    )


def disable_overtravel_switches(motor):
    """테스트용 P-OT/N-OT 리미트 해제."""
    print("[SETUP] P-OT/N-OT 기능 해제")
    write_u16(motor, cfg.ADDR_P03_08, 0)
    time.sleep(0.1)
    write_u16(motor, cfg.ADDR_P03_10, 0)
    time.sleep(0.1)


def restore_di3_setting(motor):
    """DI3 = S-ON, Low 유효 설정."""
    print("[SETUP] DI3 = S-ON, Low 유효")
    write_u16(motor, cfg.ADDR_P03_06, 1)
    time.sleep(0.1)
    write_u16(motor, cfg.ADDR_P03_07, 0)
    time.sleep(0.1)


def servo_on_by_forced_di(motor):
    """강제 DI 방식 Servo ON."""
    print("[SERVO ON]")
    write_u16(motor, cfg.ADDR_P0D_18, 0x003B)
    time.sleep(0.1)
    write_u16(motor, cfg.ADDR_P0D_17, 1)
    time.sleep(0.5)


def servo_off_by_forced_di(motor):
    """Servo OFF."""
    print("[SERVO OFF]")
    write_u16(motor, cfg.ADDR_P0D_08, 256)
    time.sleep(0.1)
    write_u16(motor, cfg.ADDR_P0D_17, 0)
    time.sleep(0.3)


def clear_serial_buffers(motor):
    """시리얼 버퍼 정리."""
    try:
        if motor.serial is not None and motor.serial.is_open:
            motor.serial.reset_input_buffer()
            motor.serial.reset_output_buffer()
    except Exception:
        pass


def stop_motion(motor):
    """일반 운전 정지."""
    print("[STOP]")
    write_u16(motor, cfg.ADDR_P0D_08, 256)
    time.sleep(0.1)

    try:
        write_i32(motor, cfg.ADDR_P10_42, 0)
    except Exception as e:
        print("[STOP] P10-42=0 쓰기 실패:", e)


def emergency_stop(motor):
    """긴급 정지."""
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
    """긴급정지 후 Servo OFF."""
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


def setup_position_mode(motor, move_rpm):
    """PP 위치 모드 설정."""
    print(f"[SETUP] PP 위치 모드 | {move_rpm} rpm")

    write_u16(motor, cfg.ADDR_P10_03, 1)
    time.sleep(0.01)

    write_i32(motor, cfg.ADDR_P10_23, rpm_to_command_speed(cfg.MAX_RPM_LIMIT))
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
    """절대 위치 이동 명령."""
    check_position_safety(target_pos, "target_pos")

    write_i32(motor, cfg.ADDR_P10_14, int(target_pos))
    time.sleep(0.001)

    write_u16(motor, cfg.ADDR_P0D_08, 0)
    time.sleep(0.001)

    write_u16(motor, cfg.ADDR_P0D_08, 7)
    time.sleep(0.001)


def setup_speed_search_mode(motor):
    """PV 속도 모드 설정. 로봇 전진 방향으로 접촉 탐색."""
    if cfg.FORWARD_DIRECTION not in (-1, 1) or cfg.SEARCH_RPM <= 0:
        raise ValueError('FORWARD_DIRECTION은 -1 또는 1, SEARCH_RPM은 양수여야 합니다.')
    forward_rpm = cfg.FORWARD_DIRECTION * cfg.SEARCH_RPM
    print(f"[SETUP] PV 전진 탐색 모드 | {forward_rpm} rpm")

    write_u16(motor, cfg.ADDR_P10_03, 3)
    time.sleep(0.01)

    write_i32(motor, cfg.ADDR_P10_23, rpm_to_command_speed(cfg.MAX_RPM_LIMIT))
    time.sleep(0.01)

    write_i32(motor, cfg.ADDR_P10_42, rpm_to_command_speed(forward_rpm))
    time.sleep(0.01)


def start_forward_speed_run(motor):
    """setup_speed_search_mode에서 설정한 부호의 PV 속도 운전 시작."""
    print("[RUN] 로봇 전진 속도 운전 시작")
    write_u16(motor, cfg.ADDR_P0D_08, 8)
    time.sleep(0.01)


def format_elapsed_time(seconds):
    """초를 mm:ss 문자열로 변환."""
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
    """왕복 압박 상태 출력."""
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
        f"{direction} | "
        f"{status} | "
        f"target={target_pos} | "
        f"pos={current_pos} | "
        f"error={error} | "
        f"current={current_a:.3f}A"
    )


def wait_until_position_reached(
    motor,
    target_pos,
    timeout=None,
    cycle=0,
    direction="MOVE",
    speed_rpm=0,
    cpr_start_time=None,
    ros=None,
):
    """목표 위치 도달까지 대기."""
    if timeout is None:
        timeout = cfg.MOVE_TIMEOUT

    start_time = time.time()
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
            ros.publish_compression_status()

        error = abs(int(target_pos) - int(current_pos))
        now = time.time()

        if error <= cfg.POSITION_TOLERANCE:
            print_motion_status(
                cycle=cycle,
                direction=direction,
                target_pos=target_pos,
                current_pos=current_pos,
                error=error,
                current_a=current_a,
                speed_rpm=speed_rpm,
                reached=True,
                cpr_start_time=cpr_start_time,
            )
            return True

        if now - last_log_time >= cfg.LOG_PERIOD_S:
            print_motion_status(
                cycle=cycle,
                direction=direction,
                target_pos=target_pos,
                current_pos=current_pos,
                error=error,
                current_a=current_a,
                speed_rpm=speed_rpm,
                reached=False,
                cpr_start_time=cpr_start_time,
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
