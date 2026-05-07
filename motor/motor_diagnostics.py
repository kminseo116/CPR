import motor_config as cfg
import motor_driver as drv


def decode_di_status(di_status):
    """
    DI 입력 상태 간단 해석.
    현재 코드에서는 DI3 Bit2를 Servo ON 조건으로 사용합니다.
    """
    di3 = (di_status >> 2) & 0x01
    di4 = (di_status >> 3) & 0x01
    di5 = (di_status >> 4) & 0x01

    print("\n[DI STATUS]")
    print(f"DI raw      = 0b{di_status:016b}")
    print(f"DI3 Bit2    = {di3}  # Servo ON 입력 확인용")
    print(f"DI4 Bit3    = {di4}  # P-OT 쪽 확인")
    print(f"DI5 Bit4    = {di5}  # N-OT 쪽 확인")

    if di3 == 1:
        print("[CHECK] DI3=1입니다. 현재 강제 DI 방식 기준으로 Servo ON 조건이 아닐 수 있습니다.")
    else:
        print("[OK] DI3=0입니다. Servo ON 입력 조건은 들어온 상태로 보입니다.")


def decode_bus_status(bus_status):
    """
    버스 상태워드 간단 해석.
    정확한 bit 의미는 매뉴얼 기준으로 추가 보완 가능합니다.
    """
    position_reached = bus_status & 0x0001
    speed_reached = (bus_status >> 1) & 0x0001

    print("\n[BUS STATUS]")
    print(f"bus raw          = 0b{bus_status:016b}")
    print(f"position_reached = {position_reached}")
    print(f"speed_reached    = {speed_reached}")


def explain_fault_code(fault_code):
    """
    P02-34 고장 코드 간단 대응.
    세부 코드는 매뉴얼 고장 코드표를 보면서 계속 추가하면 됩니다.
    """
    print("\n[FAULT CHECK]")
    print(f"fault_code = {fault_code}")

    if fault_code == 0:
        print("[OK] 현재 드라이버 고장 코드는 없습니다.")
        return

    print("[WARN] 고장 코드가 있습니다. 아래 항목을 우선 확인하세요.")

    common_checks = [
        "1. Servo ON 상태인지 확인",
        "2. P-OT/N-OT 과행정 입력이 들어왔는지 확인",
        "3. 엔코더 케이블/전원/커넥터 확인",
        "4. 브레이크가 풀렸는지 확인",
        "5. 목표 위치가 기계적 범위를 벗어나지 않았는지 확인",
        "6. 전원 재인가 후 같은 코드가 반복되는지 확인",
    ]

    for item in common_checks:
        print(item)


def diagnose_motor(motor):
    """
    통신은 되는 상태에서 모터 상태를 한 번에 진단합니다.

    확인 항목:
    - 고장 코드 P02-34
    - 현재 위치 P0B-07
    - 실제 속도 P0B-00
    - 전류 P0B-24
    - DI 입력 P0B-03
    - 버스 상태 P0B-04
    """
    print("\n================ MOTOR DIAGNOSE ================")

    try:
        fault_code = drv.read_u16(motor, cfg.ADDR_P02_34)
        actual_speed = drv.read_i16(motor, cfg.ADDR_P0B_00)
        di_status = drv.read_u16(motor, cfg.ADDR_P0B_03)
        bus_status = drv.read_u16(motor, cfg.ADDR_P0B_04)
        position = drv.read_current_position(motor)
        current_a = drv.read_current_a(motor)

        print("\n[BASIC STATUS]")
        print(f"position      = {position}")
        print(f"actual_speed  = {actual_speed} rpm")
        print(f"motor_current = {current_a:.3f} A")
        print(f"fault_code    = {fault_code}")

        explain_fault_code(fault_code)
        decode_di_status(di_status)
        decode_bus_status(bus_status)

        print("\n[POSITION LIMIT CHECK]")
        print(f"limit = [{cfg.MIN_POSITION_CMD}, {cfg.MAX_POSITION_CMD}]")

        if cfg.MIN_POSITION_CMD <= position <= cfg.MAX_POSITION_CMD:
            print("[OK] 현재 위치는 설정한 위치 제한 범위 안에 있습니다.")
        else:
            print("[WARN] 현재 위치가 설정한 위치 제한 범위를 벗어났습니다.")

        print("\n[CURRENT LIMIT CHECK]")
        print(f"current = {current_a:.3f} A")
        print(f"limit   = {cfg.MAX_CURRENT_A:.3f} A")

        if current_a <= cfg.MAX_CURRENT_A:
            print("[OK] 현재 전류는 설정한 제한값 이하입니다.")
        else:
            print("[WARN] 현재 전류가 제한값을 초과했습니다.")

    except Exception as e:
        print("\n[DIAG ERROR] 진단 중 통신 실패")
        print(f"error = {e}")
        print("\n[COMM CHECK LIST]")
        print("1. /dev/ttyUSB0 포트가 맞는지 확인")
        print("2. USB-RS485 어댑터 인식 확인")
        print("3. 485+ / 485- 배선 반대 여부 확인")
        print("4. GND 연결 여부 확인")
        print("5. SLAVE_ID 확인")
        print("6. BAUDRATE / PARITY / STOPBITS 확인")
        print("7. 종단저항 설정 확인")

    print("================================================\n")