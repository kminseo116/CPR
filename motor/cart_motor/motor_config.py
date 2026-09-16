"""cart_motor의 제어·드라이버·진단 코드에서 참조하는 설정.

시작/종료 절대 위치와 이동 속도 등 자주 바꾸는 값은 상단에 모았습니다.
"""

import serial


# ===== 자주 바꾸는 설정 =====
START_POS = 9000          # 시작 절대 위치 [unit]: 사용자가 정수로 지정
END_POS = -31489            # 종료 절대 위치 [unit]: 사용자가 정수로 지정
MOVE_RPM = 100            # 전진·후진 속도 [rpm]
FORWARD_DIRECTION = -1    # 실기 확인: 음수 방향이 로봇 전진, 양수 방향이 후진
ARRIVAL_TOLERANCE = 300   # unit; 300 unit = 기존 기어/리드 기준 0.2 mm
ARRIVAL_SAMPLES = 3       # 연속 확인 횟수
POLL_S = 0.02
SAFETY_PERIOD_S = 0.2



# 통신 설정
PORT = "/dev/ttyUSB0"
SLAVE_ID = 1
BAUDRATE = 19200
BYTESIZE = 8
PARITY = serial.PARITY_EVEN
STOPBITS = 1
TIMEOUT = 0.5


# 위치 운전 설정
ACCEL_FACTOR = 10
DECEL_FACTOR = 10
MAX_RPM_LIMIT = 3000       # 위치 모드 최대 속도 [rpm]
MOVE_TIMEOUT = 30.0       # 이동 제한 시간 [s]
POSITION_TOLERANCE = 2500 # 드라이버에 기록하는 위치 허용오차 [unit]
# 소프트웨어 도착 판정에는 상단의 ARRIVAL_TOLERANCE를 사용합니다.


# 안전 제한
MIN_POSITION_CMD = -300000
MAX_POSITION_CMD = 300000
MAX_CURRENT_A = 130.0
CURRENT_SCALE = 0.1
CURRENT_TRIP_COUNT_LIMIT = 5


# 기계 환산: 모터 1.5회전 = 볼스크류 1회전 = 10 mm
COMMAND_UNITS_PER_MOTOR_REV = 10000
SCREW_LEAD_MM_PER_REV = 10.0
GEAR_MOTOR_REV_PER_SCREW_REV = 1.5
MOTOR_UNITS_PER_MM = (
    COMMAND_UNITS_PER_MOTOR_REV * GEAR_MOTOR_REV_PER_SCREW_REV
) / SCREW_LEAD_MM_PER_REV  # 1500 unit/mm


# 기존 드라이버 보조 함수에서 참조하는 설정
# simple_motor.py에서는 해당 탐색/로그/대기 함수를 호출하지 않습니다.
SEARCH_RPM = 200          # 전진 탐색 속력 (양수 rpm); 부호는 FORWARD_DIRECTION 적용
TARGET_BPM = 107.0        # print_motion_status
LOG_PERIOD_S = 0.2        # wait_until_position_reached


# Modbus 레지스터 주소: 제어 및 진단 함수에서 사용
# P02 그룹
ADDR_P02_00 = 0x0200
ADDR_P02_34 = 0x0222

# P03 그룹 (08/10은 드라이버의 리미트 해제 보조 함수에서 참조)
ADDR_P03_06 = 0x0306
ADDR_P03_07 = 0x0307
ADDR_P03_08 = 0x0308
ADDR_P03_10 = 0x030A

# P0B 그룹
ADDR_P0B_00 = 0x0B00
ADDR_P0B_03 = 0x0B03
ADDR_P0B_04 = 0x0B04
ADDR_P0B_07 = 0x0B07
ADDR_P0B_24 = 0x0B18
ADDR_P0B_53 = 0x0B35

# P0D 그룹
ADDR_P0D_08 = 0x0D08
ADDR_P0D_17 = 0x0D11
ADDR_P0D_18 = 0x0D12

# P10 그룹
ADDR_P10_03 = 0x1003
ADDR_P10_07 = 0x1007
ADDR_P10_14 = 0x100E
ADDR_P10_23 = 0x1017
ADDR_P10_25 = 0x1019
ADDR_P10_27 = 0x101B
ADDR_P10_29 = 0x101D
ADDR_P10_42 = 0x102A
