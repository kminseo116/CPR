"""CPR 압박용 RS485 일체형 서보모터 설정.

이 파일에는 사용자가 자주 조정하는 값과 모터 레지스터 주소만 모아 둔다.
모터 제어 로직은 다른 파일에서 이 설정값을 가져와 사용한다.

주의
-----
1. 통신 및 레지스터 정의는 사용자가 제공한 모터 매뉴얼을 기준으로 작성했다.
2. 0x0038 현재 전류의 실제 A 환산 계수는 매뉴얼에 명시되어 있지 않다.
3. 위치 단위와 DWORD WORD 순서는 실제 장비에서 저속·무부하 시험으로 확인해야 한다.
"""

import serial


# ============================================================================
# 1. RS485 통신 설정
# ============================================================================

PORT = "/dev/ttyUSB0"
SLAVE_ID = 1

BAUDRATE = 115200
BYTESIZE = 8
PARITY = serial.PARITY_NONE
STOPBITS = 1

TIMEOUT_S = 0.30
RETRIES = 2

# 32비트 데이터의 WORD 순서.
# 장비에서 위치값이 비정상적으로 읽히면 "LOW_HIGH"로 변경해 확인한다.
DWORD_WORD_ORDER = "HIGH_LOW"


# ============================================================================
# 2. 기구 및 위치 단위 설정
# ============================================================================

# 매뉴얼 0x0008 공장 기본값 2000.
# 실제로 RS485 위치 명령의 1회전당 단위인지 장비 시험으로 확인해야 한다.
POSITION_UNITS_PER_MOTOR_REV = 2000

SCREW_LEAD_MM_PER_REV = 10.0
GEAR_MOTOR_REV_PER_SCREW_REV = 1.5

# 압박 방향으로 내려갈 때 위치값이 증가하면 +1, 감소하면 -1.
MOTOR_DIRECTION_SIGN = 1


# ============================================================================
# 3. CPR 압박 설정
# ============================================================================

DEFAULT_DEPTH_CM = 5.0
MIN_DEPTH_CM = 1.0
MAX_DEPTH_CM = 6.0

TARGET_BPM = 107.0
REPEAT_COUNT = 1000

# 속도 설정 [rpm]
INITIAL_SPEED_RPM = 200
SEARCH_SPEED_RPM = 60
COMPRESSION_SPEED_RPM = 1800

# 매뉴얼 0x002E, 0x002F. 범위 0~100, 공장 기본값 10.
ACCEL_FACTOR = 10
DECEL_FACTOR = 10


# ============================================================================
# 4. 접촉 탐색 설정
# ============================================================================

# 1순위 접촉 판단: 로드셀
CONTACT_FORCE_N = 4.5

# 접촉 탐색 이동 조건
CONTACT_SEARCH_STEP_MM = 0.5
CONTACT_SEARCH_MAX_MM = 120.0
CONTACT_SETTLE_S = 0.03

# 2순위 접촉 판단: 로드셀 토픽 지연/누락 시 전류 변화량
LOADCELL_STALE_TIMEOUT_S = 0.30
CURRENT_BASELINE_SAMPLE_COUNT = 20
CURRENT_BASELINE_SAMPLE_PERIOD_S = 0.01
CURRENT_CONTACT_BLANKING_S = 0.08
CONTACT_CURRENT_DELTA_RAW = 30
CONTACT_CURRENT_CONFIRM_COUNT = 5


# ============================================================================
# 5. 안전 설정
# ============================================================================

MIN_POSITION_CMD = -2_000_000
MAX_POSITION_CMD = 2_000_000
POSITION_TOLERANCE_UNITS = 30

MOVE_TIMEOUT_S = 10.0
STATUS_PERIOD_S = 0.05

# 0x0038 원시값 기준 소프트웨어 상한.
# 실제 장비 로그를 수집한 뒤 반드시 조정한다.
MAX_CURRENT_RAW = 450
CURRENT_TRIP_COUNT_LIMIT = 3

MAX_POSITION_ERROR_UNITS = 20_000

RETURN_TO_INITIAL_ON_STOP = True
SERVO_OFF_ON_PROGRAM_EXIT = True

# 0x0030 데이터 저장은 필요할 때만 사용한다.
SAVE_PARAMETERS_ON_SETUP = False


# ============================================================================
# 6. 모터 레지스터 주소
# ============================================================================

# 기본 설정
REG_CONTROL_MODE = 0x0000
REG_BAUDRATE = 0x0001
REG_DEVICE_ADDRESS = 0x0003
REG_ENCODER_DIV_MODE = 0x0007
REG_ENCODER_RESOLUTION = 0x0008

# 전류·속도·위치 제어 설정
REG_MAX_CURRENT = 0x000C
REG_MAX_SPEED = 0x0010
REG_POSITION_KP = 0x0011
REG_POSITION_KI = 0x0012
REG_POSITION_I_LIMIT = 0x0013

# 인에이블 및 운전 명령
REG_ENABLE_METHOD = 0x0020
REG_INTERNAL_ENABLE = 0x0021
REG_MOTOR_START = 0x0023
REG_MOTOR_PAUSE = 0x0024
REG_MOTOR_RESET = 0x0025

# 속도 및 가감속
REG_SPEED_MODE_COMMAND = 0x002C
REG_INTERNAL_SPEED = 0x002D
REG_ACCEL = 0x002E
REG_DECEL = 0x002F
REG_SAVE = 0x0030

# 상태 모니터링
REG_ALARM_STATUS = 0x0032
REG_RUN_STATUS = 0x0033
REG_CURRENT_VOLTAGE = 0x0037
REG_CURRENT_CURRENT = 0x0038
REG_CURRENT_SPEED = 0x0039

# 아래 주소는 매뉴얼의 레지스터 목록 기준이다.
# 상세 설명 페이지에는 주소 표기가 일부 다르므로 실제 장비에서 확인한다.
REG_CURRENT_PULSE_COUNT = 0x003B
REG_CURRENT_POSITION = 0x003D
REG_POSITION_ERROR = 0x003F

# 위치 명령
REG_POSITION_COMMAND_MODE = 0x0041
REG_ABSOLUTE_POSITION = 0x0042
REG_INCREMENT_POSITION = 0x0044


# ============================================================================
# 7. 상태 비트 정의
# ============================================================================

# 0x0032 알람 상태
ALARM_OVERVOLTAGE = 1 << 0
ALARM_UNDERVOLTAGE = 1 << 1
ALARM_OVERCURRENT = 1 << 2
ALARM_POSITION_DEVIATION = 1 << 3

# 0x0033 작동 상태
RUN_TARGET_REACHED = 1 << 0
RUN_ENABLED = 1 << 1
RUN_RUNNING = 1 << 2
RUN_SPEED_REACHED = 1 << 3
