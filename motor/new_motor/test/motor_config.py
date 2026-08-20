"""CPR 압박용 RS485 일체형 서보모터 설정.

이 파일에는 사용자가 자주 조정하는 값과 모터 레지스터 주소만 모아 둔다.
모터 제어 로직은 다른 파일에서 이 설정값을 가져와 사용한다.

## 주의

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

# True일 때만 DWORD 요청 프레임과 에코 원문을 출력한다.
# 일반 운전에서는 상태 로그의 가독성을 위해 False로 둔다.
SHOW_MODBUS_DEBUG = False

# 32비트 데이터의 WORD 순서.
# 장비에서 위치값이 비정상적으로 읽히면 "LOW_HIGH"로 변경해 확인한다.
DWORD_WORD_ORDER = "HIGH_LOW"


# ============================================================================
# 2. 기구 및 위치 단위 설정
# ============================================================================

# 현재 외부 DIP 설정. 위치 보정 후에만 속도 시험을 허용하고, 위치와 속도
# 보정이 모두 끝난 뒤에만 일반 위치 이동 코드를 허용한다.
DIP_MSTEP = 1000
MSTEP_POSITION_CALIBRATION_COMPLETE = True
MSTEP_SPEED_CALIBRATION_COMPLETE = True

# 0x003D 현재 위치 피드백을 축을 손으로 90도씩 돌려 확인한 값.
# 90도당 약 4096 count, 한 바퀴당 약 16384 count로 측정됐다.
FEEDBACK_COUNTS_PER_MOTOR_REV = 16384

# 외부 DIP MSTEP 1000(ON, ON, ON, OFF)에서 ±499 command unit
# 왕복 시험으로 +4078/-4077 feedback count를 측정했다.
# 양방향 1회전 명령 단위는 2004.8/2005.3, 평균은 2005.1이다.
COMMAND_UNITS_PER_MOTOR_REV = 2005

SCREW_LEAD_MM_PER_REV = 10.0
GEAR_MOTOR_REV_PER_SCREW_REV = 1.5

# 실제 90도 시험 결과: 양수 명령=반시계, 음수 명령=시계 방향.
POSITIVE_DIRECTION_NAME = "COUNTERCLOCKWISE"
NEGATIVE_DIRECTION_NAME = "CLOCKWISE"

# 현재 압박 방향은 양수(반시계)로 설정한다.
# 장착 후 압박판이 올라가면 -1(시계 방향)로 변경한다.
MOTOR_DIRECTION_SIGN = 1


# ============================================================================
# 3. CPR 압박 설정
# ============================================================================

DEFAULT_DEPTH_CM = 5.0
MIN_DEPTH_CM = 0.5
MAX_DEPTH_CM = 6.0

TARGET_BPM = 107.0

# 깊이와 BPM으로 계산한 이론 rpm에 곱하는 시험 보정값이다.
CPR_SPEED_MARGIN = 2.3
REPEAT_COUNT = 30

# 매뉴얼 0x002D 내부 속도 설정의 허용 범위는 0~3000이다.
MAX_INTERNAL_SPEED_SETTING = 3000

# MSTEP 1000 고속 3회전 왕복 시험에서 설정 97/194는 실제 약
# 96~99/194~196 rpm이었다. 설정 485는 짧은 거리와 가감속 영향으로
# 약 430~457 rpm이었으며, 전체 설정값/실제 rpm 중앙값은 1.006이었다.
SPEED_SETTING_PER_OUTPUT_RPM = 1.00

MOTOR_RATED_SPEED_RPM = 3000.0

MAX_OUTPUT_SPEED_RPM = min(
    MOTOR_RATED_SPEED_RPM,
    MAX_INTERNAL_SPEED_SETTING / SPEED_SETTING_PER_OUTPUT_RPM,
)

# MSTEP 1000, 무부하 60 mm 왕복에서 2000 rpm 도달을 확인했다.
# 실제 압박 하중에서는 아직 검증되지 않았으므로 반드시 시험 장비에서
# 1회 압박부터 확인한다.
MAX_VERIFIED_OUTPUT_SPEED_RPM = 3000.0

INITIAL_SPEED_RPM = 70.0
SEARCH_SPEED_RPM = 30.0

# 압박 속도는 motor_main에서 깊이와 TARGET_BPM으로 자동 계산한다.
# 이 값은 설정 표시와 이전 코드 호환용 상한 후보로만 유지한다.
COMPRESSION_SPEED_RPM = 2500.0

RETURN_OUTPUT_SPEED_RPM = INITIAL_SPEED_RPM
SEARCH_OUTPUT_SPEED_RPM = SEARCH_SPEED_RPM
COMPRESSION_OUTPUT_SPEED_RPM = COMPRESSION_SPEED_RPM

# 매뉴얼 0x002E, 0x002F. 범위 0~100, 공장 기본값 10.
#
# ACCEL_FACTOR = 67
# DECEL_FACTOR = 50

ACCEL_FACTOR = 50
DECEL_FACTOR = 40


# ============================================================================
# 4. 접촉 탐색 설정
# ============================================================================

# 로드셀 연결 후 자동 접촉 판정에 사용할 값. 현재 수동 시험에서는 미사용.
CONTACT_FORCE_N = 4.5

# 접촉 탐색은 아래 최대 거리까지 한 번의 증분 명령으로 연속 이동한다.
# 첫 통합 시험에서는 Enter 입력 누락 시 장거리 이동하지 않도록 제한한다.
CONTACT_SEARCH_MAX_MM = 20.0

# 실제 시험에서 0.5 mm 이동에 약 3초가 걸렸으므로 5 mm 탐색에 여유를 둔다.
CONTACT_SEARCH_TIMEOUT_S = 150.0

# 로드셀 연결 후 자동 접촉 판정에 사용할 값. 현재 수동 시험에서는 미사용.
LOADCELL_STALE_TIMEOUT_S = 0.30

CURRENT_BASELINE_SAMPLE_COUNT = 20
CURRENT_BASELINE_SAMPLE_PERIOD_S = 0.01
CURRENT_CONTACT_BLANKING_S = 0.08

CONTACT_CURRENT_DELTA_RAW = 30
CONTACT_CURRENT_CONFIRM_COUNT = 5


# ============================================================================
# 5. 안전 설정
# ============================================================================

# 프로그램 시작과 각 CPR 실행 전에 물리적으로 복귀할 피드백 위치다.
START_FEEDBACK_POSITION = 100

MIN_POSITION_CMD = -2_000_000
MAX_POSITION_CMD = 2_000_000

POSITION_TOLERANCE_UNITS = 30


# ----------------------------------------------------------------------------
# 일반 위치 이동 목표 초과 판정
# ----------------------------------------------------------------------------

MIN_FEEDBACK_OVERSHOOT_COUNTS = 300
MAX_FEEDBACK_OVERSHOOT_RATIO = 0.015


# ----------------------------------------------------------------------------
# 반대 방향 이동 판정
# ----------------------------------------------------------------------------

# +22, -156 count와 같이 방향 전환 직후 발생하는 작은 위치 흔들림은
# 실제 반대 방향 구동으로 판단하지 않는다.
#
# 이 값 이상 반대 방향으로 이동한 경우에만 MOTOR_ERROR로 처리한다.
DIRECTION_ERROR_MIN_COUNTS = 1000


# ----------------------------------------------------------------------------
# 일반 이동 시간 및 상태 확인
# ----------------------------------------------------------------------------

MOVE_TIMEOUT_S = 30.0

STATUS_PERIOD_S = 0.05
MOVE_STATUS_LOG_PERIOD_S = 1.0


# ----------------------------------------------------------------------------
# CPR 고속 왕복 명령 설정
# ----------------------------------------------------------------------------

# CPR 왕복 전용 빠른 명령·위치 확인 설정.
# 접촉 탐색과 초기 복귀에는 적용하지 않는다.
#
# 왕복 중에는 위치만 읽고 상세 상태는 사이클 후 확인한다.
CPR_COMMAND_EDGE_DELAY_S = 0.02
CPR_POSITION_POLL_PERIOD_S = 0.003

# 위치 읽기의 순간 지연으로 최고속도가 과장되지 않도록 이 시간 구간의
# 위치 변화로 이동 중 속도를 추정한다.
# 추가 Modbus 요청은 발생하지 않는다.
CPR_SPEED_ESTIMATE_WINDOW_S = 0.05


# ----------------------------------------------------------------------------
# CPR 목표 위치 과도 초과 안전 판정
# ----------------------------------------------------------------------------

# 고속 이동에서는 한 번의 RS485 위치 읽기 사이에 목표점을 통과할 수 있다.
#
# 목표 통과 자체는 즉시 방향 전환으로 처리하고,
# 아래 한도를 넘는 큰 초과만 비정상 이동으로 판정한다.
#
# 일반 이동의 정밀 초과 한도와는 별도다.
CPR_MIN_EMERGENCY_OVERSHOOT_COUNTS = 4096
CPR_MAX_EMERGENCY_OVERSHOOT_RATIO = 0.25


# ----------------------------------------------------------------------------
# CPR 방향 전환 관성 허용
# ----------------------------------------------------------------------------

# 방향 전환 명령 직후에는 관성 때문에 기존 방향으로 조금 더 움직일 수 있다.
#
# 일반 복귀는 시간과 거리를 함께 사용하고,
# 빠른 CPR은 실제 복귀 방향 움직임이 시작되기 전까지
# 아래 거리 안의 관성만 허용한다.
CPR_DIRECTION_CHANGE_GRACE_S = 0.10
CPR_DIRECTION_CHANGE_GRACE_COUNTS = 4096


# ----------------------------------------------------------------------------
# CPR 하강 / 복귀 완료 판정
# ----------------------------------------------------------------------------

# [추가]
# 고속 CPR 압박 하강 완료 허용오차.
#
# 압박 깊이 정확도가 중요하므로 복귀보다 엄격하게 0.3%를 허용한다.
#
# 예:
# 목표 63,898 count일 때
# 허용오차 ≈ 192 count
#
# 따라서 약 63,706 count 이상 이동하면 하강 완료로 인정한다.
#
# 2.6 cm 압박 기준 거리 허용오차는 약 0.078 mm이다.
CPR_DOWN_COMPLETION_TOLERANCE_RATIO = 0.003


# 고속 CPR 복귀는 관성과 위치 명령 환산 차이로 접촉 기준점 직전에서
# 정지할 수 있다.
#
# 복귀는 압박 깊이보다 정밀도가 덜 중요하므로 이동량의 1.5%를 허용한다.
# 이후 초기 위치 복귀가 잔차를 보정한다.
CPR_RETURN_COMPLETION_TOLERANCE_RATIO = 0.015


# ----------------------------------------------------------------------------
# 이동 제한시간 계산
# ----------------------------------------------------------------------------

# 실제 이동거리와 출력축 rpm으로 motor_main의 제한시간을 계산한다.
MOVE_TIMEOUT_MIN_S = 3.0
MOVE_TIMEOUT_FACTOR = 2.0
MOVE_TIMEOUT_MARGIN_S = 2.0


# ----------------------------------------------------------------------------
# 전류 안전 설정
# ----------------------------------------------------------------------------

# MSTEP 1000 고속 시험에서 100 rpm 압박 방향 2964 raw,
# 500 rpm 압박 방향 3921 raw의 단발성 피크가 관찰됐다.
#
# 20000 이상이 3회 연속일 때만 정지시켜
# 단발 피크와 지속 과전류를 구분하며,
# 하중 로그를 보고 조정한다.
MAX_CURRENT_RAW = 20_000
CURRENT_TRIP_COUNT_LIMIT = 3


# ----------------------------------------------------------------------------
# 위치 편차 안전 설정
# ----------------------------------------------------------------------------

MAX_POSITION_ERROR_UNITS = 20_000


# ----------------------------------------------------------------------------
# 정지 및 종료 동작
# ----------------------------------------------------------------------------

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