import serial


# ============================================================
# 핵심 수정 값
# ============================================================

# 초기 위치 이동 속도 (rpm)
INITIAL_MOVE_RPM = 300

# 접촉 위치 탐색 속도 (rpm)
SEARCH_RPM = 50

# 왕복 속도 (rpm) // 목표 압박 bpm: 117bpm -> 모터는 2106rpm (1bpm = 18rpm)
RECIP_RPM = 300 #[rpm]

# 접촉 힘 기준
CONTACT_FORCE_N = 5.0

# 전류 제한
MAX_CURRENT_A = 50.0 #[A]

# 위치 이동 제한 시간 (초) -> 일단 길게 잡음
MOVE_TIMEOUT = 30.0

# ============================================================
# 로그 설정
# ============================================================

# 이동 중 상태 로그 출력 주기
LOG_PERIOD_S = 0.2

# ============================================================
# 통신 설정
# ============================================================

PORT     = "/dev/ttyUSB0"
SLAVE_ID = 1

BAUDRATE = 19200
BYTESIZE = 8
PARITY   = serial.PARITY_EVEN
STOPBITS = 1
TIMEOUT  = 0.5

# 32bit 데이터 word 순서 (테스트 결과 LOW_HIGH 확인)
WORD_ORDER = "LOW_HIGH"


# ============================================================
# 기계 파라미터
# ============================================================

# 10000 지령단위/s = 1 rev/s = 60 rpm → 1바퀴 = 10000 지령단위
COMMAND_UNITS_PER_MOTOR_REV = 10000

# 볼스크류 리드: 1바퀴 회전 시 10 mm 이동
SCREW_LEAD_MM_PER_REV = 10.0

# 기어비: 모터 1.5바퀴 → 볼스크류 1바퀴
GEAR_MOTOR_REV_PER_SCREW_REV = 1.5

# 압박 깊이를 입력받지 않는 경우 motor_config.py의 TRAVEL_MM 고정값 사용
# 입력 기능을 끈 경우 사용할 기본 왕복 이동 거리
DEFAULT_TRAVEL_MM = 60.0

DEFAULT_TRAVEL_SCREW_REV = DEFAULT_TRAVEL_MM / SCREW_LEAD_MM_PER_REV
DEFAULT_TRAVEL_MOTOR_REV = DEFAULT_TRAVEL_SCREW_REV * GEAR_MOTOR_REV_PER_SCREW_REV
DEFAULT_TRAVEL_UNITS = int(DEFAULT_TRAVEL_MOTOR_REV * COMMAND_UNITS_PER_MOTOR_REV)


# ============================================================
# 동작 설정
# ============================================================

# 시작 시 초기 위치로 이동할지 여부
MOVE_TO_INITIAL_ON_START = True

# 초기 위치 (P0B-07 / P10-14 기준 지령단위)
INITIAL_POS = 0

# # 초기 위치 이동 속도 (rpm)
# INITIAL_MOVE_RPM = 300

# # 접촉 위치 탐색 속도 (rpm)
# SEARCH_RPM = 30

# # 왕복 속도 (rpm) // 목표 압박 bpm: 117bpm -> 모터는 2106rpm (1bpm = 18rpm)
# RECIP_RPM = 500

# 가감속
# * 4  → 목표 속도까지 약 0.25초
# * 6  → 목표 속도까지 약 0.17초
# * 8  → 목표 속도까지 약 0.125초
# * 10 → 목표 속도까지 약 0.10초
ACCEL_FACTOR = 3
DECEL_FACTOR = 3

# PV 탐색 중 허용 최대 속도 제한 (rpm)
MAX_RPM_LIMIT = 2500

# 왕복 반복 횟수
REPEAT_COUNT = 1000

# # 접촉 힘 기준
# CONTACT_FORCE_N = 2.0


# ============================================================
# 압박 깊이 입력 설정
# ============================================================

# True  = 코드 실행 중 키보드로 압박 깊이 입력
# False = config_02.py의 TRAVEL_MM 고정값 사용
USE_INPUT_COMPRESSION_DEPTH = True

# 최대 압박 깊이 [cm]
MAX_COMPRESSION_DEPTH_CM = 6.0


# ============================================================
# 안전 제한
# ============================================================

# 위치 제한 (물리 범위보다 반드시 좁게 설정)
MIN_POSITION_CMD = -200000
MAX_POSITION_CMD =  200000

# # 전류 제한
# MAX_CURRENT_A = 50.0

## P0B-24 스케일 (상전류 유효값 A)
# CURRENT_SCALE = 0.1

# 연속 전류 초과 횟수 도달 시 정지
CURRENT_TRIP_COUNT_LIMIT = 5

# 위치 도달 허용 오차 (지령단위)
POSITION_TOLERANCE = 500

# # 위치 이동 제한 시간 (초)
# MOVE_TIMEOUT = 30.0

# 하드웨어 리미트 유지 여부
KEEP_HARDWARE_LIMITS = True


# ============================================================
# Modbus 레지스터 주소 (매뉴얼 기반)
# ============================================================

# ----- P02 그룹 -----
ADDR_P02_00 = 0x0200    # P02-00 제어 모드 선택, 9 = Modbus 버스 모드
ADDR_P02_34 = 0x0222    # P02-34 고장 코드

# ----- P03 그룹: DI 입력 설정 -----
ADDR_P03_06 = 0x0306    # P03-06 DI3 기능 선택, 1 = S-ON
ADDR_P03_07 = 0x0307    # P03-07 DI3 논리 선택, 0 = Low 유효
ADDR_P03_08 = 0x0308    # P03-08 DI4 기능 선택, 기본값 14 = P-OT
ADDR_P03_10 = 0x030A    # P03-10 DI5 기능 선택, 기본값 15 = N-OT

# ----- P0B 그룹: 모니터링 -----
ADDR_P0B_00 = 0x0B00    # P0B-00 실제 모터 속도 rpm
ADDR_P0B_03 = 0x0B03    # P0B-03 DI 입력 상태
ADDR_P0B_04 = 0x0B04    # P0B-04 버스 상태워드
ADDR_P0B_07 = 0x0B07    # P0B-07 Absolute position counter
ADDR_P0B_24 = 0x0B18    # P0B-24 상전류 유효값 A
ADDR_P0B_26 = 0x0B1A    # P0B-26 DC 버스 전압, 0.1V 단위
ADDR_P0B_53 = 0x0B35    # P0B-53 위치 편차 카운터

# ----- P0D 그룹: 버스 제어 / 강제 DI -----
ADDR_P0D_08 = 0x0D08    # P0D-08 버스 제어워드
ADDR_P0D_17 = 0x0D11    # P0D-17 DIDO 강제 입력/출력 사용
ADDR_P0D_18 = 0x0D12    # P0D-18 DI 강제 입력값

# ----- P10 그룹: 버스 위치/속도 제어 -----
ADDR_P10_03 = 0x1003    # P10-03 운전 모드, 1 = PP 위치모드, 3 = PV 속도모드
ADDR_P10_07 = 0x1007    # P10-07 위치 도달 window
ADDR_P10_14 = 0x100E    # P10-14 PP 목표 위치
ADDR_P10_23 = 0x1017    # P10-23 최대 속도 제한 [지령단위/s]
ADDR_P10_25 = 0x1019    # P10-25 PP 이동 속도 [지령단위/s]
ADDR_P10_27 = 0x101B    # P10-27 PP 가속도 [지령단위/s^2]
ADDR_P10_29 = 0x101D    # P10-29 PP 감속도 [지령단위/s^2]
ADDR_P10_42 = 0x102A    # P10-42 PV 모드 속도 [지령단위/s]
