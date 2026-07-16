"""
motor_config.py

모터 제어 프로그램에서 사용하는 모든 설정값을 모아둔 파일이다.

수정할 값:
- 속도
- 위치 제한
- 통신 포트
- 압박 깊이 범위
- 흉부 높이 비율 기반 압박 깊이 계산 기준
"""

import serial


# ============================================================
# 핵심 동작 속도 및 기본 안전값
# ============================================================

# 초기 위치 이동 속도 [rpm]
INITIAL_MOVE_RPM = 300

# 접촉 위치 탐색 속도 [rpm]
SEARCH_RPM = 200

# 왕복 압박 속도 [rpm]
# 실제 BPM은 부하, 가감속, 위치 도달 허용오차에 따라 달라질 수 있음
RECIP_RPM = 3000

# 로드셀 합산 힘이 이 값 이상이면 흉부 접촉으로 판단 [N]
CONTACT_FORCE_N = 4.5

# 전류 제한 [A]
MAX_CURRENT_A = 130.0

# 위치 이동 제한 시간 [s]
MOVE_TIMEOUT = 30.0


# ============================================================
# BPM 안정화 설정
# ============================================================

TARGET_BPM = 107.0
BPM_TOLERANCE = 3.0

MIN_ALLOWED_BPM = TARGET_BPM - BPM_TOLERANCE
MAX_ALLOWED_BPM = TARGET_BPM + BPM_TOLERANCE

TARGET_CYCLE_DT = 60.0 / TARGET_BPM
THEORY_MIN_ALLOWED_CYCLE_DT = 60.0 / MAX_ALLOWED_BPM
THEORY_MAX_ALLOWED_CYCLE_DT = 60.0 / MIN_ALLOWED_BPM

# 실제 제어용 cycle time 제한값
# FORWARD 시간이 이 값을 넘으면 목표 위치 도달 여부와 관계없이 압박 완료 처리
MIN_ALLOWED_CYCLE_DT = 0.548
MAX_ALLOWED_CYCLE_DT = 0.555


# ============================================================
# 로그 설정
# ============================================================

LOG_PERIOD_S = 0.2


# ============================================================
# 통신 설정
# ============================================================

PORT = "/dev/motor_rs485"
SLAVE_ID = 1

BAUDRATE = 19200
BYTESIZE = 8
PARITY = serial.PARITY_EVEN
STOPBITS = 1
TIMEOUT = 0.5

# 현재 ISL60-PR 사용 환경에서 확인된 32bit word 순서
WORD_ORDER = "LOW_HIGH"


# ============================================================
# 기계 파라미터
# ============================================================

# 모터 1회전 = 10000 지령단위
COMMAND_UNITS_PER_MOTOR_REV = 10000

# 볼스크류 리드: 볼스크류 1회전당 10 mm 이동
SCREW_LEAD_MM_PER_REV = 10.0

# 기어비: 모터 1.5회전 = 볼스크류 1회전
GEAR_MOTOR_REV_PER_SCREW_REV = 1.5

# motor unit → mm 변환값
# 10000 unit = motor 1 rev
# motor 1.5 rev = screw 1 rev
# screw 1 rev = 10 mm
# 따라서 15000 unit = 10 mm, 1500 unit = 1 mm
MOTOR_UNITS_PER_MM = (
    COMMAND_UNITS_PER_MOTOR_REV * GEAR_MOTOR_REV_PER_SCREW_REV
) / SCREW_LEAD_MM_PER_REV


# ============================================================
# 기본 압박 깊이 설정
# ============================================================

# 자동 깊이를 사용하지 않을 때의 기본 압박 깊이 [mm]
DEFAULT_TRAVEL_MM = 50.0

DEFAULT_TRAVEL_SCREW_REV = DEFAULT_TRAVEL_MM / SCREW_LEAD_MM_PER_REV
DEFAULT_TRAVEL_MOTOR_REV = DEFAULT_TRAVEL_SCREW_REV * GEAR_MOTOR_REV_PER_SCREW_REV
DEFAULT_TRAVEL_UNITS = int(DEFAULT_TRAVEL_MOTOR_REV * COMMAND_UNITS_PER_MOTOR_REV)


# ============================================================
# 동작 설정
# ============================================================

# 시작 시 초기 위치로 이동할지 여부
MOVE_TO_INITIAL_ON_START = True

# 초기 위치 -> 전원 재연결 후 복귀할 위쪽 안전 기준 위치
INITIAL_POS = 49147

# 가감속 계수
ACCEL_FACTOR = 12
DECEL_FACTOR = 12

# PV 탐색 중 허용 최대 속도 제한 [rpm]
MAX_RPM_LIMIT = 2500

# 왕복 반복 횟수
REPEAT_COUNT = 1000

# ============================================================
# 로봇 설치 높이 / 흉부 높이 계산 설정
# ============================================================

# 받침면/등판면으로부터 초기 위치 상태의 압박봉 끝단까지의 높이 [mm]
INITIAL_PLATE_HEIGHT_FROM_SUPPORT_MM = 235.0

# 계산된 흉부 높이의 유효 범위 [mm] -> 비정상적인 흉부 높이가 계산되는 것을 방지 범위
MIN_VALID_CHEST_HEIGHT_MM = 100.0
MAX_VALID_CHEST_HEIGHT_MM = 350.0


# ============================================================
# 자동 압박 깊이 보정 설정
# ============================================================

# True이면 접촉 위치 기반 흉부 높이 추정으로 압박 깊이를 자동 계산
USE_AUTO_COMPRESSION_DEPTH = True

# 성인 CPR 압박 깊이 제한 범위 [mm]
# 최종 목표 깊이는 항상 이 범위 안으로 제한한다.
MIN_COMPRESSION_DEPTH_MM = 50.0
MAX_COMPRESSION_DEPTH_MM = 60.0

# ============================================================
# 흉부 높이 기반 압박 깊이 매핑 설정
# ============================================================

# 현재 우리의 마네킹 흉부 높이 약 160 mm 기준
# 낮은 흉부 기준: 140 mm 이하 → 50 mm 압박
# 높은 흉부 기준: 200 mm 이상 → 60 mm 압박
# 140~200 mm 사이는 선형 보간으로 50~60 mm 사이 깊이를 계산한다.
LOW_CHEST_HEIGHT_MM = 140.0
HIGH_CHEST_HEIGHT_MM = 200.0

# recoil 확인 기준 [N]
# BACKWARD 복귀 후에도 로드셀 합산 힘이 이 값보다 크면 recoil 부족 가능성 경고
RECOIL_FORCE_THRESHOLD_N = 30.0

# ============================================================
# 압박 깊이 수동 입력 설정
# 자동 깊이를 끄는 경우에만 사용
# ============================================================

USE_INPUT_COMPRESSION_DEPTH = True
MAX_COMPRESSION_DEPTH_CM = 6.0


# ============================================================
# 안전 제한
# ============================================================

# 위치 제한
MIN_POSITION_CMD = -300000
MAX_POSITION_CMD = 300000

# 전류 스케일
CURRENT_SCALE = 0.1

# 연속 전류 초과 횟수 도달 시 정지
CURRENT_TRIP_COUNT_LIMIT = 5

# 위치 도달 허용 오차 [unit]
POSITION_TOLERANCE = 2500

# 하드웨어 리미트 유지 여부
KEEP_HARDWARE_LIMITS = True


# ============================================================
# Modbus 레지스터 주소
# ============================================================

# P02 그룹
ADDR_P02_00 = 0x0200
ADDR_P02_34 = 0x0222

# P03 그룹
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
ADDR_P0B_26 = 0x0B1A
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