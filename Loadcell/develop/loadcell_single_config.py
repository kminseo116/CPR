"""loadcell_single_config.py — 단일 로드셀 설정"""

# ── 사용 채널 ──────────────────────────────────────────────────
# 현재 실행 로그에서 channel=1로 확인됨
ACTIVE_CHANNEL = 1
CHANNEL_LABEL = "LOADCELL"

# ── 로드셀 스펙 ───────────────────────────────────────────────
FULL_SCALE_KG = 100.0

# 채널별 Full Scale Output
# CH0 = 1.844 mV/V → 0.001844 V/V
# CH1 = 1.710 mV/V → 0.001710 V/V
# CH2 = 1.785 mV/V → 0.001785 V/V
#
# 현재 ACTIVE_CHANNEL=1이므로 CH1 값 사용
FULL_SCALE_RATIO = 0.0018

GRAVITY = 9.80665

# 1 kg당 출력 전압비
RATIO_PER_KG = FULL_SCALE_RATIO / FULL_SCALE_KG

# 정격 힘
RATED_FORCE_N = FULL_SCALE_KG * GRAVITY

# ── 샘플링 및 필터 ────────────────────────────────────────────
DATA_INTERVAL_MS = 10       # 100 Hz
MOVING_AVG_WINDOW = 5       # 이동평균 샘플 수

# ── 영점 보정 ─────────────────────────────────────────────────
ZERO_CAL_DURATION_S = 2.0
ZERO_CAL_SAMPLE_HZ = 100
ZERO_CAL_MIN_SAMPLES = 15

# /motor_stop=True를 받은 후 영점 보정까지 기다리는 시간
ZERO_DELAY_AFTER_MOTOR_STOP_S = 5.0

# 5초 후 현재 힘의 절댓값이 이 값 이하일 때만 영점 보정
ZERO_SAFE_FORCE_N = 10.0

# ── 과부하 임계값 ─────────────────────────────────────────────
# status_code
# 0 = NORMAL
# 1 = OVERLOAD_WARNING
# 2 = OVERLOAD_TRIP
# 3 = HARD_STOP 또는 fault latch
OL_WARN_N = 800.0
OL_TRIP_N = 1000.0
OL_STOP_N = 1100.0

# OL_TRIP_N 이상이 이 횟수만큼 연속 검출되면 트립
OL_CONFIRM = 5

# ── 압박 피크 검출 ────────────────────────────────────────────
# 이 값 이상이면 압박 시작으로 판단
COMPRESS_START_N = 120.0

# 압박 중 이 값 이하로 내려오면 압박 종료로 판단
COMPRESS_END_N = 90.0

# ── 실행 주기 ─────────────────────────────────────────────────
PRINT_PERIOD_S = 0.1
PUBLISH_PERIOD_S = 0.01


def print_summary() -> None:
    print(
        f"[Config] channel={ACTIVE_CHANNEL}  "
        f"FULL_SCALE_RATIO={FULL_SCALE_RATIO:.6f}  "
        f"OL_WARN/TRIP/STOP="
        f"{OL_WARN_N:.1f}/{OL_TRIP_N:.1f}/{OL_STOP_N:.1f} N  "
        f"ZERO_DELAY={ZERO_DELAY_AFTER_MOTOR_STOP_S:.1f} s"
    )