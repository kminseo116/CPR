"""loadcell_config.py — 단일 로드셀 설정"""

# ── 사용 채널 ──────────────────────────────────────────────────
# 실제로 연결한 PhidgetBridge 채널 번호로 변경한다.
ACTIVE_CHANNEL = 0
CHANNEL_LABEL = "LOADCELL"

# ── 로드셀 스펙 ───────────────────────────────────────────────
FULL_SCALE_KG = 100.0
FULL_SCALE_RATIO = 0.001844  # CH0 Full Scale Output: 1.844 mV/V
GRAVITY = 9.80665
RATIO_PER_KG = FULL_SCALE_RATIO / FULL_SCALE_KG
RATED_FORCE_N = FULL_SCALE_KG * GRAVITY

# ── 샘플링 / 필터 ─────────────────────────────────────────────
DATA_INTERVAL_MS = 10       # 100 Hz
MOVING_AVG_WINDOW = 5

# ── 영점 보정 ─────────────────────────────────────────────────
ZERO_CAL_DURATION_S = 2.0
ZERO_CAL_SAMPLE_HZ = 100
ZERO_CAL_MIN_SAMPLES = 15

# /loadcell_reset 수신 후 영점 보정
ZERO_DELAY_AFTER_RESET_S = 3.0
ZERO_SAFE_FORCE_N = 10.0

# ── 오버로드 임계값 ───────────────────────────────────────────
# 단일 로드셀에 직접 적용되는 힘 기준 [N]
OL_WARN_N = 800.0
OL_TRIP_N = 1000.0
OL_STOP_N = 1100.0
OL_CONFIRM = 5

# ── 압박 피크 측정 ────────────────────────────────────────────
COMPRESS_START_N = 120.0
COMPRESS_END_N = 90.0

# ── 주기 ──────────────────────────────────────────────────────
PRINT_PERIOD_S = 0.1
PUBLISH_PERIOD_S = 0.01


def print_summary() -> None:
    print(
        f"[Config] channel={ACTIVE_CHANNEL}  "
        f"FULL_SCALE_RATIO={FULL_SCALE_RATIO:.6f}  "
        f"OL_WARN/TRIP/STOP="
        f"{OL_WARN_N:.1f}/{OL_TRIP_N:.1f}/{OL_STOP_N:.1f} N"
    )