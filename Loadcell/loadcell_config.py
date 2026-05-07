"""loadcell_config.py — 로드셀 설정"""

# ── 채널 ──────────────────────────────────────────────────────
ACTIVE_CHANNELS: list[int]     = [0, 1, 2]   # 3채널: [0, 1, 2]
CHANNEL_LABELS:  dict[int, str] = {0: "LEFT", 1: "RIGHT", 2: "CENTER"}

# ── 로드셀 스펙 ───────────────────────────────────────────────
FULL_SCALE_KG    = 100.0
FULL_SCALE_RATIO = 0.001844
GRAVITY          = 9.80665
RATIO_PER_KG     = FULL_SCALE_RATIO / FULL_SCALE_KG
RATED_FORCE_N    = FULL_SCALE_KG * GRAVITY   # ≈ 980.7 N

# ── 샘플링 / 필터 ─────────────────────────────────────────────
DATA_INTERVAL_MS  = 10    # 100 Hz
MOVING_AVG_WINDOW = 5    # 이동 평균 윈도우 (샘플 수)

# ── 영점 보정 ─────────────────────────────────────────────────
ZERO_CAL_DURATION_S  = 2.0
ZERO_CAL_SAMPLE_HZ   = 100
ZERO_CAL_MIN_SAMPLES = 15

# ── 편심 경보 ─────────────────────────────────────────────────
TILT_MIN_FORCE_N  = 80.0    # 전체 힘이 ?N 이상일 때만 편심 판단
SHARE_WARN_THRESH = 0.70    # 한 로드셀이 전체 힘의 70% 이상 받을 때 편심 경고
IMBALANCE_WARN_N  = 150.0   # 로드셀 간 차이가 ?N 이상일 때 편심 경고
# 값 조정 필요

# ── 오버로드 임계값 ───────────────────────────────────────────
# CPR 마네킹 테스트용: 실제 압박 힘 기준 [N]
OL_WARN_N = 400.0    # 경고: 압박 힘이 높음
OL_TRIP_N = 500.0    # 트립: 모터 정지 요청
OL_STOP_N = 600.0    # 하드스탑: 즉시 정지 요청
# 값 조정 필요

# OL_WARN_N   = RATED_FORCE_N * OL_WARN_RATIO
# OL_TRIP_N   = RATED_FORCE_N * OL_TRIP_RATIO
# OL_STOP_N   = RATED_FORCE_N * OL_STOP_RATIO
OL_CONFIRM  = 3

# ── peak 값 측정 설정 ─────────────────────────────────────────────────
COMPRESS_START_N = 5.0   # 이 값 이상이면 압박 시작으로 판단
COMPRESS_END_N   = 3.0   # 이 값 이하로 내려오면 압박 종료로 판단

# ── 주기 ──────────────────────────────────────────────────────
PRINT_PERIOD_S   = 0.1      # 화면에 보이는 속도
PUBLISH_PERIOD_S = 0.01     # 실제 계산 속도
# 피크 값 계산과 터미널 출력 주기가 다르기 때문에 피크 값이 터미널에 출력 안 될 수 있음


def print_summary() -> None:
    print(
        f"[Config] channels={ACTIVE_CHANNELS}  "
        f"OL_WARN/TRIP/STOP="
        f"{OL_WARN_N:.1f}/{OL_TRIP_N:.1f}/{OL_STOP_N:.1f} N  "
        f"TILT_MIN={TILT_MIN_FORCE_N:.1f} N  "
        f"SHARE_WARN={SHARE_WARN_THRESH:.2f}  "
        f"IMB_WARN={IMBALANCE_WARN_N:.1f} N"
    )
