"""loadcell_config.py — 로드셀 설정"""

# 로드셀 하나용
# # ── 채널 ──────────────────────────────────────────────────────
# ACTIVE_CHANNELS: list[int]     = [0]   # 3채널: [0, 1, 2]
# CHANNEL_LABELS:  dict[int, str] = {0: "0"}

# # ── 로드셀 스펙 ───────────────────────────────────────────────
# FULL_SCALE_KG = 100.0

# CHANNEL_FULL_SCALE_RATIO: dict[int, float] = {
#     0: 0.0018
# }

# # 로드셀 3개용
# # ── 채널 ──────────────────────────────────────────────────────
# ACTIVE_CHANNELS: list[int]     = [1, 2, 3]   # 3채널: [0, 1, 2]
# CHANNEL_LABELS:  dict[int, str] = {3: "0", 2: "1", 1: "2"}

# # ── 로드셀 스펙 ───────────────────────────────────────────────
# FULL_SCALE_KG = 100.0

# # 채널별 Full Scale Output [V/V]
# # CH0 = 1.844 mV/V, CH1 = 1.710 mV/V, CH2 = 1.785 mV/V
# CHANNEL_FULL_SCALE_RATIO: dict[int, float] = {
#     3: 0.001844,
#     2: 0.001710,
#     1: 0.001785,
# }

# 로드셀 3개용
# ── 채널 ──────────────────────────────────────────────────────
ACTIVE_CHANNELS: list[int]     = [0, 1, 2]
CHANNEL_LABELS:  dict[int, str] = {0: "0", 1: "1", 2: "2"}

# ── 로드셀 스펙 ───────────────────────────────────────────────
FULL_SCALE_KG = 100.0

# 채널별 Full Scale Output [V/V]
# CH0 = 1.844 mV/V, CH1 = 1.710 mV/V, CH2 = 1.785 mV/V
CHANNEL_FULL_SCALE_RATIO: dict[int, float] = {
    0: 0.001844,
    1: 0.001710,
    2: 0.001785,
}

# 기본값: 예외 상황 또는 구버전 코드 호환용
FULL_SCALE_RATIO = CHANNEL_FULL_SCALE_RATIO[ACTIVE_CHANNELS[0]]

GRAVITY = 9.80665

# 채널별 kg 환산 계수 [V/V per kg]
CHANNEL_RATIO_PER_KG: dict[int, float] = {
    channel: ratio / FULL_SCALE_KG
    for channel, ratio in CHANNEL_FULL_SCALE_RATIO.items()
}

# 기본값: 예외 상황 또는 구버전 코드 호환용
RATIO_PER_KG = FULL_SCALE_RATIO / FULL_SCALE_KG

RATED_FORCE_N = FULL_SCALE_KG * GRAVITY   # ≈ 980.7 N


def get_ratio_per_kg(channel_id: int) -> float:
    """채널별 전압비-kg 변환 계수를 반환한다."""
    try:
        return CHANNEL_RATIO_PER_KG[channel_id]
    except KeyError as exc:
        raise KeyError(
            f"CHANNEL_FULL_SCALE_RATIO에 channel {channel_id} 설정이 없습니다."
        ) from exc

# ── 샘플링 / 필터 ─────────────────────────────────────────────
DATA_INTERVAL_MS  = 10    # 100 Hz
MOVING_AVG_WINDOW = 5    # 이동 평균 윈도우 (샘플 수)

# ── 영점 보정 ─────────────────────────────────────────────────
# 현재 약 2.5초 걸림
ZERO_CAL_DURATION_S  = 2.0
ZERO_CAL_SAMPLE_HZ   = 100
ZERO_CAL_MIN_SAMPLES = 15

# ── reset 후 영점 보정 ───────────────────────────────────
# 시간지연
ZERO_DELAY_AFTER_RESET_S = 3.0
# /loadcell_reset 수신 시, 현재 합산 힘이 이 값 이하일 때만 zero calibration 수행
ZERO_SAFE_TOTAL_N = 10.0

# ── 편심 경보 ─────────────────────────────────────────────────
TILT_MIN_FORCE_N  = 200.0    # 전체 힘이 ?N 이상일 때만 편심 판단
SHARE_WARN_THRESH = 0.80    # 한 로드셀이 전체 힘의 80% 이상 받을 때 편심 경고
IMBALANCE_WARN_N  = 300.0   # 로드셀 간 차이가 ?N 이상일 때 편심 경고
# 값 조정 필요

# ── 오버로드 임계값 ───────────────────────────────────────────
# CPR 마네킹 테스트용: 실제 압박 힘 기준 [N]
# status_code:
# 0 = NORMAL
# 1 = OVERLOAD_WARNING
# 2 = OVERLOAD_TRIP
# 3 = OVERLOAD_HARD_STOP 또는 LATCH
OL_WARN_N = 800.0    # 경고: 압박 힘이 높음
OL_TRIP_N = 1000.0    # 트립: 모터 정지 요청
OL_STOP_N = 1100.0    # 하드스탑: 즉시 정지 요청

OL_CONFIRM  = 5

# ── peak 값 측정 설정 ─────────────────────────────────────────────────
COMPRESS_START_N = 120.0   # 이 값 이상이면 압박 시작으로 판단
COMPRESS_END_N   = 90.0   # 이 값 이하로 내려오면 압박 종료로 판단

# ── 주기 ──────────────────────────────────────────────────────
PRINT_PERIOD_S   = 0.1      # 화면에 보이는 속도
PUBLISH_PERIOD_S = 0.01     # 실제 계산 속도
# 피크 값 계산과 터미널 출력 주기가 다르기 때문에 피크 값이 터미널에 출력 안 될 수 있음


def print_summary() -> None:
    print(
        f"[Config] channels={ACTIVE_CHANNELS}  "
        f"CH_RATIO={CHANNEL_FULL_SCALE_RATIO}  "
        f"OL_WARN/TRIP/STOP="
        f"{OL_WARN_N:.1f}/{OL_TRIP_N:.1f}/{OL_STOP_N:.1f} N  "
        f"TILT_MIN={TILT_MIN_FORCE_N:.1f} N  "
        f"SHARE_WARN={SHARE_WARN_THRESH:.2f}  "
        f"IMB_WARN={IMBALANCE_WARN_N:.1f} N"
    )
