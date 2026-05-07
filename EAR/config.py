# =========================
# 모든 파라미터 상수 정의
# =========================

# 디버그 플래그
DEBUG = False   # True로 바꾸면 MOTION RAW / EMA 수치 표시

# -------------------------
# 랜드마크 인덱스
# -------------------------
LEFT_EYE_LANDMARK  = [362, 385, 387, 263, 373, 380]
RIGHT_EYE_LANDMARK = [33, 160, 158, 133, 153, 144]
MOTION_LANDMARKS   = [1, 152, 234, 454, 10]    # 코, 턱, 좌우 외곽, 이마


LEFT_EYE_CENTER_IDX  = 33
RIGHT_EYE_CENTER_IDX = 263

# -------------------------
# 상태 정의
# -------------------------
STATE_WAIT_OPEN   = 0
STATE_CALIBRATING = 1
STATE_RUNNING     = 2

# -------------------------
# EAR 파라미터
# -------------------------
ABS_OPEN_EAR       = 0.25   # 캘리브레이션 시작 기준 절대 EAR
ABS_CLOSED_EAR     = 0.15   # WAIT_OPEN 단계 임시 CLOSED 판단 기준
EAR_WIDTH_MIN      = 1e-6   # 0 나누기 방지
CALIBRATION_FRAMES = 60     # 캘리브레이션 수집 프레임 수 (약 2초)

# -------------------------
# 눈 상태 파라미터
# -------------------------
BLINK_THRESHOLD       = 0.25    # 눈 감긴 시간이 이보다 짧으면 깜빡임
BLINK_ACTIVE_GAP      = 1.0     # 마지막 깜빡임 후 BLINK 반응 유지 시간 (초)
NO_RESPONSE_THRESHOLD = 1.5     # 눈 감긴 시간이 이 이상이면 NO RESPONSE (초)

# -------------------------
# 움직임 파라미터
# -------------------------
MOTION_THRESHOLD   = 0.030  # 움직임 판단 EMA 임계값 -> 이 값을 이용해서 움직임 정도를 정함
MOTION_HOLD_TIME   = 0.25   # threshold 초과 유지 시간 (초) ->  권장 범위: 0.15 ~ 0.40
MOTION_EMA_ALPHA   = 0.25   # EMA 계수
MOTION_HISTORY_LEN = 8      # 최근 프레임 저장 개수
MOTION_COMPARE_GAP = 6      # 현재 vs N프레임 전 비교

# -------------------------
# 카메라 설정
# -------------------------
CAMERA_INDEX  = 4
WINDOW_NAME   = "EAR_AND_MOTION_RESPONSE"
FRAME_WIDTH   = 640
FRAME_HEIGHT  = 360
