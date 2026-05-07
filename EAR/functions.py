# =========================
# functions.py
# 유틸 함수 + 렌더링 함수
# =========================

import cv2
import numpy as np

from config import (
    LEFT_EYE_CENTER_IDX, RIGHT_EYE_CENTER_IDX,
    EAR_WIDTH_MIN,
    ABS_OPEN_EAR, ABS_CLOSED_EAR,
    NO_RESPONSE_THRESHOLD,
    DEBUG,
)

# =========================
# 유틸 함수
# =========================

def euclidean(p1, p2):
    """두 점 사이의 유클리드 거리"""
    return np.linalg.norm(p1 - p2)


def get_pixel_coord(landmarks, idx, w, h):
    """정규화된 랜드마크를 픽셀 좌표로 변환"""
    lm = landmarks[idx]
    return np.array([lm.x * w, lm.y * h], dtype=np.float32)


def compute_ear(landmarks, eye_indices, w, h):
    """Eye Aspect Ratio(EAR) 계산"""
    pts = [get_pixel_coord(landmarks, idx, w, h) for idx in eye_indices]
    p1, p2, p3, p4, p5, p6 = pts

    width = euclidean(p1, p4)
    if width < EAR_WIDTH_MIN:
        return 0.0

    return (euclidean(p2, p6) + euclidean(p3, p5)) / (2.0 * width)


def get_motion_points(landmarks, indices, w, h):
    """코를 원점으로 하는 상대 좌표 반환 (카메라 평행이동 영향 제거)"""
    points = np.array(
        [get_pixel_coord(landmarks, idx, w, h) for idx in indices],
        dtype=np.float32
    )
    nose = get_pixel_coord(landmarks, 1, w, h)
    return points - nose


def get_eye_distance(landmarks, w, h):
    """눈 사이 거리 (motion score 정규화 기준)"""
    left  = get_pixel_coord(landmarks, LEFT_EYE_CENTER_IDX, w, h)
    right = get_pixel_coord(landmarks, RIGHT_EYE_CENTER_IDX, w, h)
    return max(euclidean(left, right), 1.0)


def compute_motion_score(current_points, ref_points, norm_dist):
    """현재 프레임과 과거 프레임 간 평균 이동 거리 (정규화)"""
    if ref_points is None:
        return 0.0
    displacements = np.linalg.norm(current_points - ref_points, axis=1)
    return np.mean(displacements) / max(norm_dist, 1.0)


def draw_landmark_group(image, landmarks, indices, w, h, color, radius=2):
    """랜드마크 그룹을 이미지에 시각화"""
    for idx in indices:
        pixel = get_pixel_coord(landmarks, idx, w, h).astype(int)
        cv2.circle(image, tuple(pixel), radius, color, -1)


# =========================
# 렌더링 함수
# =========================

def render_wait_open(image, ear):
    """STATE_WAIT_OPEN 화면 표시"""
    if ear < ABS_CLOSED_EAR:
        state_str = "EYES CLOSED"
    elif ear > ABS_OPEN_EAR:
        state_str = "EYES OPEN"
    else:
        state_str = "PARTIAL"

    cv2.putText(image, state_str, (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 3)


def render_running(image, eye_state_text, closed_duration,
                   blink_active, motion_response, final_response,
                   motion_score_raw, motion_score_ema):
    
    """STATE_RUNNING 화면 표시"""
    cv2.putText(image, f"EYE STATE: {eye_state_text}", (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)

    if closed_duration is not None:
        cv2.putText(image, f"CLOSED TIME: {closed_duration:.2f}s", (30, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        if closed_duration >= NO_RESPONSE_THRESHOLD:
            cv2.putText(image, "NO RESPONSE (EYE)", (30, 155), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    if blink_active:
        cv2.putText(image, "BLINKING", (30, 185), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    if DEBUG:
        cv2.putText(image, f"MOTION RAW: {motion_score_raw:.4f}", (30, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)
        cv2.putText(image, f"MOTION EMA: {motion_score_ema:.4f}", (30, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 150, 0), 2)

    if motion_response:
        cv2.putText(image, "MOTION RESPONSE", (30, 280),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)

    color = (0, 255, 0) if final_response else (0, 0, 255)
    label = "FINAL: RESPONSE" if final_response else "FINAL: NO RESPONSE"

    cv2.putText(image, label, (330, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 3)
