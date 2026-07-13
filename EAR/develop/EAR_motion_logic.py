"""
EAR_motion_logic.py

Face motion and CPR phase-based irregular motion logic.
"""

import time
from collections import deque

from EAR_config import *
from EAR_functions import compute_motion_score


class MotionResponseLogic:
    def __init__(self):
        self.motion_points_history = deque(maxlen=MOTION_HISTORY_LEN)
        self.motion_start_time = None
        self.motion_score_ema = 0.0

        # Compression status for CPR phase calculation
        self.compression_count = 0
        self.compression_bpm = 0.0
        self.cpr_period_s = None
        self.last_compression_count = None
        self.last_compression_time = None

        # CPR phase-based motion baseline
        self.phase_motion_baseline = [0.0 for _ in range(CPR_PHASE_BIN_COUNT)]
        self.phase_motion_initialized = [False for _ in range(CPR_PHASE_BIN_COUNT)]
        self.phase_motion_residual = 0.0
        self.phase_motion_response_start_time = None

    def reset_tracking(self):
        self.motion_points_history.clear()
        self.motion_start_time = None
        self.motion_score_ema = 0.0

    def add_motion_points(self, current_points):
        self.motion_points_history.append(current_points.copy())

    def reset_phase_motion_baseline(self):
        self.phase_motion_baseline = [0.0 for _ in range(CPR_PHASE_BIN_COUNT)]
        self.phase_motion_initialized = [False for _ in range(CPR_PHASE_BIN_COUNT)]
        self.phase_motion_response_start_time = None
        self.phase_motion_residual = 0.0

    def update_motor_status(self, data):
        """
        /motor_compression_status

        data[0] = compression_count
        data[1] = compression_bpm
        data[2] = compression_time_s
        data[3] = compression_depth_cm
        """
        if len(data) < 4:
            return

        self.compression_count = int(data[0])
        self.compression_bpm = float(data[1])

        if self.compression_bpm > 0:
            self.cpr_period_s = 60.0 / self.compression_bpm

        if self.last_compression_count is None:
            self.last_compression_count = self.compression_count
            self.last_compression_time = time.time()

        elif self.compression_count != self.last_compression_count:
            self.last_compression_count = self.compression_count
            self.last_compression_time = time.time()

    def get_cpr_phase(self):
        """
        Returns current CPR cycle phase in [0.0, 1.0).
        None is returned when compression status is not available.
        """
        if self.cpr_period_s is None:
            return None
        if self.last_compression_time is None:
            return None
        if self.cpr_period_s <= 0:
            return None

        elapsed = time.time() - self.last_compression_time
        return (elapsed % self.cpr_period_s) / self.cpr_period_s

    def update(self, current_points, eye_dist, current_time, is_cpr_running):
        """
        Returns:
            dict with:
            - motion_score_raw: float
            - motion_score_ema: float
            - motion_response: bool
            - phase_motion_response: bool
            - phase_motion_residual: float
        """

        motion_score_raw = 0.0
        motion_response = False
        phase_motion_response = False

        # -------------------------
        # 1) Face motion score
        # -------------------------
        if len(self.motion_points_history) >= MOTION_COMPARE_GAP:
            ref_points = self.motion_points_history[-MOTION_COMPARE_GAP]
            motion_score_raw = compute_motion_score(current_points, ref_points, eye_dist)
        else:
            motion_score_raw = 0.0

        self.motion_score_ema = (
            MOTION_EMA_ALPHA * motion_score_raw
            + (1.0 - MOTION_EMA_ALPHA) * self.motion_score_ema
        )

        if self.motion_score_ema >= MOTION_THRESHOLD:
            if self.motion_start_time is None:
                self.motion_start_time = current_time
            if current_time - self.motion_start_time >= MOTION_HOLD_TIME:
                motion_response = True
        else:
            self.motion_start_time = None
            motion_response = False

        # -------------------------
        # 2) CPR phase-based irregular motion decision
        # -------------------------
        if is_cpr_running:
            phase = self.get_cpr_phase()

            if phase is not None:
                phase_idx = int(phase * CPR_PHASE_BIN_COUNT)
                if phase_idx >= CPR_PHASE_BIN_COUNT:
                    phase_idx = CPR_PHASE_BIN_COUNT - 1

                current_motion = float(self.motion_score_ema)

                if not self.phase_motion_initialized[phase_idx]:
                    self.phase_motion_baseline[phase_idx] = current_motion
                    self.phase_motion_initialized[phase_idx] = True

                phase_baseline = self.phase_motion_baseline[phase_idx]
                self.phase_motion_residual = current_motion - phase_baseline

                # Update baseline only when current motion is close to the expected
                # periodic CPR artifact. Large deviations are treated as irregular
                # motion candidates and are not absorbed into baseline.
                if self.phase_motion_residual < CPR_PHASE_RESIDUAL_THRESHOLD:
                    self.phase_motion_baseline[phase_idx] = (
                        CPR_PHASE_BASELINE_ALPHA * current_motion
                        + (1.0 - CPR_PHASE_BASELINE_ALPHA)
                        * self.phase_motion_baseline[phase_idx]
                    )

                phase_baseline = self.phase_motion_baseline[phase_idx]
                self.phase_motion_residual = current_motion - phase_baseline

                if self.phase_motion_residual >= CPR_PHASE_RESIDUAL_THRESHOLD:
                    if self.phase_motion_response_start_time is None:
                        self.phase_motion_response_start_time = current_time

                    if (
                        current_time - self.phase_motion_response_start_time
                        >= CPR_PHASE_RESIDUAL_HOLD_TIME
                    ):
                        phase_motion_response = True
                else:
                    self.phase_motion_response_start_time = None
                    phase_motion_response = False

            else:
                self.phase_motion_response_start_time = None
                self.phase_motion_residual = 0.0
                phase_motion_response = False

        else:
            self.reset_phase_motion_baseline()
            phase_motion_response = False

        self.add_motion_points(current_points)

        return {
            "motion_score_raw": motion_score_raw,
            "motion_score_ema": self.motion_score_ema,
            "motion_response": motion_response,
            "phase_motion_response": phase_motion_response,
            "phase_motion_residual": self.phase_motion_residual,
        }
