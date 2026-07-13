"""
EAR_eye_logic.py

Eye-state and EAR residual logic for CPR patient response detection.
"""

from EAR_config import *


class EyeResponseLogic:
    def __init__(self):
        self.eye_closed_start_time = None
        self.eye_open_start_time = None
        self.eye_partial_start_time = None
        self.last_blink_time = None

        self.open_duration = 0.0
        self.partial_duration = 0.0

        self.ear_baseline_ema = 0.0
        self.ear_residual = 0.0
        self.ear_residual_start_time = None

    def reset_cpr_ear_baseline(self):
        self.ear_baseline_ema = 0.0
        self.ear_residual = 0.0
        self.ear_residual_start_time = None

    def update(self, ear, ear_open_th, ear_closed_th, current_time, is_cpr_running):
        """
        Returns:
            dict with:
            - eye_state_text: OPEN / PARTIAL / CLOSED / BLINKING
            - eye_response: True / False / None
            - partial_response: bool
            - blink_active: bool
            - closed_duration: float or None
            - ear_residual_response: bool
            - open_duration: float
            - partial_duration: float
            - ear_baseline_ema: float
            - ear_residual: float
        """

        eye_state_text = "N/A"
        eye_response = False
        partial_response = False
        ear_residual_response = False
        closed_duration = None

        self.open_duration = 0.0
        self.partial_duration = 0.0

        # -------------------------
        # 1) Eye-state decision
        # -------------------------
        if ear <= ear_closed_th:
            eye_state_text = "CLOSED"

            self.eye_open_start_time = None
            self.eye_partial_start_time = None

            if self.eye_closed_start_time is None:
                self.eye_closed_start_time = current_time

            closed_duration = current_time - self.eye_closed_start_time

            if closed_duration >= NO_RESPONSE_THRESHOLD:
                eye_response = False
            else:
                eye_response = None

        elif ear >= ear_open_th:
            eye_state_text = "OPEN"

            if self.eye_open_start_time is None:
                self.eye_open_start_time = current_time

            self.open_duration = current_time - self.eye_open_start_time

            if self.open_duration >= OPEN_HOLD_TIME:
                eye_response = True
            else:
                eye_response = None

            if self.eye_closed_start_time is not None:
                closed_duration = current_time - self.eye_closed_start_time
                if closed_duration < BLINK_THRESHOLD:
                    self.last_blink_time = current_time

            self.eye_closed_start_time = None
            self.eye_partial_start_time = None
            closed_duration = None

        else:
            eye_state_text = "PARTIAL"

            if self.eye_partial_start_time is None:
                self.eye_partial_start_time = current_time

            self.partial_duration = current_time - self.eye_partial_start_time
            eye_response = None

            if self.partial_duration >= PARTIAL_HOLD_TIME:
                partial_response = True
            else:
                partial_response = False

            self.eye_open_start_time = None
            closed_duration = None

        blink_active = (
            self.last_blink_time is not None
            and (current_time - self.last_blink_time <= BLINK_ACTIVE_GAP)
        )

        if blink_active:
            eye_state_text = "BLINKING"

        # -------------------------
        # 2) CPR EAR residual compensation
        # -------------------------
        if is_cpr_running:
            if self.ear_baseline_ema == 0.0:
                self.ear_baseline_ema = ear

            # Do not allow confirmed open-eye response to be absorbed into baseline.
            if eye_response is not True:
                self.ear_baseline_ema = (
                    EAR_BASELINE_ALPHA * ear
                    + (1.0 - EAR_BASELINE_ALPHA) * self.ear_baseline_ema
                )

            self.ear_residual = ear - self.ear_baseline_ema

            if self.ear_residual >= EAR_RESIDUAL_THRESHOLD:
                if self.ear_residual_start_time is None:
                    self.ear_residual_start_time = current_time

                if current_time - self.ear_residual_start_time >= EAR_RESIDUAL_HOLD_TIME:
                    ear_residual_response = True
            else:
                self.ear_residual_start_time = None
                ear_residual_response = False

        else:
            self.reset_cpr_ear_baseline()
            ear_residual_response = False

        return {
            "eye_state_text": eye_state_text,
            "eye_response": eye_response,
            "partial_response": partial_response,
            "blink_active": blink_active,
            "closed_duration": closed_duration,
            "ear_residual_response": ear_residual_response,
            "open_duration": self.open_duration,
            "partial_duration": self.partial_duration,
            "ear_baseline_ema": self.ear_baseline_ema,
            "ear_residual": self.ear_residual,
        }
