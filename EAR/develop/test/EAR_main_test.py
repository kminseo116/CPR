"""
CPR 로봇이 체결되면 사람의 얼굴이 인식되고 EAR 값 계산을 통해 사람의 눈 움직임을 통해 의식 여부 확인 알고리즘
1) 얼굴 랜드마크 검출
2) EAR로 눈 상태 판단
3) 얼굴 기준점 이동량으로 움직임 반응 판단
결과: RESPONSE / NO RESPONSE
눈 판단: 눈 뜸(OPEN), 실눈(Partial), 감음(Closed), 1.5s 동안 눈을 감고 있으면 No respond, 깜빡임(BLINKING)
"""

import cv2
import time
import numpy as np
import mediapipe as mp
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Float32, Bool, Float32MultiArray
from sensor_msgs.msg import Image

from EAR_config import *
from EAR_functions import *

mp_face_mesh = mp.solutions.face_mesh


class ConsciousnessDetectorNode(Node):

    def __init__(self):
        super().__init__('Response_detector')

        # -------------------------
        # ROS2 퍼블리셔
        # -------------------------
        self.pub_eye_state    = self.create_publisher(String,  'Response/eye_state',      10)  # OPEN / PARTIAL / CLOSED
        self.pub_ear          = self.create_publisher(Float32, 'Response/ear',            10)  # EAR 수치
        self.pub_closed_time  = self.create_publisher(Float32, 'Response/closed_duration',10)  # 눈 감은 시간
        self.pub_motion_score = self.create_publisher(Float32, 'Response/motion_score',   10)  # 고개 움직임 EMA 값
        self.pub_final        = self.create_publisher(String,  'Response/final_response', 10)  # RESPONSE / NO_RESPONSE
        
        # -------------------------
        # 디버그용 퍼블리셔
        # -------------------------
        self.pub_is_cpr_running = self.create_publisher(Bool,    'Response/is_cpr_running',   10)
        self.pub_partial_resp   = self.create_publisher(Bool,    'Response/partial_response', 10)
        self.pub_open_duration  = self.create_publisher(Float32, 'Response/open_duration',    10)
        self.pub_partial_duration = self.create_publisher(Float32, 'Response/partial_duration', 10)
        self.pub_motion_baseline = self.create_publisher(Float32,'Response/motion_baseline',10)
        self.pub_motion_residual = self.create_publisher(Float32,'Response/motion_residual',10)
        self.pub_motion_residual_resp = self.create_publisher(Bool,'Response/motion_residual_response',10)
        self.pub_ear_baseline = self.create_publisher(Float32, 'Response/ear_baseline', 10)
        self.pub_ear_residual = self.create_publisher(Float32, 'Response/ear_residual', 10)
        self.pub_ear_residual_resp = self.create_publisher(Bool, 'Response/ear_residual_response', 10)
        self.pub_cpr_bpm = self.create_publisher(Float32, 'Response/cpr_bpm', 10)
        self.pub_cpr_phase = self.create_publisher(Float32, 'Response/cpr_phase', 10)
        self.pub_phase_motion_residual = self.create_publisher(Float32, 'Response/phase_motion_residual', 10)
        self.pub_phase_motion_response = self.create_publisher(Bool, 'Response/phase_motion_response', 10)

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        # UI에 영상을 보냄
        self.pub_frame        = self.create_publisher(Image, '/ear/frame', image_qos)  # UI 표시용 주석 포함 EAR 영상

        # -------------------------
        # ROS2 구독자
        # -------------------------
        self.sub_motor_state = self.create_subscription(String, '/motor_state', self.motor_state_callback, 10)  # 모터 상태 구독
        self.sub_motor_status = self.create_subscription(Float32MultiArray, '/motor_compression_status', self.motor_status_callback, 10)
        
        # -------------------------
        # 카메라 초기화
        # -------------------------
        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        if not self.cap.isOpened():
            self.get_logger().error(f"I cannot open the camera. index={CAMERA_INDEX}")
            raise SystemExit

        # -------------------------
        # 상태 변수 초기화
        # -------------------------
        self.state            = STATE_WAIT_OPEN
        self.ear_open_samples = []
        self.ear_open_th      = None
        self.ear_closed_th    = None

        self.motion_points_history = deque(maxlen=MOTION_HISTORY_LEN)
        self.eye_closed_start_time = None
        self.last_blink_time       = None
        self.motion_start_time     = None
        self.motion_score_ema      = 0.0

        self.ear_baseline_ema = 0.0
        self.ear_residual = 0.0
        self.ear_residual_start_time = None

        self.motion_baseline_ema = 0.0
        self.motion_residual_start_time = None
        self.motion_residual = 0.0
        self.last_motion_residual_response_time = None

        self.motor_state = "unknown"        # 모터의 상태를 통해 CPR 로봇이 압박 중인지 확인
        self.is_cpr_running = False
        self.last_motor_state_time = None   # 마지막 /motor_state 수신 시간

        self.compression_count = 0
        self.compression_bpm = 0.0
        self.cpr_period_s = None
        self.last_compression_count = None
        self.last_compression_time = None

        self.phase_motion_baseline = [0.0 for _ in range(CPR_PHASE_BIN_COUNT)]
        self.phase_motion_initialized = [False for _ in range(CPR_PHASE_BIN_COUNT)]
        self.phase_motion_residual = 0.0
        self.phase_motion_response_start_time = None

        self.eye_open_start_time = None     # 일정 시간 이상 유지될 때만 반응 후보로 인정
        self.eye_partial_start_time = None

        # 테스트 용
        self.mock_motion_start_time = time.time()

        # -------------------------
        # FaceMesh 초기화
        # -------------------------
        self.face_mesh = mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

        # -------------------------
        # 타이머 (30fps)
        # -------------------------
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info("Response_Detector_Node Start")

    # ==============================
    # 메인 콜백 (매 프레임 실행)
    # ==============================
    def motor_state_callback(self, msg):
        prev_cpr_running = self.is_cpr_running

        self.motor_state = msg.data.strip().lower()
        self.last_motor_state_time = time.time()

        if self.motor_state == "compressing":
            self.is_cpr_running = True
        else:
            self.is_cpr_running = False

        # CPR이 새로 시작되는 순간 mock motion 시간 초기화
        if self.is_cpr_running and not prev_cpr_running:
            self.mock_motion_start_time = time.time()

    def motor_status_callback(self, msg):
        if len(msg.data) < 4:
            return

        self.compression_count = int(msg.data[0])
        self.compression_bpm = float(msg.data[1])

        if self.compression_bpm > 0:
            self.cpr_period_s = 60.0 / self.compression_bpm

        # compression_count가 바뀌는 순간을 한 압박 주기의 기준점으로 사용
        if self.last_compression_count is None:
            self.last_compression_count = self.compression_count
            self.last_compression_time = time.time()

        elif self.compression_count != self.last_compression_count:
            self.last_compression_count = self.compression_count
            self.last_compression_time = time.time()

    def get_cpr_phase(self):
        if self.cpr_period_s is None:
            return None

        if self.last_compression_time is None:
            return None

        if self.cpr_period_s <= 0:
            return None

        elapsed = time.time() - self.last_compression_time
        phase = (elapsed % self.cpr_period_s) / self.cpr_period_s

        return phase

    def timer_callback(self):
        success, image = self.cap.read()
        if not success:
            self.get_logger().warn("Ignoring empty camera frame.")
            return

        h, w, _ = image.shape

        # FaceMesh 처리
        image.flags.writeable = False
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results   = self.face_mesh.process(image_rgb)
        image.flags.writeable = True

        # 프레임마다 초기화
        eye_state_text   = "N/A"
        eye_response     = False
        partial_response  = False
        motion_response  = False
        ear_residual_response = False
        motion_residual_response = False
        phase_motion_response = False
        motion_score_raw = 0.0
        closed_duration  = None

        self.open_duration = 0.0
        self.partial_duration = 0.0

        if results.multi_face_landmarks:
            face_landmarks = results.multi_face_landmarks[0]

            # EAR 계산
            left_ear  = compute_ear(face_landmarks.landmark, LEFT_EYE_LANDMARK,  w, h)
            right_ear = compute_ear(face_landmarks.landmark, RIGHT_EYE_LANDMARK, w, h)
            ear = (left_ear + right_ear) / 2.0

            # cv2.putText(image, f"EAR: {ear:.3f}", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 3)

            # 랜드마크 시각화
            draw_landmark_group(image, face_landmarks.landmark, LEFT_EYE_LANDMARK,  w, h, (0, 0, 255), 1)
            draw_landmark_group(image, face_landmarks.landmark, RIGHT_EYE_LANDMARK, w, h, (0, 0, 255), 1)
            draw_landmark_group(image, face_landmarks.landmark, MOTION_LANDMARKS,   w, h, (255, 0, 255), 2)

            # ==============================
            # 상태 머신
            # ==============================
            if self.state == STATE_WAIT_OPEN:
                # render_wait_open(image, ear)
                if ear >= ABS_OPEN_EAR:
                    self.ear_open_samples.clear()
                    self.motion_points_history.clear()
                    self.motion_score_ema  = 0.0
                    self.motion_start_time = None
                    self.state = STATE_CALIBRATING

            elif self.state == STATE_CALIBRATING:
                self.ear_open_samples.append(ear)
                cv2.putText(image, "CALIBRATING...", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

                current_points = get_motion_points(face_landmarks.landmark, MOTION_LANDMARKS, w, h)
                self.motion_points_history.append(current_points.copy())

                if len(self.ear_open_samples) >= CALIBRATION_FRAMES:
                    mean_open_ear      = np.mean(self.ear_open_samples)
                    self.ear_open_th   = mean_open_ear * 0.80
                    self.ear_closed_th = mean_open_ear * 0.40
                    self.state = STATE_RUNNING

                    self.get_logger().info("[Calibration Done]")
                    self.get_logger().info(f"  OPEN EAR Mean : {mean_open_ear:.3f}")
                    self.get_logger().info(f"  EAR_OPEN_TH   : {self.ear_open_th:.3f}")
                    self.get_logger().info(f"  EAR_CLOSED_TH : {self.ear_closed_th:.3f}")

            elif self.state == STATE_RUNNING:
                current_time = time.time()

                # --- /motor_state timeout 처리 ---
                if self.last_motor_state_time is None:
                    self.is_cpr_running = False
                elif current_time - self.last_motor_state_time > MOTOR_STATE_TIMEOUT:
                    self.is_cpr_running = False
                    self.motor_state = "timeout"

                # --- 1) 눈 상태 판단 ---
                if ear <= self.ear_closed_th:
                    eye_state_text = "CLOSED"

                    self.eye_open_start_time = None
                    self.eye_partial_start_time = None

                    if self.eye_closed_start_time is None:
                        self.eye_closed_start_time = current_time

                    closed_duration = current_time - self.eye_closed_start_time
                    
                    if closed_duration >= NO_RESPONSE_THRESHOLD:
                        eye_response = False   # 확정
                    else:
                        eye_response = None    # 아직 판단 안 함


                elif ear >= self.ear_open_th:
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

                else:   # PARTIAL
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
                    self.last_blink_time is not None and
                    (current_time - self.last_blink_time <= BLINK_ACTIVE_GAP)
                )

                # --- CPR 중 EAR 흔들림 보정 ---
                if self.is_cpr_running:
                    # 처음 CPR 상태에 들어왔을 때 baseline 초기화
                    if self.ear_baseline_ema == 0.0:
                        self.ear_baseline_ema = ear

                    # 확실한 OPEN 상태에서는 baseline이 눈뜸 값을 따라가지 않도록 제한
                    if eye_response is not True:
                        self.ear_baseline_ema = (
                            EAR_BASELINE_ALPHA * ear +
                            (1.0 - EAR_BASELINE_ALPHA) * self.ear_baseline_ema
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
                    self.ear_baseline_ema = 0.0
                    self.ear_residual = 0.0
                    self.ear_residual_start_time = None
                    ear_residual_response = False

                if blink_active:
                    eye_state_text = "BLINKING"

                # --- 2) 얼굴 움직임 판단 ---
                current_points = get_motion_points(face_landmarks.landmark, MOTION_LANDMARKS, w, h)
                eye_dist       = get_eye_distance(face_landmarks.landmark, w, h)

                if len(self.motion_points_history) >= MOTION_COMPARE_GAP:
                    ref_points       = self.motion_points_history[-MOTION_COMPARE_GAP]
                    motion_score_raw = compute_motion_score(current_points, ref_points, eye_dist)
                else:
                    motion_score_raw = 0.0

                self.motion_score_ema = (
                    MOTION_EMA_ALPHA * motion_score_raw +
                    (1.0 - MOTION_EMA_ALPHA) * self.motion_score_ema
                )

                # --- Mock motion test ---
                if MOCK_MOTION_TEST and self.is_cpr_running and self.cpr_period_s is not None:
                    mock_elapsed = current_time - self.mock_motion_start_time

                    # CPR BPM과 같은 주기의 규칙적 motion 생성
                    regular_motion = MOCK_MOTION_BASE + MOCK_MOTION_AMP * abs(
                        np.sin(2.0 * np.pi * mock_elapsed / self.cpr_period_s)
                    )

                    # 특정 시간 구간에만 불규칙 motion 추가
                    irregular_motion = 0.0
                    if MOCK_IRREGULAR_START <= mock_elapsed <= MOCK_IRREGULAR_END:
                        irregular_motion = MOCK_IRREGULAR_BOOST

                    self.motion_score_ema = regular_motion + irregular_motion
                # ------------------------------------------------------------

                if self.motion_score_ema >= MOTION_THRESHOLD:
                    if self.motion_start_time is None:
                        self.motion_start_time = current_time
                    if current_time - self.motion_start_time >= MOTION_HOLD_TIME:
                        motion_response = True
                else:
                    self.motion_start_time = None
                    motion_response        = False

                # --- CPR 중 고개 움직임 보정 ---
                if self.is_cpr_running:
                    # 처음 CPR 상태에 들어왔을 때 baseline 초기화
                    if self.motion_baseline_ema == 0.0:
                        self.motion_baseline_ema = self.motion_score_ema

                    # baseline 업데이트 전에 임시 residual 계산
                    provisional_residual = self.motion_score_ema - self.motion_baseline_ema

                    # residual이 작을 때만 CPR 기본 흔들림으로 보고 baseline 업데이트
                    # residual이 크면 환자 고개 움직임 후보이므로 baseline이 따라가지 않게 제한
                    if provisional_residual < MOTION_RESIDUAL_THRESHOLD:
                        self.motion_baseline_ema = (
                            MOTION_BASELINE_ALPHA * self.motion_score_ema +
                            (1.0 - MOTION_BASELINE_ALPHA) * self.motion_baseline_ema
                        )

                    # 업데이트 제한 후 최종 residual 계산
                    self.motion_residual = self.motion_score_ema - self.motion_baseline_ema

                    if self.motion_residual >= MOTION_RESIDUAL_THRESHOLD:
                        if self.motion_residual_start_time is None:
                            self.motion_residual_start_time = current_time

                        if current_time - self.motion_residual_start_time >= MOTION_RESIDUAL_HOLD_TIME:
                            motion_residual_response = True
                            self.last_motion_residual_response_time = current_time
                    else:
                        self.motion_residual_start_time = None
                        motion_residual_response = False

                    # 한 번 감지된 고개 반응은 짧은 시간 동안 유지
                    if self.last_motion_residual_response_time is not None:
                        if current_time - self.last_motion_residual_response_time <= MOTION_RESPONSE_LATCH_TIME:
                            motion_residual_response = True

                else:
                    self.motion_baseline_ema = 0.0
                    self.motion_residual_start_time = None
                    self.motion_residual = 0.0
                    self.last_motion_residual_response_time = None
                    motion_residual_response = False

                # --- CPR phase 기반 불규칙 고개 움직임 판단 ---
                if self.is_cpr_running:
                    phase = self.get_cpr_phase()

                    if phase is not None:
                        phase_idx = int(phase * CPR_PHASE_BIN_COUNT)

                        if phase_idx >= CPR_PHASE_BIN_COUNT:
                            phase_idx = CPR_PHASE_BIN_COUNT - 1

                        current_motion = float(self.motion_score_ema)

                        # 해당 phase bin의 baseline 초기화
                        if not self.phase_motion_initialized[phase_idx]:
                            self.phase_motion_baseline[phase_idx] = current_motion
                            self.phase_motion_initialized[phase_idx] = True

                        phase_baseline = self.phase_motion_baseline[phase_idx]
                        self.phase_motion_residual = current_motion - phase_baseline

                        # 해당 phase에서 평소보다 많이 튀지 않을 때만 baseline 업데이트
                        # 크게 튀면 불규칙 움직임 후보이므로 baseline에 흡수하지 않음
                        if self.phase_motion_residual < CPR_PHASE_RESIDUAL_THRESHOLD:
                            self.phase_motion_baseline[phase_idx] = (
                                CPR_PHASE_BASELINE_ALPHA * current_motion +
                                (1.0 - CPR_PHASE_BASELINE_ALPHA) * self.phase_motion_baseline[phase_idx]
                            )

                        # 업데이트 후 residual 재계산
                        phase_baseline = self.phase_motion_baseline[phase_idx]
                        self.phase_motion_residual = current_motion - phase_baseline

                        if self.phase_motion_residual >= CPR_PHASE_RESIDUAL_THRESHOLD:
                            if self.phase_motion_response_start_time is None:
                                self.phase_motion_response_start_time = current_time

                            if current_time - self.phase_motion_response_start_time >= CPR_PHASE_RESIDUAL_HOLD_TIME:
                                phase_motion_response = True
                        else:
                            self.phase_motion_response_start_time = None
                            phase_motion_response = False

                    else:
                        self.phase_motion_response_start_time = None
                        self.phase_motion_residual = 0.0
                        phase_motion_response = False

                else:
                    self.phase_motion_baseline = [0.0 for _ in range(CPR_PHASE_BIN_COUNT)]
                    self.phase_motion_initialized = [False for _ in range(CPR_PHASE_BIN_COUNT)]
                    self.phase_motion_response_start_time = None
                    self.phase_motion_residual = 0.0
                    phase_motion_response = False

                self.motion_points_history.append(current_points.copy())

                # --- 3) 최종 판단 ---
                if self.is_cpr_running:
                    # CPR 압박 중:
                    # EAR residual과 CPR phase 기반 motion residual을 사용하여
                    # CPR 주기성 흔들림과 다른 불규칙 움직임만 반응으로 판단

                    if eye_response is True:
                        final_response = True
                    elif ear_residual_response:
                        final_response = True
                    elif phase_motion_response:
                        final_response = True
                    else:
                        final_response = False

                else:
                    # CPR 압박 전:
                    # 흔들림이 적으므로 기존 판단 사용

                    if eye_response is True:
                        final_response = True
                    elif partial_response is True:
                        final_response = True
                    elif motion_response:
                        final_response = True
                    else:
                        final_response = False

                # ==============================
                # ROS2 토픽 퍼블리시
                # ==============================
                msg_eye = String()
                msg_eye.data = eye_state_text
                self.pub_eye_state.publish(msg_eye)

                msg_ear = Float32()
                msg_ear.data = float(ear)
                self.pub_ear.publish(msg_ear)

                msg_closed_time = Float32()
                msg_closed_time.data = float(closed_duration) if closed_duration is not None else 0.0
                self.pub_closed_time.publish(msg_closed_time)

                msg_motion = Float32()
                msg_motion.data = float(self.motion_score_ema)
                self.pub_motion_score.publish(msg_motion)

                msg_final = String()
                msg_final.data = "RESPONSE" if final_response else "NO_RESPONSE"
                self.pub_final.publish(msg_final)

                # ==============================
                # 디버그 토픽 퍼블리시
                # ==============================
                msg_cpr = Bool()
                msg_cpr.data = bool(self.is_cpr_running)
                self.pub_is_cpr_running.publish(msg_cpr)

                msg_partial = Bool()
                msg_partial.data = bool(partial_response)
                self.pub_partial_resp.publish(msg_partial)

                msg_open_duration = Float32()
                msg_open_duration.data = float(self.open_duration)
                self.pub_open_duration.publish(msg_open_duration)

                msg_partial_duration = Float32()
                msg_partial_duration.data = float(self.partial_duration)
                self.pub_partial_duration.publish(msg_partial_duration)

                msg_motion_baseline = Float32()
                msg_motion_baseline.data = float(self.motion_baseline_ema)
                self.pub_motion_baseline.publish(msg_motion_baseline)

                msg_motion_residual = Float32()
                msg_motion_residual.data = float(self.motion_residual)
                self.pub_motion_residual.publish(msg_motion_residual)

                msg_motion_residual_resp = Bool()
                msg_motion_residual_resp.data = bool(motion_residual_response)
                self.pub_motion_residual_resp.publish(msg_motion_residual_resp)

                msg_ear_baseline = Float32()
                msg_ear_baseline.data = float(self.ear_baseline_ema)
                self.pub_ear_baseline.publish(msg_ear_baseline)

                msg_ear_residual = Float32()
                msg_ear_residual.data = float(self.ear_residual)
                self.pub_ear_residual.publish(msg_ear_residual)

                msg_ear_residual_resp = Bool()
                msg_ear_residual_resp.data = bool(ear_residual_response)
                self.pub_ear_residual_resp.publish(msg_ear_residual_resp)

                msg_cpr_bpm = Float32()
                msg_cpr_bpm.data = float(self.compression_bpm)
                self.pub_cpr_bpm.publish(msg_cpr_bpm)

                phase = self.get_cpr_phase()
                msg_cpr_phase = Float32()
                msg_cpr_phase.data = float(phase) if phase is not None else -1.0
                self.pub_cpr_phase.publish(msg_cpr_phase)

                msg_phase_motion_residual = Float32()
                msg_phase_motion_residual.data = float(self.phase_motion_residual)
                self.pub_phase_motion_residual.publish(msg_phase_motion_residual)

                msg_phase_motion_response = Bool()
                msg_phase_motion_response.data = bool(phase_motion_response)
                self.pub_phase_motion_response.publish(msg_phase_motion_response)

                # 화면 표시
                render_running(image, eye_state_text, closed_duration,
                               blink_active, motion_response, final_response,
                               motion_score_raw, self.motion_score_ema)

        else:
            # 얼굴 없음
            # cv2.putText(image, "NO FACE", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
            self.motion_points_history.clear()
            self.motion_start_time = None
            self.motion_score_ema  = 0.0

        # EAR 화면 표시/ UI에 대신 뜨도록 해서 주석처리
        cv2.imshow(WINDOW_NAME, image)
        if cv2.waitKey(1) & 0xFF == 27:
            self.destroy_node()
        self.pub_frame.publish(self.make_image_msg(image))

    def make_image_msg(self, image):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ear_camera"
        msg.height = image.shape[0]
        msg.width = image.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = image.strides[0]
        msg.data = image.tobytes()
        return msg

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        self.face_mesh.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = ConsciousnessDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
