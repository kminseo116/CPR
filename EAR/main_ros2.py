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
from std_msgs.msg import String, Float32

from config import *
from functions import *

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
        motion_response  = False
        motion_score_raw = 0.0
        closed_duration  = None

        if results.multi_face_landmarks:
            face_landmarks = results.multi_face_landmarks[0]

            # EAR 계산
            left_ear  = compute_ear(face_landmarks.landmark, LEFT_EYE_LANDMARK,  w, h)
            right_ear = compute_ear(face_landmarks.landmark, RIGHT_EYE_LANDMARK, w, h)
            ear = (left_ear + right_ear) / 2.0

            cv2.putText(image, f"EAR: {ear:.3f}", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 3)

            # 랜드마크 시각화
            draw_landmark_group(image, face_landmarks.landmark, LEFT_EYE_LANDMARK,  w, h, (0, 0, 255), 2)
            draw_landmark_group(image, face_landmarks.landmark, RIGHT_EYE_LANDMARK, w, h, (0, 0, 255), 2)
            draw_landmark_group(image, face_landmarks.landmark, MOTION_LANDMARKS,   w, h, (255, 0, 255), 3)

            # ==============================
            # 상태 머신
            # ==============================
            if self.state == STATE_WAIT_OPEN:
                render_wait_open(image, ear)
                if ear >= ABS_OPEN_EAR:
                    self.ear_open_samples.clear()
                    self.motion_points_history.clear()
                    self.motion_score_ema  = 0.0
                    self.motion_start_time = None
                    self.state = STATE_CALIBRATING

            elif self.state == STATE_CALIBRATING:
                self.ear_open_samples.append(ear)
                cv2.putText(image, "CALIBRATING...", (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)

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

                # --- 1) 눈 상태 판단 ---
                if ear <= self.ear_closed_th:
                    eye_state_text = "CLOSED"

                    if self.eye_closed_start_time is None:
                        self.eye_closed_start_time = current_time

                    closed_duration = current_time - self.eye_closed_start_time
                    
                    if closed_duration >= NO_RESPONSE_THRESHOLD:
                        eye_response = False   # 확정
                    else:
                        eye_response = None    # 아직 판단 안 함


                elif ear >= self.ear_open_th:
                    eye_state_text  = "OPEN"
                    eye_response    = True      # 현재 눈으로 반응이 보이는지

                    if self.eye_closed_start_time is not None:
                        closed_duration = current_time - self.eye_closed_start_time
                        if closed_duration < BLINK_THRESHOLD:
                            self.last_blink_time = current_time

                    self.eye_closed_start_time = None
                    closed_duration            = None

                else:   # PARTIAL
                    eye_state_text             = "PARTIAL"
                    eye_response               = True
                    closed_duration            = None
                    # self.eye_closed_start_time = None

                blink_active = (
                    self.last_blink_time is not None and
                    (current_time - self.last_blink_time <= BLINK_ACTIVE_GAP)
                )

                if blink_active:
                    eye_state_text = "BLINKING"

                # --- 2) 얼굴 움직임 판단 ---
                current_points = get_motion_points(face_landmarks.landmark, MOTION_LANDMARKS, w, h)
                eye_dist       = get_eye_distance(face_landmarks.landmark, w, h)

                if len(self.motion_points_history) >= MOTION_COMPARE_GAP:
                    ref_points       = self.motion_points_history[0]
                    motion_score_raw = compute_motion_score(current_points, ref_points, eye_dist)
                else:
                    motion_score_raw = 0.0

                self.motion_score_ema = (
                    MOTION_EMA_ALPHA * motion_score_raw +
                    (1.0 - MOTION_EMA_ALPHA) * self.motion_score_ema
                )

                if self.motion_score_ema >= MOTION_THRESHOLD:
                    if self.motion_start_time is None:
                        self.motion_start_time = current_time
                    if current_time - self.motion_start_time >= MOTION_HOLD_TIME:
                        motion_response = True
                else:
                    self.motion_start_time = None
                    motion_response        = False

                self.motion_points_history.append(current_points.copy())

                # --- 3) 최종 판단 ---
                if motion_response or blink_active:
                    final_response = True
                elif eye_response is True:
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

                # 화면 표시
                render_running(image, eye_state_text, closed_duration,
                               blink_active, motion_response, final_response,
                               motion_score_raw, self.motion_score_ema)

        else:
            # 얼굴 없음
            cv2.putText(image, "NO FACE", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
            self.motion_points_history.clear()
            self.motion_start_time = None
            self.motion_score_ema  = 0.0

        cv2.imshow(WINDOW_NAME, image)
        if cv2.waitKey(1) & 0xFF == 27:
            self.destroy_node()

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
