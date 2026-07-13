"""
EAR_main.py

기능:
1) MediaPipe FaceMesh 기반 EAR 계산
2) 눈 상태 판단: OPEN / PARTIAL / CLOSED / BLINKING
3) CPR 중 EAR baseline residual로 눈 반응 보정
4) /motor_compression_status의 BPM/count를 이용한 CPR phase 기반 고개 움직임 보정
5) 최종 결과: RESPONSE / NO_RESPONSE
"""

import cv2
import time
import numpy as np
import mediapipe as mp

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Float32, Float32MultiArray
from sensor_msgs.msg import Image

from EAR_config import *
from EAR_functions import (
    compute_ear,
    draw_landmark_group,
    get_eye_distance,
    get_motion_points,
    render_running,
)
from EAR_eye_logic import EyeResponseLogic
from EAR_motion_logic import MotionResponseLogic


mp_face_mesh = mp.solutions.face_mesh


class ConsciousnessDetectorNode(Node):
    def __init__(self):
        super().__init__("Response_detector")

        # -------------------------
        # ROS2 publishers
        # -------------------------
        self.pub_eye_state = self.create_publisher(String, "Response/eye_state", 10)
        self.pub_ear = self.create_publisher(Float32, "Response/ear", 10)
        self.pub_closed_time = self.create_publisher(Float32, "Response/closed_duration", 10)
        self.pub_motion_score = self.create_publisher(Float32, "Response/motion_score", 10)
        self.pub_final = self.create_publisher(String, "Response/final_response", 10)

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.pub_frame = self.create_publisher(Image, "/ear/frame", image_qos)

        # -------------------------
        # ROS2 subscribers
        # -------------------------
        self.sub_motor_state = self.create_subscription(
            String,
            "/motor_state",
            self.motor_state_callback,
            10,
        )
        self.sub_motor_status = self.create_subscription(
            Float32MultiArray,
            "/motor_compression_status",
            self.motor_status_callback,
            10,
        )

        # -------------------------
        # Camera
        # -------------------------
        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        if not self.cap.isOpened():
            self.get_logger().error(f"I cannot open the camera. index={CAMERA_INDEX}")
            raise SystemExit

        # -------------------------
        # State machine
        # -------------------------
        self.state = STATE_WAIT_OPEN
        self.ear_open_samples = []
        self.ear_open_th = None
        self.ear_closed_th = None

        # CPR state
        self.motor_state = "unknown"
        self.is_cpr_running = False
        self.last_motor_state_time = None

        # Logic modules
        self.eye_logic = EyeResponseLogic()
        self.motion_logic = MotionResponseLogic()

        # -------------------------
        # FaceMesh
        # -------------------------
        self.face_mesh = mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info("Response_Detector_Node Start")

    # ==============================
    # ROS callbacks
    # ==============================
    def motor_state_callback(self, msg):
        self.motor_state = msg.data.strip().lower()
        self.last_motor_state_time = time.time()
        self.is_cpr_running = self.motor_state == "compressing"

    def motor_status_callback(self, msg):
        self.motion_logic.update_motor_status(msg.data)

    # ==============================
    # Main frame callback
    # ==============================
    def timer_callback(self):
        success, image = self.cap.read()
        if not success:
            self.get_logger().warn("Ignoring empty camera frame.")
            return

        h, w, _ = image.shape

        image.flags.writeable = False
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.face_mesh.process(image_rgb)
        image.flags.writeable = True

        if not results.multi_face_landmarks:
            self.motion_logic.reset_tracking()
            self.pub_frame.publish(self.make_image_msg(image))
            return

        face_landmarks = results.multi_face_landmarks[0]

        # -------------------------
        # EAR calculation
        # -------------------------
        left_ear = compute_ear(face_landmarks.landmark, LEFT_EYE_LANDMARK, w, h)
        right_ear = compute_ear(face_landmarks.landmark, RIGHT_EYE_LANDMARK, w, h)
        ear = (left_ear + right_ear) / 2.0

        # UI frame annotation
        draw_landmark_group(image, face_landmarks.landmark, LEFT_EYE_LANDMARK, w, h, (0, 0, 255), 1)
        draw_landmark_group(image, face_landmarks.landmark, RIGHT_EYE_LANDMARK, w, h, (0, 0, 255), 1)
        draw_landmark_group(image, face_landmarks.landmark, MOTION_LANDMARKS, w, h, (255, 0, 255), 2)

        # ==============================
        # State machine
        # ==============================
        if self.state == STATE_WAIT_OPEN:
            if ear >= ABS_OPEN_EAR:
                self.ear_open_samples.clear()
                self.motion_logic.reset_tracking()
                self.state = STATE_CALIBRATING

        elif self.state == STATE_CALIBRATING:
            self.ear_open_samples.append(ear)

            current_points = get_motion_points(face_landmarks.landmark, MOTION_LANDMARKS, w, h)
            self.motion_logic.add_motion_points(current_points)

            if len(self.ear_open_samples) >= CALIBRATION_FRAMES:
                mean_open_ear = np.mean(self.ear_open_samples)
                self.ear_open_th = mean_open_ear * 0.80
                self.ear_closed_th = mean_open_ear * 0.40
                self.state = STATE_RUNNING

                self.get_logger().info("[Calibration Done]")
                self.get_logger().info(f"  OPEN EAR Mean : {mean_open_ear:.3f}")
                self.get_logger().info(f"  EAR_OPEN_TH   : {self.ear_open_th:.3f}")
                self.get_logger().info(f"  EAR_CLOSED_TH : {self.ear_closed_th:.3f}")

        elif self.state == STATE_RUNNING:
            self.run_detection(image, face_landmarks, ear, w, h)

        self.pub_frame.publish(self.make_image_msg(image))

    def run_detection(self, image, face_landmarks, ear, w, h):
        current_time = time.time()

        # /motor_state timeout
        if self.last_motor_state_time is None:
            self.is_cpr_running = False
        elif current_time - self.last_motor_state_time > MOTOR_STATE_TIMEOUT:
            self.is_cpr_running = False
            self.motor_state = "timeout"

        eye_result = self.eye_logic.update(
            ear=ear,
            ear_open_th=self.ear_open_th,
            ear_closed_th=self.ear_closed_th,
            current_time=current_time,
            is_cpr_running=self.is_cpr_running,
        )

        current_points = get_motion_points(face_landmarks.landmark, MOTION_LANDMARKS, w, h)
        eye_dist = get_eye_distance(face_landmarks.landmark, w, h)

        motion_result = self.motion_logic.update(
            current_points=current_points,
            eye_dist=eye_dist,
            current_time=current_time,
            is_cpr_running=self.is_cpr_running,
        )

        final_response = self.decide_final_response(
            eye_response=eye_result["eye_response"],
            partial_response=eye_result["partial_response"],
            ear_residual_response=eye_result["ear_residual_response"],
            motion_response=motion_result["motion_response"],
            phase_motion_response=motion_result["phase_motion_response"],
        )

        self.publish_response(
            eye_state_text=eye_result["eye_state_text"],
            ear=ear,
            closed_duration=eye_result["closed_duration"],
            motion_score_ema=motion_result["motion_score_ema"],
            final_response=final_response,
        )

        render_running(
            image,
            eye_result["eye_state_text"],
            eye_result["closed_duration"],
            eye_result["blink_active"],
            motion_result["motion_response"],
            final_response,
            motion_result["motion_score_raw"],
            motion_result["motion_score_ema"],
        )

    def decide_final_response(
        self,
        eye_response,
        partial_response,
        ear_residual_response,
        motion_response,
        phase_motion_response,
    ):
        if self.is_cpr_running:
            # CPR 압박 중:
            # CPR 주기성 흔들림과 다른 EAR/motion 반응만 RESPONSE로 판단
            return bool(
                eye_response is True
                or ear_residual_response
                or phase_motion_response
            )

        # 비압박 상태:
        # 흔들림이 적으므로 기존 눈/고개 움직임 판단 사용
        return bool(
            eye_response is True
            or partial_response is True
            or motion_response
        )

    def publish_response(
        self,
        eye_state_text,
        ear,
        closed_duration,
        motion_score_ema,
        final_response,
    ):
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
        msg_motion.data = float(motion_score_ema)
        self.pub_motion_score.publish(msg_motion)

        msg_final = String()
        msg_final.data = "RESPONSE" if final_response else "NO_RESPONSE"
        self.pub_final.publish(msg_final)

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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
