#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

import pyrealsense2 as rs
import numpy as np
import cv2
import sys
import time

from ultralytics import YOLO


# ============================================================
# 설정값
# ============================================================

# YOLO COCO person class
PERSON_CLASS_ID = 41

# YOLO confidence threshold
CONFIDENCE_THRESHOLD = 0.5

# Bounding Box 중심에서 좌우/상하 몇 pixel을 사용할지
ROI_HALF_SIZE = 10
# → 10이면 약 20 x 20 pixel 영역

# 정상적인 Depth로 인정할 범위
MIN_DEPTH = 0.2       # [m]
MAX_DEPTH = 5.0       # [m]


class PatientDepthNode(Node):

    def __init__(self):

        super().__init__('patient_depth_node')

        # ========================================================
        # ROS2 설정
        # ========================================================
        self.measure_start_time = None
        self.measure_duration = 10.0

        # goal_pose 완료 전에는 카메라 측정 비활성화
        self.detection_enabled = False

        # Depth 값이 한 번 발행되면 다시 발행하지 않도록 설정
        self.depth_published = False

        # goal_pose 완료 토픽 구독
        self.create_subscription(
            Bool,
            '/nav_complete',
            self.nav_complete_callback,
            10
        )

        # 최종 Depth 값 발행
        self.depth_pub = self.create_publisher(
            Float32,
            '/patient_distance',
            10
        )


        # ========================================================
        # RealSense 초기화
        # ========================================================

        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8,30)


        # --------------------------------------------------------
        # Depth를 Color frame에 맞춤
        # --------------------------------------------------------

        self.align = rs.align(rs.stream.color)


        # ========================================================
        # YOLO
        # ========================================================

        self.model = YOLO('yolo11n.pt')


        # ========================================================
        # 최근 Depth 저장
        # ========================================================

        self.depth_buffer = []


        # ========================================================
        # RealSense 시작
        # ========================================================

        self.pipeline.start(self.config)

        self.get_logger().info("==========================================")
        self.get_logger().info("Patient Depth Node")
        self.get_logger().info("==========================================")
        self.get_logger().info("/nav_complete 대기 중...")
        self.get_logger().info("==========================================")


        # ========================================================
        # Timer
        # ========================================================

        self.timer = self.create_timer(
            0.05,
            self.camera_callback
        )


    # ============================================================
    # goal_pose 완료 토픽 수신
    # ============================================================

    def nav_complete_callback(self, msg):

        if msg.data:

            self.depth_buffer.clear()

            self.depth_published = False
            self.detection_enabled = True

            self.measure_start_time = time.time()

            self.get_logger().info(
                "goal_pose 완료 → 10초간 Depth 측정 시작"
            )


    # ============================================================
    # 카메라 동작
    # ============================================================

    def camera_callback(self):

        # goal_pose가 완료되지 않았으면 카메라 측정 안 함
        if not self.detection_enabled:
            return

        # Depth를 이미 발행했으면 다시 측정 안 함
        if self.depth_published:
            return


        # =================================================
        # Frame 획득
        # =================================================

        frames = self.pipeline.wait_for_frames()

        aligned_frames = self.align.process(frames)

        depth_frame = (aligned_frames.get_depth_frame())
        color_frame = (aligned_frames.get_color_frame())


        if not depth_frame or not color_frame:
            return


        color_image = np.asanyarray(color_frame.get_data())


        # =================================================
        # YOLO
        # =================================================

        results = self.model(color_image, verbose=False)

        person_found = False


        # =================================================
        # Detection
        # =================================================

        for result in results:

            if result.boxes is None:
                continue


            for box in result.boxes:

                class_id = int(box.cls[0].cpu().numpy())
                confidence = float(box.conf[0].cpu().numpy())

                # -----------------------------------------
                # 사람만 사용
                # -----------------------------------------

                if class_id != PERSON_CLASS_ID:
                    continue

                if confidence < CONFIDENCE_THRESHOLD:
                    continue


                person_found = True


                # =========================================
                # Bounding Box
                # =========================================

                x1, y1, x2, y2 = (box.xyxy[0].cpu().numpy().astype(int))

                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2


                # =========================================
                # 중심 주변 ROI
                # =========================================

                roi_x1 = max(0, center_x - ROI_HALF_SIZE)
                roi_x2 = min(depth_frame.get_width(), center_x + ROI_HALF_SIZE)


                roi_y1 = max(0, center_y - ROI_HALF_SIZE)
                roi_y2 = min(depth_frame.get_height(), center_y + ROI_HALF_SIZE)


                # =========================================
                # ROI 내부 Depth 수집
                # =========================================

                depth_values = []


                for y in range(roi_y1, roi_y2):

                    for x in range(roi_x1, roi_x2):

                        # get_distance()
                        # 단위 = meter
                        depth = (depth_frame.get_distance(x, y))


                        # --------------------------------
                        # 비정상값 제거
                        # --------------------------------

                        if (MIN_DEPTH
                            < depth
                            < MAX_DEPTH
                        ):

                            depth_values.append(
                                depth
                            )


                # =========================================
                # 유효 Depth가 없을 경우
                # =========================================

                if len(depth_values) == 0:

                    cv2.putText(
                        color_image,
                        "No valid depth",
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 0, 255),
                        2
                    )

                    continue


                # =========================================
                # 현재 프레임의 Depth
                # =========================================

                frame_depth = float(np.median(depth_values))


                # =========================================
                # 최근 Depth 저장
                # =========================================

                self.depth_buffer.append(frame_depth)


                # =========================================
                # 여러 프레임 Median
                # =========================================

                filtered_depth = float(np.median(self.depth_buffer))

                # =========================================
                # Terminal 출력
                # =========================================

                output = (

                    f"Raw: {frame_depth:.3f} m | "

                    f"Filtered: "
                    f"{filtered_depth:.3f} m | "

                    f"Valid pixels: "
                    f"{len(depth_values)}"

                )


                sys.stdout.write(
                    "\r"
                    + output.ljust(120)
                )

                sys.stdout.flush()


                # =========================================
                # Bounding Box 출력
                # =========================================

                cv2.rectangle(color_image,
                    (x1, y1), (x2, y2),
                    (64, 224, 208), 2)


                # =========================================
                # 중심점 표시
                # =========================================

                cv2.circle(color_image,(center_x, center_y),
                    4, (0, 255, 0), -1)


                # =========================================
                # Depth 측정 ROI 표시
                # =========================================

                cv2.rectangle(color_image,(roi_x1, roi_y1),(roi_x2, roi_y2),
                    (255, 0, 0),2)


                # =========================================
                # 화면에 Depth 표시
                # =========================================

                depth_text = (

                    f"Depth: "
                    f"{filtered_depth:.2f} m"

                )


                cv2.putText(
                    color_image,
                    depth_text,
                    (x1, max(20, y1 - 30)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )


                # =========================================
                # Depth 값 ROS2 발행
                # =========================================

                elapsed_time = time.time() - self.measure_start_time
                remaining_time = max(0.0, self.measure_duration - elapsed_time)

                self.get_logger().info(
                    f"Depth: {filtered_depth:.3f} m | "
                    f"남은 시간: {remaining_time:.1f} sec"
                )

                # 10초 동안 측정 후 최종 Depth 발행
                if elapsed_time >= self.measure_duration:

                    final_depth = float(np.median(self.depth_buffer))

                    msg = Float32()
                    msg.data = final_depth

                    self.depth_pub.publish(msg)

                    print()
                    self.get_logger().info(
                        f"최종 Depth: {final_depth:.3f} m"
                    )

                    self.get_logger().info(
                        "/patient_distance 발행 완료"
                    )

                    # 한 번만 발행하도록 설정
                    self.depth_published = True

                    # Depth 측정 종료
                    self.detection_enabled = False


                # -----------------------------------------
                # 현재는 첫 번째 사람만 사용
                # -----------------------------------------

                break


            if person_found:
                break


        # =================================================
        # 사람이 없을 경우
        # =================================================

        if not person_found:

            # self.depth_buffer.clear()

            sys.stdout.write(
                "\r"
                + "Person not detected".ljust(120)
            )

            sys.stdout.flush()


        # =================================================
        # 화면 출력
        # =================================================

        cv2.imshow("Patient Depth Test",color_image)

        key = cv2.waitKey(1) & 0xFF

        # ESC
        if key == 27:

            print()


    # ============================================================
    # 종료
    # ============================================================

    def destroy_node(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
        super().destroy_node()


def main():

    rclpy.init()

    node = PatientDepthNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":

    main()