#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, String


class MotorRosBridge(Node):
    def __init__(self):
        super().__init__("motor_ros_bridge")

        # =========================
        # SUB 수신값
        # =========================
        self.loadcell_total = 0.0
        self.loadcell_stop_request = False
        self.loadcell_status_code = 0
        self.loadcell_warning = False

        # 이전 상태 저장용: 같은 로그 반복 출력 방지
        self.prev_loadcell_stop_request = False
        self.prev_loadcell_status_code = 0
        self.prev_loadcell_warning = False

        self.motor_start = False
        self.motor_stop = False

        # =========================
        # PUB 상태값
        # =========================
        self.motor_state = "IDLE"
        self.contact_position = 0
        self.target_position = 0

        self.compression_count = 0
        self.compression_bpm = 0.0
        self.cpr_start_time = None
        self.last_compression_time = None

        # =========================
        # Publisher
        # =========================
        self.pub_abs_pos = self.create_publisher(Int32, "/motor_absolute_position", 10)     # 모터 절대 위치값(P0B-07)
        self.pub_contact_pos = self.create_publisher(Int32, "/motor_contact_position", 10)  # 로드셀 접촉 순간 저장한 위치값
        self.pub_target_pos = self.create_publisher(Int32, "/motor_target_position", 10)    # 목표 위치값
        self.pub_count = self.create_publisher(Int32, "/motor_compression_count", 10)       # 압박 횟수
        self.pub_bpm = self.create_publisher(Float32, "/motor_compression_bpm", 10)         # 압박 속도[bpm]
        self.pub_time = self.create_publisher(Float32, "/motor_compression_time", 10)       # 압박 진행 시간[초]
        self.pub_current_a = self.create_publisher(Float32, "/motor_current_a", 10)         # 모터 전류값[A], P0B-24 기준
        self.pub_state = self.create_publisher(String, "/motor_state", 10)                  # 모터 상태(IDLE, SEARCHING, RECIPROCATING, STOPPED 등)

        # =========================
        # Subscriber
        # =========================
        self.create_subscription(Float32, "/loadcell_total", self.cb_loadcell_total, 10)            # 로드셀 합산 힘[N], 접촉 감지 기준값   
        self.create_subscription(Bool, "/loadcell_stop_request", self.cb_loadcell_stop_request, 10) # 로드셀 기준 모터 정지 요청
        self.create_subscription(Int32, "/loadcell_status_code", self.cb_loadcell_status_code, 10)  # 로드셀 상태 코드, 0=정상 1=경고 2=트립 3=래치
        self.create_subscription(Bool, "/loadcell_warning", self.cb_loadcell_warning, 10)           # 로드셀 편심 경고, 압박봉 재정비 필요

        self.create_subscription(Bool, "/motor_start", self.cb_motor_start, 10)                     # 모터 시작 명령, UI 시작 버튼에서 발행 예정
        self.create_subscription(Bool, "/motor_stop", self.cb_motor_stop, 10)                       # 모터 정지 명령, UI 정지 버튼에서 발행 예정

        self.get_logger().info("motor_ros_bridge started")

    # =========================
    # Callback
    # =========================
    def cb_loadcell_total(self, msg):
        self.loadcell_total = float(msg.data)

    def cb_loadcell_stop_request(self, msg):
        new_value = bool(msg.data)

        # 값이 바뀔 때만 로그 출력
        if new_value != self.prev_loadcell_stop_request:
            if new_value:
                self.get_logger().error("[LOADCELL] stop_request=True")
            else:
                self.get_logger().info("[LOADCELL] stop_request=False")

        self.loadcell_stop_request = new_value
        self.prev_loadcell_stop_request = new_value

    def cb_loadcell_status_code(self, msg):
        new_code = int(msg.data)

        # 코드가 바뀔 때만 로그 출력
        if new_code != self.prev_loadcell_status_code:
            if new_code == 0:
                self.get_logger().info("[LOADCELL] status_code=0 NORMAL")
            elif new_code == 1:
                self.get_logger().warn("[LOADCELL] status_code=1 WARNING")
            elif new_code == 2:
                self.get_logger().error("[LOADCELL] status_code=2 TRIP")
            elif new_code == 3:
                self.get_logger().error("[LOADCELL] status_code=3 LATCHED_FAULT")
            else:
                self.get_logger().error(f"[LOADCELL] unknown status_code={new_code}")

        self.loadcell_status_code = new_code
        self.prev_loadcell_status_code = new_code

    def cb_loadcell_warning(self, msg):
        new_value = bool(msg.data)

        # 값이 바뀔 때만 로그 출력
        if new_value != self.prev_loadcell_warning:
            if new_value:
                self.get_logger().warn("[LOADCELL] 편심 경고: 압박봉 재정비 필요")
            else:
                self.get_logger().info("[LOADCELL] warning cleared")

        self.loadcell_warning = new_value
        self.prev_loadcell_warning = new_value

    def cb_motor_start(self, msg):
        if msg.data:
            self.motor_start = True
            self.get_logger().info("[MOTOR] start command received")

    def cb_motor_stop(self, msg):
        if msg.data:
            self.motor_stop = True
            self.get_logger().error("[MOTOR] stop command received")

    # =========================
    # ROS spin
    # =========================
    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.0)

    # =========================
    # 정지 조건
    # =========================
    def should_stop(self):
        self.spin_once()
        return (
            self.motor_stop
            or self.loadcell_stop_request
            or self.loadcell_status_code >= 2
        )

    # =========================
    # Publish 함수
    # =========================
    def publish_state(self, state):
        self.motor_state = str(state)
        self.pub_state.publish(String(data=self.motor_state))
        print(f"[ROS STATE] {self.motor_state}")

    def publish_absolute_position(self, pos):
        self.pub_abs_pos.publish(Int32(data=int(pos)))

    def publish_current_a(self, current_a):
        self.pub_current_a.publish(Float32(data=float(current_a)))

    def publish_target_position(self, target_pos):
        self.target_position = int(target_pos)
        self.pub_target_pos.publish(Int32(data=int(target_pos)))

    def publish_contact_position(self, contact_pos):
        self.contact_position = int(contact_pos)
        self.pub_contact_pos.publish(Int32(data=int(contact_pos)))

    def publish_compression_time(self):
        if self.cpr_start_time is None:
            elapsed = 0.0
        else:
            elapsed = time.time() - self.cpr_start_time

        self.pub_time.publish(Float32(data=float(elapsed)))

    def update_compression_count(self):
        """
        왕복 1회 = 압박 1회
        """
        now = time.time()

        self.compression_count += 1

        if self.last_compression_time is not None:
            dt = now - self.last_compression_time
            if dt > 0:
                self.compression_bpm = 60.0 / dt

        self.last_compression_time = now

        self.pub_count.publish(Int32(data=int(self.compression_count)))
        self.pub_bpm.publish(Float32(data=float(self.compression_bpm)))

        print(
            f"[ROS COMPRESSION] count={self.compression_count} | "
            f"bpm={self.compression_bpm:.1f}"
        )