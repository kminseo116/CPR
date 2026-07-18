#!/usr/bin/env python3
"""CPR 모터 제어용 ROS2 입출력 브리지."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32, String


class MotorRosBridge(Node):
    """로드셀·시작·정지 토픽을 받고 모터 상태를 발행한다."""

    def __init__(self) -> None:
        super().__init__("new_motor_ros_bridge")

        self._initialize_input_state()
        self._initialize_compression_state()
        self._create_publishers()
        self._create_subscribers()

    # ======================================================================
    # 1. 내부 상태 초기화
    # ======================================================================

    def _initialize_input_state(self) -> None:
        self.loadcell_total = 0.0
        self.loadcell_stop_request = False
        self.last_loadcell_time: float | None = None

        self.motor_start = False
        self.motor_stop = False

    def _initialize_compression_state(self) -> None:
        self.compression_count = 0
        self.compression_bpm = 0.0
        self.compression_depth_cm = 0.0
        self.cpr_start_time: float | None = None
        self.last_compression_time: float | None = None

    # ======================================================================
    # 2. Publisher / Subscriber 생성
    # ======================================================================

    def _create_publishers(self) -> None:
        self.pub_position = self.create_publisher(
            Int32,
            "/motor_absolute_position",
            10,
        )
        self.pub_current = self.create_publisher(
            Float32,
            "/motor_current_raw",
            10,
        )
        self.pub_state = self.create_publisher(
            String,
            "/motor_state",
            10,
        )
        self.pub_status = self.create_publisher(
            Float32MultiArray,
            "/motor_compression_status",
            10,
        )
        self.pub_contact_source = self.create_publisher(
            String,
            "/motor_contact_source",
            10,
        )

    def _create_subscribers(self) -> None:
        self.create_subscription(
            Float32,
            "/loadcell_total",
            self._on_loadcell_total,
            10,
        )
        self.create_subscription(
            Bool,
            "/loadcell_stop_request",
            self._on_loadcell_stop,
            10,
        )
        self.create_subscription(
            Bool,
            "/motor_start",
            self._on_motor_start,
            10,
        )
        self.create_subscription(
            Bool,
            "/motor_stop",
            self._on_motor_stop,
            10,
        )

    # ======================================================================
    # 3. Subscriber 콜백
    # ======================================================================

    def _on_loadcell_total(self, msg: Float32) -> None:
        self.loadcell_total = float(msg.data)
        self.last_loadcell_time = time.monotonic()

    def _on_loadcell_stop(self, msg: Bool) -> None:
        self.loadcell_stop_request = bool(msg.data)

    def _on_motor_start(self, msg: Bool) -> None:
        if msg.data:
            self.motor_start = True

    def _on_motor_stop(self, msg: Bool) -> None:
        if msg.data:
            self.motor_stop = True

    # ======================================================================
    # 4. 입력 상태 확인
    # ======================================================================

    def spin_once(self, timeout_sec: float = 0.0) -> None:
        rclpy.spin_once(self, timeout_sec=timeout_sec)

    def get_stop_reason(self) -> str | None:
        """현재 정지 요청의 원인을 반환한다."""

        if self.motor_stop:
            return "MOTOR_STOP"

        if self.loadcell_stop_request:
            return "LOADCELL_STOP_REQUEST"

        return None

    def should_stop(self) -> bool:
        """정지 요청 존재 여부를 반환한다."""
        return self.get_stop_reason() is not None

    def is_loadcell_stale(self, timeout_s: float) -> bool:
        self.spin_once()

        if self.last_loadcell_time is None:
            return True

        elapsed = time.monotonic() - self.last_loadcell_time
        return elapsed > timeout_s

    def clear_start_stop_flags(self) -> None:
        self.motor_start = False
        self.motor_stop = False
        self.loadcell_stop_request = False

    # ======================================================================
    # 5. 기본 상태 발행
    # ======================================================================

    def publish_state(self, state: str) -> None:
        self.pub_state.publish(String(data=state))
        print(f"[ROS STATE] {state}")

    def publish_contact_source(self, source: str) -> None:
        self.pub_contact_source.publish(String(data=source))

    def publish_motor_status(
        self,
        position: int,
        current_raw: int,
    ) -> None:
        self.pub_position.publish(Int32(data=int(position)))
        self.pub_current.publish(Float32(data=float(current_raw)))

    # ======================================================================
    # 6. 압박 상태 계산 및 발행
    # ======================================================================

    def reset_compression(self) -> None:
        self.compression_count = 0
        self.compression_bpm = 0.0
        self.compression_depth_cm = 0.0
        self.cpr_start_time = time.monotonic()
        self.last_compression_time = None
        self.publish_compression_status()

    def count_compression(self, depth_cm: float) -> None:
        now = time.monotonic()
        self.compression_count += 1

        if self.last_compression_time is not None:
            cycle_time_s = now - self.last_compression_time
            if cycle_time_s > 0:
                self.compression_bpm = 60.0 / cycle_time_s

        self.last_compression_time = now
        self.compression_depth_cm = float(depth_cm)
        self.publish_compression_status()

    def publish_compression_status(self) -> None:
        if self.cpr_start_time is None:
            elapsed_s = 0.0
        else:
            elapsed_s = time.monotonic() - self.cpr_start_time

        message = Float32MultiArray()
        message.data = [
            float(self.compression_count),
            float(self.compression_bpm),
            float(elapsed_s),
            float(self.compression_depth_cm),
        ]
        self.pub_status.publish(message)
