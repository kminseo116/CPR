#!/usr/bin/env python3
"""
loadcell_main.py — 단일 로드셀 모니터링 및 ROS 퍼블리셔

단일 Phidget VoltageRatioInput 채널의 값을 힘(N)으로 변환한다.
기존 모터 코드와의 호환을 위해 측정 힘은 /loadcell_total로 발행한다.
"""

import threading
import time
from collections import deque
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32

from Phidget22.Devices.VoltageRatioInput import BridgeGain, VoltageRatioInput
from Phidget22.PhidgetException import PhidgetException

import loadcell_config as cfg


class LoadCellChannel:
    """단일 Phidget 로드셀 채널의 수신, 필터링 및 오버로드 판정."""

    def __init__(self, channel_id: int) -> None:
        self.channel_id = channel_id
        self._phidget = VoltageRatioInput()

        self._offset = 0.0
        self._window: deque[float] = deque(maxlen=cfg.MOVING_AVG_WINDOW)
        self._latest_ratio: Optional[float] = None
        self._latest_force_n: Optional[float] = None
        self._lock = threading.Lock()

        self._warn = False
        self._trip = False
        self._hard_stop = False
        self._trip_count = 0

    def _on_attach(self, ch: VoltageRatioInput) -> None:
        ch.setBridgeGain(BridgeGain.BRIDGE_GAIN_128)
        ch.setDataInterval(cfg.DATA_INTERVAL_MS)
        print(f"[Attach] channel={self.channel_id}")

    def _on_detach(self, _ch: VoltageRatioInput) -> None:
        print(f"[Detach] channel={self.channel_id}")

    def _on_error(self, _ch: VoltageRatioInput, code: int, description: str) -> None:
        print(f"[Error] channel={self.channel_id} code={code} desc={description}")

    def _on_ratio_change(self, _ch: VoltageRatioInput, ratio: float) -> None:
        with self._lock:
            self._latest_ratio = float(ratio)

    def open(self) -> None:
        self._phidget.setChannel(self.channel_id)
        self._phidget.setOnAttachHandler(self._on_attach)
        self._phidget.setOnDetachHandler(self._on_detach)
        self._phidget.setOnErrorHandler(self._on_error)
        self._phidget.setOnVoltageRatioChangeHandler(self._on_ratio_change)
        self._phidget.openWaitForAttachment(5000)

    def close(self) -> None:
        try:
            self._phidget.close()
        except Exception:
            pass

    def calibrate_zero(self) -> None:
        """무부하 상태의 평균 전압비를 영점 오프셋으로 저장한다."""
        time.sleep(0.1)
        print(f"[Zero] channel={self.channel_id} calibrating...")

        samples: list[float] = []
        interval_s = 1.0 / cfg.ZERO_CAL_SAMPLE_HZ
        end_time = time.time() + cfg.ZERO_CAL_DURATION_S

        while time.time() < end_time:
            try:
                samples.append(float(self._phidget.getVoltageRatio()))
            except PhidgetException:
                pass
            time.sleep(interval_s)

        if len(samples) < cfg.ZERO_CAL_MIN_SAMPLES:
            raise RuntimeError(
                f"영점 보정 실패: 유효 샘플={len(samples)} "
                f"(최소 {cfg.ZERO_CAL_MIN_SAMPLES} 필요)"
            )

        with self._lock:
            self._offset = sum(samples) / len(samples)
            self._latest_force_n = 0.0
            self._window.clear()

        print(f"[Zero] channel={self.channel_id} offset={self._offset:+.9f}")

    def update_force(self) -> Optional[float]:
        with self._lock:
            ratio = self._latest_ratio
            offset = self._offset

        if ratio is None:
            return None

        self._window.append(ratio - offset)
        avg_ratio = sum(self._window) / len(self._window)
        kg = avg_ratio / cfg.RATIO_PER_KG
        force_n = kg * cfg.GRAVITY

        with self._lock:
            self._latest_force_n = force_n

        self._check_overload(force_n)
        return force_n

    def get_force(self) -> Optional[float]:
        with self._lock:
            return self._latest_force_n

    def get_state(self) -> dict[str, bool]:
        return {
            "warn": self._warn,
            "trip": self._trip,
            "hard_stop": self._hard_stop,
        }

    def reset_fault(self) -> None:
        self._warn = False
        self._trip = False
        self._hard_stop = False
        self._trip_count = 0

    def _check_overload(self, force_n: float) -> None:
        self._warn = force_n >= cfg.OL_WARN_N

        if force_n >= cfg.OL_STOP_N:
            self._hard_stop = True
            self._trip = True
            self._trip_count = cfg.OL_CONFIRM
            return

        if force_n >= cfg.OL_TRIP_N:
            self._trip_count += 1
            if self._trip_count >= cfg.OL_CONFIRM:
                self._trip = True
        else:
            self._trip_count = 0
            self._trip = False
            self._hard_stop = False


class LoadCellRosPublisher(Node):
    """단일 로드셀 측정값을 ROS 토픽으로 발행한다."""

    def __init__(self) -> None:
        super().__init__("loadcell_monitor_publisher")

        self._channel = LoadCellChannel(cfg.ACTIVE_CHANNEL)
        self._last_print = 0.0
        self._fault_latch = False
        self._stop_req = False

        self._compressing = False
        self._current_peak_force = 0.0

        self._zero_pending = False
        self._zero_request_time = 0.0

        self._setup_publishers()
        self._reset_sub = self.create_subscription(
            Bool,
            "/loadcell_reset",
            self._reset_callback,
            10,
        )

        self._channel.open()
        self._channel.calibrate_zero()

        self.create_timer(cfg.PUBLISH_PERIOD_S, self._tick)
        self.get_logger().info(
            f"Single loadcell publisher started. channel={cfg.ACTIVE_CHANNEL}"
        )

    def _setup_publishers(self) -> None:
        self._pubs = {
            # 기존 모터 코드가 이 토픽을 구독하므로 이름을 유지한다.
            "total": self.create_publisher(Float32, "/loadcell_total", 10),
            "stop": self.create_publisher(Bool, "/loadcell_stop_request", 10),
            "code": self.create_publisher(Int32, "/loadcell_status_code", 10),
            "peak": self.create_publisher(Float32, "/compression_peak_force", 10),
        }

    def _update_compression_peak(self, force_n: float) -> None:
        if not self._compressing:
            if force_n >= cfg.COMPRESS_START_N:
                self._compressing = True
                self._current_peak_force = force_n
            return

        if force_n > self._current_peak_force:
            self._current_peak_force = force_n

        if force_n <= cfg.COMPRESS_END_N:
            self._pubs["peak"].publish(Float32(data=float(self._current_peak_force)))
            self.get_logger().info(
                f"Compression peak force published: {self._current_peak_force:.1f} N"
            )
            self._compressing = False
            self._current_peak_force = 0.0

    def _reset_callback(self, msg: Bool) -> None:
        if not msg.data:
            return

        self._fault_latch = False
        self._stop_req = False
        self._channel.reset_fault()

        self._zero_pending = True
        self._zero_request_time = time.time()
        self.get_logger().info(
            f"Zero calibration scheduled after {cfg.ZERO_DELAY_AFTER_RESET_S:.1f} s."
        )

    def _handle_delayed_zero_calibration(self, force_n: float) -> None:
        if not self._zero_pending:
            return

        if time.time() - self._zero_request_time < cfg.ZERO_DELAY_AFTER_RESET_S:
            return

        self._zero_pending = False

        if abs(force_n) > cfg.ZERO_SAFE_FORCE_N:
            self.get_logger().warn(
                f"Delayed zero calibration skipped. Current force={force_n:.1f} N "
                f"> safe limit={cfg.ZERO_SAFE_FORCE_N:.1f} N"
            )
            return

        self.get_logger().info(
            f"Delayed zero calibration started. Current force={force_n:.1f} N"
        )
        self._channel.calibrate_zero()
        self._compressing = False
        self._current_peak_force = 0.0
        self.get_logger().info("Delayed loadcell zero calibration completed.")

    def _tick(self) -> None:
        force_n = self._channel.update_force()
        if force_n is None:
            return

        self._handle_delayed_zero_calibration(force_n)
        self._update_compression_peak(force_n)

        state = self._channel.get_state()
        if state["hard_stop"] or state["trip"]:
            self._fault_latch = True
            self._stop_req = True

        if self._fault_latch or state["hard_stop"]:
            status_code = 3
        elif state["trip"]:
            status_code = 2
        elif state["warn"]:
            status_code = 1
        else:
            status_code = 0

        # 단일 센서 값이지만 기존 호환성을 위해 /loadcell_total에 발행한다.
        self._pubs["total"].publish(Float32(data=float(force_n)))
        self._pubs["stop"].publish(Bool(data=self._stop_req))
        self._pubs["code"].publish(Int32(data=status_code))

        self._print_status(force_n, state)

    def _print_status(self, force_n: float, state: dict[str, bool]) -> None:
        now = time.time()
        if now - self._last_print < cfg.PRINT_PERIOD_S:
            return
        self._last_print = now

        if state["hard_stop"]:
            overload = "HARD_STOP"
        elif state["trip"]:
            overload = "TRIP"
        elif state["warn"]:
            overload = "WARN"
        else:
            overload = "NORMAL"

        print(
            f"{cfg.CHANNEL_LABEL}[CH{cfg.ACTIVE_CHANNEL}]:{force_n:7.1f} N  "
            f"STATUS:{overload}  STOP_REQ:{self._stop_req}"
        )

    def destroy_node(self) -> None:
        self._channel.close()
        super().destroy_node()


def main() -> None:
    cfg.print_summary()
    rclpy.init()
    node: Optional[LoadCellRosPublisher] = None

    try:
        node = LoadCellRosPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()