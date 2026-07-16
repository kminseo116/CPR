#!/usr/bin/env python3
"""
loadcell_single_main.py — 단일 로드셀 모니터링 및 ROS 퍼블리셔

기능
1. 단일 Phidget 로드셀의 전압비를 힘(N)으로 변환
2. /loadcell_total 발행
3. 압박 1회 최대 힘 /compression_peak_force 발행
4. 과부하 상태 /loadcell_status_code 발행
5. 과부하 발생 시 /loadcell_stop_request 발행
6. /motor_stop=True 수신 후 5초 뒤 영점 보정
"""

import threading
import time
from collections import deque
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32

from Phidget22.Devices.VoltageRatioInput import (
    BridgeGain,
    VoltageRatioInput,
)
from Phidget22.PhidgetException import PhidgetException

import loadcell_single_config as cfg


class LoadCellChannel:
    """단일 Phidget 로드셀 채널의 수신, 필터링 및 과부하 판정."""

    def __init__(self, channel_id: int) -> None:
        self.channel_id = channel_id
        self._phidget = VoltageRatioInput()

        # 영점 오프셋
        self._offset = 0.0

        # 이동평균 필터
        self._window: deque[float] = deque(
            maxlen=cfg.MOVING_AVG_WINDOW
        )

        # 최신 측정값
        self._latest_ratio: Optional[float] = None
        self._latest_force_n: Optional[float] = None

        # Phidget 이벤트 스레드와 ROS 스레드 간 데이터 보호
        self._lock = threading.Lock()

        # 과부하 상태
        self._warn = False
        self._trip = False
        self._hard_stop = False
        self._trip_count = 0

    # ── Phidget 이벤트 콜백 ────────────────────────────────────
    def _on_attach(self, ch: VoltageRatioInput) -> None:
        ch.setBridgeGain(BridgeGain.BRIDGE_GAIN_128)
        ch.setDataInterval(cfg.DATA_INTERVAL_MS)

        print(f"[Attach] channel={self.channel_id}")

    def _on_detach(self, _ch: VoltageRatioInput) -> None:
        print(f"[Detach] channel={self.channel_id}")

    def _on_error(
        self,
        _ch: VoltageRatioInput,
        code: int,
        description: str,
    ) -> None:
        print(
            f"[Error] channel={self.channel_id} "
            f"code={code} desc={description}"
        )

    def _on_ratio_change(
        self,
        _ch: VoltageRatioInput,
        ratio: float,
    ) -> None:
        with self._lock:
            self._latest_ratio = float(ratio)

    # ── Phidget 연결 ───────────────────────────────────────────
    def open(self) -> None:
        """설정한 PhidgetBridge 채널을 연다."""
        self._phidget.setChannel(self.channel_id)

        self._phidget.setOnAttachHandler(self._on_attach)
        self._phidget.setOnDetachHandler(self._on_detach)
        self._phidget.setOnErrorHandler(self._on_error)
        self._phidget.setOnVoltageRatioChangeHandler(
            self._on_ratio_change
        )

        self._phidget.openWaitForAttachment(5000)

    def close(self) -> None:
        """Phidget 채널을 닫는다."""
        try:
            self._phidget.close()
        except Exception:
            pass

    # ── 영점 보정 ─────────────────────────────────────────────
    def calibrate_zero(self) -> None:
        """무부하 상태의 평균 전압비를 영점 오프셋으로 저장한다."""
        time.sleep(0.1)

        print(
            f"[Zero] channel={self.channel_id} calibrating..."
        )

        samples: list[float] = []

        interval_s = 1.0 / cfg.ZERO_CAL_SAMPLE_HZ
        end_time = time.time() + cfg.ZERO_CAL_DURATION_S

        while time.time() < end_time:
            try:
                ratio = self._phidget.getVoltageRatio()
                samples.append(float(ratio))

            except PhidgetException:
                pass

            time.sleep(interval_s)

        if len(samples) < cfg.ZERO_CAL_MIN_SAMPLES:
            raise RuntimeError(
                f"영점 보정 실패: 유효 샘플={len(samples)}, "
                f"최소 샘플={cfg.ZERO_CAL_MIN_SAMPLES}"
            )

        offset = sum(samples) / len(samples)

        with self._lock:
            self._offset = offset
            self._latest_force_n = 0.0
            self._window.clear()

        print(
            f"[Zero] channel={self.channel_id} "
            f"offset={self._offset:+.9f}"
        )

    # ── 힘 계산 ────────────────────────────────────────────────
    def update_force(self) -> Optional[float]:
        """최신 전압비를 힘(N)으로 변환하고 과부하를 판정한다."""
        with self._lock:
            ratio = self._latest_ratio
            offset = self._offset

        if ratio is None:
            return None

        # 영점 오프셋 제거
        corrected_ratio = ratio - offset

        # 이동평균 필터 적용
        self._window.append(corrected_ratio)
        avg_ratio = sum(self._window) / len(self._window)

        # 전압비 → kg → N
        kg = avg_ratio / cfg.RATIO_PER_KG
        force_n = kg * cfg.GRAVITY

        with self._lock:
            self._latest_force_n = force_n

        self._check_overload(force_n)

        return force_n

    def get_force(self) -> Optional[float]:
        with self._lock:
            return self._latest_force_n

    # ── 과부하 상태 ────────────────────────────────────────────
    def get_state(self) -> dict[str, bool]:
        return {
            "warn": self._warn,
            "trip": self._trip,
            "hard_stop": self._hard_stop,
        }

    def reset_fault(self) -> None:
        """과부하 상태를 초기화한다."""
        self._warn = False
        self._trip = False
        self._hard_stop = False
        self._trip_count = 0

    def _check_overload(self, force_n: float) -> None:
        """현재 힘을 기준으로 과부하 상태를 갱신한다."""
        self._warn = force_n >= cfg.OL_WARN_N

        # 하드스탑: 한 번만 초과해도 즉시 정지
        if force_n >= cfg.OL_STOP_N:
            self._hard_stop = True
            self._trip = True
            self._trip_count = cfg.OL_CONFIRM
            return

        # 트립: 임계값을 연속으로 초과할 때 확정
        if force_n >= cfg.OL_TRIP_N:
            self._trip_count += 1

            if self._trip_count >= cfg.OL_CONFIRM:
                self._trip = True

        else:
            self._trip_count = 0
            self._trip = False
            self._hard_stop = False


class LoadCellRosPublisher(Node):
    """단일 로드셀 측정값과 상태를 ROS 토픽으로 발행한다."""

    def __init__(self) -> None:
        super().__init__("loadcell_monitor_publisher")

        self._channel = LoadCellChannel(cfg.ACTIVE_CHANNEL)

        self._last_print = 0.0

        # 트립 또는 하드스탑 이후 정지 요청을 유지
        self._fault_latch = False
        self._stop_req = False

        # 압박 1회 최대 힘 추적
        self._compressing = False
        self._current_peak_force = 0.0

        # /motor_stop 수신 후 영점 보정 예약
        self._zero_pending = False
        self._zero_request_time = 0.0

        self._setup_publishers()

        # 모터 정지 명령 구독
        self._motor_stop_sub = self.create_subscription(
            Bool,
            "/motor_stop",
            self._motor_stop_callback,
            10,
        )

        # 로드셀 연결
        self._channel.open()

        # 프로그램 시작 시 초기 영점 보정
        self._channel.calibrate_zero()

        # 메인 타이머
        self.create_timer(
            cfg.PUBLISH_PERIOD_S,
            self._tick,
        )

        self.get_logger().info(
            f"Single loadcell publisher started. "
            f"channel={cfg.ACTIVE_CHANNEL}"
        )

    # ── ROS 퍼블리셔 ───────────────────────────────────────────
    def _setup_publishers(self) -> None:
        """사용할 ROS 퍼블리셔를 생성한다."""
        self._pubs = {
            # 단일 센서 값이지만 기존 모터 코드 호환을 위해 이름 유지
            "total": self.create_publisher(
                Float32,
                "/loadcell_total",
                10,
            ),

            # 과부하 시 모터 정지 요청
            "stop": self.create_publisher(
                Bool,
                "/loadcell_stop_request",
                10,
            ),

            # 0=정상, 1=경고, 2=트립, 3=정지 래치
            "code": self.create_publisher(
                Int32,
                "/loadcell_status_code",
                10,
            ),

            # 압박 1회 최대 힘
            "peak": self.create_publisher(
                Float32,
                "/compression_peak_force",
                10,
            ),
        }

    # ── 압박 피크 계산 ─────────────────────────────────────────
    def _update_compression_peak(self, force_n: float) -> None:
        """압박 1회 동안의 최대 힘을 추적해 종료 시 발행한다."""

        # 압박 시작 전
        if not self._compressing:
            if force_n >= cfg.COMPRESS_START_N:
                self._compressing = True
                self._current_peak_force = force_n

            return

        # 압박 중 최대 힘 갱신
        if force_n > self._current_peak_force:
            self._current_peak_force = force_n

        # 압박 종료
        if force_n <= cfg.COMPRESS_END_N:
            peak_msg = Float32()
            peak_msg.data = float(self._current_peak_force)

            self._pubs["peak"].publish(peak_msg)

            self.get_logger().info(
                f"Compression peak force published: "
                f"{self._current_peak_force:.1f} N"
            )

            self._compressing = False
            self._current_peak_force = 0.0

    # ── /motor_stop 콜백 ──────────────────────────────────────
    def _motor_stop_callback(self, msg: Bool) -> None:
        """
        /motor_stop=True 수신 시 과부하 상태를 초기화하고
        5초 후 영점 보정을 예약한다.

        /motor_stop=False는 무시한다.
        """
        if not msg.data:
            return

        # 과부하 상태와 정지 요청 초기화
        self._fault_latch = False
        self._stop_req = False
        self._channel.reset_fault()

        # 진행 중이던 압박 피크 계산 초기화
        self._compressing = False
        self._current_peak_force = 0.0

        # 영점 보정 예약
        self._zero_pending = True
        self._zero_request_time = time.time()

        self.get_logger().info(
            f"/motor_stop=True received. "
            f"Zero calibration scheduled after "
            f"{cfg.ZERO_DELAY_AFTER_MOTOR_STOP_S:.1f} s."
        )

    # ── 지연 영점 보정 ─────────────────────────────────────────
    def _handle_delayed_zero_calibration(
        self,
        force_n: float,
    ) -> None:
        """모터 정지 신호 수신 후 5초가 지나면 영점 보정한다."""
        if not self._zero_pending:
            return

        elapsed_s = time.time() - self._zero_request_time

        if elapsed_s < cfg.ZERO_DELAY_AFTER_MOTOR_STOP_S:
            return

        # 같은 요청에 대해 반복 수행하지 않도록 먼저 False 처리
        self._zero_pending = False

        # 아직 로드셀에 힘이 걸려 있으면 영점 보정하지 않음
        if abs(force_n) > cfg.ZERO_SAFE_FORCE_N:
            self.get_logger().warn(
                f"Delayed zero calibration skipped. "
                f"Current force={force_n:.1f} N, "
                f"safe limit={cfg.ZERO_SAFE_FORCE_N:.1f} N"
            )
            return

        self.get_logger().info(
            f"Delayed zero calibration started. "
            f"Current force={force_n:.1f} N"
        )

        self._channel.calibrate_zero()

        self._compressing = False
        self._current_peak_force = 0.0

        self.get_logger().info(
            "Delayed loadcell zero calibration completed."
        )

    # ── 상태 코드 ──────────────────────────────────────────────
    def _get_status_code(
        self,
        state: dict[str, bool],
    ) -> int:
        """
        상태 코드:
        0 = NORMAL
        1 = OVERLOAD_WARNING
        2 = OVERLOAD_TRIP
        3 = HARD_STOP 또는 fault latch
        """
        if self._fault_latch:
            return 3

        if state["hard_stop"]:
            return 3

        if state["trip"]:
            return 2

        if state["warn"]:
            return 1

        return 0

    # ── 메인 타이머 ────────────────────────────────────────────
    def _tick(self) -> None:
        """PUBLISH_PERIOD_S 주기로 실행되는 메인 처리 함수."""

        # 단일 로드셀 힘 계산
        force_n = self._channel.update_force()

        if force_n is None:
            return

        # /motor_stop 수신 후 5초 지연 영점 보정
        self._handle_delayed_zero_calibration(force_n)

        # 압박 1회 최대 힘 추적
        self._update_compression_peak(force_n)

        # 과부하 상태 확인
        state = self._channel.get_state()

        # 트립 또는 하드스탑 발생 시 정지 요청 래치
        if state["trip"] or state["hard_stop"]:
            self._fault_latch = True
            self._stop_req = True

        status_code = self._get_status_code(state)

        # 실시간 힘 발행
        total_msg = Float32()
        total_msg.data = float(force_n)
        self._pubs["total"].publish(total_msg)

        # 정지 요청 발행
        stop_msg = Bool()
        stop_msg.data = bool(self._stop_req)
        self._pubs["stop"].publish(stop_msg)

        # 상태 코드 발행
        code_msg = Int32()
        code_msg.data = int(status_code)
        self._pubs["code"].publish(code_msg)

        self._print_status(
            force_n=force_n,
            state=state,
            status_code=status_code,
        )

    # ── 터미널 출력 ────────────────────────────────────────────
    def _print_status(
        self,
        force_n: float,
        state: dict[str, bool],
        status_code: int,
    ) -> None:
        """현재 힘과 과부하 상태를 터미널에 출력한다."""
        now = time.time()

        if now - self._last_print < cfg.PRINT_PERIOD_S:
            return

        self._last_print = now

        if state["hard_stop"]:
            status_text = "HARD_STOP"

        elif state["trip"]:
            status_text = "TRIP"

        elif state["warn"]:
            status_text = "WARN"

        else:
            status_text = "NORMAL"

        print(
            f"{cfg.CHANNEL_LABEL}[CH{cfg.ACTIVE_CHANNEL}]: "
            f"{force_n:7.1f} N  "
            f"STATUS:{status_text}({status_code})  "
            f"STOP_REQ:{self._stop_req}  "
            f"ZERO_PENDING:{self._zero_pending}"
        )

    # ── 종료 ───────────────────────────────────────────────────
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