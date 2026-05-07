#!/usr/bin/env python3
"""
loadcell_monitor.py — 로드셀 모니터링 & ROS 퍼블리셔
Phidget VoltageRatioInput으로 로드셀 데이터를 수집하고,
힘(N) 변환 및 통계 계산 후 ROS 토픽으로 퍼블리시한다.
"""
import time
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32

from Phidget22.Devices.VoltageRatioInput import BridgeGain, VoltageRatioInput
from Phidget22.PhidgetException import PhidgetException

import loadcell_config as cfg


@dataclass
class ForceStats:
    """한 틱에서 계산된 힘 통계와 경보 상태를 담는 컨테이너."""

    # 힘 측정값
    forces:    list[float]  # 채널별 힘 [N]
    total:     float        # 전체 합산 힘 [N]
    imbalance: float        # 채널 간 힘 차이 [N] — 압박봉 기울기 지표

    # 편심(하중 편중) 경보
    tilt_active:  bool            # 편심 판정 활성화 여부 (합산 힘 >= TILT_MIN_FORCE_N 일 때)
    warning:      bool            # 편심 경고 발생 여부
    warn_reasons: list[str] = field(default_factory=list)  # 경고 사유 목록

    # 오버로드 상태
    overload_trip: bool = False   # 트립 임계값을 연속으로 초과함
    hard_stop:     bool = False   # 하드스탑 임계값을 즉시 초과함


def _compute_stats(
    channel_force_pairs: list[tuple[int, float]],
    channel_states:      dict[int, dict],
) -> ForceStats:
    """채널별 (ID, 힘N) 목록과 오버로드 상태로부터 ForceStats를 계산한다."""

    forces    = [force for _, force in channel_force_pairs]
    total     = sum(forces)
    imbalance = max(forces) - min(forces) if len(forces) >= 2 else 0.0

    # 편심 경보: 합산 힘이 최소값 이상일 때만 판정
    tilt_active  = total >= cfg.TILT_MIN_FORCE_N
    warning      = False
    warn_reasons = []

    if tilt_active:
        share_ratio = imbalance / total
        if share_ratio >= cfg.SHARE_WARN_THRESH:
            warning = True
            warn_reasons.append(f"share>={cfg.SHARE_WARN_THRESH:.2f}")
        if imbalance >= cfg.IMBALANCE_WARN_N:
            warning = True
            warn_reasons.append(f"imb>={cfg.IMBALANCE_WARN_N:.0f}N")

    # 채널별 오버로드 상태 집계
    overload_trip = False
    hard_stop     = False
    for state in channel_states.values():
        if state["trip"]:      overload_trip = True
        if state["hard_stop"]: hard_stop     = True

    return ForceStats(
        forces=forces,
        total=total,
        imbalance=imbalance,
        tilt_active=tilt_active,
        warning=warning,
        warn_reasons=warn_reasons,
        overload_trip=overload_trip,
        hard_stop=hard_stop,
    )


class LoadCellChannel:
    """Phidget 로드셀 채널 1개의 데이터 수신 / 필터링 / 오버로드 판정."""

    def __init__(self, channel_id: int) -> None:
        self.channel_id = channel_id
        self._phidget   = VoltageRatioInput()

        self._offset    = 0.0                                              # 영점 보정 오프셋
        self._window: deque[float] = deque(maxlen=cfg.MOVING_AVG_WINDOW)  # 이동 평균 버퍼
        self._latest_ratio:   Optional[float] = None  # 최신 전압비 (이벤트 스레드에서 갱신)
        self._latest_force_n: Optional[float] = None  # 최신 힘 [N]
        self._lock = threading.Lock()                  # _latest_ratio / _latest_force_n 보호용
        # -> 현재 초기 값은 None이고, 피젯이 연결되고 값이 [타입]으로 숫자가 데이터가 들어온다.

        # 오버로드 상태
        self._warn      = False
        self._trip      = False
        self._hard_stop = False
        self._trip_count = 0  # 트립 임계값 연속 초과 횟수 (OL_CONFIRM 도달 시 트립 확정)

    # Phidget 라이브러리가 별도 스레드에서 아래 핸들러들을 호출한다.
    def _on_attach(self, ch: VoltageRatioInput) -> None:
        ch.setBridgeGain(BridgeGain.BRIDGE_GAIN_128)   # 브리지 게인 128배 설정
        ch.setDataInterval(cfg.DATA_INTERVAL_MS)       # 데이터 수신 주기 설정
        print(f"[Attach] channel={self.channel_id}")

    def _on_detach(self, _ch: VoltageRatioInput) -> None:
        print(f"[Detach] channel={self.channel_id}")

    def _on_error(self, _ch: VoltageRatioInput, code: int, description: str) -> None:
        print(f"[Error]  channel={self.channel_id}  code={code}  desc={description}")

    def _on_ratio_change(self, _ch: VoltageRatioInput, ratio: float) -> None:
        # 이벤트 스레드에서 호출되므로 lock으로 보호
        with self._lock:
            self._latest_ratio = float(ratio)


    def open(self) -> None:
        """Phidget 채널을 열고 이벤트 핸들러를 등록한다."""
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
        """하중이 없는 상태에서 전압비를 샘플링해 영점 오프셋을 저장한다."""
        time.sleep(0.5)  # 센서 안정화 대기
        print(f"[Zero]   channel={self.channel_id} calibrating...")

        samples = []
        dt    = 1.0 / cfg.ZERO_CAL_SAMPLE_HZ
        end     = time.time() + cfg.ZERO_CAL_DURATION_S

        while time.time() < end:
            try:
                samples.append(self._phidget.getVoltageRatio())
            except PhidgetException:
                pass
            time.sleep(dt)

        if len(samples) < cfg.ZERO_CAL_MIN_SAMPLES:
            raise RuntimeError(
                f"영점 보정 실패: 유효 샘플={len(samples)} (최소 {cfg.ZERO_CAL_MIN_SAMPLES} 필요)"
            )

        self._offset = sum(samples) / len(samples)
        self._window.clear()
        print(f"[Zero]   channel={self.channel_id}  offset={self._offset:+.9f}")

    def update_force(self) -> Optional[float]:
        """최신 전압비로 이동 평균 힘(N)을 계산하고 오버로드 판정을 수행한다."""
        with self._lock:
            ratio = self._latest_ratio

        if ratio is None:
            return None

        # 오프셋 제거 후 이동 평균 버퍼에 추가
        self._window.append(ratio - self._offset)

        # 이동 평균 전압비 → kg 환산 → N 환산
        avg_ratio = sum(self._window) / len(self._window)
        kg        = avg_ratio / cfg.RATIO_PER_KG
        force_n   = kg * cfg.GRAVITY

        with self._lock:
            self._latest_force_n = force_n

        self._check_overload(force_n)
        return force_n

    def get_force(self) -> Optional[float]:
        with self._lock:
            return self._latest_force_n

    def get_state(self) -> dict:
        return {
            "warn":      self._warn,
            "trip":      self._trip,
            "hard_stop": self._hard_stop,
        }
    
    def reset_fault(self) -> None:
        """채널의 경고/트립/하드스탑 상태를 초기화한다."""
        self._warn = False
        self._trip = False
        self._hard_stop = False
        self._trip_count = 0

    # 오버로드 판정(OL은 오버로드 의미)
    def _check_overload(self, force_n: float) -> None:
        """힘 값에 따라 경고 / 트립 / 하드스탑 상태를 갱신한다."""
        self._warn = force_n >= cfg.OL_WARN_N

        if force_n >= cfg.OL_STOP_N:
            # 하드스탑: trip_count 무관하게 즉시 확정
            self._hard_stop  = True
            self._trip       = True
            self._trip_count = cfg.OL_CONFIRM
            return

        if force_n >= cfg.OL_TRIP_N:
            # 트립: 연속 OL_CONFIRM회 초과 시 확정 (순간 피크 오동작 방지)
            self._trip_count += 1
            if self._trip_count >= cfg.OL_CONFIRM:
                self._trip = True
        else:
            # 정상 범위 복귀 시 상태 초기화
            self._trip_count = 0
            self._trip       = False
            self._hard_stop  = False


# ROS 노드 
class LoadCellRosPublisher(Node):
    """모든 채널을 관리하고 측정값을 ROS 토픽으로 퍼블리시하는 노드."""

    def __init__(self) -> None:
        super().__init__('loadcell_monitor_publisher')

        self._channels    = [LoadCellChannel(cid) for cid in cfg.ACTIVE_CHANNELS]
        self._last_print  = 0.0
        self._fault_latch = False  # trip/hard_stop 발생 후 래치 — 수동 리셋 전까지 유지
        self._stop_req    = False  # 모터 정지 요청 플래그

        self._compressing = False
        self._current_peak_force = 0.0

        #  토픽 발행 부분
        self._setup_publishers()

        self._reset_sub = self.create_subscription(
            Bool,
            '/loadcell_reset',
            self._reset_callback,
            10
        )

        for ch in self._channels:
            ch.open()

        for ch in self._channels:
            ch.calibrate_zero()

        # Enter 키 입력을 별도 스레드에서 감지해 종료 처리
        # threading.Thread(target=self._wait_for_enter, daemon=True).start()
        self.create_timer(cfg.PUBLISH_PERIOD_S, self._tick)
        self.get_logger().info("LoadCell 퍼블리셔 시작. Enter로 종료.")

    def _update_compression_peak(self, total_force: float) -> None:
        """압박 1회 동안 total force의 peak를 추적하고, 종료 시 1회 발행한다."""
        # 아직 압박 중이 아니면 시작 조건 확인
        if not self._compressing:
            if total_force >= cfg.COMPRESS_START_N:
                self._compressing = True                # 지금 압박 중인지 여부 (True/False)
                self._current_peak_force = total_force  # 현재 압박에서 최고 힘
            return

        # 압박 중이면 peak 갱신
        if total_force > self._current_peak_force:
            self._current_peak_force = total_force

        # 종료 조건
        if total_force <= cfg.COMPRESS_END_N:
            peak_msg = Float32()
            peak_msg.data = float(self._current_peak_force)
            self._pubs["peak"].publish(peak_msg)

            self.get_logger().info(
                f"Compression peak force published: {self._current_peak_force:.1f} N"
            )

            self._compressing = False
            self._current_peak_force = 0.0

    def _reset_faults(self) -> None:
        """래치 및 채널 fault 상태를 모두 초기화한다."""
        self._fault_latch = False
        self._stop_req = False

        for ch in self._channels:
            ch.reset_fault()

        self.get_logger().info("Loadcell fault reset completed.")

    def _reset_callback(self, msg: Bool) -> None:
        """True가 들어오면 fault reset 수행."""
        if msg.data:
            self._reset_faults()

    # 초기화 
    def _setup_publishers(self) -> None:
        """ROS 퍼블리셔를 생성하고 dict에 저장한다."""
        def pub(msg_type, topic):
            return self.create_publisher(msg_type, topic, 10)

        self._pubs = {
            "each":  pub(Float32MultiArray, '/loadcell_each'),         # 채널별 힘 배열 [N]
            "total": pub(Float32,           '/loadcell_total'),        # 합산 힘 [N]
            "imb":   pub(Float32,           '/loadcell_imbalance'),    # 채널 간 힘 차이 [N]
            "tilt":  pub(Bool,              '/loadcell_tilt_active'),  # 편심 판정 활성화 여부
            "warn":  pub(Bool,              '/loadcell_warning'),      # 편심 경고
            "stop":  pub(Bool,              '/loadcell_stop_request'), # 모터 정지 요청, 오버로드(로드셀 정격 이상 쓴 경우 발생)
            "code":  pub(Int32,             '/loadcell_status_code'),  # 0=정상 1=경고 2=트립 3=래치
            "peak":  pub(Float32,           '/compression_peak_force'),# 순간 압박 값
        }

    # 종료 
    def _wait_for_enter(self) -> None:
        input()
        self.get_logger().info("종료 중...")
        rclpy.shutdown()

    def destroy_node(self) -> None:
        for ch in self._channels:
            ch.close()
        super().destroy_node()

    # 메인 루프 (PUBLISH_PERIOD_S 마다 호출), 한 사이클 
    def _tick(self) -> None:
        # 힘 계산
        for ch in self._channels:
            ch.update_force()

        # 유효한 힘 값이 있는 채널만 추림
        channel_force_pairs = [
            (ch.channel_id, force)
            for ch in self._channels
            if (force := ch.get_force()) is not None
        ]
        if not channel_force_pairs:
            return

        # 통계 계산
        channel_states = {ch.channel_id: ch.get_state() for ch in self._channels}
        stats = _compute_stats(channel_force_pairs, channel_states)

        # 압박 peak force 추적 및 종료 시 발행
        self._update_compression_peak(stats.total)

        # trip 또는 hard_stop 발생 시 래치 — 리셋 토픽 sub 전까지 stop_req 유지
        if stats.hard_stop or stats.overload_trip:
            self._fault_latch = True
            self._stop_req    = True

        self._publish(stats)
        self._print_status(channel_force_pairs, stats, channel_states)

    # 퍼블리시 
    def _publish(self, stats: ForceStats) -> None:
        publishers = self._pubs

        # 상태 코드: 0=정상 / 1=경고 / 2=트립 / 3=래치된 fault
        if self._fault_latch:
            status_code = 3
        elif stats.hard_stop or stats.overload_trip:
            status_code = 2
        elif stats.warning:
            status_code = 1
        else:
            status_code = 0

        msg = Float32MultiArray()
        msg.data = stats.forces
        publishers["each"].publish(msg)

        for key, message in [
            ("total", Float32(data=stats.total)),
            ("imb",   Float32(data=stats.imbalance)),
            ("tilt",  Bool(data=stats.tilt_active)),
            ("warn",  Bool(data=stats.warning)),
            ("stop",  Bool(data=self._stop_req)),
            ("code",  Int32(data=status_code)),
        ]:
            publishers[key].publish(message)

    #  터미널 출력 
    def _print_status(self, channel_force_pairs, stats,channel_states) -> None:
        now = time.time()
        if now - self._last_print < cfg.PRINT_PERIOD_S:
            return
        self._last_print = now

        def channel_label(channel_id: int) -> str:
            return cfg.CHANNEL_LABELS.get(channel_id, f"ch{channel_id}")

        # 채널별 힘 출력
        forces_str = "  ".join(
            f"{channel_label(cid)}:{force:7.1f}N"
            for cid, force in channel_force_pairs
        )

        # 오버로드 상태 문자열 조합
        overload_parts = []
        for cid, state in channel_states.items():
            # 오버로드 상태인 채널만 처리
            if state["warn"] or state["trip"] or state["hard_stop"]:
                if state["hard_stop"]:
                    level = "HARD_STOP"
                elif state["trip"]:
                    level = "TRIP"
                else:
                    level = "WARN"
                overload_parts.append(f"{channel_label(cid)}:{level}")

        if overload_parts:
            overload_str = "  OVERLOAD[" + ", ".join(overload_parts) + "]"
        else:
            overload_str = ""

        if stats.tilt_active:
            tilt_str = "TILT=ON"
        else:
            tilt_str = "TILT=OFF"

        if stats.warning:
            warning_str = "  WARN(" + ", ".join(stats.warn_reasons) + ")"
        else:
            warning_str = ""

        print(
            f"{forces_str}  |  "
            f"SUM:{stats.total:7.1f}N  "
            f"IMB:{stats.imbalance:6.1f}N  "
            f"{tilt_str}"
            f"{warning_str}"
            f"{overload_str}"
        )


def main() -> None:
    # cfg.print_summary()
    rclpy.init()
    node = None
    try:
        node = LoadCellRosPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
