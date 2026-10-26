#!/usr/bin/env python3
"""
loadcell_monitor.py — 로드셀 모니터링 & ROS 퍼블리셔
Phidget VoltageRatioInput으로 로드셀 데이터를 수집하고,
힘(N) 변환 및 통계 계산 후 ROS 토픽으로 퍼블리시한다.
"""
import time
import threading
import sys
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32

from Phidget22.Devices.VoltageRatioInput import BridgeGain, VoltageRatioInput
from Phidget22.PhidgetException import PhidgetException

import loadcell_config as cfg


# === CSV recording only: embedded existing V3 SessionRecorder (no extra file required) ===
# Async worker writes files without changing legacy ROS/control logic.

import csv
import json
import queue
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

SAMPLE_FIELDS = [
    'record_time_ns', 'timestamp_utc', 'elapsed_s', 'sample_index',
    'ratio_ch1', 'ratio_ch2', 'ratio_ch3',
    'receive_time_ch1_ns', 'receive_time_ch2_ns', 'receive_time_ch3_ns',
    'force_ch1_n', 'force_ch2_n', 'force_ch3_n', 'total_force_n',
    'offset_ch1', 'offset_ch2', 'offset_ch3', 'calibration_id',
    'sensor_valid', 'sensor_reason', 'overload_status', 'stop_request',
]
EVENT_FIELDS = ['record_time_ns', 'elapsed_s', 'event', 'detail']
RAW_FIELDS = ['receive_time_ns', 'elapsed_s', 'channel_id', 'voltage_ratio', 'calibration_id']


class SessionRecorder:
    """One recorder per session; callbacks enqueue, worker writes to disk."""

    def __init__(self, root: str | Path, metadata: dict, queue_size: int = 50000):
        self.start_ns = time.monotonic_ns()
        suffix = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        self.directory = Path(root) / suffix
        self.directory.mkdir(parents=True, exist_ok=False)
        self._queue = queue.Queue(maxsize=queue_size)
        self._loss = 0
        self._state_lock = threading.Lock()  # serialize producer admission against close sentinel
        self._loss_lock = threading.Lock()
        self._rejected_on_close = 0
        self._closing = False
        self._worker_error: str | None = None
        self._thread = threading.Thread(target=self._writeLoop, daemon=False)
        meta = {
            **metadata,
            'session_id': suffix,
            'start_monotonic_ns': self.start_ns,
            'start_wall_time_utc': datetime.now(timezone.utc).isoformat(),
            'note': 'receive_time_ns is PC callback arrival time, not sensor capture time',
        }
        (self.directory / 'meta.json').write_text(
            json.dumps(meta, ensure_ascii=False, indent=2, default=str),
            encoding='utf-8',
        )
        self._thread.start()
        self.event('RECORDING_STARTED')

    @property
    def writer_error(self) -> str | None:
        return self._worker_error

    def record(self, kind: str, row: dict):
        # Admission and sentinel positioning must be mutually exclusive.
        # Never silently accept a write after the close sentinel was queued.
        with self._state_lock:
            if self._closing:
                self._rejected_on_close += 1
                raise RuntimeError('CSV session closing/closed: rejected write')
            if self._worker_error is not None:
                raise RuntimeError(f'CSV worker failed: {self._worker_error}')
            try:
                self._queue.put_nowait((kind, row.copy()))
            except queue.Full:
                with self._loss_lock:
                    self._loss += 1
                raise RuntimeError('CSV queue overflow: data lost; session cannot be trusted')

    def event(self, name: str, detail: str = ''):
        now_ns = time.monotonic_ns()
        self.record('event', {
            'record_time_ns': now_ns,
            'elapsed_s': (now_ns - self.start_ns) / 1e9,
            'event': name,
            'detail': detail,
        })

    def _writeLoop(self):
        files = {}
        writers = {}
        last_flush = time.monotonic()
        try:
            for kind, name, fields in (
                ('sample', 'samples.csv', SAMPLE_FIELDS),
                ('event', 'events.csv', EVENT_FIELDS),
                ('raw', 'channel_events.csv', RAW_FIELDS),
            ):
                file = (self.directory / name).open('w', newline='', encoding='utf-8')
                files[kind] = file
                writer = csv.DictWriter(file, fieldnames=fields, extrasaction='ignore')
                writer.writeheader()
                writers[kind] = writer
            while True:
                try:
                    item = self._queue.get(timeout=0.2)
                except queue.Empty:
                    item = None
                if item is not None:
                    try:
                        if item == ('__CLOSE__', None):
                            break
                        kind, row = item
                        writers[kind].writerow(row)
                    finally:
                        self._queue.task_done()
                if time.monotonic() - last_flush >= 1:
                    for file in files.values():
                        file.flush()
                    last_flush = time.monotonic()
        except BaseException as exc:
            self._worker_error = f'{type(exc).__name__}: {exc}'
        finally:
            for file in files.values():
                try:
                    file.flush()
                    file.close()
                except BaseException as exc:
                    if self._worker_error is None:
                        self._worker_error = f'File close/flush {type(exc).__name__}: {exc}'

    def close(self):
        close_errors = []
        with self._state_lock:
            if self._closing:
                raise RuntimeError('SessionRecorder.close was already attempted')
            # LAST event must be placed before close sentinel while producers are barred.
            with self._loss_lock:
                loss = self._loss
            if self._worker_error is None:
                now_ns = time.monotonic_ns()
                stop_row = {
                    'record_time_ns': now_ns,
                    'elapsed_s': (now_ns - self.start_ns) / 1e9,
                    'event': 'RECORDING_STOPPED',
                    'detail': f'queue_drop_count={loss}',
                }
                try:
                    self._queue.put_nowait(('event', stop_row))
                except queue.Full:
                    close_errors.append('queue full when adding recording stop event')
            self._closing = True
        # No producer can append after this point; enqueue sentinel in FIFO order.
        if self._thread.is_alive():
            try:
                self._queue.put(('__CLOSE__', None), timeout=0.5)
            except queue.Full:
                close_errors.append('queue full when closing (writer unresponsive)')
            self._thread.join(timeout=10)
        if self._thread.is_alive():
            close_errors.append('CSV worker did not finish within shutdown deadline')
        writer_error = self._worker_error
        with self._loss_lock:
            loss = self._loss
        with self._state_lock:
            rejected = self._rejected_on_close
        closed_cleanly = not (close_errors or writer_error or loss or rejected)
        status = {
            'closed_cleanly': closed_cleanly,
            'queue_drop_count': loss,
            'rejected_during_close': rejected,
            'writer_error': writer_error,
            'close_errors': close_errors,
        }
        # Best effort only: a full disk can prevent even the status file write.
        try:
            (self.directory / 'recording_status.json').write_text(
                json.dumps(status, ensure_ascii=False, indent=2), encoding='utf-8')
        except OSError as exc:
            raise RuntimeError(f'CSV INCOMPLETE; could not write status file: {exc}') from exc
        if not closed_cleanly:
            raise RuntimeError(f'CSV INCOMPLETE ({status}); directory={self.directory}')
        return self.directory

# === End CSV recording infrastructure ===


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
    overload_warn: bool = False   # 경고 임계값을 초과함
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

    if tilt_active and total > 0.0:
        max_force = max(forces)
        max_share = max_force / total

        if max_share >= cfg.SHARE_WARN_THRESH:
            warning = True
            warn_reasons.append(f"max_share={max_share:.2f}>={cfg.SHARE_WARN_THRESH:.2f}")
        if imbalance >= cfg.IMBALANCE_WARN_N:
            warning = True
            warn_reasons.append(f"imb>={cfg.IMBALANCE_WARN_N:.0f}N")

    # 채널별 오버로드 상태 집계
    overload_warn = False
    overload_trip = False
    hard_stop     = False

    for state in channel_states.values():
        if state["warn"]:
            overload_warn = True
        if state["trip"]:
            overload_trip = True
        if state["hard_stop"]:
            hard_stop = True

    return ForceStats(
        forces=forces,
        total=total,
        imbalance=imbalance,
        tilt_active=tilt_active,
        warning=warning,
        warn_reasons=warn_reasons,
        overload_warn=overload_warn,
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
        self._latest_receive_ns: Optional[int] = None  # PC callback 도착 시각 (CSV 전용)
        self._raw_record_callback = None               # CSV 기록 중일 때만 사용
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
        receive_ns = time.monotonic_ns()               # 원본 전압비 CSV 기록용 시간
        with self._lock:
            self._latest_ratio = float(ratio)
            self._latest_receive_ns = receive_ns
        # 기존 로드셀 계산/ROS 제어에는 영향이 없도록 기록 오류는 별도 처리.
        if self._raw_record_callback is not None:
            self._raw_record_callback(self.channel_id, float(ratio), receive_ns)


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
        time.sleep(0.1)  # 센서 안정화 대기
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
        # 채널별 Full Scale Output이 다르므로 채널별 계수로 변환한다.
        avg_ratio    = sum(self._window) / len(self._window)
        ratio_per_kg = cfg.get_ratio_per_kg(self.channel_id)
        kg           = avg_ratio / ratio_per_kg
        force_n      = kg * cfg.GRAVITY

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
        # CSV 세션: 기존 힘 계산/안전 판정/ROS 퍼블리시는 전혀 변경하지 않음.
        self._recorder = None
        self._recorder_lock = threading.Lock()
        self._recorder_root = Path(__file__).resolve().parent / 'recordings'
        self._csv_sample_index = 0
        self._calibration_id = 1
        self._recording_error_reported = False

        self._compressing = False
        self._current_peak_force = 0.0

        self._zero_pending = False      # 모터 정지 후 영점조절
        self._zero_request_time = 0.0

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

        # 센서 연결/초기 영점 보정 이후에만 CSV 기록을 허용.
        for ch in self._channels:
            ch._raw_record_callback = self._record_raw_ratio
        if sys.stdin.isatty():
            threading.Thread(target=self._keyboard_loop, daemon=True).start()
            self.get_logger().info('CSV: s+Enter=기록 시작, e+Enter=저장, Ctrl+C=종료')
        else:
            self.get_logger().warning('stdin이 터미널이 아니어서 s/e 키 입력이 비활성화됩니다.')

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

    def _get_current_total_force(self) -> Optional[float]:
        """현재 저장된 채널별 힘을 합산한다. 아직 값이 없으면 None을 반환한다."""
        forces = [ch.get_force() for ch in self._channels]

        if any(force is None for force in forces):
            return None

        return float(sum(force for force in forces if force is not None))

    def _zero_calibration_if_safe(self) -> None:
        """
        현재 합산 힘이 충분히 작을 때만 영점 보정을 수행한다.
        압박판이 로드셀을 누르고 있는 상태에서 offset이 잘못 잡히는 것을 방지한다.
        """
        total_force = self._get_current_total_force()

        if total_force is None:
            self.get_logger().warn(
                "Zero calibration skipped. Loadcell force is not ready yet."
            )
            return

        if abs(total_force) > cfg.ZERO_SAFE_TOTAL_N:
            self.get_logger().warn(
                f"Zero calibration skipped. "
                f"Current total force={total_force:.1f} N "
                f"> safe limit={cfg.ZERO_SAFE_TOTAL_N:.1f} N"
            )
            return

        self.get_logger().info(
            f"Zero calibration started. Current total force={total_force:.1f} N"
        )

        for ch in self._channels:
            ch.calibrate_zero()

        # peak 추적 상태도 초기화
        self._compressing = False
        self._current_peak_force = 0.0

        self._calibration_id += 1  # CSV 메타데이터에만 반영
        self.get_logger().info("Loadcell zero calibration completed.")

    def _handle_delayed_zero_calibration(self, total_force: float) -> None:
        """reset 후 일정 시간이 지난 뒤, 안전 조건에서 영점 보정을 수행한다."""
        if not self._zero_pending:
            return

        elapsed = time.time() - self._zero_request_time

        if elapsed < cfg.ZERO_DELAY_AFTER_RESET_S:
            return

        # 한 번 시도했으면 pending 해제
        self._zero_pending = False

        if abs(total_force) > cfg.ZERO_SAFE_TOTAL_N:
            self.get_logger().warn(
                f"Delayed zero calibration skipped. "
                f"Current total force={total_force:.1f} N "
                f"> safe limit={cfg.ZERO_SAFE_TOTAL_N:.1f} N"
            )
            return

        self.get_logger().info(
            f"Delayed zero calibration started. "
            f"Current total force={total_force:.1f} N"
        )

        for ch in self._channels:
            ch.calibrate_zero()

        self._compressing = False
        self._current_peak_force = 0.0

        self._calibration_id += 1  # CSV 메타데이터에만 반영
        self.get_logger().info("Delayed loadcell zero calibration completed.")

    def _reset_callback(self, msg: Bool) -> None:
        """True가 들어오면 fault reset 후, 일정 시간 뒤 영점 보정을 예약한다."""
        if msg.data:
            self._reset_faults()

            self._zero_pending = True
            self._zero_request_time = time.time()

            self.get_logger().info(
                f"Zero calibration scheduled after "
                f"{cfg.ZERO_DELAY_AFTER_RESET_S:.1f} s."
            )
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

    # === CSV recording only: existing motor-facing publishers and stop rules untouched ===
    def _report_recording_error(self, exc: Exception) -> None:
        if not self._recording_error_reported:
            print(f'[REC ERROR] {type(exc).__name__}: {exc}', flush=True)
            self._recording_error_reported = True

    def _record_raw_ratio(self, channel_id: int, ratio: float, receive_ns: int) -> None:
        with self._recorder_lock:
            recorder = self._recorder
            if recorder is None:
                return
            try:
                recorder.record('raw', {
                    'receive_time_ns': receive_ns,
                    'elapsed_s': (receive_ns - recorder.start_ns) / 1e9,
                    'channel_id': channel_id,
                    'voltage_ratio': ratio,
                    'calibration_id': self._calibration_id,
                })
            except Exception as exc:
                self._report_recording_error(exc)

    def _record_sample(self, channel_force_pairs, stats) -> None:
        with self._recorder_lock:
            recorder = self._recorder
            if recorder is None:
                return
            self._csv_sample_index += 1
            now_ns = time.monotonic_ns()
            row = {
                'record_time_ns': now_ns,
                'timestamp_utc': datetime.now(timezone.utc).isoformat(),
                'elapsed_s': (now_ns - recorder.start_ns) / 1e9,
                'sample_index': self._csv_sample_index,
                'total_force_n': stats.total if len(channel_force_pairs) == len(self._channels) else '',
                'calibration_id': self._calibration_id,
                'sensor_valid': int(len(channel_force_pairs) == len(self._channels)),
                'sensor_reason': '' if len(channel_force_pairs) == len(self._channels) else 'MISSING_CHANNEL',
                'overload_status': int(self._fault_latch),
                'stop_request': int(self._stop_req),
            }
            # 실제 Phidget CH0/1/2 -> CSV ch1/2/3 (편심 분석 파일 형식 유지).
            for csv_ch, channel in enumerate(self._channels, start=1):
                with channel._lock:
                    ratio = channel._latest_ratio
                    receive_ns = channel._latest_receive_ns
                    force = channel._latest_force_n
                    offset = channel._offset
                row[f'ratio_ch{csv_ch}'] = ratio
                row[f'receive_time_ch{csv_ch}_ns'] = receive_ns
                row[f'force_ch{csv_ch}_n'] = force
                row[f'offset_ch{csv_ch}'] = offset
            try:
                recorder.record('sample', row)
            except Exception as exc:
                self._report_recording_error(exc)

    def _start_recording(self) -> None:
        with self._recorder_lock:
            if self._recorder is not None:
                print('[REC] already recording', flush=True)
                return
            cfg_data = {k: v for k, v in vars(cfg).items()
                        if k.isupper() and isinstance(v, (str, int, float, bool, dict, list, type(None)))}
            meta = {
                'config': cfg_data,
                'channels': {str(ch.channel_id): {'label': cfg.CHANNEL_LABELS.get(ch.channel_id),
                                                  'offset': ch._offset} for ch in self._channels},
                'csv_channel_map': {f'ch{i}': ch.channel_id
                                    for i, ch in enumerate(self._channels, start=1)},
                'force_source': 'existing loadcell_main.py calculation, unchanged',
                'raw_channel_events_available': True,
                'motor_control_changed': False,
                'note': 'CSV only; original ROS publishers and stop policy remain unchanged',
            }
            try:
                self._recorder = SessionRecorder(self._recorder_root, meta)
            except Exception as exc:
                self._report_recording_error(exc)
                return
            self._recording_error_reported = False
            self._csv_sample_index = 0
            print(f'[REC] started {self._recorder.directory}', flush=True)

    def _stop_recording(self) -> None:
        with self._recorder_lock:
            recorder = self._recorder
            self._recorder = None
        if recorder is not None:
            try:
                saved = recorder.close()
                print(f'[REC] saved {saved}', flush=True)
            except Exception as exc:
                self._report_recording_error(exc)
                print(f'[REC] incomplete {recorder.directory}', flush=True)

    def _keyboard_loop(self) -> None:
        while True:
            try:
                cmd = input().strip().lower()
            except (EOFError, OSError):
                return
            if cmd == 's':
                self._start_recording()
            elif cmd == 'e':
                self._stop_recording()

    # 종료 
    def _wait_for_enter(self) -> None:
        input()
        self.get_logger().info("종료 중...")
        rclpy.shutdown()

    def destroy_node(self) -> None:
        # 기존 종료 경로에 파일 닫기만 추가한다.
        self._stop_recording()
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

        # reset 후 예약된 영점 보정 처리
        self._handle_delayed_zero_calibration(stats.total)

        # 압박 peak force 추적 및 종료 시 발행
        self._update_compression_peak(stats.total)

        # trip 또는 hard_stop 발생 시 래치 — 리셋 토픽 sub 전까지 stop_req 유지
        if stats.hard_stop or stats.overload_trip:
            self._fault_latch = True
            self._stop_req    = True

        self._publish(stats)
        self._print_status(channel_force_pairs, stats, channel_states)
        self._record_sample(channel_force_pairs, stats)

    # 퍼블리시 
    def _publish(self, stats: ForceStats) -> None:
        publishers = self._pubs

        # 편심 경고는 /loadcell_warning으로만 publish한다.
        if self._fault_latch:
            status_code = 3
        elif stats.hard_stop:
            status_code = 3
        elif stats.overload_trip:
            status_code = 2
        elif stats.overload_warn:
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
            ("warn",  Bool(data=stats.warning)),       # 편심 경고
            ("stop",  Bool(data=self._stop_req)),      # 과하중 trip/hard_stop에서만 True
            ("code",  Int32(data=status_code)),        # 과하중 상태만 표시
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
