#!/usr/bin/env python3
"""
loadcell_monitor.py — 로드셀 모니터링 & ROS 퍼블리셔
Phidget VoltageRatioInput으로 로드셀 데이터를 수집하고,
힘(N) 변환 및 통계 계산 후 ROS 토픽으로 퍼블리시한다.
"""
import time
import threading
import json
import select
import sys
from pathlib import Path
from datetime import datetime, timezone
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32

from Phidget22.Devices.VoltageRatioInput import BridgeGain, VoltageRatioInput
from Phidget22.PhidgetException import PhidgetException

import loadcell_config_v4 as cfg
from loadcell_recorder_v3 import SessionRecorder


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
        self._attached = False
        self._sensor_error = False
        self._last_receive_ns: int | None = None
        self._sample_seq = 0
        self._processed_seq = 0
        self._raw_recorder_callback = None
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
        with self._lock:
            self._attached = True
            self._sensor_error = False
        print(f"[Attach] channel={self.channel_id}")

    def _on_detach(self, _ch: VoltageRatioInput) -> None:
        with self._lock:
            self._attached = False
            self._latest_force_n = None
        print(f"[Detach] channel={self.channel_id}")

    def _on_error(self, _ch: VoltageRatioInput, code: int, description: str) -> None:
        with self._lock:
            self._sensor_error = True
        print(f"[Error]  channel={self.channel_id}  code={code}  desc={description}")

    def _on_ratio_change(self, _ch: VoltageRatioInput, ratio: float) -> None:
        # 이벤트 스레드에서 호출되므로 lock으로 보호
        receive_ns = time.monotonic_ns()
        with self._lock:
            self._latest_ratio = float(ratio)
            self._last_receive_ns = receive_ns
            self._sample_seq += 1
        if self._raw_recorder_callback is not None:
            self._raw_recorder_callback(self.channel_id, float(ratio), receive_ns)


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

        with self._lock:
            self._offset = sum(samples) / len(samples)
            self._window.clear()
            self._latest_force_n = None
            self._processed_seq = self._sample_seq
        print(f"[Zero]   channel={self.channel_id}  offset={self._offset:+.9f}")

    def update_force(self) -> Optional[float]:
        """최신 전압비로 이동 평균 힘(N)을 계산하고 오버로드 판정을 수행한다."""
        with self._lock:
            ratio = self._latest_ratio
            if ratio is None or self._sample_seq == self._processed_seq or not self._attached:
                return self._latest_force_n
            self._processed_seq = self._sample_seq
            self._window.append(ratio - self._offset)
        # Moving average updated only after a NEW Phidget callback.


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
        warn_limit = cfg.V3_CHANNEL_WARN_N
        trip_limit = cfg.V3_CHANNEL_TRIP_N
        if warn_limit is None or trip_limit is None:
            return  # overall stop_request remains latched while unvalidated
        self._warn = force_n >= warn_limit

        if force_n >= trip_limit:
            self._hard_stop = True
            self._trip = True
            return
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
        self._recorder: SessionRecorder | None = None
        self._recorder_lock = threading.Lock()
        self._sample_index = 0
        self._calibration_id = 0
        self._node_shutdown_requested = False
        self._last_motor_state = 'UNKNOWN'
        self._motor_last_received_ns: int | None = None
        self._last_print  = 0.0
        self._fault_latch = False  # armed 이후 센서 fault/과하중을 래치
        self._stop_req    = True   # 시작 시 보호 요청. 새 센서 측정 전에는 구동 불허.
        self._armed = False
        self._calibration_end_ns: int | None = None
        self._recorder_failed = False
        self._last_interlock_report = None

        self._compressing = False
        self._current_peak_force = 0.0

        self._zero_pending = False      # 모터 정지 후 영점조절
        self._zero_request_time = 0.0

        #  토픽 발행 부분
        self._setup_publishers()

        from std_msgs.msg import String, Float32MultiArray
        self.create_subscription(String, '/motor_state', self._onMotorState, 10)
        self.create_subscription(Float32MultiArray, '/motor_compression_status', self._onMotorCompression, 10)
        for ch in self._channels:
            ch._raw_recorder_callback = self._onRawReading

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
        self._calibration_id += 1
        # Zero calibration leaves _latest_force_n=None and processed_seq at the
        # last sample. Require a NEW sample from EACH channel after this instant.
        self._calibration_end_ns = time.monotonic_ns()

        if sys.stdin.isatty():
            threading.Thread(target=self._keyboardLoop, daemon=True).start()
        # Enter 키 입력을 별도 스레드에서 감지해 종료 처리
        # threading.Thread(target=self._wait_for_enter, daemon=True).start()
        self.create_timer(cfg.PUBLISH_PERIOD_S, self._tick)
        self.get_logger().info("V3 Loadcell: s<Enter>=record, e<Enter>=stop, m<Enter>=marker, x<Enter>=shutdown after MOTOR_OFF confirmation")
        self.get_logger().info("[LEGACY MOTOR ROS] /loadcell_total (Float32), /loadcell_stop_request (Bool)")
        approved, reason = cfg.validateSafetyLimits()
        if not approved:
            self.get_logger().warn(f"[SAFETY LOCKOUT] {reason}; recording still available, motor movement not authorized")

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

        self.get_logger().info("Delayed loadcell zero calibration completed.")

    def _reset_callback(self, msg: Bool) -> None:
        """True가 들어오면 fault reset 후, 일정 시간 뒤 영점 보정을 예약한다."""
        if not msg.data:
            return
        # Remote reset is intentionally disabled: zeroing blocks ROS callbacks
        # and releasing a fault while a motor is enabled is unsafe.
        self.get_logger().warn('V3 remote zero/reset REFUSED: stop and disable motor, restart loadcell node for supervised calibration.')
        self._event('REMOTE_RESET_REFUSED')
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
        try:
            self._stopRecording()
        finally:
            # Even if CSV close fails, release all sensor handles.
            for ch in self._channels:
                ch.close()
            super().destroy_node()

    # V3 recorder and safety helpers
    def _onMotorState(self, msg):
        self._last_motor_state = str(msg.data)
        self._motor_last_received_ns = time.monotonic_ns()
        self._event('MOTOR_STATE', self._last_motor_state)

    def _onMotorCompression(self, msg):
        self._event('MOTOR_COMPRESSION', json.dumps(list(msg.data)))

    def _record(self, kind, row):
        with self._recorder_lock:
            if self._recorder is not None:
                try:
                    self._recorder.record(kind, row)
                except Exception as exc:
                    self._reportRecorderFailure(exc)

    def _event(self, name, detail=''):
        with self._recorder_lock:
            if self._recorder is not None:
                try:
                    self._recorder.event(name, detail)
                except Exception as exc:
                    self._reportRecorderFailure(exc)

    def _onRawReading(self, channel_id, ratio, receive_ns):
        with self._recorder_lock:
            recorder = self._recorder
            if recorder is not None:
                try:
                    recorder.record('raw', {
                    'receive_time_ns':receive_ns,
                    'elapsed_s':(receive_ns-recorder.start_ns)/1e9,
                    'channel_id':channel_id,
                    'voltage_ratio':ratio,'calibration_id':self._calibration_id,
                    })
                except Exception as exc:
                    self._reportRecorderFailure(exc)

    def _startRecording(self):
        with self._recorder_lock:
            if self._recorder is not None:
                print('[REC] already recording');return
            cfg_data={k:v for k,v in vars(cfg).items() if k.isupper() and isinstance(v,(str,int,float,bool,dict,list,type(None)))}
            meta={'config':cfg_data,
                  'channels':{str(ch.channel_id):{'label':cfg.CHANNEL_LABELS.get(ch.channel_id),
                      'offset':ch._offset} for ch in self._channels},
                  'motor_state_last_seen':self._last_motor_state,
                  'csv_channel_map': {f'ch{i}': int(cid)
                      for i, cid in enumerate(cfg.ACTIVE_CHANNELS, start=1)},
                  'safety_limits_check': cfg.validateSafetyLimits()[1],
                  'warning':'Research prototype; no patient use; motion requires verified safety limits'}
            new_recorder = SessionRecorder(cfg.V3_RECORDER_ROOT, meta)
            self._recorder = new_recorder
            self._recorder_failed = False  # a failed *previous* session is not a sensor fault
            print(f'[REC] started {self._recorder.directory}', flush=True)

    def _stopRecording(self):
        with self._recorder_lock:
            recorder=self._recorder
            self._recorder=None  # stop new writes before the recorder places its close sentinel
        if recorder is not None:
            try:
                result=recorder.close()
                print(f'[REC] saved {result}',flush=True)
            except Exception as exc:
                self._reportRecorderFailure(exc)
                print(f'[REC FAILURE] session {recorder.directory} is NOT complete: {exc}',flush=True)
                # The interactive keyboard must catch this; recording failure is NOT a safety trip.
                raise

    def _reportRecorderFailure(self, exc):
        if not self._recorder_failed:
            print(f'[REC FAILURE] {exc}',flush=True)
        self._recorder_failed = True

    def _keyboardLoop(self):
        # Recorder errors are reported to the operator but MUST NOT kill this thread.
        # This path does not control the loadcell safety publisher.
        while not self._node_shutdown_requested:
            try:
                line = input().strip().lower()
            except (EOFError, OSError):
                return
            try:
                if line == 's':
                    self._startRecording()
                elif line == 'e':
                    self._stopRecording()
                elif line == 'm':
                    self._event('ALIGNMENT_CHANGED_BY_OPERATOR')
                elif line == 'x':
                    print('[SAFETY] confirm motor is stopped AND drive physically inhibited. Type MOTOR_OFF:')
                    try:
                        phrase = input().strip()
                    except (EOFError, OSError):
                        continue
                    if phrase == 'MOTOR_OFF':
                        # Stop recording first; even an incomplete CSV must not block node shutdown.
                        try:
                            self._stopRecording()
                        finally:
                            self._node_shutdown_requested = True
                    else:
                        print('[SAFETY] shutdown canceled')
            except Exception as exc:
                print(f'[COMMAND ERROR] {type(exc).__name__}: {exc}; keyboard remains active', flush=True)

    def _tick(self) -> None:
        if self._node_shutdown_requested:
            rclpy.shutdown();return
        self._sample_index+=1
        now_ns=time.monotonic_ns()
        for ch in self._channels:ch.update_force()
        snapshots=[]
        for ch in self._channels:
            with ch._lock:
                snapshots.append((ch.channel_id,ch._latest_ratio,ch._last_receive_ns,
                    ch._attached,ch._sensor_error,ch._offset,ch._latest_force_n))
        stale_ns=int(cfg.V3_SENSOR_STALE_TIMEOUT_S*1e9)
        invalid=[]
        for channel_id,ratio,received,attached,error,offset,force in snapshots:
            if not attached:invalid.append(f'ch{channel_id}:DETACHED')
            elif error:invalid.append(f'ch{channel_id}:ERROR')
            elif received is None:invalid.append(f'ch{channel_id}:NO_SAMPLE')
            elif now_ns-received>stale_ns:invalid.append(f'ch{channel_id}:STALE')
            elif force is None:invalid.append(f'ch{channel_id}:NOT_READY')
        # Legacy motor uses these same two ROS2 topics. Interlock stays asserted
        # until the physically reviewed loadcell limits have been approved.
        limits_valid, limit_reason = cfg.validateSafetyLimits()
        sensor_invalid = list(invalid)
        # A first timer tick may precede the first fresh callback after zeroing.
        # This is NOT a running failure: fail closed but do not permanently latch.
        fresh_since_calibration = (self._calibration_end_ns is not None and
            all(received is not None and received > self._calibration_end_ns
                for _,_,received,_,_,_,_ in snapshots))
        if not self._armed and not invalid and fresh_since_calibration:
            self._armed = True
            self.get_logger().info('[SAFETY] all channels armed after new measurements')
        if not self._armed:
            invalid.append('ARMING_WAIT_FOR_FRESH_SAMPLES')
        state={ch.channel_id:ch.get_state() for ch in self._channels}
        channel_pairs=[(cid,force) for cid,_,_,_,_,_,force in snapshots if force is not None]
        stats=_compute_stats(channel_pairs,state) if channel_pairs else ForceStats([],0,0,False,False)
        if limits_valid and self._armed and len(channel_pairs)==len(self._channels):
            if any(force>=cfg.V3_CHANNEL_TRIP_N for _,force in channel_pairs) or stats.total>=cfg.V3_TOTAL_TRIP_N:
                self._fault_latch=True
        if self._armed and (stats.overload_trip or stats.hard_stop or invalid):
            self._fault_latch=True
        with self._recorder_lock:
            active_recorder = self._recorder
            if active_recorder is not None and active_recorder.writer_error is not None:
                self._reportRecorderFailure(active_recorder.writer_error)
        # CSV failures are data-quality events, NOT safety events. Never latch motor stop for these.
        # When limits are not signed off, remain unarmed FOR MOTION, but keep
        # sensor arming separate to avoid a permanent startup false latch.
        self._stop_req=bool(self._fault_latch or invalid or not limits_valid)
        interlock_reason = ('LATCHED_SENSOR_FAULT' if self._fault_latch else
                            ';'.join(invalid) if invalid else
                            limit_reason if not limits_valid else 'READY')
        if interlock_reason != self._last_interlock_report:
            self._last_interlock_report = interlock_reason
            if self._stop_req:
                self.get_logger().warn(f'[MOTOR COMPAT] STOP asserted: {interlock_reason}')
            else:
                self.get_logger().info('[MOTOR COMPAT] Valid sensor/approved limits; publishing loadcell force and stop=False')
        # Never publish an incomplete/stale/unapproved sum as valid /loadcell_total.
        if not invalid and limits_valid:
            self._update_compression_peak(stats.total)
            self._publish(stats)
        else:
            self._pubs['stop'].publish(Bool(data=True))
            self._pubs['code'].publish(Int32(data=3))
        with self._recorder_lock:
            rec=self._recorder
            if rec:
                d={cid:(ratio,tm,offset,force) for cid,ratio,tm,_,_,offset,force in snapshots}
                row={'record_time_ns':now_ns,'timestamp_utc':datetime.now(timezone.utc).isoformat(),
                    'elapsed_s':(now_ns-rec.start_ns)/1e9,'sample_index':self._sample_index,
                    'total_force_n':stats.total if len(channel_pairs)==len(self._channels) else '',
                    'calibration_id':self._calibration_id,
                    'sensor_valid':int(self._armed and not sensor_invalid),
                    'sensor_reason':';'.join(sensor_invalid),
                    'overload_status':int(self._fault_latch),
                    'stop_request':int(self._stop_req)}
                # Source Phidget channels are 0/1/2, CSV schema remains ch1/ch2/ch3.
                for csv_ch, physical_ch in enumerate(cfg.ACTIVE_CHANNELS, start=1):
                    ratio, tm, offset, force = d.get(physical_ch, (None, None, None, None))
                    row[f'ratio_ch{csv_ch}'] = ratio
                    row[f'receive_time_ch{csv_ch}_ns'] = tm
                    row[f'force_ch{csv_ch}_n'] = force
                    row[f'offset_ch{csv_ch}'] = offset
                try:
                    rec.record('sample',row)
                except Exception as exc:
                    self._reportRecorderFailure(exc)
                    # Do not modify sensor validity, stop request or a previously valid force publish.
        self._print_status(channel_pairs,stats,state) if channel_pairs else None

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
    except Exception as exc:
        # Some ROS2 distributions signal shutdown from a timer as an exception.
        if 'ExternalShutdownException' not in type(exc).__name__:
            raise
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
