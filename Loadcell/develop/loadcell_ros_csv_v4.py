#!/usr/bin/env python3
"""Read-only ROS2 data logger for an EXISTING legacy loadcell publisher.

Does not instantiate Phidget devices, motor drivers, publishers, or actuator controls.
Keep legacy loadcell_main.py as the ONLY publisher of /loadcell_stop_request.
CSV schema matches V3 `recorded_filtered` eccentricity analysis.
"""
from __future__ import annotations

import json
import math
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

from loadcell_recorder_v3 import SessionRecorder

# These are logging/quality parameters, NOT authorization for motor motion.
kMaxFrameGapS = 0.30
kRosCheckPeriodS = 1.0


class LegacyForceCsvLogger(Node):
    def __init__(self, output_root: str | Path = 'recordings'):
        super().__init__('legacy_loadcell_csv_observer_v4')
        # Consult the SAME legacy config used by loadcell_main.py to label array order.
        import loadcell_config as legacy_cfg

        physical = tuple(legacy_cfg.ACTIVE_CHANNELS)
        if len(physical) != 3 or len(set(physical)) != 3:
            raise ValueError('Expected exactly 3 ACTIVE_CHANNELS in legacy loadcell_config.py')
        self._physical_channels = physical
        self._root = Path(output_root)
        self._recorder = None
        self._recorder_lock = threading.Lock()
        self._shutdown_requested = threading.Event()
        self._sample_index = 0
        self._last_frame_ns = None
        self._last_total = None
        self._last_total_ns = None
        self._last_stop = None
        self._last_stop_ns = None
        self._motor_state = 'UNKNOWN'
        self._calibration_id = 1
        self._record_error = None
        self._invalid_frame_count = 0
        self._status_report_ns = 0

        # Strictly subscriptions: no create_publisher, no motor driver.
        self.create_subscription(Float32MultiArray, '/loadcell_each', self._onForces, 100)
        self.create_subscription(Float32, '/loadcell_total', self._onTotal, 30)
        self.create_subscription(Bool, '/loadcell_stop_request', self._onStop, 30)
        self.create_subscription(String, '/motor_state', self._onMotorState, 30)
        self.create_subscription(Float32MultiArray, '/motor_compression_status', self._onMotorCompression, 30)
        self.create_subscription(Bool, '/loadcell_reset', self._onReset, 10)
        self.create_timer(kRosCheckPeriodS, self._healthCheck)

        if not sys.stdin.isatty():
            self.get_logger().warning('stdin is not a terminal: interactive s/e commands disabled')
        else:
            self._keyboard_thread = threading.Thread(target=self._keyboardLoop, daemon=True)
            self._keyboard_thread.start()
        self.get_logger().info('READ-ONLY CSV observer: s+Enter=start, e+Enter=save, x+Enter=exit')
        self.get_logger().info(f'ROS /loadcell_each index order -> CSV ch1/ch2/ch3: {physical}')
        self.get_logger().warning('This observer is NOT a motor safety monitor or motor motion approval.')

    def _onForces(self, msg):
        now_ns = time.monotonic_ns()
        try:
            arr = [float(value) for value in msg.data]
            if len(arr) != 3 or not all(math.isfinite(value) for value in arr):
                raise ValueError('Expected three finite forces')
        except (ValueError, TypeError):
            self._invalid_frame_count += 1
            self._recordEvent('INVALID_FORCE_FRAME', f'length={len(msg.data)}')
            return

        gap = None if self._last_frame_ns is None else (now_ns - self._last_frame_ns) / 1e9
        self._last_frame_ns = now_ns
        self._sample_index += 1
        with self._recorder_lock:
            rec = self._recorder
            if rec is None:
                return
            if gap is not None and gap > kMaxFrameGapS:
                try:
                    rec.event('FORCE_STREAM_GAP', f'gap_s={gap:.3f}')
                except Exception as exc:
                    self._reportError(exc)
            # Capture callbacks provide arrival time, NOT physical per-channel sample time.
            # Using one ROS arrival timestamp for each channel is an explicit limitation.
            row = {
                'record_time_ns': now_ns,
                'timestamp_utc': datetime.now(timezone.utc).isoformat(),
                'elapsed_s': (now_ns - rec.start_ns)/1e9,
                'sample_index': self._sample_index,
                'force_ch1_n': arr[0],
                'force_ch2_n': arr[1],
                'force_ch3_n': arr[2],
                'total_force_n': sum(arr),
                'receive_time_ch1_ns': now_ns,
                'receive_time_ch2_ns': now_ns,
                'receive_time_ch3_ns': now_ns,
                'calibration_id': self._calibration_id,
                'sensor_valid': 0 if self._record_error else 1,
                'sensor_reason': ('LOGGER_ERROR' if self._record_error else 'ROS_FILTERED_VALUES_NO_RAW_TIMESTAMP'),
                'overload_status': '',  # Legacy status not enough to reproduce V3 latch.
                'stop_request': '' if self._last_stop is None else int(self._last_stop),
            }
            try:
                rec.record('sample', row)
            except Exception as exc:
                self._reportError(exc)

    def _onTotal(self, msg):
        self._last_total = float(msg.data)
        self._last_total_ns = time.monotonic_ns()

    def _onStop(self, msg):
        value = bool(msg.data)
        self._last_stop_ns = time.monotonic_ns()
        if self._last_stop is not None and self._last_stop != value:
            self._recordEvent('LEGACY_STOP_REQUEST', f'value={int(value)}')
        self._last_stop = value

    def _onMotorState(self, msg):
        self._motor_state = str(msg.data)
        self._recordEvent('MOTOR_STATE', self._motor_state)

    def _onMotorCompression(self, msg):
        self._recordEvent('MOTOR_COMPRESSION', json.dumps([float(x) for x in msg.data]))

    def _onReset(self, msg):
        if msg.data:
            # Actual zeroing is delayed in legacy code, so request time is only an approximation.
            self._calibration_id += 1
            self._recordEvent('LEGACY_RESET_REQUESTED', 'Actual zero-completion time is not observable')
            self.get_logger().warning('Legacy reset requested: split recordings before/after recalibration for analysis')

    def _healthCheck(self):
        now_ns = time.monotonic_ns()
        if self._last_frame_ns is None:
            self.get_logger().warning('No /loadcell_each messages observed; start legacy loadcell_main.py first')
        elif (now_ns - self._last_frame_ns) / 1e9 > kMaxFrameGapS:
            self.get_logger().warning('Loadcell force messages missing/stale; check legacy publisher')
        if self._record_error:
            self.get_logger().error(f'CSV recording error: {self._record_error}')

    def _recordEvent(self, name, detail=''):
        with self._recorder_lock:
            if self._recorder is not None:
                try:
                    self._recorder.event(name, detail)
                except Exception as exc:
                    self._reportError(exc)

    def _reportError(self, exc):
        self._record_error = f'{type(exc).__name__}: {exc}'
        print(f'[REC ERROR] {self._record_error}', flush=True)

    def _startRecording(self):
        with self._recorder_lock:
            if self._recorder is not None:
                print('[REC] Already recording', flush=True)
                return
            if self._last_frame_ns is None or (time.monotonic_ns() - self._last_frame_ns) / 1e9 > kMaxFrameGapS:
                print('[REC] REFUSED: no fresh /loadcell_each; start legacy loadcell node', flush=True)
                return
            meta = {
                'recorder_mode': 'READ_ONLY_LEGACY_ROS_OBSERVER',
                'config': {'ACTIVE_CHANNELS': list(self._physical_channels),
                           'CHANNEL_FULL_SCALE_RATIO': {},
                           'FULL_SCALE_KG': None, 'GRAVITY': None},
                'csv_channel_map': {'ch1': self._physical_channels[0],
                                    'ch2': self._physical_channels[1],
                                    'ch3': self._physical_channels[2]},
                'motor_state_last_seen': self._motor_state,
                'raw_channel_events_available': False,
                'force_source': 'ROS2 /loadcell_each Float32MultiArray (filtered)',
                'force_timestamp_semantics': 'ROS subscriber arrival, same stamp for three channels',
                'warning': ('Passive data observer. Does not validate motor safety. '
                            'No raw voltage ratios, no sensor capture timestamps; '
                            'analysis must use force_source=recorded_filtered.'),
            }
            self._record_error = None
            self._recorder = SessionRecorder(self._root, meta)
            print(f'[REC] started {self._recorder.directory}', flush=True)

    def _stopRecording(self):
        with self._recorder_lock:
            recorder = self._recorder
            self._recorder = None
        if recorder is not None:
            try:
                path = recorder.close()
                print(f'[REC] saved {path}', flush=True)
            except Exception as exc:
                self._reportError(exc)
                print(f'[REC INCOMPLETE] {recorder.directory}', flush=True)
                raise

    def _keyboardLoop(self):
        while not self._shutdown_requested.is_set():
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
                    self._recordEvent('ALIGNMENT_CHANGED_BY_OPERATOR')
                elif line == 'x':
                    self._stopRecording()
                    self._shutdown_requested.set()
                    return
            except Exception as exc:
                self._reportError(exc)

    def shouldExit(self):
        return self._shutdown_requested.is_set()

    def destroy_node(self):
        self._shutdown_requested.set()
        try:
            self._stopRecording()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = LegacyForceCsvLogger()
        while rclpy.ok() and not node.shouldExit():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print('[REC] Ctrl+C; closing recording (does NOT stop motor)', flush=True)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
