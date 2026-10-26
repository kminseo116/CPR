"""Failure-reporting asynchronous CSV recorder; no ROS/Phidget dependency.

A worker exception is surfaced through writer_error and recording_status.json.
A complete session cannot be reported if the worker died or records were lost.
"""
from __future__ import annotations

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
