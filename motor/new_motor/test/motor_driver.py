"""RS485 Modbus-RTU 서보모터 드라이버.

이 파일의 역할
--------------
1. Modbus 레지스터 읽기/쓰기
2. Servo ON/OFF, 정지, 리셋
3. 절대 위치 이동
4. 현재 위치·속도·전류·알람 읽기
5. 위치 제한 및 알람 해석
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from collections import deque
from datetime import datetime
from typing import Callable, TypeVar

import minimalmodbus

import motor_config as cfg


T = TypeVar("T")


@dataclass(frozen=True)
class MotorStatus:
    """한 시점의 모터 상태."""

    position: int
    speed_rpm: int
    current_raw: int
    position_error: int
    alarm: int
    run_status: int


@dataclass(frozen=True)
class MotorCycleStatus:
    """빠른 반복 운전 중 필요한 최소 안전 상태."""

    position: int
    current_raw: int
    alarm: int
    run_status: int


class MotorError(RuntimeError):
    """모터 통신 또는 제어 오류."""


class MotorDriver:
    """새 RS485 일체형 서보모터 제어 클래스."""

    def __init__(
        self,
        port: str = cfg.PORT,
        slave_id: int = cfg.SLAVE_ID,
    ) -> None:
        self.instrument = minimalmodbus.Instrument(port, slave_id)
        self._configure_serial()
        # 메모리 이력: 고장 시 추가 통신 없이 마지막 정상 상태를 볼 수 있다.
        self.events = deque(maxlen=48)
        self.last_position = None
        self.last_cycle_status = None
        self.last_status = None
        self.last_communication_error = None
        self.record_event("드라이버 연결 객체 생성", f"port={port}, slave={slave_id}")

    # ======================================================================
    # 1. 통신 초기화 및 재시도
    # ======================================================================

    def _configure_serial(self) -> None:
        serial_port = self.instrument.serial
        serial_port.baudrate = cfg.BAUDRATE
        serial_port.bytesize = cfg.BYTESIZE
        serial_port.parity = cfg.PARITY
        serial_port.stopbits = cfg.STOPBITS
        serial_port.timeout = cfg.TIMEOUT_S

        self.instrument.mode = minimalmodbus.MODE_RTU
        self.instrument.clear_buffers_before_each_transaction = True
        self.instrument.close_port_after_each_call = False

        # USB-RS485 변환기가 송신 프레임을 그대로 돌려주는 경우 제거
        self.instrument.handle_local_echo = False

    @staticmethod
    def register_name(address: int) -> str:
        names = {
            cfg.REG_MOTOR_START: "모터 기동/정지 명령",
            cfg.REG_INTERNAL_ENABLE: "서보 활성화",
            cfg.REG_INTERNAL_SPEED: "명령 속도",
            cfg.REG_INCREMENT_POSITION: "증분 위치 명령",
            cfg.REG_CURRENT_POSITION: "현재 위치 피드백",
            cfg.REG_ALARM_STATUS: "모터 보호 알람",
            cfg.REG_RUN_STATUS: "운전 상태",
            cfg.REG_CURRENT_CURRENT: "전류 원본값",
            cfg.REG_CURRENT_VOLTAGE: "전압 원본값",
            cfg.REG_POSITION_ERROR: "위치 편차",
            cfg.REG_CURRENT_SPEED: "실제 속도 레지스터",
        }
        return names.get(address, "기타 레지스터")

    def record_event(self, action: str, detail: str = "") -> None:
        stamp = datetime.now().astimezone().isoformat(timespec="milliseconds")
        self.events.append(f"{stamp} | {action} | {detail}")

    def _retry(self, operation: Callable[[], T], description: str) -> T:
        """실패한 Modbus 요청의 종류·횟수·경과시간을 남긴다."""
        started = time.monotonic()
        last_error: Exception | None = None
        for attempt in range(1, cfg.RETRIES + 2):
            try:
                result = operation()
                if attempt > 1:
                    self.record_event("통신 재시도 복구", f"{description}, {attempt}번째 시도")
                    print(f"[COMM RECOVERED] {description} | attempt={attempt}/{cfg.RETRIES + 1} | "
                          f"elapsed={time.monotonic()-started:.3f}s", flush=True)
                return result
            except Exception as exc:
                last_error = exc
                text = str(exc).lower()
                category = ("NO_RESPONSE/TIMEOUT" if any(x in text for x in
                            ("no answer", "no communication", "timed out", "timeout"))
                            else "MODBUS_OR_SERIAL_ERROR")
                print(f"[COMM FAIL] {description} | attempt={attempt}/{cfg.RETRIES + 1} | "
                      f"category={category} | exception={type(exc).__name__} | "
                      f"detail={exc} | elapsed={time.monotonic()-started:.3f}s", flush=True)
                self.last_communication_error = f"{description}: {type(exc).__name__}: {exc}"
                self.record_event("통신 오류", f"{description}, {attempt}/{cfg.RETRIES + 1}, {type(exc).__name__}: {exc}")
                self.clear_buffers()
                if attempt <= cfg.RETRIES:
                    time.sleep(cfg.RETRY_DELAY_S)
        raise MotorError(f"{description} 실패 (총 {cfg.RETRIES + 1}회, "
                         f"{time.monotonic()-started:.3f}s, "
                         f"원인={type(last_error).__name__}: {last_error})") from last_error

    def clear_buffers(self) -> None:
        try:
            self.instrument.serial.reset_input_buffer()
            self.instrument.serial.reset_output_buffer()
        except Exception:
            pass

    # ======================================================================
    # 2. 기본 레지스터 읽기/쓰기
    # ======================================================================

    def write_u16(self, address: int, value: int) -> None:
        value = int(value)
        if not 0 <= value <= 0xFFFF:
            raise ValueError(f"u16 범위 초과: {value}")

        self._retry(
            lambda: self.instrument.write_register(
                address,
                value,
                functioncode=6,
            ),
            f"{self.register_name(address)} 쓰기(0x{address:04X}, value={value})",
        )
        if address in (cfg.REG_MOTOR_START, cfg.REG_INTERNAL_ENABLE, cfg.REG_INTERNAL_SPEED):
            self.record_event("명령 응답 수신", f"{self.register_name(address)}, value={value} (실제 동작 확인 아님)")

    def read_u16(self, address: int) -> int:
        return int(
            self._retry(
                lambda: self.instrument.read_register(
                    address,
                    0,
                    functioncode=3,
                ),
                f"{self.register_name(address)} 읽기(0x{address:04X})",
            )
        )

    def read_i16(self, address: int) -> int:
        value = self.read_u16(address)
        return value - 0x10000 if value & 0x8000 else value

    @staticmethod
    def _i32_to_words(value: int) -> list[int]:
        unsigned = int(value) & 0xFFFFFFFF
        high_word = (unsigned >> 16) & 0xFFFF
        low_word = unsigned & 0xFFFF

        if cfg.DWORD_WORD_ORDER == "HIGH_LOW":
            return [high_word, low_word]
        if cfg.DWORD_WORD_ORDER == "LOW_HIGH":
            return [low_word, high_word]

        raise ValueError(
            f"잘못된 DWORD_WORD_ORDER: {cfg.DWORD_WORD_ORDER}"
        )

    @staticmethod
    def _words_to_i32(words: list[int]) -> int:
        if len(words) != 2:
            raise ValueError("DWORD는 WORD 2개가 필요합니다.")

        if cfg.DWORD_WORD_ORDER == "HIGH_LOW":
            high_word, low_word = words
        elif cfg.DWORD_WORD_ORDER == "LOW_HIGH":
            low_word, high_word = words
        else:
            raise ValueError(
                f"잘못된 DWORD_WORD_ORDER: {cfg.DWORD_WORD_ORDER}"
            )

        unsigned = (int(high_word) << 16) | int(low_word)
        if unsigned & 0x8000_0000:
            return unsigned - 0x1_0000_0000
        return unsigned

    @staticmethod
    def _modbus_crc(data: bytes) -> int:
        """Modbus-RTU CRC16을 계산한다."""
        crc = 0xFFFF

        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1

        return crc

    def write_i32(self, address: int, value: int) -> None:
        """32비트 signed 값을 기능 코드 0x10으로 쓴다.

        현재 장비는 표준 쓰기 응답 대신 요청 프레임을 그대로 에코한다.
        명령 레지스터는 읽기 검증이 불가능하므로 에코의 일치 여부를 검사한다.
        """
        words = self._i32_to_words(value)

        request_without_crc = struct.pack(
            ">BBHHBHH",
            int(self.instrument.address),
            0x10,
            int(address),
            2,
            4,
            int(words[0]),
            int(words[1]),
        )
        crc = self._modbus_crc(request_without_crc)
        request = request_without_crc + bytes(
            [crc & 0xFF, (crc >> 8) & 0xFF]
        )

        serial_port = self.instrument.serial
        if not serial_port.is_open:
            serial_port.open()

        if cfg.SHOW_MODBUS_DEBUG:
            print(
                f"[DWORD WRITE] address=0x{address:04X}, "
                f"value={value}, words={words}"
            )

        stage = "buffer reset"
        self.record_event("위치 명령 전송 시작", f"{self.register_name(address)}, value={value}")
        try:
            serial_port.reset_input_buffer()
            serial_port.reset_output_buffer()
            stage = "write/flush"
            serial_port.write(request)
            serial_port.flush()
            stage = "read echo"
            echoed = serial_port.read(len(request))
        except Exception as exc:
            self.last_communication_error = f"{self.register_name(address)} {stage}: {type(exc).__name__}: {exc}"
            self.record_event("위치 명령 통신 실패", self.last_communication_error)
            print(f"[COMM DWORD FAIL] address=0x{address:04X} | stage={stage} | "
                  f"exception={type(exc).__name__} | detail={exc}", flush=True)
            raise MotorError(f"DWORD 통신 실패 address=0x{address:04X} stage={stage}: {exc}") from exc
        if cfg.SHOW_MODBUS_DEBUG:
            print(
                "[DWORD ECHO] "
                + " ".join(f"{byte:02X}" for byte in echoed)
            )

        if echoed != request:
            self.last_communication_error = (f"{self.register_name(address)} 에코 불일치 "
                                             f"received={len(echoed)}/{len(request)} bytes")
            self.record_event("위치 명령 에코 불일치", self.last_communication_error)
            self.clear_buffers()
            raise MotorError(
                f"DWORD 요청 에코 불일치: address=0x{address:04X}, "
                f"expected={request.hex(' ')}, actual={echoed.hex(' ')}, "
                f"received={len(echoed)}/{len(request)} bytes "
                f"({'NO_ECHO_OR_TIMEOUT' if not echoed else 'PARTIAL_OR_DIFFERENT_ECHO'})"
            )

        self.record_event("위치 명령 에코 일치", f"{self.register_name(address)}, value={value} (실행 확인 아님)")
        if cfg.SHOW_MODBUS_DEBUG:
            print(
                f"[DWORD WRITE ACCEPTED] address=0x{address:04X}, "
                f"value={value}"
            )

    def read_i32(self, address: int) -> int:
        words = self._retry(
            lambda: self.instrument.read_registers(
                address,
                2,
                functioncode=3,
            ),
            f"{self.register_name(address)} 읽기(0x{address:04X})",
        )
        return self._words_to_i32(list(words))

    # ======================================================================
    # 3. 초기 설정 및 Servo 제어
    # ======================================================================

    def verify_connection(self) -> None:
        """알람 상태를 읽어 통신 연결을 확인한다."""
        self.read_u16(cfg.REG_ALARM_STATUS)

    def setup_runtime(self) -> None:
        """재기동 없이 적용 가능한 런타임 설정을 적용한다."""
        # 이전 기동 명령이 남은 상태에서 Servo가 켜지지 않도록 먼저 정지한다.
        self.write_u16(cfg.REG_MOTOR_START, 0)
        self.write_u16(cfg.REG_ENABLE_METHOD, 1)
        self.write_u16(cfg.REG_INTERNAL_ENABLE, 0)
        self.write_u16(cfg.REG_POSITION_COMMAND_MODE, 0)
        self.write_u16(cfg.REG_ACCEL, cfg.ACCEL_FACTOR)
        self.write_u16(cfg.REG_DECEL, cfg.DECEL_FACTOR)
        # 0x0010은 장비에서 40000으로 확인됐다. 일반 실행 때 사용자가
        # 지정한 압박 속도로 덮어쓰지 않고 기존 하드웨어 상한을 유지한다.

        if cfg.SAVE_PARAMETERS_ON_SETUP:
            self.write_u16(cfg.REG_SAVE, 1)

    def enable(self) -> None:
        """내부 명령 방식으로 Servo ON."""
        # Servo ON 전에 이동 시작 명령을 반드시 해제한다.
        self.write_u16(cfg.REG_MOTOR_START, 0)
        self.write_u16(cfg.REG_ENABLE_METHOD, 1)
        self.write_u16(cfg.REG_INTERNAL_ENABLE, 1)
        time.sleep(0.05)

        enable_method = self.read_u16(cfg.REG_ENABLE_METHOD)
        internal_enable = self.read_u16(cfg.REG_INTERNAL_ENABLE)

        print(
            f"[SERVO ENABLE VERIFY] method={enable_method}, "
            f"internal_enable={internal_enable}"
        )

        if enable_method != 1 or internal_enable != 1:
            raise MotorError(
                "내부 Servo ON 검증 실패: "
                f"method={enable_method}, "
                f"internal_enable={internal_enable}"
            )

    def disable(self) -> None:
        """운동 정지 후 Servo OFF."""
        try:
            self.stop()
        finally:
            self.write_u16(cfg.REG_INTERNAL_ENABLE, 0)

    def reset(self) -> None:
        """모터 리셋 명령을 펄스 형태로 전송한다."""
        self.write_u16(cfg.REG_MOTOR_RESET, 1)
        time.sleep(0.05)
        self.write_u16(cfg.REG_MOTOR_RESET, 0)

    def stop(self) -> None:
        """모터 기동 명령을 0으로 만들어 운동을 정지한다."""
        self.write_u16(cfg.REG_MOTOR_START, 0)

    def pause(self) -> None:
        """매뉴얼의 일시정지 명령을 전송한다."""
        self.write_u16(cfg.REG_MOTOR_PAUSE, 1)

    # ======================================================================
    # 4. 위치 이동 명령
    # ======================================================================

    @staticmethod
    def output_rpm_to_speed_setting(output_rpm: float) -> int:
        """원하는 실제 출력축 rpm을 모터 내부 속도 설정값으로 변환한다."""
        output_rpm = float(output_rpm)
        if output_rpm <= 0:
            raise MotorError(f"출력축 속도는 0보다 커야 합니다: {output_rpm}")

        allowed_output_rpm = min(
            cfg.MAX_OUTPUT_SPEED_RPM,
            cfg.MAX_VERIFIED_OUTPUT_SPEED_RPM,
        )
        if output_rpm > allowed_output_rpm:
            raise MotorError(
                "요청한 출력축 속도가 허용 범위를 초과합니다: "
                f"requested={output_rpm:.2f} rpm, "
                f"allowed={allowed_output_rpm:.2f} rpm"
            )

        speed_setting = round(
            output_rpm * cfg.SPEED_SETTING_PER_OUTPUT_RPM
        )
        if speed_setting > cfg.MAX_INTERNAL_SPEED_SETTING:
            raise MotorError(
                "요청한 출력축 속도가 현재 확인된 범위를 초과합니다: "
                f"requested={output_rpm:.2f} rpm, "
                f"max={cfg.MAX_OUTPUT_SPEED_RPM:.2f} rpm, "
                f"setting={speed_setting}, "
                f"setting_limit={cfg.MAX_INTERNAL_SPEED_SETTING}"
            )
        return speed_setting

    def set_speed(self, speed_setting: int) -> None:
        """모터 내부 속도 설정값을 확인된 범위 0~3000으로 기록한다."""
        speed_setting = int(speed_setting)
        # 0x002D의 매뉴얼 상한은 3000이다. 설정 파일의 값을 잘못 높여도
        # 범위 밖 명령이 실제 장치로 전송되지 않도록 하드웨어 한계를 고정한다.
        allowed_setting = min(cfg.MAX_INTERNAL_SPEED_SETTING, 3000)
        if not 0 <= speed_setting <= allowed_setting:
            raise MotorError(
                "내부 속도 설정값 범위 초과: "
                f"value={speed_setting}, "
                f"limit={allowed_setting}"
            )
        self.write_u16(cfg.REG_INTERNAL_SPEED, speed_setting)

    def move_absolute(self, target_position: int, speed_setting: int) -> None:
        """절대 위치 모드로 목표 위치 이동을 시작한다."""
        self.check_position_limit(target_position)
        self.stop()
        self.set_speed(speed_setting)
        self.write_u16(cfg.REG_POSITION_COMMAND_MODE, 0)
        self.write_i32(cfg.REG_ABSOLUTE_POSITION, int(target_position))
        self.write_u16(cfg.REG_MOTOR_START, 1)

    def move_incremental(self, move_units: int, speed_setting: int) -> None:
        """현재 위치를 기준으로 지정한 명령 단위만큼 이동을 시작한다."""
        move_units = int(move_units)
        self.check_position_limit(move_units)

        # 직전 기동 상태를 해제해 새 이동 명령의 시작 에지를 만든다.
        self.stop()
        time.sleep(0.05)
        self.set_speed(speed_setting)
        self.write_u16(cfg.REG_POSITION_COMMAND_MODE, 1)
        self.write_i32(cfg.REG_INCREMENT_POSITION, move_units)
        time.sleep(0.05)
        self.write_u16(cfg.REG_MOTOR_START, 1)

    def prepare_incremental_motion(self, speed_setting: int) -> None:
        """반복 증분 이동 전에 속도와 위치 명령 모드를 한 번만 설정한다."""
        self.stop()
        self.set_speed(speed_setting)
        self.write_u16(cfg.REG_POSITION_COMMAND_MODE, 1)

    def move_incremental_prepared(self, move_units: int) -> None:
        """사전 설정된 속도·증분 모드로 최소 명령만 보내 이동한다.

        CPR 왕복 전용 경로다. 매번 속도와 모드를 다시 쓰지 않고 기동
        해제, 증분값, 기동 명령만 전송한다.
        """
        move_units = int(move_units)
        self.check_position_limit(move_units)

        self.stop()
        time.sleep(cfg.CPR_COMMAND_EDGE_DELAY_S)
        self.write_i32(cfg.REG_INCREMENT_POSITION, move_units)
        time.sleep(cfg.CPR_COMMAND_EDGE_DELAY_S)
        self.write_u16(cfg.REG_MOTOR_START, 1)

    # ======================================================================
    # 5. 상태 읽기
    # ======================================================================

    def read_position(self) -> int:
        position = self.read_i32(cfg.REG_CURRENT_POSITION)
        self.last_position = (time.monotonic(), position)
        return position

    def read_status(self) -> MotorStatus:
        status = MotorStatus(
            position=self.read_position(),
            speed_rpm=self.read_i16(cfg.REG_CURRENT_SPEED),
            current_raw=self.read_u16(cfg.REG_CURRENT_CURRENT),
            position_error=self.read_i32(cfg.REG_POSITION_ERROR),
            alarm=self.read_u16(cfg.REG_ALARM_STATUS),
            run_status=self.read_u16(cfg.REG_RUN_STATUS),
        )
        self.last_status = (time.monotonic(), status)
        return status

    def read_cycle_status(self) -> MotorCycleStatus:
        """사이클 사이에 필요한 상태만 세 번의 통신으로 읽는다.

        알람과 운전 상태는 연속 레지스터라 한 번에 읽고, 전류를 읽은 뒤
        다음 증분 이동의 기준으로 쓸 현재 위치를 마지막에 읽는다.
        방향·속도·이동량은 위치 샘플과 명령값으로 계산하므로 읽지 않는다.
        """
        alarm_and_run = self._retry(
            lambda: self.instrument.read_registers(
                cfg.REG_ALARM_STATUS,
                2,
                functioncode=3,
            ),
            "사이클 알람/운전 상태 읽기",
        )
        if len(alarm_and_run) != 2:
            raise MotorError(
                "사이클 알람/운전 상태 길이 오류: "
                f"received={len(alarm_and_run)}"
            )

        current_raw = self.read_u16(cfg.REG_CURRENT_CURRENT)
        position = self.read_position()
        status = MotorCycleStatus(
            position=position,
            current_raw=current_raw,
            alarm=int(alarm_and_run[0]),
            run_status=int(alarm_and_run[1]),
        )
        self.last_cycle_status = (time.monotonic(), status)
        return status

    # ======================================================================
    # 6. 안전 검사 및 상태 해석
    # ======================================================================

    @staticmethod
    def check_position_limit(position: int) -> None:
        if cfg.MIN_POSITION_CMD <= int(position) <= cfg.MAX_POSITION_CMD:
            return

        raise MotorError(
            f"위치 제한 초과: {position}, "
            f"허용 범위=[{cfg.MIN_POSITION_CMD}, {cfg.MAX_POSITION_CMD}]"
        )

    @staticmethod
    def decode_alarm(alarm: int) -> list[str]:
        alarms: list[str] = []

        if alarm & cfg.ALARM_OVERVOLTAGE:
            alarms.append("OVERVOLTAGE")
        if alarm & cfg.ALARM_UNDERVOLTAGE:
            alarms.append("UNDERVOLTAGE")
        if alarm & cfg.ALARM_OVERCURRENT:
            alarms.append("OVERCURRENT")
        if alarm & cfg.ALARM_POSITION_DEVIATION:
            alarms.append("POSITION_DEVIATION")

        return alarms

    @staticmethod
    def target_reached(status: MotorStatus, target_position: int) -> bool:
        position_ok = (
            abs(status.position - target_position)
            <= cfg.POSITION_TOLERANCE_UNITS
        )
        status_bit_ok = bool(status.run_status & cfg.RUN_TARGET_REACHED)
        return position_ok or status_bit_ok