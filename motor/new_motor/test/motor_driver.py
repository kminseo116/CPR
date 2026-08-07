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

    def _retry(self, operation: Callable[[], T], description: str) -> T:
        last_error: Exception | None = None

        for _ in range(cfg.RETRIES + 1):
            try:
                return operation()
            except Exception as exc:
                last_error = exc
                self.clear_buffers()

        raise MotorError(f"{description} 실패: {last_error}")

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
            f"WORD 쓰기 0x{address:04X}",
        )

    def read_u16(self, address: int) -> int:
        return int(
            self._retry(
                lambda: self.instrument.read_register(
                    address,
                    0,
                    functioncode=3,
                ),
                f"WORD 읽기 0x{address:04X}",
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
        """32비트 signed 값을 기능 코드 0x10으로 쓰고 다시 읽어 검증한다."""
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

        last_error: Exception | None = None

        for attempt in range(cfg.RETRIES + 1):
            try:
                serial_port.reset_input_buffer()
                serial_port.reset_output_buffer()

                print(
                    f"[DWORD WRITE] address=0x{address:04X}, "
                    f"value={value}, words={words}, attempt={attempt + 1}"
                )

                serial_port.write(request)
                serial_port.flush()

                time.sleep(0.05)
                response = serial_port.read(serial_port.in_waiting or 8)
                print(
                    "[DWORD RESPONSE] "
                    + " ".join(f"{byte:02X}" for byte in response)
                )

                time.sleep(0.05)
                readback = self.read_i32(address)
                print(f"[DWORD VERIFY] write={value}, read={readback}")

                if readback != int(value):
                    raise MotorError(
                        f"DWORD 쓰기 검증 실패: address=0x{address:04X}, "
                        f"write={value}, read={readback}"
                    )

                return
            except Exception as exc:
                last_error = exc
                self.clear_buffers()
                time.sleep(0.05)

        raise MotorError(
            f"DWORD 쓰기 0x{address:04X} 실패: {last_error}"
        )

    def read_i32(self, address: int) -> int:
        words = self._retry(
            lambda: self.instrument.read_registers(
                address,
                2,
                functioncode=3,
            ),
            f"DWORD 읽기 0x{address:04X}",
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
        self.write_u16(cfg.REG_ENABLE_METHOD, 1)
        self.write_u16(cfg.REG_POSITION_COMMAND_MODE, 0)
        self.write_u16(cfg.REG_ACCEL, cfg.ACCEL_FACTOR)
        self.write_u16(cfg.REG_DECEL, cfg.DECEL_FACTOR)
        self.write_u16(
            cfg.REG_MAX_SPEED,
            min(cfg.COMPRESSION_SPEED_RPM, 50_000),
        )

        if cfg.SAVE_PARAMETERS_ON_SETUP:
            self.write_u16(cfg.REG_SAVE, 1)

    def enable(self) -> None:
        """내부 명령 방식으로 Servo ON."""
        self.write_u16(cfg.REG_ENABLE_METHOD, 1)
        self.write_u16(cfg.REG_INTERNAL_ENABLE, 1)
        time.sleep(0.1)

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

    def set_speed(self, rpm: int) -> None:
        """내부 속도를 매뉴얼 범위 0~3000 rpm으로 제한해 설정한다."""
        limited_rpm = max(0, min(int(rpm), 3000))
        self.write_u16(cfg.REG_INTERNAL_SPEED, limited_rpm)

    def move_absolute(self, target_position: int, speed_rpm: int) -> None:
        """절대 위치 모드로 목표 위치 이동을 시작한다."""
        self.check_position_limit(target_position)
        self.set_speed(speed_rpm)
        self.write_u16(cfg.REG_POSITION_COMMAND_MODE, 0)
        self.write_i32(cfg.REG_ABSOLUTE_POSITION, int(target_position))
        self.write_u16(cfg.REG_MOTOR_START, 1)

    # ======================================================================
    # 5. 상태 읽기
    # ======================================================================

    def read_position(self) -> int:
        return self.read_i32(cfg.REG_CURRENT_POSITION)

    def read_status(self) -> MotorStatus:
        return MotorStatus(
            position=self.read_position(),
            speed_rpm=self.read_i16(cfg.REG_CURRENT_SPEED),
            current_raw=self.read_u16(cfg.REG_CURRENT_CURRENT),
            position_error=self.read_i32(cfg.REG_POSITION_ERROR),
            alarm=self.read_u16(cfg.REG_ALARM_STATUS),
            run_status=self.read_u16(cfg.REG_RUN_STATUS),
        )

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
