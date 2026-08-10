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

        serial_port.reset_input_buffer()
        serial_port.reset_output_buffer()
        serial_port.write(request)
        serial_port.flush()

        echoed = serial_port.read(len(request))
        if cfg.SHOW_MODBUS_DEBUG:
            print(
                "[DWORD ECHO] "
                + " ".join(f"{byte:02X}" for byte in echoed)
            )

        if echoed != request:
            self.clear_buffers()
            raise MotorError(
                f"DWORD 요청 에코 불일치: address=0x{address:04X}, "
                f"expected={request.hex(' ')}, actual={echoed.hex(' ')}"
            )

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
        time.sleep(0.1)

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

        if output_rpm > cfg.MAX_VERIFIED_OUTPUT_SPEED_RPM:
            raise MotorError(
                "요청한 출력축 속도가 현재 실제 검증 범위를 초과합니다: "
                f"requested={output_rpm:.2f} rpm, "
                f"verified_limit={cfg.MAX_VERIFIED_OUTPUT_SPEED_RPM:.2f} rpm"
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
        if not 0 <= speed_setting <= cfg.MAX_INTERNAL_SPEED_SETTING:
            raise MotorError(
                "내부 속도 설정값 범위 초과: "
                f"value={speed_setting}, "
                f"limit={cfg.MAX_INTERNAL_SPEED_SETTING}"
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
