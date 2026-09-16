#!/usr/bin/env python3
"""ISL60-PR: /motor_start 전진, /motor_back 초기 위치 복귀.

START_POS와 END_POS에 지정한 엔코더 절대 위치 사이를 이동합니다.
사용자 설정은 motor_config.py 상단에서 변경합니다.
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String, Float32

import motor_config as cfg
import motor_driver as drv
import motor_diagnostics as diag



class SimpleMotor(Node):
    def __init__(self):
        super().__init__('simple_motor')
        self.start_requested = False
        self.back_requested = False
        self.stop_requested = False
        self.emergency_latched = False
        self.state = 'SETUP'
        self.start_pos = None
        self.end_pos = None
        self.motor = None
        self.target = None
        self.motion_started = None
        self.last_safety = 0.0
        self.arrival_count = 0

        self.create_subscription(Bool, '/motor_start', self.on_start, 10)
        self.create_subscription(Bool, '/motor_back', self.on_back, 10)
        self.create_subscription(Bool, '/motor_stop', self.on_stop, 10)
        self.state_pub = self.create_publisher(String, '/motor_state', 10)
        self.pos_pub = self.create_publisher(Int32, '/motor_absolute_position', 10)
        self.start_pub = self.create_publisher(Int32, '/motor_start_position', 10)
        self.end_pub = self.create_publisher(Int32, '/motor_end_position', 10)
        self.target_pub = self.create_publisher(Int32, '/motor_target_position', 10)
        self.current_pub = self.create_publisher(Float32, '/motor_current_a', 10)

    def on_start(self, msg):
        if msg.data:
            self.start_requested = True

    def on_back(self, msg):
        if msg.data:
            self.back_requested = True

    def on_stop(self, msg):
        if msg.data:
            self.stop_requested = True

    def set_state(self, value):
        if value != self.state:
            self.state = value
            self.get_logger().info(f'STATE: {value}')
        self.state_pub.publish(String(data=value))

    def setup(self):
        if type(cfg.START_POS) is not int or type(cfg.END_POS) is not int:
            raise ValueError('START_POS와 END_POS를 엔코더 절대 위치 정수(unit)로 지정하세요.')
        drv.check_position_safety(cfg.START_POS, 'configured start')
        drv.check_position_safety(cfg.END_POS, 'configured end')
        if abs(cfg.END_POS - cfg.START_POS) <= 2 * cfg.ARRIVAL_TOLERANCE:
            raise ValueError('시작·종료 위치 간격은 도착 허용오차의 2배보다 커야 합니다.')
        self.start_pos = cfg.START_POS
        self.end_pos = cfg.END_POS
        if not (0 < cfg.MOVE_RPM <= cfg.MAX_RPM_LIMIT):
            raise ValueError('MOVE_RPM 범위 오류')
        self.motor = drv.create_motor_connection(cfg.PORT, cfg.SLAVE_ID)
        drv.check_fault(self.motor)
        # 기존 설정을 유지하되, 하드웨어 리미트 비활성화는 수행하지 않음.
        drv.write_u16(self.motor, cfg.ADDR_P02_00, 9)
        time.sleep(0.1)
        drv.restore_di3_setting(self.motor)
        drv.setup_position_mode(self.motor, cfg.MOVE_RPM)
        drv.servo_on_by_forced_di(self.motor)
        drv.check_fault(self.motor)
        pos = drv.read_current_position(self.motor)
        drv.check_position_safety(pos, 'current position')
        self.start_pub.publish(Int32(data=self.start_pos))
        self.end_pub.publish(Int32(data=self.end_pos))
        self.pos_pub.publish(Int32(data=pos))
        self.get_logger().info(
            f'START_POS={self.start_pos}, END_POS={self.end_pos}, '
            f'travel={abs(self.end_pos - self.start_pos) / cfg.MOTOR_UNITS_PER_MM}mm, speed={cfg.MOVE_RPM}rpm; '
            '실제 전진 방향 및 기계 간섭 확인 필수'
        )
        if abs(pos - self.start_pos) <= cfg.ARRIVAL_TOLERANCE:
            self.set_state('READY')
        else:
            self.set_state('NOT_AT_START')
            self.get_logger().info('/motor_back=True로 지정한 시작 위치로 이동하세요.')

    def command_move(self, target, new_state):
        drv.check_position_safety(target, 'target')
        drv.check_fault(self.motor)
        drv.setup_position_mode(self.motor, cfg.MOVE_RPM)
        self.target = target
        self.motion_started = time.monotonic()
        self.last_safety = 0.0
        self.arrival_count = 0
        self.target_pub.publish(Int32(data=target))
        self.set_state(new_state)
        drv.move_absolute_position(self.motor, target)

    def halt(self):
        # 기존 드라이버의 일반 정지 명령을 우선 사용.
        drv.stop_motion(self.motor)
        self.target = None

    def tick(self):
        if self.stop_requested:
            self.get_logger().error('/motor_stop=True: 정지 상태 래치 (재시작 필요)')
            drv.emergency_stop(self.motor)
            self.emergency_latched = True
            self.set_state('STOPPED')
            return False

        now = time.monotonic()
        if now - self.last_safety >= cfg.SAFETY_PERIOD_S:
            drv.check_fault(self.motor)
            drv.check_current_safety(self.motor)
            current = drv.read_current_a(self.motor)
            self.current_pub.publish(Float32(data=float(current)))
            self.last_safety = now

        pos = drv.read_current_position(self.motor)
        drv.check_position_safety(pos, 'current position')
        self.pos_pub.publish(Int32(data=pos))
        self.start_pub.publish(Int32(data=self.start_pos))
        self.end_pub.publish(Int32(data=self.end_pos))
        self.state_pub.publish(String(data=self.state))

        # back 명령은 전진 중에도 우선 처리: 감속 정지 후 시작 위치로 복귀.
        if self.back_requested:
            self.back_requested = False
            self.start_requested = False
            if self.state == 'FORWARD_MOVING':
                self.halt()
                time.sleep(0.15)
                pos = drv.read_current_position(self.motor)
            if self.state in ('FORWARD_MOVING', 'AT_END', 'NOT_AT_START'):
                if abs(pos - self.start_pos) <= cfg.ARRIVAL_TOLERANCE:
                    self.set_state('READY')
                else:
                    self.command_move(self.start_pos, 'BACKWARD_MOVING')
            elif self.state == 'BACKWARD_MOVING':
                self.get_logger().info('이미 후진 중입니다.')
            else:
                self.get_logger().info('시작 위치 대기 중: back 명령 무시')
            return True

        if self.start_requested:
            self.start_requested = False
            if self.state == 'READY':
                self.command_move(self.end_pos, 'FORWARD_MOVING')
            else:
                self.get_logger().info('READY 상태가 아니므로 start 명령 무시')

        if self.state in ('FORWARD_MOVING', 'BACKWARD_MOVING'):
            if now - self.motion_started > cfg.MOVE_TIMEOUT:
                raise TimeoutError(f'{self.state} 목표 위치 도달 시간 초과')
            if abs(pos - self.target) <= cfg.ARRIVAL_TOLERANCE:
                self.arrival_count += 1
            else:
                self.arrival_count = 0
            if self.arrival_count >= cfg.ARRIVAL_SAMPLES:
                completed = self.state
                self.halt()
                self.set_state('AT_END' if completed == 'FORWARD_MOVING' else 'READY')
                self.get_logger().info(f'{completed} 완료, 실제 위치={pos}')
        return True

    def shutdown_motor(self, fault=False):
        if self.motor is None:
            if fault:
                self.set_state('FAULT')
            return
        if fault:
            try:
                diag.diagnose_motor(self.motor)
            except Exception as exc:
                self.get_logger().error(f'진단 실패: {exc}')
            try:
                drv.emergency_stop(self.motor)
            except Exception as exc:
                self.get_logger().error(f'긴급정지 통신 실패: {exc}')
            self.set_state('FAULT')
        elif not self.emergency_latched:
            try:
                drv.stop_motion(self.motor)
            except Exception as exc:
                self.get_logger().error(f'정지 명령 실패: {exc}')
        # 브레이크가 없는 축이므로 서보 OFF 자동 실행 금지: 중력 하강 위험.
        # 정지 유지/전원 차단은 장비의 독립 안전회로로 처리해야 합니다.


def main():
    rclpy.init()
    node = SimpleMotor()
    fault = False
    try:
        node.setup()
        node.get_logger().info('전진: /motor_start=True | 후진: /motor_back=True | 긴급정지: /motor_stop=True')
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=cfg.POLL_S)
            if not node.tick():
                break
    except KeyboardInterrupt:
        node.get_logger().warn('Ctrl+C: 정지 명령 실행')
    except Exception as exc:
        fault = True
        node.get_logger().error(f'운전 오류: {exc}')
    finally:
        node.shutdown_motor(fault=fault)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
