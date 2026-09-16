#!/usr/bin/env python3

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
        self.target = None
        self.motor = None

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
            self.get_logger().info('/motor_start=True 수신')

    def on_back(self, msg):
        if msg.data:
            self.back_requested = True
            self.get_logger().info('/motor_back=True 수신')

    def on_stop(self, msg):
        if msg.data:
            self.stop_requested = True
            self.get_logger().error('/motor_stop=True 수신')

    def set_state(self, value):
        if value != self.state:
            self.state = value
            self.get_logger().info(f'STATE: {value}')
        self.state_pub.publish(String(data=value))

    def setup(self):
        if type(cfg.START_POS) is not int or type(cfg.END_POS) is not int:
            raise ValueError('START_POS와 END_POS를 정수(unit)로 지정하세요.')

        drv.check_position_safety(cfg.START_POS, 'configured start')
        drv.check_position_safety(cfg.END_POS, 'configured end')

        if abs(cfg.END_POS - cfg.START_POS) <= 2 * cfg.ARRIVAL_TOLERANCE:
            raise ValueError('START_POS와 END_POS 간격이 너무 작습니다.')

        if not (0 < cfg.MOVE_RPM <= cfg.MAX_RPM_LIMIT):
            raise ValueError('MOVE_RPM 범위 오류')

        self.start_pos = cfg.START_POS
        self.end_pos = cfg.END_POS

        self.motor = drv.create_motor_connection(cfg.PORT, cfg.SLAVE_ID)

        drv.check_fault(self.motor)

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

        travel_mm = abs(self.end_pos - self.start_pos) / cfg.MOTOR_UNITS_PER_MM

        print("\n========== MOTOR SETTING ==========")
        print(f"현재 위치 : {pos}")
        print(f"START_POS : {self.start_pos}")
        print(f"END_POS   : {self.end_pos}")
        print(f"이동거리  : {travel_mm:.2f} mm")
        print(f"속도      : {cfg.MOVE_RPM} rpm")
        print("===================================\n")

        # 실행 시 항상 START_POS로 자동 복귀
        if abs(pos - self.start_pos) <= cfg.ARRIVAL_TOLERANCE:
            self.set_state('READY')
            self.get_logger().info(f'이미 START_POS에 있습니다. 현재 위치={pos}')
        else:
            self.get_logger().info(
                f'초기 위치 자동 복귀 | 현재={pos} -> START_POS={self.start_pos}'
            )
            self.command_move(self.start_pos, 'INITIAL_MOVING')

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

        self.get_logger().info(f'MOVE | target={target} | state={new_state}')
        drv.move_absolute_position(self.motor, target)

    def halt(self):
        drv.stop_motion(self.motor)
        self.target = None

    def tick(self):
        if self.stop_requested:
            self.get_logger().error('/motor_stop=True : 긴급 정지')
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

        # BACK 명령
        if self.back_requested:
            self.back_requested = False
            self.start_requested = False

            if self.state == 'FORWARD_MOVING':
                self.get_logger().info('전진 중 BACK 명령 -> 정지 후 복귀')
                self.halt()
                time.sleep(0.15)
                pos = drv.read_current_position(self.motor)

            if self.state in ('FORWARD_MOVING', 'AT_END'):
                if abs(pos - self.start_pos) <= cfg.ARRIVAL_TOLERANCE:
                    self.set_state('READY')
                else:
                    self.command_move(self.start_pos, 'BACKWARD_MOVING')

            elif self.state == 'BACKWARD_MOVING':
                self.get_logger().info('이미 START_POS로 복귀 중입니다.')

            elif self.state == 'INITIAL_MOVING':
                self.get_logger().info('현재 초기 위치로 자동 복귀 중입니다.')

            elif self.state == 'READY':
                self.get_logger().info('이미 START_POS에 있습니다.')

            return True

        # START 명령
        if self.start_requested:
            self.start_requested = False

            if self.state == 'READY':
                self.get_logger().info(
                    f'전진 시작 | {self.start_pos} -> {self.end_pos}'
                )
                self.command_move(self.end_pos, 'FORWARD_MOVING')
            else:
                self.get_logger().info(
                    f'현재 상태={self.state}, READY가 아니므로 START 무시'
                )

        # 이동 중 도착 확인
        if self.state in ('INITIAL_MOVING', 'FORWARD_MOVING', 'BACKWARD_MOVING'):
            if now - self.motion_started > cfg.MOVE_TIMEOUT:
                raise TimeoutError(f'{self.state} 목표 위치 도달 시간 초과')

            error = abs(pos - self.target)

            if error <= cfg.ARRIVAL_TOLERANCE:
                self.arrival_count += 1
            else:
                self.arrival_count = 0

            if self.arrival_count >= cfg.ARRIVAL_SAMPLES:
                completed = self.state
                actual_pos = pos

                self.halt()

                if completed == 'INITIAL_MOVING':
                    self.set_state('READY')
                    self.get_logger().info(
                        f'초기 위치 복귀 완료 | START_POS={self.start_pos} | 실제={actual_pos}'
                    )

                elif completed == 'FORWARD_MOVING':
                    self.set_state('AT_END')
                    self.get_logger().info(
                        f'전진 완료 | END_POS={self.end_pos} | 실제={actual_pos}'
                    )

                elif completed == 'BACKWARD_MOVING':
                    self.set_state('READY')
                    self.get_logger().info(
                        f'복귀 완료 | START_POS={self.start_pos} | 실제={actual_pos}'
                    )

        return True

    def shutdown_motor(self, fault=False):
        if self.motor is None:
            if fault:
                self.set_state('FAULT')
            return

        # Fault 발생
        if fault:
            try:
                diag.diagnose_motor(self.motor)
            except Exception as exc:
                self.get_logger().error(f'진단 실패: {exc}')

            try:
                drv.emergency_stop(self.motor)
            except Exception as exc:
                self.get_logger().error(f'긴급정지 실패: {exc}')

            try:
                drv.servo_off_by_forced_di(self.motor)
                self.get_logger().info('Servo OFF 완료')
            except Exception as exc:
                self.get_logger().error(f'Servo OFF 실패: {exc}')

            self.set_state('FAULT')
            return

        # /motor_stop=True로 종료한 경우
        if self.emergency_latched:
            try:
                drv.servo_off_by_forced_di(self.motor)
                self.get_logger().info('긴급정지 후 Servo OFF 완료')
            except Exception as exc:
                self.get_logger().error(f'Servo OFF 실패: {exc}')

            return

        # Ctrl+C 또는 정상 종료
        try:
            drv.stop_motion(self.motor)
        except Exception as exc:
            self.get_logger().error(f'정지 실패: {exc}')

        try:
            drv.servo_off_by_forced_di(self.motor)
            self.get_logger().info('Servo OFF 완료')
        except Exception as exc:
            self.get_logger().error(f'Servo OFF 실패: {exc}')


def main():
    rclpy.init()
    node = SimpleMotor()
    fault = False

    try:
        node.setup()

        node.get_logger().info(
            '/motor_start=True -> 전진 | '
            '/motor_back=True -> 복귀 | '
            '/motor_stop=True -> 긴급정지'
        )

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=cfg.POLL_S)

            if not node.tick():
                break

    except KeyboardInterrupt:
        node.get_logger().warn('Ctrl+C: 모터 정지')

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