#!/usr/bin/env python3

import time

import motor_config as cfg
import motor_driver as drv


def main():
    motor = drv.create_motor_connection(cfg.PORT, cfg.SLAVE_ID)

    try:
        print("=== 로봇 전진 테스트 ===")

        # Modbus 버스 모드
        drv.write_u16(motor, cfg.ADDR_P02_00, 9)
        time.sleep(0.1)

        # DI3 = Servo ON 설정
        drv.restore_di3_setting(motor)

        # 속도 모드 설정
        drv.setup_speed_search_mode(motor)

        # Servo ON
        drv.servo_on_by_forced_di(motor)

        drv.check_status(motor, "전진 시작 전")

        print(f"[START] 전진 시작 | 속도 지령 = {cfg.FORWARD_DIRECTION * cfg.SEARCH_RPM} rpm")

        # 로봇 전진 시작 (음수 속도 지령)
        drv.start_forward_speed_run(motor)

        # 5초간 전진
        time.sleep(5.0)

        print("[STOP] 모터 정지")

        drv.stop_motion(motor)
        time.sleep(0.3)

        # Servo OFF
        drv.servo_off_by_forced_di(motor)

        print("=== 전진 테스트 완료 ===")

    except KeyboardInterrupt:
        print("\n[Ctrl+C] 사용자 정지")
        drv.safe_stop(motor)

    except Exception as e:
        print(f"\n[ERROR] {e}")
        drv.safe_stop(motor)


if __name__ == "__main__":
    main()
