#!/usr/bin/env python3

import sys
import termios
import tty
import time

import motor_config as cfg
from motor_driver import MotorDriver


RPM = 100.0
MOVE_REV = 1.0

MOVE_UNITS = int(
    cfg.COMMAND_UNITS_PER_MOTOR_REV * MOVE_REV
)


def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(
            fd,
            termios.TCSADRAIN,
            old
        )

    return key


def main():
    driver = MotorDriver()
    servo_on = False

    try:
        print("[SETUP] 통신 확인")
        driver.verify_connection()

        print("[SETUP] 모터 설정")
        driver.setup_runtime()

        print("[SETUP] Servo ON")
        driver.enable()
        servo_on = True

        time.sleep(0.2)

        speed_setting = (
            driver.output_rpm_to_speed_setting(RPM)
        )

        print("\n========================")
        print(" W : 정방향 (+)")
        print(" S : 역방향 (-)")
        print(" SPACE : 정지")
        print(" Q : 종료 + Servo OFF")
        print("========================")
        print(f"RPM = {RPM}")
        print(f"MOVE_UNITS = {MOVE_UNITS}")
        print(f"SPEED_SETTING = {speed_setting}")

        while True:
            key = get_key().lower()

            if key == "w":
                print("\n[W] 정방향 +")

                driver.move_incremental(
                    +MOVE_UNITS,
                    speed_setting
                )

            elif key == "s":
                print("\n[S] 역방향 -")

                driver.move_incremental(
                    -MOVE_UNITS,
                    speed_setting
                )

            elif key == " ":
                print("\n[STOP]")
                driver.stop()

            elif key == "q":
                print("\n[Q] 종료")
                break

    except KeyboardInterrupt:
        print("\n[Ctrl+C] 종료")

    except Exception as e:
        print(
            f"\n[ERROR] "
            f"{type(e).__name__}: {e}"
        )

    finally:
        # 먼저 모터 정지
        try:
            print("[EXIT] Motor STOP")
            driver.stop()
        except Exception as e:
            print(f"[STOP ERROR] {e}")

        # 그 다음 Servo OFF
        if servo_on:
            try:
                print("[EXIT] Servo OFF")
                driver.disable()
                servo_on = False
            except Exception as e:
                print(f"[SERVO OFF ERROR] {e}")

        print("[EXIT] 종료 완료")


if __name__ == "__main__":
    main()