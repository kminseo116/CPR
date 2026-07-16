"""
motor_main.py

실행 진입점이다.
이 파일만 실행하면 나머지 파일들이 import되어 전체 CPR 모터 제어가 실행된다.

실행:
    python3 motor_main.py
"""

import rclpy

from motor_ros_node_02 import MotorRosBridge

import motor_config as cfg
import motor_driver as drv
import motor_diagnostics as diag

from motor_sequence import (
    UserStopRequested,
    return_to_initial_and_shutdown,
    run_sequence,
)


def main():
    rclpy.init()

    ros = MotorRosBridge()
    motor = drv.create_motor_connection(cfg.PORT, cfg.SLAVE_ID)

    try:
        print("\n[READY] /motor_start=True 를 기다립니다.")
        print('실행: ros2 topic pub /motor_start std_msgs/Bool "data: true" --once')
        print('정지: ros2 topic pub /motor_stop std_msgs/Bool "data: true" --once\n')

        while rclpy.ok():
            rclpy.spin_once(ros, timeout_sec=0.1)

            if ros.motor_start:
                ros.motor_start = False
                ros.motor_stop = False

                try:
                    run_sequence(motor, ros)

                except UserStopRequested as e:
                    print("\n[USER STOP]", e)
                    return_to_initial_and_shutdown(motor, ros, reason=str(e))

                print("\n[READY] 다시 /motor_start=True 를 기다립니다.")

    except KeyboardInterrupt:
        print("\n[KEYBOARD INTERRUPT] 사용자 Ctrl+C 중단")
        drv.safe_stop(motor)

    except UserStopRequested as e:
        print("\n[USER STOP]", e)
        return_to_initial_and_shutdown(motor, ros, reason=str(e))

    except Exception as e:
        print("\n[ERROR] 오류 발생:", e)

        try:
            diag.diagnose_motor(motor)
        except Exception as diag_e:
            print("[DIAG ERROR]", diag_e)

        drv.safe_stop(motor)
        ros.publish_state("SAFE_STOP")

    finally:
        ros.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()