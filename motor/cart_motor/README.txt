실행: ROS 2 Humble / Python 환경에서 pip install pyserial minimalmodbus 필요
같은 디렉터리에서 python3 simple_motor.py
전진: ros2 topic pub --once /motor_start std_msgs/msg/Bool '{data: true}'
후진: ros2 topic pub --once /motor_back std_msgs/msg/Bool '{data: true}'
정지: ros2 topic pub --once /motor_stop std_msgs/msg/Bool '{data: true}'

motor_config.py 상단 START_POS, END_POS를 엔코더 절대 위치 정수(unit)로 지정하고 MOVE_RPM 조정.
두 위치는 필수 설정값이며 None 상태에서는 모터 연결 전에 오류로 종료함.
예: START_POS = 10000, END_POS = 55000 (예시일 뿐 실제 장비 위치에 맞게 지정).
1 mm = 1500 unit. END_POS가 START_POS보다 작아도 사용 가능.
두 위치는 설정된 위치 제한 이내이고 간격은 600 unit보다 커야 함.
기존 motor_config.py의 PORT 및 위치 제한, 모터 스케일과 리미트 설정 확인.
실행 시 자동 이동하지 않음. 현재 위치가 시작 위치 허용오차 이내면 READY.
시작 위치가 아니면 NOT_AT_START: /motor_back=True로 시작 위치에 도착한 뒤 전진 가능.
/motor_start=True는 지정한 END_POS로 이동, /motor_back=True는 START_POS로 이동.
모터가 움직이기 전 장비를 안전 위치에 두고, 상하 기계적 한계와 회전 방향 확인.
브레이크 없는 수직 축은 전원 차단/서보 OFF시 낙하할 수 있음. 물리적 안전장치 필수.
오프라인 가상 모터 시뮬레이션을 수행했으며 실제 장비 동작 검증은 하지 않았음.
