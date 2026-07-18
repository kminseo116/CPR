# 새 RS485 모터 CPR 압박 코드

## 파일 구성

| 파일 | 역할 |
|---|---|
| `motor_config.py` | 통신, 속도, 깊이, 접촉, 안전, 레지스터 설정 |
| `motor_driver.py` | Modbus 통신과 모터 직접 제어 |
| `motor_ros_node.py` | ROS2 토픽 입력·출력 |
| `motor_main.py` | CPR 전체 실행 순서 |
| `register_check.py` | 레지스터 주소와 DWORD WORD 순서 확인 |

## 전체 실행 순서

```text
/motor_start=True
→ 통신 확인
→ 모터 리셋
→ Servo ON
→ 현재 위치를 INITIAL_POS로 저장
→ 압박 깊이 입력
→ 무부하 전류 기준 측정
→ 접촉 탐색
   1순위: 로드셀 4.5 N 이상
   2순위: 로드셀 stale + 전류 증가 지속
→ 목표 깊이까지 압박
→ 접촉 위치까지 복귀
→ 반복
→ 일반 STOP 또는 반복 완료
→ INITIAL_POS 복귀
→ Servo ON 유지
```

과전류 또는 모터 오류 발생 시에는 자동 복귀하지 않는다.

```text
고장 감지
→ 모터 정지
→ FAULT_LATCHED 상태
→ 자동 초기 위치 복귀 안 함
```

## 사용자가 주로 조정하는 값

`motor_config.py`

```python
PORT = "/dev/ttyUSB0"
MOTOR_DIRECTION_SIGN = 1

DEFAULT_DEPTH_CM = 5.0
REPEAT_COUNT = 1000

INITIAL_SPEED_RPM = 200
SEARCH_SPEED_RPM = 60
COMPRESSION_SPEED_RPM = 1800

CONTACT_FORCE_N = 4.5
CONTACT_CURRENT_DELTA_RAW = 30
MAX_CURRENT_RAW = 450
```

`CONTACT_CURRENT_DELTA_RAW`와 `MAX_CURRENT_RAW`는 실제 장비 전류 로그를 보고 조정해야 한다.

## 실행

```bash
pip install minimalmodbus pyserial
source /opt/ros/humble/setup.bash
python3 motor_main.py
```

시작:

```bash
ros2 topic pub /motor_start std_msgs/Bool "data: true" --once
```

정지:

```bash
ros2 topic pub /motor_stop std_msgs/Bool "data: true" --once
```

접촉 판단 출처 확인:

```bash
ros2 topic echo /motor_contact_source
```

## 최초 무부하 시험값

```python
DEFAULT_DEPTH_CM = 0.5
REPEAT_COUNT = 1
SEARCH_SPEED_RPM = 20
COMPRESSION_SPEED_RPM = 100
CONTACT_SEARCH_STEP_MM = 0.2
```

장비 방향, 위치 단위, 전류 raw 값을 확인한 뒤 단계적으로 속도와 깊이를 올린다.
