# rear_ackermann_controller

Nav2의 `/cmd_vel`을 후륜 좌우 조향각과 구동 선속도로 변환하는 ROS 2 Python 패키지입니다.

## 임시 파라미터

- 휠베이스 L: 2.0 m
- 후륜 윤거 T: 1.3 m
- 최대 조향각: 30 deg

## 출력 토픽

`/rear_ackermann_cmd` (`std_msgs/msg/Float32MultiArray`)

- `data[0]`: 왼쪽 후륜 조향각 [rad]
- `data[1]`: 오른쪽 후륜 조향각 [rad]
- `data[2]`: 구동 목표 선속도 [m/s]

## 설치

```bash
cd ~/cpr_auto_ws/src
unzip rear_ackermann_controller.zip
cd ~/cpr_auto_ws
colcon build --symlink-install --packages-select rear_ackermann_controller
source install/setup.bash
```

## 실행

```bash
ros2 launch rear_ackermann_controller rear_ackermann.launch.py
```

## 테스트

좌회전 곡률:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 1.0}, angular: {z: 0.3}}"
```

출력 확인:

```bash
ros2 topic echo /rear_ackermann_cmd
```
