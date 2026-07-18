# 실제 모터 없는 CPR 알고리즘 시험

## 목적

실제 RS485 모터를 연결하지 않고 다음 로직을 확인합니다.

1. 로드셀 4.5 N 접촉 검출
2. `START_POS` 및 `END_POS` 계산
3. `START_POS ↔ END_POS` 반복 압박
4. 정상 STOP 후 `INITIAL_POS` 복귀 및 Servo ON 유지
5. 로드셀 STOP 후 `INITIAL_POS` 복귀 및 Servo ON 유지
6. 과전류 시 자동 복귀 금지, Servo ON 유지, FAULT 전환

## 파일

- `motor_main.py`: 실제 알고리즘
- `motor_config.py`: 실제 설정
- `mock_components.py`: 가상 모터와 가상 ROS 브리지
- `motor_mock_test.py`: 시험 시나리오 실행기

## 실행

ROS2와 기존 Python 환경에서 폴더로 이동한 뒤 실행합니다.

```bash
python3 motor_mock_test.py
```

정상 결과:

```text
=== 결과: 4/4 PASS ===
```

## 주의

이 시험은 알고리즘 분기와 위치 계산을 확인하는 소프트웨어 시험입니다. 다음 항목은 실제 장비 시험이 필요합니다.

- 실제 위치 레지스터 주소
- DWORD WORD 순서
- 위치 unit/rev
- 실제 이동 방향 부호
- 전류 raw 임계값
- 실제 가속·감속 및 CPR 속도
- 정전 또는 Servo OFF 시 압박봉 기계 거동
