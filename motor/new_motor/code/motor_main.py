#!/usr/bin/env python3
"""호환 실행 진입점.

실제 CPR 시퀀스는 motor_main_v2.py에만 유지한다. 이전 구현이 설정과
달라지는 문제를 막기 위해 이 파일은 최신 진입점을 그대로 호출한다.
"""

from motor_main_v2 import main


if __name__ == "__main__":
    main()
