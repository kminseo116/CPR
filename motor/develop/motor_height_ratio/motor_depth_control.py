"""
motor_depth_control.py

흉부 높이 비율 기반 자동 압박 깊이 제어 알고리즘을 담당하는 파일이다.

역할:
1. motor unit ↔ mm/cm 변환
2. 접촉 위치 기반 로봇 기준 흉부 높이 추정
3. 흉부 높이 × 비율로 초기 목표 압박 깊이 계산
4. 최종 깊이를 50~60 mm로 제한
5. recoil 상태 확인
"""

import motor_config as cfg


def clamp(value, low, high):
    """value를 low~high 범위로 제한한다."""
    return max(low, min(value, high))


def units_to_mm(position_units):
    """모터 지령단위 unit을 mm로 변환한다."""
    return float(position_units) / float(cfg.MOTOR_UNITS_PER_MM)


def mm_to_units(mm):
    """mm를 모터 지령단위 unit으로 변환한다."""
    return int(float(mm) * float(cfg.MOTOR_UNITS_PER_MM))


def position_units_to_cm(position_units):
    """모터 위치 차이 unit을 cm로 변환한다."""
    return units_to_mm(position_units) / 10.0


def calculate_initial_to_contact_mm(initial_pos, contact_pos):
    """
    초기 위치에서 흉부 접촉 위치까지 압박봉이 이동한 거리 [mm]를 계산한다.

    이 값은 흉부 높이가 아니라,
    INITIAL_POS에서 contact_pos까지 압박봉이 내려간 이동거리이다.
    """
    delta_units = abs(int(contact_pos) - int(initial_pos))
    return units_to_mm(delta_units)


def estimate_chest_height_mm(initial_pos, contact_pos):
    """
    로봇 기준 흉부 높이 [mm]를 계산한다.

    흉부 높이 =
        받침면/등판면 기준 초기 압박봉 끝단 높이
        - 초기 위치에서 흉부 접촉 위치까지 이동한 거리
    """
    initial_to_contact_mm = calculate_initial_to_contact_mm(
        initial_pos,
        contact_pos,
    )

    chest_height_mm = (
        cfg.INITIAL_PLATE_HEIGHT_FROM_SUPPORT_MM
        - initial_to_contact_mm
    )

    if chest_height_mm < cfg.MIN_VALID_CHEST_HEIGHT_MM:
        print(
            f"[WARN] 계산된 흉부 높이가 너무 작음: "
            f"{chest_height_mm:.1f} mm < {cfg.MIN_VALID_CHEST_HEIGHT_MM:.1f} mm"
        )

    if chest_height_mm > cfg.MAX_VALID_CHEST_HEIGHT_MM:
        print(
            f"[WARN] 계산된 흉부 높이가 너무 큼: "
            f"{chest_height_mm:.1f} mm > {cfg.MAX_VALID_CHEST_HEIGHT_MM:.1f} mm"
        )

    return chest_height_mm


def calculate_initial_depth_mm(chest_height_mm):
    """
    흉부 높이 기반 초기 압박 깊이 계산.

    기준:
    - LOW_CHEST_HEIGHT_MM 이하이면 MIN_COMPRESSION_DEPTH_MM 적용
    - HIGH_CHEST_HEIGHT_MM 이상이면 MAX_COMPRESSION_DEPTH_MM 적용
    - 그 사이에서는 선형 보간으로 50~60 mm 사이 깊이 계산

    예:
    LOW_CHEST_HEIGHT_MM = 140 mm
    HIGH_CHEST_HEIGHT_MM = 200 mm
    MIN_COMPRESSION_DEPTH_MM = 50 mm
    MAX_COMPRESSION_DEPTH_MM = 60 mm

    chest_height 140 mm -> depth 50.0 mm
    chest_height 160 mm -> depth 53.3 mm
    chest_height 180 mm -> depth 56.7 mm
    chest_height 200 mm -> depth 60.0 mm
    """
    chest_height_mm = float(chest_height_mm)

    low_h = float(cfg.LOW_CHEST_HEIGHT_MM)
    high_h = float(cfg.HIGH_CHEST_HEIGHT_MM)

    min_depth = float(cfg.MIN_COMPRESSION_DEPTH_MM)
    max_depth = float(cfg.MAX_COMPRESSION_DEPTH_MM)

    if high_h <= low_h:
        print(
            f"[WARN] 흉부 높이 매핑 기준 오류: "
            f"LOW={low_h:.1f}, HIGH={high_h:.1f}. "
            f"기본 최소 깊이 {min_depth:.1f} mm 사용"
        )
        return min_depth

    if chest_height_mm <= low_h:
        target_depth_mm = min_depth
        region = "LOW_CHEST_MIN_DEPTH"

    elif chest_height_mm >= high_h:
        target_depth_mm = max_depth
        region = "HIGH_CHEST_MAX_DEPTH"

    else:
        ratio = (chest_height_mm - low_h) / (high_h - low_h)
        target_depth_mm = min_depth + ratio * (max_depth - min_depth)
        region = "LINEAR_MAPPING"

    target_depth_mm = clamp(
        target_depth_mm,
        cfg.MIN_COMPRESSION_DEPTH_MM,
        cfg.MAX_COMPRESSION_DEPTH_MM,
    )

    print(
        f"[DEPTH MAP] chest_height={chest_height_mm:.1f} mm | "
        f"height_range={low_h:.1f}~{high_h:.1f} mm | "
        f"depth_range={min_depth:.1f}~{max_depth:.1f} mm | "
        f"target_depth={target_depth_mm:.1f} mm "
        f"({target_depth_mm / 10.0:.1f} cm) | "
        f"region={region}"
    )

    return target_depth_mm


def check_recoil_force(loadcell_total_n):
    """
    BACKWARD 복귀 후 recoil이 충분한지 확인한다.

    loadcell_total이 아직 크면 압박판이 흉부/마네킹을 충분히 놓지 못했을 수 있다.
    이 경우 경고만 출력하고, 즉시 정지는 하지 않는다.
    """
    if loadcell_total_n is None:
        return False, "NO_LOADCELL_TOTAL"

    loadcell_total_n = float(loadcell_total_n)

    if loadcell_total_n > cfg.RECOIL_FORCE_THRESHOLD_N:
        return False, "RECOIL_FORCE_REMAIN"

    return True, "RECOIL_OK"


def get_forward_target(start_pos, depth_mm):
    """
    압박 방향 목표 위치를 계산한다.

    기존 코드가 contact_pos + travel_units 방향으로 압박했으므로
    같은 방향을 유지한다.

    만약 실제 장비에서 방향이 반대라면 아래 return을
    start_pos - mm_to_units(depth_mm)로 바꾸면 된다.
    """
    return int(start_pos + mm_to_units(depth_mm))


def get_manual_depth_mm():
    """
    자동 깊이 기능을 끄는 경우 사용할 수동 깊이 입력 함수.
    """
    if not cfg.USE_INPUT_COMPRESSION_DEPTH:
        return float(cfg.DEFAULT_TRAVEL_MM)

    while True:
        try:
            depth_cm = round(float(input("\n[INPUT] 압박 깊이(cm) 입력 예: 5.0 > ")), 1)
        except ValueError:
            print("[ERROR] 숫자로 입력하세요.")
            continue

        if depth_cm <= 0:
            print("[ERROR] 0보다 큰 값을 입력하세요.")
            continue

        if depth_cm > cfg.MAX_COMPRESSION_DEPTH_CM:
            print(f"[ERROR] 최대 {cfg.MAX_COMPRESSION_DEPTH_CM:.1f}cm 이하로 입력하세요.")
            continue

        return depth_cm * 10.0
