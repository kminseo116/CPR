#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class RearAckermannNode(Node):
    """
    /cmd_vel을 후륜 아커만 조향 명령으로 변환합니다.

    입력:
      /cmd_vel (geometry_msgs/msg/Twist)
        linear.x  : 차량 기준 선속도 [m/s]
        angular.z : 차량 기준 요 각속도 [rad/s]

    출력:
      /rear_ackermann_cmd (std_msgs/msg/Float32MultiArray)
        data[0] : 왼쪽 후륜 조향각 [rad]
        data[1] : 오른쪽 후륜 조향각 [rad]
        data[2] : 구동 목표 선속도 [m/s]

    조향각 부호:
      + : 차량 기준 왼쪽
      - : 차량 기준 오른쪽
    """

    def __init__(self) -> None:
        super().__init__('rear_ackermann_node')

        # 차량 제원 및 제한값
        self.declare_parameter('wheelbase_m', 1.075)
        self.declare_parameter('rear_track_m', 0.336)
        self.declare_parameter('max_steering_deg', 30.0)
        self.declare_parameter('linear_epsilon', 1.0e-4)
        self.declare_parameter('angular_epsilon', 1.0e-4)
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/rear_ackermann_cmd')

        # yaml 파일에서 값 가져옴
        self.wheelbase_m = float(self.get_parameter('wheelbase_m').value)
        self.rear_track_m = float(self.get_parameter('rear_track_m').value)
        self.max_steering_rad = math.radians(
            float(self.get_parameter('max_steering_deg').value)
        )
        self.linear_epsilon = float(
            self.get_parameter('linear_epsilon').value
        )
        self.angular_epsilon = float(
            self.get_parameter('angular_epsilon').value
        )
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        # 입력값 검증
        if self.wheelbase_m <= 0.0:
            raise ValueError('wheelbase_m은 0보다 커야 합니다.')
        if self.rear_track_m <= 0.0:
            raise ValueError('rear_track_m은 0보다 커야 합니다.')
        if not 0.0 < self.max_steering_rad < math.pi / 2.0:
            raise ValueError(
                'max_steering_deg는 0도보다 크고 90도보다 작아야 합니다.'
            )

        self.publisher = self.create_publisher(
            Float32MultiArray,
            output_topic,
            10
        )
        self.subscription = self.create_subscription(
            Twist,
            input_topic,
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info(
            '후륜 아커만 변환 노드 시작 | '
            f'L={self.wheelbase_m:.3f} m, '
            f'T={self.rear_track_m:.3f} m, '
            f'max={math.degrees(self.max_steering_rad):.1f} deg'
        )

    def publish_command(
        self,
        left_angle_rad: float,
        right_angle_rad: float,
        drive_velocity_mps: float
    ) -> None:
        msg = Float32MultiArray()
        msg.data = [
            float(left_angle_rad),
            float(right_angle_rad),
            float(drive_velocity_mps),
        ]
        self.publisher.publish(msg)

    def cmd_vel_callback(self, msg: Twist) -> None:
        v = float(msg.linear.x)         # 선속도: 직진/ 후진
        omega = float(msg.angular.z)    # 요 각속도: 회전

        # 정지 -> 조향 중앙 복귀
        if (
            abs(v) < self.linear_epsilon
            and abs(omega) < self.angular_epsilon
        ):
            # (왼쪽 조향각, 오른쪽 조향각, 구동 속도)
            self.publish_command(0.0, 0.0, 0.0)
            return

        # 아커만 차량은 일반적으로 제자리 회전 불가
        if abs(v) < self.linear_epsilon:
            self.publish_command(0.0, 0.0, 0.0)
            self.get_logger().warning(
                f'제자리 회전 명령 거부: v={v:.4f}, omega={omega:.4f}'
            )
            return

        # 직진 또는 후진 직진
        if abs(omega) < self.angular_epsilon:
            self.publish_command(0.0, 0.0, v)
            return

        # 곡률 k = omega / v, curvature
        curvature = omega / v   # k
        requested_radius = 1.0 / abs(curvature) # R, Nav2가 요구한 회전반경

        half_track = self.rear_track_m / 2.0

        # 안쪽 후륜이 최대 조향각에 도달할 때의 최소 회전반경
        minimum_radius = (
            self.wheelbase_m / math.tan(self.max_steering_rad)
            + half_track
        )

        radius = max(requested_radius, minimum_radius)

        if requested_radius < minimum_radius:
            self.get_logger().warning(
                '요구 회전반경이 조향 한계를 초과했습니다. '
                f'요구 R={requested_radius:.3f} m → '
                f'제한 R={radius:.3f} m'
            )

        inner_mag = math.atan(self.wheelbase_m / (radius - half_track))
        outer_mag = math.atan(self.wheelbase_m / (radius + half_track))

        # 곡률 > 0: 경로가 차량 기준 왼쪽으로 굽음
        # 후륜 조향은 반대 방향(오른쪽, 음수)으로 꺾임
        # 왼쪽 바퀴가 안쪽, 오른쪽 바퀴가 바깥쪽
        if curvature > 0.0:
            left_angle = -inner_mag
            right_angle = -outer_mag
            direction = 'LEFT_CURVE'
        else:
            # 곡률 < 0: 경로가 오른쪽으로 굽음
            # 후륜은 왼쪽(양수)으로 꺾임
            # 오른쪽 바퀴가 안쪽, 왼쪽 바퀴가 바깥쪽
            left_angle = outer_mag
            right_angle = inner_mag
            direction = 'RIGHT_CURVE'

        self.publish_command(left_angle, right_angle, v)

        self.get_logger().info(
            f'[{direction}] '
            f'v={v:.3f} m/s, omega={omega:.3f} rad/s, '
            f'R={radius:.3f} m | '
            f'left={math.degrees(left_angle):.2f} deg, '
            f'right={math.degrees(right_angle):.2f} deg'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RearAckermannNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_command(0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
