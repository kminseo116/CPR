import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time


class MockMotorStatusPublisher(Node):
    def __init__(self):
        super().__init__('mock_motor_status_pub')

        self.pub = self.create_publisher(
            Float32MultiArray,
            '/motor_compression_status',
            10
        )

        self.bpm = 107.0
        self.depth_cm = 5.0
        self.count = 0
        self.start_time = time.time()
        self.period = 60.0 / self.bpm

        self.timer = self.create_timer(self.period, self.publish_status)

    def publish_status(self):
        self.count += 1

        msg = Float32MultiArray()
        msg.data = [
            float(self.count),
            float(self.bpm),
            float(time.time() - self.start_time),
            float(self.depth_cm),
        ]

        self.pub.publish(msg)

        self.get_logger().info(
            f"count={self.count}, bpm={self.bpm}, time={msg.data[2]:.2f}, depth={self.depth_cm}"
        )


def main():
    rclpy.init()
    node = MockMotorStatusPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()