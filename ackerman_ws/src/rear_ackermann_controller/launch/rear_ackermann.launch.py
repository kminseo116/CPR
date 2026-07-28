from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory(
        'rear_ackermann_controller'
    )
    params_file = os.path.join(
        package_share,
        'config',
        'rear_ackermann.yaml'
    )

    return LaunchDescription([
        Node(
            package='rear_ackermann_controller',
            executable='rear_ackermann_node',
            name='rear_ackermann_node',
            output='screen',
            parameters=[params_file],
            additional_env={
                'RCUTILS_CONSOLE_OUTPUT_FORMAT': '{message}'
            },
        )
    ])
