from glob import glob
from setuptools import find_packages, setup

package_name = 'rear_ackermann_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='/cmd_vel 기반 후륜 아커만 조향각 변환 노드',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'rear_ackermann_node = '
            'rear_ackermann_controller.rear_ackermann_node:main',
        ],
    },
)
