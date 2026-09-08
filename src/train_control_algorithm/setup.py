from glob import glob
from setuptools import find_packages, setup

package_name = 'train_control_algorithm'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/config', glob('config/*.yaml')),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zengxianwei',
    maintainer_email='1668657526@qq.com',
    description='Speed tracking node with PID state machine for train uncoupling robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'speed_track_node = train_control_algorithm.longitudinal_pid_ros_node:main',
            'test_upstream_node = train_control_algorithm.test_upstream_node:main',
        ],
    },
)
