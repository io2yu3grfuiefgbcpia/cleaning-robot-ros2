import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'cleaning_robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yys',
    maintainer_email='yys@todo.todo',
    description='清扫机器人控制模块',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cleaning_controller = cleaning_robot_control.cleaning_controller:main',
            'cleaning_dashboard = cleaning_robot_control.cleaning_dashboard:main',
            'keyboard_teleop = cleaning_robot_control.keyboard_teleop:main',
            'track_motor_driver = cleaning_robot_control.track_motor_driver:main',
        ],
    },
)
