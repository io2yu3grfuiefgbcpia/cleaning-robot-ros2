from setuptools import find_packages, setup

package_name = 'cleaning_robot_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yys',
    maintainer_email='yys@todo.todo',
    description='清扫机器人感知模块',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stereo_processor = cleaning_robot_perception.stereo_processor:main',
            'depth_camera_processor_node = cleaning_robot_perception.depth_camera_processor:main',
            'orbbec_camera_node = cleaning_robot_perception.orbbec_camera_node:main',
        ],
    },
)
