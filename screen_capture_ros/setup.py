from setuptools import setup
import os
from glob import glob

package_name = 'screen_capture_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kehdgus9434',
    maintainer_email='kehdgus9434@example.com',
    description='scrcpy 화면 캡처를 위한 ROS2 패키지',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'screen_capture = screen_capture_ros.screen_capture_publisher:main',
            'screenshot_capture = screen_capture_ros.screenshot_publisher:main',
            'multi_capture = screen_capture_ros.multi_capture_publisher:main',
            'direct_capture = screen_capture_ros.direct_capture_publisher:main',
            'memory_capture = screen_capture_ros.direct_memory_publisher:main',
            'stable_capture = screen_capture_ros.stable_screen_capture_publisher:main',
            'pipe_capture = screen_capture_ros.pipe_screen_capture_publisher:main',
            'pure_memory_capture = screen_capture_ros.memory_capture_publisher:main',
        ],
    },
)
