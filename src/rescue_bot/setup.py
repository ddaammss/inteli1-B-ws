# v0.000
# file: setup.py
# date: 2026-03-11
# changes:
# - rescue_nav_node_sucees entry point 추가
# - robot6_control_node entry point 추가
# - rescue_stt_node entry point 추가

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rescue_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chans',
    maintainer_email='ahwkt46@gmail.com',
    description='rescue robot orchestration package',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'database = rescue_bot.database.srd_database_node:main',
            'rescue_vision_node = rescue_bot.analyzer.rescue_vision_node:main',
            'rescue_control_node = rescue_bot.analyzer.rescue_control_node:main',
            'rescue_nav_node = rescue_bot.analyzer.rescue_nav_node:main',
            'rescue_stt_node = rescue_bot.analyzer.rescue_stt_node:main',
            'test_webcam_pub = rescue_bot.analyzer.test_webcam_pub:main',
        ],
    },
)