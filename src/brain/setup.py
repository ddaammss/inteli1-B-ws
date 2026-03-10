from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'brain'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 런치 파일을 설치하기 위한 경로 설정
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gom',
    maintainer_email='wndyd293@gmail.com',
    description='SRD Vision Analyzer and Database Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 실행 명령어 = 패키지.파일명:함수명
            'analyzer = brain.srd_advanced_analyzer:main',
            'database = brain.srd_database_node:main',
        ],
    },
)