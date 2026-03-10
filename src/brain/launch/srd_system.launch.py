import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    분석 노드(Analyzer)와 데이터베이스 노드(Database)를 동시에 실행하는 런치 파일
    """
    return LaunchDescription([
        # 1. 시각 분석 노드 실행
        Node(
            package='brain',
            executable='analyzer',
            name='srd_analyzer_node',
            parameters=[{'model_path': 'yolo11n-pose.pt'}],
            output='screen'
        ),
        # 2. 데이터베이스 로깅 노드 실행
        Node(
            package='brain',
            executable='database',
            name='srd_database_node',
            output='screen'
        )
    ])