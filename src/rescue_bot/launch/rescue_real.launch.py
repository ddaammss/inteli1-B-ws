# rescue_real.launch.py v0.000 2026-03-14
# [이번 버전에서 수정된 사항]
# - 실로봇용 rescue_bot 노드(control/nav/stt) 실행 런치 파일 추가

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Rescue Robot 6 실로봇용 런치 파일.
    - rescue_control_node: 메인 비전 분석 및 제어
    - rescue_nav_node: 목표 수신 및 주행 제어
    - rescue_stt_node: 구조 대화 시나리오 수행
    """

    control_node = Node(
        package='rescue_bot',
        executable='rescue_control_node',
        name='robot6_control_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    nav_node = Node(
        package='rescue_bot',
        executable='rescue_nav_node',
        name='rescue_nav_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    stt_node = Node(
        package='rescue_bot',
        executable='rescue_stt_node',
        name='rescue_dialogue_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        control_node,
        nav_node,
        stt_node,
    ])
