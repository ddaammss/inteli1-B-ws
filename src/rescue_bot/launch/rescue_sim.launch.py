# rescue_sim.launch.py v0.100 2026-03-14
# [이번 버전에서 수정된 사항]
# - launch 역할 분리를 위해 시뮬레이터 전용 런치 파일로 설명 정리
# - 불필요한 import 제거

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


ARGUMENTS = [
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Ignition World (warehouse, depot, maze, small_room)'),
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
    DeclareLaunchArgument('nav2', default_value='true',
                          choices=['true', 'false'],
                          description='Whether to launch Nav2'),
    DeclareLaunchArgument('slam', default_value='false',
                          choices=['true', 'false'],
                          description='SLAM (true) or Localization with existing map (false)'),
    DeclareLaunchArgument('localization', default_value='true',
                          choices=['true', 'false'],
                          description='Localization with existing map'),
    DeclareLaunchArgument('map', default_value='/home/gom/maps/warehouse_map.yaml',
                          description='Full path to map yaml file to load'),
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'],
                          description='Open RViz'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of robot spawn pose.'))


def generate_launch_description():
    # ──────────────────────────────────────────────
    # 1. TurtleBot4 Ignition Gazebo 시뮬레이터
    # ──────────────────────────────────────────────
    pkg_tb4_ignition = get_package_share_directory('turtlebot4_ignition_bringup')
    tb4_ignition_launch = PathJoinSubstitution(
        [pkg_tb4_ignition, 'launch', 'turtlebot4_ignition.launch.py'])

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([tb4_ignition_launch]),
        launch_arguments=[
            ('namespace',     'robot6'),
            ('world',         LaunchConfiguration('world')),
            ('model',         LaunchConfiguration('model')),
            ('nav2',          LaunchConfiguration('nav2')),
            ('slam',          LaunchConfiguration('slam')),
            ('localization',  LaunchConfiguration('localization')),
            ('map',           LaunchConfiguration('map')),
            ('rviz',          LaunchConfiguration('rviz')),
            ('x',             LaunchConfiguration('x')),
            ('y',             LaunchConfiguration('y')),
            ('z',             LaunchConfiguration('z')),
            ('yaw',           LaunchConfiguration('yaw')),
        ]
    )

    # ──────────────────────────────────────────────
    # 2. rescue_bot 노드들 (5초 딜레이 후 기동)
    #    - 가제보가 완전히 뜬 뒤 노드가 시작되도록
    # ──────────────────────────────────────────────
    control_node = Node(
        package='rescue_bot',
        executable='rescue_control_node',
        name='robot6_control_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    nav_node = Node(
        package='rescue_bot',
        executable='rescue_nav_node',
        name='rescue_nav_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    stt_node = Node(
        package='rescue_bot',
        executable='rescue_stt_node',
        name='rescue_dialogue_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    delayed_rescue_nodes = TimerAction(
        period=5.0,
        actions=[
            control_node,
            nav_node,
            stt_node,
        ]
    )

    # ──────────────────────────────────────────────
    # Launch Description 조합
    # ──────────────────────────────────────────────
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(sim_launch)
    ld.add_action(delayed_rescue_nodes)
    return ld
