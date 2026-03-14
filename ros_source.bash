#!/bin/bash
# ros_source.bash
# Usage: source ros_source.bash
# rescue_bot + TurtleBot4 환경을 한 번에 설정합니다.

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1. ROS 2 Humble 기본 환경
source /opt/ros/humble/setup.bash

# 2. TurtleBot4 워크스페이스 (turtlebot4_navigation 등 포함)
if [ -f ~/turtlebot4_ws/install/setup.bash ]; then
    source ~/turtlebot4_ws/install/setup.bash
    echo "[ros_source] TurtleBot4 워크스페이스 소스 완료"
else
    echo "[ros_source] WARNING: turtlebot4_ws 를 찾을 수 없습니다. (~/turtlebot4_ws)"
fi

# 3. rescue_bot 워크스페이스
source "${WORKSPACE_DIR}/install/setup.bash"

# 4. colcon 격리 설치 시 rescue_bot prefix가 AMENT_PREFIX_PATH에 자동으로 추가되지 않는
#    문제를 수동으로 보완합니다.
export AMENT_PREFIX_PATH="${WORKSPACE_DIR}/install/rescue_bot:${AMENT_PREFIX_PATH}"
export PYTHONPATH="${WORKSPACE_DIR}/install/rescue_bot/lib/python3.10/site-packages:${PYTHONPATH}"

echo "[ros_source] 환경 설정 완료!"
echo "[ros_source] AMENT_PREFIX_PATH = ${AMENT_PREFIX_PATH}"
echo ""
echo "[ros_source] 런치 명령어:"
echo "  ros2 launch rescue_bot rescue_real.launch.py"
echo "  ros2 launch rescue_bot rescue_sim.launch.py"
echo "  ros2 launch rescue_bot rescue_web.launch.py"
