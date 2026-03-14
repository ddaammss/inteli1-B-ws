# v0.300
# file: rescue_nav_node.py
# date: 2026-03-13
# changes:
# - v0.300: 예시 코드 스타일에 맞춰 navigator API 중심으로 리팩토링.
#           Dock/Undock ActionClient 직접 구현 제거 후 navigator.dock()/undock() 사용.
#           PoseStamped 수동 생성 축소 후 navigator.getPoseStamped() 우선 사용.
#           dock_status 구독 제거 후 navigator.getDockedStatus() 기반 상태 조회로 단순화.
#           기존 기능(1m standoff, pre-dock 후 dock, 성공 시 arrived 발행)은 유지.
# - v0.200: initial pose 설정 방식을 /robot6/initialpose 직접 퍼블리시에서 navigator.setInitialPose() 기반으로 변경.
#           victim 목표 주행 시 1.0 m standoff goal 계산 추가.
#           큐가 비면 pre-dock 위치로 먼저 이동한 뒤 dock action 수행하도록 변경.
#           Nav 성공 시에만 arrived 발행하도록 수정.
# - v0.100: initial pose 직접 퍼블리시 추가, pre-dock -> dock 시퀀스 초안 추가.

"""
rescue_nav_node.py
==================
TurtleBot B - 요구조자 위치 토픽 수신 → 큐 순차 주행 → 미션 완료 대기 → pre-dock → dock

동작 흐름:
  1. /rescue/victim_pose(PointStamped) 또는 /rescue/victim_pose_stamped(PoseStamped) 수신
  2. map 프레임으로 변환 후 goal_queue(FIFO)에 적재
  3. 워커 스레드가 하나씩 꺼내 robot6 현재 위치 기준 1.0m standoff goal 계산
  4. Nav 성공 시에만 /robot6/mission/arrived 발행
  5. /robot6/tts/done(True) 수신 시 다음 목표 진행
  6. 큐가 비면 pre-dock 위치로 이동 후 dock action 수행

주의:
  - INITIAL_POSE_*, PRE_DOCK_*, DOCK_REF_* 는 실맵 기준으로 반드시 수정해야 합니다.
  - pre-dock 실패 시 direct dock action은 시도하지 않습니다. 정렬 실패 상태에서 바로 dock 하면 실기기에서 더 위험할 수 있습니다.
"""

import math
import threading
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped, Quaternion
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator


# ════════════════════════════════════════════════
# 설정 상수
# ════════════════════════════════════════════════
VICTIM_POINT_TOPIC = '/rescue/victim_pose'
VICTIM_POSE_TOPIC = '/rescue/victim_pose_stamped'
AMCL_POSE_TOPIC = '/robot6/amcl_pose'
ARRIVED_TOPIC = '/robot6/mission/arrived'
TTS_DONE_TOPIC = '/robot6/tts/done'

TF_STABLE_WAIT_SEC = 2.0
QUEUE_WAIT_SEC = 0.3
MISSION_WAIT_TIMEOUT = 120.0
STANDOFF_DISTANCE_M = 1.0

# localization 초기 위치 (실맵 기준으로 수정 필요)
INITIAL_POSE_X = 0.0
INITIAL_POSE_Y = 0.0
INITIAL_POSE_YAW = 0.0

# 큐가 비었을 때 이동할 pre-dock 위치와, 바라볼 dock 기준점 (실맵 기준으로 수정 필요)
PRE_DOCK_X = 0.40
PRE_DOCK_Y = 0.00
DOCK_REF_X = 0.00
DOCK_REF_Y = 0.00


# ════════════════════════════════════════════════
# 헬퍼
# ════════════════════════════════════════════════
def yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )


def quaternion_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def make_pose_stamped(node: Node, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation = yaw_to_quaternion(yaw)
    return pose


# ════════════════════════════════════════════════
# 메인 노드
# ════════════════════════════════════════════════
class RescueNavNode(Node):
    def __init__(self):
        super().__init__('rescue_nav_node')

        self.goal_queue: deque = deque()
        self.queue_lock = threading.Lock()
        self.queue_event = threading.Event()
        self.mission_event = threading.Event()

        self.nav_active = False
        self.tf_ready = False
        self._shutdown = False
        self.navigator = None

        self.current_goal = None
        self.robot_pose = None  # (x, y, yaw)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(PointStamped, VICTIM_POINT_TOPIC, self.victim_point_callback, 10)
        self.create_subscription(PoseStamped, VICTIM_POSE_TOPIC, self.victim_pose_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, AMCL_POSE_TOPIC, self.amcl_pose_callback, 10)
        self.create_subscription(Bool, TTS_DONE_TOPIC, self.mission_finished_callback, 10)

        self.arrived_pub = self.create_publisher(Bool, ARRIVED_TOPIC, 10)

        self.get_logger().info(f'[Init] TF 안정화 대기 중... ({TF_STABLE_WAIT_SEC}초 후 시작)')
        self._start_timer = self.create_timer(TF_STABLE_WAIT_SEC, self._on_tf_ready)

        self._worker = threading.Thread(target=self._goal_worker, daemon=True, name='GoalWorker')
        self._worker.start()

        self.create_timer(3.0, self._status_log)
        self.get_logger().info('[Init] 노드 초기화 완료. Navigator 준비 대기 중...')

    # ──────────────────────────────────────────────
    # Navigator 초기화
    # ──────────────────────────────────────────────
    def init_navigator(self):
        self.get_logger().info('[Init] Navigator 초기화 시작...')
        self.navigator = TurtleBot4Navigator(namespace='/robot6')

        initial_pose = self._build_nav_pose(INITIAL_POSE_X, INITIAL_POSE_Y, INITIAL_POSE_YAW)
        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info(
            f'[Init] setInitialPose 적용 → x={INITIAL_POSE_X:.3f}, y={INITIAL_POSE_Y:.3f}, yaw={INITIAL_POSE_YAW:.3f}'
        )

        self.navigator.waitUntilNav2Active()
        self.get_logger().info(f'[Init] Nav2 활성화 완료. 현재 도킹 상태: {self._is_docked()}')

    def _build_nav_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        if self.navigator is not None:
            try:
                return self.navigator.getPoseStamped([x, y], yaw)
            except Exception:
                pass
        return make_pose_stamped(self, x, y, yaw)

    def _is_docked(self) -> bool:
        if self.navigator is None:
            return True
        try:
            return bool(self.navigator.getDockedStatus())
        except Exception:
            return True

    # ──────────────────────────────────────────────
    # 초기화 / 상태 콜백
    # ──────────────────────────────────────────────
    def _on_tf_ready(self):
        self.tf_ready = True
        self.get_logger().info('[TF] TF 안정화 완료. 요구조자 토픽 대기 중...')
        self._start_timer.cancel()

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.robot_pose = (x, y, yaw)

    def mission_finished_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info('[Mission] 미션 완료 신호 수신됨.')
            self.mission_event.set()

    # ──────────────────────────────────────────────
    # 요구조자 위치 수신
    # ──────────────────────────────────────────────
    def victim_point_callback(self, msg: PointStamped):
        self.get_logger().info(
            f'[Victim] PointStamped 수신 → frame={msg.header.frame_id}, x={msg.point.x:.3f}, y={msg.point.y:.3f}'
        )
        self._enqueue_victim(msg.point.x, msg.point.y, msg.header.frame_id)

    def victim_pose_callback(self, msg: PoseStamped):
        self.get_logger().info(
            f'[Victim] PoseStamped 수신 → frame={msg.header.frame_id}, x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}'
        )
        self._enqueue_victim(msg.pose.position.x, msg.pose.position.y, msg.header.frame_id)

    def _enqueue_victim(self, x: float, y: float, frame: str):
        if not self.tf_ready:
            self.get_logger().warn('[Queue] TF 미준비. 해당 위치 무시.')
            return

        map_x, map_y = x, y
        if frame != 'map':
            try:
                pt = PointStamped()
                pt.header.frame_id = frame
                pt.header.stamp = self.get_clock().now().to_msg()
                pt.point.x, pt.point.y, pt.point.z = x, y, 0.0
                pt_map = self.tf_buffer.transform(pt, 'map', timeout=Duration(seconds=1.0))
                map_x, map_y = pt_map.point.x, pt_map.point.y
                self.get_logger().info(f'[TF] {frame} → map 변환: ({map_x:.3f}, {map_y:.3f})')
            except Exception as e:
                self.get_logger().error(f'[TF] 변환 실패: {e}')
                return

        with self.queue_lock:
            self.goal_queue.append((map_x, map_y))
            q_len = len(self.goal_queue)

        self.get_logger().info(
            f'[Queue] ✚ 목표 추가 → victim map({map_x:.3f}, {map_y:.3f}) | 큐 길이: {q_len}'
        )
        self.queue_event.set()

    # ──────────────────────────────────────────────
    # 목표 워커
    # ──────────────────────────────────────────────
    def _goal_worker(self):
        self.get_logger().info('[Worker] 목표 워커 시작.')

        while not self._shutdown and self.navigator is None:
            time.sleep(0.5)
        if self._shutdown:
            return

        self.get_logger().info('[Worker] Navigator 준비 완료. 목표 대기 시작.')

        while not self._shutdown and rclpy.ok():
            goal = self._pop_next_goal()

            if goal is None:
                self._go_predock_and_dock()
                self.get_logger().info('[Worker] 큐 비어있음 → 새 목표 대기...')
                self.queue_event.wait()
                self.queue_event.clear()
                continue

            victim_x, victim_y = goal
            self._do_undock()

            standoff_pose = self._compute_standoff_pose(victim_x, victim_y)
            self.current_goal = (standoff_pose.pose.position.x, standoff_pose.pose.position.y)
            self.nav_active = True

            self.get_logger().info(
                f'[Worker] ▶ victim=({victim_x:.3f}, {victim_y:.3f}) → '
                f'standoff=({self.current_goal[0]:.3f}, {self.current_goal[1]:.3f})'
            )

            success, result = self._go_to_pose_blocking(standoff_pose)

            if success:
                self._publish_arrived(True)
                self._wait_for_mission_completion()
            else:
                self.get_logger().warn(f'[Worker] 목표 실패. arrived 미발행. result={result}')

            self.nav_active = False
            self.current_goal = None

        self.get_logger().info('[Worker] 워커 종료.')

    def _pop_next_goal(self):
        with self.queue_lock:
            if self.goal_queue:
                return self.goal_queue.popleft()
        return None

    # ──────────────────────────────────────────────
    # 주행 목표 계산
    # ──────────────────────────────────────────────
    def _compute_standoff_pose(self, victim_x: float, victim_y: float) -> PoseStamped:
        if self.robot_pose is not None:
            robot_x, robot_y, _ = self.robot_pose
        else:
            robot_x, robot_y = INITIAL_POSE_X, INITIAL_POSE_Y
            self.get_logger().warn('[Goal] amcl_pose 미수신 상태. initial pose 기준으로 standoff 방향 계산.')

        dx = victim_x - robot_x
        dy = victim_y - robot_y
        distance = math.hypot(dx, dy)

        if distance < 1e-6:
            yaw = 0.0
            goal_x = victim_x - STANDOFF_DISTANCE_M
            goal_y = victim_y
        else:
            yaw = math.atan2(dy, dx)
            if distance <= STANDOFF_DISTANCE_M:
                goal_x = robot_x
                goal_y = robot_y
                self.get_logger().warn('[Goal] 요구조자와 이미 1m 이내. 현재 위치를 goal로 사용.')
            else:
                goal_x = victim_x - STANDOFF_DISTANCE_M * math.cos(yaw)
                goal_y = victim_y - STANDOFF_DISTANCE_M * math.sin(yaw)

        return self._build_nav_pose(goal_x, goal_y, yaw)

    def _compute_predock_pose(self) -> PoseStamped:
        yaw = math.atan2(DOCK_REF_Y - PRE_DOCK_Y, DOCK_REF_X - PRE_DOCK_X)
        return self._build_nav_pose(PRE_DOCK_X, PRE_DOCK_Y, yaw)

    # ──────────────────────────────────────────────
    # 주행 / 미션 완료 대기
    # ──────────────────────────────────────────────
    def _go_to_pose_blocking(self, pose: PoseStamped):
        try:
            self.navigator.goToPose(pose)
            while rclpy.ok() and not self._shutdown:
                if self.navigator.isTaskComplete():
                    break
                time.sleep(QUEUE_WAIT_SEC)

            result = self.navigator.getResult()
            self.get_logger().info(
                f'[Nav] 완료 → x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}, result={result}'
            )
            return self._is_nav_success(result), result
        except Exception as e:
            self.get_logger().error(f'[Nav] goToPose 실패: {e}')
            return False, str(e)

    def _is_nav_success(self, result) -> bool:
        if result is None:
            return False

        name = getattr(result, 'name', None)
        if isinstance(name, str):
            return name.upper() in ('SUCCEEDED', 'SUCCESS')

        text = str(result).lower()
        if 'succeed' in text or 'success' in text:
            return True
        if 'fail' in text or 'cancel' in text or 'unknown' in text:
            return False

        value = getattr(result, 'value', None)
        if isinstance(value, int):
            return value == 0

        return False

    def _publish_arrived(self, arrived: bool):
        msg = Bool()
        msg.data = arrived
        self.arrived_pub.publish(msg)
        self.get_logger().info(f'[Worker] 📢 arrived={arrived} 발행')

    def _wait_for_mission_completion(self):
        self.get_logger().info(f'[Worker] ⏳ 구조 미션 대기 중... ({TTS_DONE_TOPIC})')
        self.mission_event.clear()
        finished = self.mission_event.wait(timeout=MISSION_WAIT_TIMEOUT)

        if finished:
            self.get_logger().info('[Worker] ✅ TTS/구조 완료 신호 수신. 다음 동작으로 이동.')
        else:
            self.get_logger().warn(f'[Worker] ⚠️ 미션 대기 시간 초과({MISSION_WAIT_TIMEOUT}초). 강제 진행.')

    def _go_predock_and_dock(self):
        if self._is_docked():
            return

        predock_pose = self._compute_predock_pose()
        self.get_logger().info(
            f'[Dock] pre-dock 이동 시작 → x={predock_pose.pose.position.x:.3f}, '
            f'y={predock_pose.pose.position.y:.3f}'
        )

        success, result = self._go_to_pose_blocking(predock_pose)
        if not success:
            self.get_logger().error(f'[Dock] pre-dock 이동 실패. direct dock 생략. result={result}')
            return

        self.get_logger().info('[Dock] pre-dock 정렬 완료. dock 실행.')
        self._do_dock()

    # ──────────────────────────────────────────────
    # 도킹 / 언도킹
    # ──────────────────────────────────────────────
    def _do_dock(self):
        if self._is_docked():
            return

        try:
            self.get_logger().info('[Dock] 도킹 시작...')
            self.navigator.dock()
            self.get_logger().info(f'[Dock] 도킹 호출 완료. 현재 상태={self._is_docked()}')
        except Exception as e:
            self.get_logger().error(f'[Dock] 도킹 실패: {e}')

    def _do_undock(self):
        if not self._is_docked():
            return

        try:
            self.get_logger().info('[Dock] 새 목표 수신 → 언도킹 시작...')
            self.navigator.undock()
            self.get_logger().info(f'[Dock] 언도킹 호출 완료. 현재 상태={self._is_docked()}')
        except Exception as e:
            self.get_logger().error(f'[Dock] 언도킹 실패: {e}')

    # ──────────────────────────────────────────────
    # 로그 / 종료
    # ──────────────────────────────────────────────
    def _status_log(self):
        with self.queue_lock:
            q_len = len(self.goal_queue)
        self.get_logger().info(
            f'[Status] 도킹={self._is_docked()} | 주행중={self.nav_active} | '
            f'현재목표={self.current_goal} | 대기목표={q_len}개 | robot_pose={self.robot_pose}'
        )

    def shutdown(self):
        self.get_logger().info('[Shutdown] 종료 시작...')
        self._shutdown = True
        self.queue_event.set()

        try:
            if self.navigator is not None:
                self.navigator.cancelTask()
        except Exception:
            pass

        try:
            if self.navigator is not None and not self._is_docked():
                self.get_logger().info('[Shutdown] pre-dock 후 도킹 복귀 시도...')
                self._go_predock_and_dock()
        except Exception:
            pass

        self.get_logger().info('[Shutdown] 완료.')


# ════════════════════════════════════════════════
# main
# ════════════════════════════════════════════════
def main():
    rclpy.init()
    node = RescueNavNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    init_thread = threading.Thread(target=node.init_navigator, daemon=True)
    init_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C 감지 → 종료')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
