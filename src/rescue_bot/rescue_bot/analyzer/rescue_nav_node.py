"""
rescue_nav_node.py
==================
TurtleBot B - 요구조자 위치 토픽 수신 → 큐 순서대로 순차 주행 → 완료/큐 빔 시 도킹

동작 흐름:
  1. /rescue/victim_pose (PointStamped) 또는
     /rescue/victim_pose_stamped (PoseStamped) 으로 요구조자 위치 수신
     → deque(goal_queue) 에 순서대로 적재
  2. 목표 큐 실행 워커 스레드가 큐에서 하나씩 꺼내 순차 주행
  3. 목표 도달(성공/실패 무관) 후 큐가 비어 있으면 → 도킹
  4. 도킹 중 새 목표가 들어오면 → 언도킹 → 주행 재개
  5. Ctrl+C : 현재 task 취소 → 도킹 → 종료

토픽 상수 (상단에서 수정):
  VICTIM_POINT_TOPIC  : /rescue/victim_pose         (PointStamped)
  VICTIM_POSE_TOPIC   : /rescue/victim_pose_stamped (PoseStamped)
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.action import ActionClient

from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from std_msgs.msg import Bool

from tf2_ros import Buffer, TransformListener

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions
from irobot_create_msgs.action import Dock, Undock

import subprocess
from collections import deque
import threading
import math
import time


# ════════════════════════════════════════════════
#  설정 상수 (환경에 맞게 수정)
# ════════════════════════════════════════════════
VICTIM_POINT_TOPIC  = '/rescue/victim_pose'           # PointStamped
VICTIM_POSE_TOPIC   = '/rescue/victim_pose_stamped'   # PoseStamped
NAV_GOAL_YAW        = 0.0                             # 목표 도착 방향 (rad)
TF_STABLE_WAIT_SEC  = 2.0                             # TF 안정화 대기 시간 (초)
QUEUE_WAIT_SEC      = 0.3                             # 큐 폴링 주기 (초)
ARRIVED_TOPIC       = '/robot6/mission/arrived'       # Control 노드 전달용 도착 신호



# ════════════════════════════════════════════════
#  헬퍼: PoseStamped 생성
# ════════════════════════════════════════════════
def make_goal_pose(node: Node, x: float, y: float, yaw: float = NAV_GOAL_YAW) -> PoseStamped:
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp    = node.get_clock().now().to_msg()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0.0
    goal.pose.orientation = Quaternion(
        x=0.0, y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0)
    )
    return goal


# ════════════════════════════════════════════════
#  메인 노드
# ════════════════════════════════════════════════
class RescueNavNode(Node):
    """
    목표 큐 기반 순차 구조 주행 노드

    ┌─────────────────────────────────────────────────┐
    │  토픽 수신 → goal_queue (deque, FIFO)           │
    │                                                 │
    │  [GoalWorker 스레드]                            │
    │    큐에서 목표 꺼냄                             │
    │      있음 → 언도킹(필요시) → goToPose → 완료  │
    │      없음 → 도킹 → queue_event 대기             │
    │    새 목표 수신 → queue_event.set() → 재개      │
    └─────────────────────────────────────────────────┘
    """

    # ──────────────────────────────────────────────
    #  초기화
    # ──────────────────────────────────────────────
    def __init__(self):
        super().__init__('rescue_nav_node')

        # ── 상태 변수 ────────────────────────────────
        self.goal_queue  : deque          = deque()
        self.queue_lock  : threading.Lock = threading.Lock()
        self.queue_event : threading.Event = threading.Event()  # 새 목표 도착 신호

        self.is_docked    = True    # 현재 도킹 상태
        self.nav_active   = False   # 주행 중 여부
        self.tf_ready     = False   # TF 준비 여부
        self._shutdown    = False   # 종료 플래그
        self.navigator    = None    # init_navigator() 에서 초기화

        self.current_goal : tuple = None  # 현재 주행 중인 목표 (map_x, map_y)

        # ── TF ──────────────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── 도킹/언도킹 액션 클라이언트 ──────────────
        self._dock_client   = ActionClient(self, Dock,   '/robot6/dock')
        self._undock_client = ActionClient(self, Undock, '/robot6/undock')

        # ── 구독 ─────────────────────────────────────
        self.create_subscription(
            PointStamped, VICTIM_POINT_TOPIC, self.victim_point_callback, 10)
        self.create_subscription(
            PoseStamped, VICTIM_POSE_TOPIC, self.victim_pose_callback, 10)

        # ── 발행(Publish) ───────────────────────────
        self.arrived_pub = self.create_publisher(Bool, ARRIVED_TOPIC, 10)

        # ── TF 안정화 타이머 ─────────────────────────
        self.get_logger().info(
            f'[Init] TF 안정화 대기 중... ({TF_STABLE_WAIT_SEC}초 후 시작)')
        self._start_timer = self.create_timer(TF_STABLE_WAIT_SEC, self._on_tf_ready)

        # ── 큐 워커 스레드 ────────────────────────────
        self._worker = threading.Thread(
            target=self._goal_worker, daemon=True, name='GoalWorker')
        self._worker.start()

        # ── 주기 상태 로그 ────────────────────────────
        self.create_timer(3.0, self._status_log)

        self.get_logger().info('[Init] 노드 초기화 완료. Navigator 준비 대기 중...')

    # ──────────────────────────────────────────────
    #  Navigator 초기화 (executor.spin() 이후 호출)
    # ──────────────────────────────────────────────
    def init_navigator(self):
        self.get_logger().info('[Init] Navigator 초기화 시작...')
        self.navigator = TurtleBot4Navigator(namespace='/robot6')

        # 실제 도킹 상태 반영 (강제 도킹 없이 현재 상태 그대로 사용)
        self.is_docked = self.navigator.getDockedStatus()
        self.get_logger().info(f'[Init] 현재 도킹 상태: {self.is_docked}')

        self.get_logger().info('[Init] Nav2 활성화 완료. 도킹 대기 상태.')

    # ──────────────────────────────────────────────
    #  TF 준비 완료
    # ──────────────────────────────────────────────
    def _on_tf_ready(self):
        self.tf_ready = True
        self.get_logger().info('[TF] TF 안정화 완료. 요구조자 토픽 대기 중...')
        self._start_timer.cancel()

    # ──────────────────────────────────────────────
    #  요구조자 위치 수신 콜백
    # ──────────────────────────────────────────────
    def victim_point_callback(self, msg: PointStamped):
        self.get_logger().info(
            f'[Victim] PointStamped 수신 → frame={msg.header.frame_id}, '
            f'x={msg.point.x:.3f}, y={msg.point.y:.3f}')
        self._enqueue_victim(msg.point.x, msg.point.y, msg.header.frame_id)

    def victim_pose_callback(self, msg: PoseStamped):
        self.get_logger().info(
            f'[Victim] PoseStamped 수신 → frame={msg.header.frame_id}, '
            f'x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}')
        self._enqueue_victim(
            msg.pose.position.x, msg.pose.position.y, msg.header.frame_id)

    # ──────────────────────────────────────────────
    #  큐 적재
    # ──────────────────────────────────────────────
    def _enqueue_victim(self, x: float, y: float, frame: str):
        """수신 좌표를 map 프레임으로 변환 후 goal_queue 에 FIFO 적재"""
        if not self.tf_ready:
            self.get_logger().warn('[Queue] TF 미준비. 해당 위치 무시.')
            return

        # map 프레임 아닐 경우 TF 변환
        map_x, map_y = x, y
        if frame != 'map':
            try:
                pt = PointStamped()
                pt.header.frame_id = frame
                pt.header.stamp    = self.get_clock().now().to_msg()
                pt.point.x, pt.point.y, pt.point.z = x, y, 0.0
                pt_map = self.tf_buffer.transform(
                    pt, 'map', timeout=Duration(seconds=1.0))
                map_x, map_y = pt_map.point.x, pt_map.point.y
                self.get_logger().info(
                    f'[TF] {frame} → map 변환: ({map_x:.3f}, {map_y:.3f})')
            except Exception as e:
                self.get_logger().error(f'[TF] 변환 실패: {e}')
                return

        with self.queue_lock:
            self.goal_queue.append((map_x, map_y))
            q_len = len(self.goal_queue)

        self.get_logger().info(
            f'[Queue] ✚ 목표 추가 → map({map_x:.3f}, {map_y:.3f}) | '
            f'큐 길이: {q_len}')

        # 워커에 새 목표 도착 알림 (도킹 대기 중이면 깨움)
        self.queue_event.set()

    # ══════════════════════════════════════════════
    #  목표 큐 워커 스레드 (핵심 로직)
    # ══════════════════════════════════════════════
    def _goal_worker(self):
        """
        백그라운드 스레드 메인 루프

        [루프]
          ① 큐에서 목표 꺼내기
          ② 목표 있음  → 언도킹(필요 시) → goToPose → 완료 폴링
          ③ 목표 없음  → 도킹 → queue_event 대기 (새 목표 올 때까지 블로킹)
        """
        self.get_logger().info('[Worker] 목표 워커 시작.')

        # navigator 준비될 때까지 대기
        while not self._shutdown and self.navigator is None:
            time.sleep(0.5)
        if self._shutdown:
            return
        self.get_logger().info('[Worker] Navigator 준비 완료. 목표 대기 시작.')

        while not self._shutdown and rclpy.ok():

            # ─ ① 큐에서 다음 목표 꺼내기 ─────────────
            goal = self._pop_next_goal()

            # ─ ③ 큐 비어있음 ─────────────────────────
            if goal is None:
                self._do_dock()
                self.get_logger().info('[Worker] 큐 비어있음 → 새 목표 대기...')
                self.queue_event.wait()   # 새 목표 수신 시 _enqueue_victim 에서 set()
                self.queue_event.clear()
                continue                  # 루프 처음으로 돌아가 큐 재확인

            # ─ ② 목표 실행 ───────────────────────────
            map_x, map_y = goal

            # 도킹 상태이면 먼저 언도킹
            self._do_undock()

            # Nav2 goal 전송
            self.nav_active   = True
            self.current_goal = (map_x, map_y)
            goal_pose = make_goal_pose(self, map_x, map_y)

            self.get_logger().info(
                f'[Worker] ▶ 주행 시작 → map({map_x:.3f}, {map_y:.3f})')
            self.navigator.goToPose(goal_pose)

            # 완료 폴링 (블로킹) ─────────────────────
            while rclpy.ok() and not self._shutdown:
                if self.navigator.isTaskComplete():
                    break
                time.sleep(QUEUE_WAIT_SEC)

            # 결과 확인
            try:
                result = self.navigator.getResult()
                self.get_logger().info(
                    f'[Worker] ✔ 목표 완료 → map({map_x:.3f}, {map_y:.3f}) '
                    f'| 결과: {result}')
            except Exception as e:
                self.get_logger().warn(f'[Worker] 결과 조회 실패: {e}')

            # ─ Control 노드에 도착 신호 전송 ────────
            self.get_logger().info(f'[Worker] 📢 오케스트레이터(Control)에 도착 신호(True) 발송')
            msg = Bool()
            msg.data = True
            self.arrived_pub.publish(msg)

            self.nav_active   = False
            self.current_goal = None


            # 다음 루프 : 큐 확인 → 있으면 바로 실행, 없으면 도킹

        self.get_logger().info('[Worker] 워커 종료.')

    # ──────────────────────────────────────────────
    #  큐에서 다음 목표 꺼내기
    # ──────────────────────────────────────────────
    def _pop_next_goal(self):
        with self.queue_lock:
            if self.goal_queue:
                return self.goal_queue.popleft()
        return None

    # ──────────────────────────────────────────────
    #  도킹 / 언도킹 (ActionClient 방식)
    # ──────────────────────────────────────────────
    def _do_dock(self):
        """이미 도킹 상태이면 skip"""
        if self.is_docked:
            return
        self.get_logger().info('[Dock] 큐 완료 → 도킹 시작...')
        
        if not self._dock_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('[Dock] 도킹 서버를 찾을 수 없습니다!')
            return
            
        goal_msg = Dock.Goal()
        future = self._dock_client.send_goal_async(goal_msg)
        
        # [수정된 부분] spin 대신 future가 완료될 때까지 블로킹 루프
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
            
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('[Dock] 도킹 명령 거부됨.')
            return
            
        result_future = goal_handle.get_result_async()
        
        # [수정된 부분] 마찬가지로 결과가 나올 때까지 대기
        while rclpy.ok() and not result_future.done():
            time.sleep(0.1)
            
        self.is_docked = True
        self.get_logger().info('[Dock] ■ 도킹 완료. 대기 상태.')
    def _do_undock(self):
        """이미 언도킹 상태이면 skip"""
        if not self.is_docked:
            return
            
        self.get_logger().info('[Dock] 새 목표 수신 → 언도킹 시작...')
        
        if not self._undock_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('[Dock] 언도킹 서버를 찾을 수 없습니다!')
            return
            
        goal_msg = Undock.Goal()
        future = self._undock_client.send_goal_async(goal_msg)
        
        # [수정된 부분] spin_until_future_complete 대신 while 루프로 대기
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
            
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('[Dock] 언도킹 명령 거부됨.')
            return
            
        result_future = goal_handle.get_result_async()
        
        # [수정된 부분] 결과가 나올 때까지 대기
        while rclpy.ok() and not result_future.done():
            time.sleep(0.1)
            
        self.is_docked = False
        self.get_logger().info('[Dock] ▲ 언도킹 완료.')

    # ──────────────────────────────────────────────
    #  주기 상태 로그
    # ──────────────────────────────────────────────
    def _status_log(self):
        with self.queue_lock:
            q_len = len(self.goal_queue)
        self.get_logger().info(
            f'[Status] 도킹={self.is_docked} | '
            f'주행중={self.nav_active} | 현재목표={self.current_goal} | '
            f'대기 목표={q_len}개')

    # ──────────────────────────────────────────────
    #  종료 처리
    # ──────────────────────────────────────────────
    def shutdown(self):
        self.get_logger().info('[Shutdown] 종료 시작...')
        self._shutdown = True
        self.queue_event.set()  # 대기 중인 워커 깨우기
        try:
            self.navigator.cancelTask()
        except Exception:
            pass
        try:
            if not self.is_docked:
                self.get_logger().info('[Shutdown] 도킹 복귀 중...')
                self._do_dock()
        except Exception:
            pass
        self.get_logger().info('[Shutdown] 완료.')


# ════════════════════════════════════════════════
#  main
# ════════════════════════════════════════════════
def main():
    rclpy.init()
    node = RescueNavNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # executor 가 돌기 시작한 뒤 navigator 초기화 (블로킹 방지)
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


# ════════════════════════════════════════════════
#  사용 방법 (README)
# ════════════════════════════════════════════════
#
#  [구조]
#    TurtleBot A → /rescue/victim_pose 퍼블리시 (요구조자 발견 시)
#    TurtleBot B → 이 노드 실행
#
#  [동작 시나리오]
#    1. 최초 기동       → 도킹 상태로 대기
#    2. 목표 1 수신     → 언도킹 → 목표1 주행 시작
#    3. 주행 중 목표 2,3 수신 → 큐에 순서대로 적재 (현재 주행 계속)
#    4. 목표1 완료      → 큐에서 목표2 꺼냄 → 목표2 주행
#    5. 목표2 완료      → 큐에서 목표3 꺼냄 → 목표3 주행
#    6. 목표3 완료      → 큐 비어있음 → 도킹
#    7. 새 목표 수신    → 언도킹 → 주행 재개 (2번부터 반복)
#
#  [실행]
#    ros2 run <your_pkg> rescue_nav_node
#
#  [테스트용 수동 퍼블리시]
#    ros2 topic pub --once /rescue/victim_pose geometry_msgs/PointStamped \
#      "{header: {frame_id: 'map'}, point: {x: 1.5, y: 0.5, z: 0.0}}"
#
#    ros2 topic pub --once /rescue/victim_pose geometry_msgs/PointStamped \
#      "{header: {frame_id: 'map'}, point: {x: 3.0, y: -1.0, z: 0.0}}"
