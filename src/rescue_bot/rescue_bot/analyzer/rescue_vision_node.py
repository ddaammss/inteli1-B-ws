# v0.610
"""
SRD Pose Emergency ROS2 Node
===========================

이 파일은 실제 TurtleBot4 환경에서 사용하는 ROS2 노드 파일이다.
역할은 명확하다.

1) compressed image 토픽을 구독한다.
2) JPEG 이미지를 OpenCV 프레임으로 디코딩한다.
3) PoseEmergencyEngine을 호출한다.
4) 분석 결과(단일 emergency_level string)와 시각화 이미지(compressed)를 publish 한다.

즉, 이 파일은 ROS2 입출력 담당이고,
실제 포즈 분석 로직 자체는 srd_pose_emergency_core.py 에 있다.
"""
from typing import List

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

try:
    from .srd_pose_emergency_core import AnalyzerConfig, PoseEmergencyEngine
except ImportError:
    from .rescue_vision_core import AnalyzerConfig, PoseEmergencyEngine


class RescueVisionNode(Node):
    """실시간 카메라 영상을 받아 위급 상황 여부를 판단하는 ROS2 노드(Node) 메인 클래스.

    ROS2(Robot Operating System 2)의 노드는 하나의 독립적인 프로그램 단위입니다.
    이 노드는 카메라 이미지를 구독(Sub)하고, 내부 코어 엔진(PoseEmergencyEngine)으로 분석한 뒤,
    그 결과(위급 단계 문자열, 뼈대가 그려진 그림)를 다시 발행(Pub)하는 중계소 역할을 합니다.

    [구독(Subscribe) 토픽]
    - input_image_topic : 로봇 카메라에서 들어오는 압축된 원본 이미지 (CompressedImage)

    [발행(Publish) 토픽]
    - emergency_level_topic : NORMAL, CRITICAL 등 최종 판정된 하나의 텍스트 결과 (String)
    - image_result_topic : 관리자 대시보드에서 볼 수 있게 뼈대가 그려진 시각화 이미지 (CompressedImage)
    """

    def __init__(self):
        # 부모 클래스(Node)의 초기화 함수를 호출해 ROS 네트워크에 이 노드의 이름을 알립니다.
        super().__init__("srd_pose_emergency_node")

        # --------------------------------------------------------------
        # ROS2 파라미터(Parameter) 선언 및 가져오기
        # 파라미터는 노드를 실행할 때(launch 파일이나 터미널 명령어) 동적으로 바꿀 수 있는 설정값들입니다.
        # 코드 수정 없이도 토픽 이름이나 모델 경로를 바꿀 수 있어 유연합니다.
        # --------------------------------------------------------------
        # 1. 사용할 파라미터들의 이름과 '기본값(default)'을 미리 시스템에 등록합니다.
        self.declare_parameter("model_path", "yolo11n-pose.pt")
        self.declare_parameter("input_image_topic", "/camera/image_raw/compressed")
        self.declare_parameter("emergency_level_topic", "/robot6/emergency_level") # 최종 판정 문자열 발행
        self.declare_parameter("image_result_topic", "/robot6/image_result/compressed") # 뼈대 그려진 그림 발행
        self.declare_parameter("publish_annotated", True) # 뼈대 이미지를 실제로 보낼지 말지 On/Off 스위치
        self.declare_parameter("show_debug", True)        # 이미지 밑에 글씨(각도 등 디버깅 정보)를 띄울지 On/Off 스위치

        # 2. 등록된 구멍에서 실제 값들을 꺼내와서 파이썬 변수에 저장합니다.
        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        input_image_topic = self.get_parameter("input_image_topic").get_parameter_value().string_value
        emergency_level_topic = self.get_parameter("emergency_level_topic").get_parameter_value().string_value
        image_result_topic = self.get_parameter("image_result_topic").get_parameter_value().string_value
        
        # boolean(참/거짓) 스위치들은 콜백 함수 안에서 매번 확인해야 하므로 self 변수로 격상시켜 저장합니다.
        self.publish_annotated = self.get_parameter("publish_annotated").get_parameter_value().bool_value
        show_debug = self.get_parameter("show_debug").get_parameter_value().bool_value

        # --------------------------------------------------------------
        # 분석 코어 엔진 설정 및 생성 (다른 파일에 있는 로직 덩어리를 불러옵니다)
        # --------------------------------------------------------------
        # 엔진에 전달할 세팅 꾸러미(Config)를 만듭니다.
        cfg = AnalyzerConfig(model_path=model_path, show_debug=show_debug)
        # 세팅 꾸러미를 넣어서 메인 분석 뇌(Engine) 하나를 생성해둡니다.
        self.engine = PoseEmergencyEngine(cfg)

        # --------------------------------------------------------------
        # ROS2 퍼블리셔(발행자)와 서브스크라이버(구독자) 생성
        # --------------------------------------------------------------
        # 분석 결과 문자열을 바깥(ROS 네트워크)으로 뿌려줄 스피커(Publisher) 생성 (큐 사이즈 10)
        self.emergency_level_pub = self.create_publisher(String, emergency_level_topic, 10)
        
        # 뼈대가 예쁘게 그려진 이미지를 바깥으로 뿌려줄 스피커 생성
        self.image_result_pub = self.create_publisher(CompressedImage, image_result_topic, 10)

        # 원본 카메라 압축 이미지를 받을 귀(Subscriber) 생성
        # 이미지가 한 장 들어올 때마다 `self.image_callback` 이라는 지정된 함수가 자동으로 호출됩니다!
        self.image_sub = self.create_subscription(
            CompressedImage,
            input_image_topic,
            self.image_callback,
            10,
        )

        # 프로그램이 잘 켜졌다고 터미널 창에 초록색 글씨로 안내문(로그)을 하나 남겨줍니다.
        self.get_logger().info(
            f"SRD Pose Emergency Node started. input={input_image_topic}, emergency_level={emergency_level_topic}, annotated={image_result_topic}"
        )

    # ------------------------------------------------------------------
    # 이미지 디코딩 / 인코딩 보조 함수 (통신 포맷 <-> 인공지능 포맷 변환기)
    # ------------------------------------------------------------------
    @staticmethod
    def _decode_compressed_image(msg: CompressedImage) -> np.ndarray:
        """[ROS2 CompressedImage 메시지 -> OpenCV 배열(BGR)] 변환 함수
        
        와이파이 등으로 로봇 영상을 보낼 때는 데이터가 너무 커서 보통 JPEG로 압축해서 쏩니다.
        근데 AI 엔진(YOLO)은 압축된 파일 상태로는 그림을 볼 수 없기 때문에,
        이를 다시 숫자 배열(numpy array) 비트맵 형태로 쫙 펴주는 작업(디코딩)이 필요합니다.
        """
        # 1. 1차원 바이트 배열 찌꺼기를 numpy 단위 배열로 읽습니다.
        np_arr = np.frombuffer(msg.data, np.uint8)
        # 2. OpenCV를 이용해 메모리에 올려 진짜 컬러 이미지 그림 판(frame)으로 복원합니다.
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError("Failed to decode compressed image.")
        return frame

    @staticmethod
    def _encode_compressed_image(frame: np.ndarray, header_stamp, format_str: str = "jpeg") -> CompressedImage:
        """[OpenCV 배열(BGR) -> ROS2 CompressedImage 메시지] 변환 함수
        
        AI가 원본 그림 위에 뼈대를 다 그렸으면, 다시 와이파이로 관제 서버에 보내야 합니다.
        날것의 그림 데이터(frame)를 그대로 보내면 트래픽이 터지므로, 다시 JPEG로 압축(인코딩)해줍니다.
        """
        # 1. OpenCV 이미지 배열을 ".jpg" 포맷으로 압축합니다.
        ok, encoded = cv2.imencode(".jpg", frame)
        if not ok:
            raise ValueError("Failed to encode annotated image.")

        # 2. ROS2 통신 규격에 맞는 '빈 봉투(CompressedImage)'를 하나 만듭니다.
        msg = CompressedImage()
        # 3. 우표(시간표)를 붙입니다. (언제 찍힌 사진인지 기록)
        msg.header.stamp = header_stamp
        msg.format = format_str
        # 4. 봉투 안에 압축한 진짜 그림 데이터를 바이트 형태로 바꿔 담습니다.
        msg.data = encoded.tobytes()
        return msg

    # ------------------------------------------------------------------
    # 메인 콜백 함수 (카메라에서 새 사진이 도착할 때마다 무한 반복 실행됨)
    # ------------------------------------------------------------------
    def image_callback(self, msg: CompressedImage) -> None:
        """ROS2 시스템이 카메라 압축 영상을 받을 때마다 자동으로 호출해 주는 함수입니다.
        이 프로그램의 '심장이 뛰는 곳'입니다.
        """
        try:
            # 1) 통신용 압축 파일(msg)을 AI가 볼 수 있는 그림판(frame)으로 변환합니다.
            frame = self._decode_compressed_image(msg)

            # 2) "분석 코어 엔진아, 이 그림 받아서 뼈대 그리고 위급 상황인지 알려줘!" 하고 지시합니다.
            # 엔진은 뼈대 그려진 사진(annotated)과, 최종 위급 단계 글자(emergency_level)를 줍니다.
            annotated, emergency_level = self.engine.analyze_frame_with_emergency_level(frame)

            # 3) 엔진 결과 배포 (퍼블리시)
            # 만약 화면에 사람이 단 한 명이라도 있어서 위급수준 결과(emergency_level)가 나왔다면:
            if emergency_level is not None:
                # ROS2 문자열 빈 봉투(String)를 만들고
                em_msg = String()
                # 글자를 적어서 넣은 뒤
                em_msg.data = emergency_level
                # 바깥세상(ROS 네트워크)으로 외칩니다! "현재 상태: CRITICAL!!"
                self.emergency_level_pub.publish(em_msg)

            # 4) 시각화 이미지 배포 (퍼블리시)
            # 파라미터에서 뼈대 그림을 만들라고 켜놨다면(publish_annotated == True):
            if self.publish_annotated:
                # 뼈대가 그려진 그림(annotated)을 다시 통신용 압축 파일로 포장합니다.
                annotated_msg = self._encode_compressed_image(annotated, msg.header.stamp)
                # 바깥세상으로 사진 통째로 쏴줍니다. (RQT 같은 모니터링 툴에서 볼 수 있음)
                self.image_result_pub.publish(annotated_msg)

        # 프로그램이 돌다가 뭔가 에러가 나면 멈춰버리지 않고 빨간 글씨(Error)로 에러 내용만 띄웁니다.
        except Exception as exc:
            self.get_logger().error(f"image_callback failed: {exc}")


def main(args=None):
    """터미널에서 이 파이썬 파일을 직접 실행할 때 제일 먼저 불리는 시작점입니다."""
    # 1. ROS2 시스템 통신망에 접속(초기화)합니다.
    rclpy.init(args=args)
    
    # 2. 우리가 만든 노드(로봇 지능 단위)를 하나 메모리에 찍어냅니다.
    node = RescueVisionNode()
    try:
        # 3. 노드가 절대 꺼지지 않고 무한루프를 돌며 계속 귀를 열고 일하게 만듭니다. (Spin)
        rclpy.spin(node)
    finally:
        # 4. 사용자가 Ctrl+C를 눌러서 프로그램을 끄면 
        # 점잖게 노드를 메모리에서 부수고, ROS2 접속을 끊고 종료합니다.
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
