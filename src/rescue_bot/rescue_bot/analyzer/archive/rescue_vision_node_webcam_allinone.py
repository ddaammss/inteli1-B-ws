# v0.620 All-in-One Webcam ROS2 Node
"""
Rescue Bot Vision Node - Direct Webcam Version (v0.620)
=====================================================
이 파일은 별도의 publisher(터틀봇 등) 없이, 
노드 자체에서 "웹캠"을 구동하고 즉시 분석(v0.620)까지 수행하는 All-in-One 노드이다.

실행 방법:
python3 rescue_vision_node_webcam_allinone.py
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import time

# 동일 디렉토리에 있는 PoseEmergencyEngine을 사용하거나, 
# 만약 경로가 꼬일 경우를 대비해 본 파일에 Engine 클래스를 포함하거나 import 한다.
# 여기서는 사용자 편의를 위해 v0.620 로직이 담긴 rescue_vision_core를 import 한다.
try:
    from rescue_vision_core import AnalyzerConfig, PoseEmergencyEngine
except ImportError:
    # 패키지 구조에서 실행될 경우를 대비
    from .rescue_vision_core import AnalyzerConfig, PoseEmergencyEngine

class RescueVisionWebcamNode(Node):
    def __init__(self):
        super().__init__('rescue_vision_webcam_node')
        
        # 파라미터 선언
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("emergency_level_topic", "/robot6/srd/emergency_level")
        self.declare_parameter("image_result_topic", "/robot6/srd/image_result/compressed")
        
        cam_idx = self.get_parameter("camera_index").get_parameter_value().integer_value
        em_topic = self.get_parameter("emergency_level_topic").get_parameter_value().string_value
        img_topic = self.get_parameter("image_result_topic").get_parameter_value().string_value
        
        # 엔진 초기화 (v0.620)
        self.engine = PoseEmergencyEngine()
        
        # 퍼블리셔 초기화
        self.em_pub = self.create_publisher(String, em_topic, 10)
        self.img_pub = self.create_publisher(CompressedImage, img_topic, 10)
        
        # 웹캠 초기화
        self.cap = cv2.VideoCapture(cam_idx)
        if not self.cap.isOpened():
            self.get_logger().warn(f"카메라 {cam_idx}번 실패. 1번 시도 중...")
            self.cap = cv2.VideoCapture(1)
            
        if not self.cap.isOpened():
            self.get_logger().error("카메라를 찾을 수 없습니다!")
            return

        self.get_logger().info(f"✅ 웹캠(v0.620) All-in-One 노드 시작!")
        self.get_logger().info(f"발행 토픽: {em_topic}, {img_topic}")

        # 타이머 주기 (15 FPS)
        self.timer = self.create_timer(1.0 / 15.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret: return

        # v0.620 엔진 분석 호출
        annotated, results_list = self.engine.analyze_frame_with_results(frame)
        emergency_level = self.engine.extract_frame_emergency_level(results_list)

        # 결과 발행 (Text)
        if emergency_level:
            msg = String()
            msg.data = emergency_level
            self.em_pub.publish(msg)

        # 결과 발행 (Image)
        success, encoded = cv2.imencode('.jpg', annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if success:
            img_msg = CompressedImage()
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "camera_link"
            img_msg.format = "jpeg"
            img_msg.data = encoded.tobytes()
            self.img_pub.publish(img_msg)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RescueVisionWebcamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
