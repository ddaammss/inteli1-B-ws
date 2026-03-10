import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher_test')
        self.publisher_ = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', 10)
        self.timer = self.create_timer(1.0 / 15.0, self.timer_callback) # 15 fps
        self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            self.get_logger().error("웹캠(0번)을 열 수 없습니다!")
        else:
            self.get_logger().info("웹캠 퍼블리셔 시작! 토픽: /camera/image_raw/compressed")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # 압축 이미지로 인코딩 (quality 80)
        success, encoded_image = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if success:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            msg.format = "jpeg"
            msg.data = encoded_image.tobytes()
            self.publisher_.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
