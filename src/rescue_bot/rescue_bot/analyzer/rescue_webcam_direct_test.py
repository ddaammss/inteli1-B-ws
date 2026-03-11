import cv2
import argparse
from rescue_vision_core import AnalyzerConfig, PoseEmergencyEngine

def main():
    parser = argparse.ArgumentParser(description="Direct UI Test for Rescue Vision Core (No ROS2)")
    parser.add_argument("--device", type=int, default=0, help="Webcam device ID (default 0)")
    parser.add_argument("--model", type=str, default="yolo11n-pose.pt", help="Path to YOLO pose model")
    args = parser.parse_args()

    # 설정 및 코어 엔진 생성
    cfg = AnalyzerConfig(model_path=args.model, show_debug=True)
    engine = PoseEmergencyEngine(cfg)

    # 웹캠 연결
    cap = cv2.VideoCapture(args.device)
    if not cap.isOpened():
        print(f"웹캠({args.device}번)을 열 수 없습니다!")
        return

    print("====================================")
    print("SRD Rescue Vision Direct Test Started")
    print("====================================")
    print("Press 'q' or 'ESC' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break

        # 코어 엔진에 프레임을 집어넣어 결과 이미지를 받아옵니다
        annotated_frame, emergency_level = engine.analyze_frame_with_emergency_level(frame)

        # OpenCV 창에 결과 표시
        cv2.imshow("Rescue Vision Direct Test", annotated_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break

    # 종료
    cap.release()
    cv2.destroyAllWindows()
    print("Direct Test Stopped.")

if __name__ == "__main__":
    main()
