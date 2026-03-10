import cv2
import numpy as np
import time
import os
import threading
import json
from collections import defaultdict, deque

# ROS 2 관련 라이브러리
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ultralytics import YOLO
from PIL import ImageFont, ImageDraw, Image
from deepface import DeepFace

# TensorFlow 환경 설정
os.environ['TF_FORCE_GPU_ALLOW_GROWTH'] = 'true'
os.environ['CUDA_VISIBLE_DEVICES'] = '-1' 

class SrdAnalyzerNode(Node):
    """
    YOLO11-Pose + DeepFace 분석 후 ROS 2 토픽으로 결과를 발행하는 노드
    """
    def __init__(self):
        super().__init__('srd_analyzer_node')
        
        self.declare_parameter('model_path', 'yolo11n-pose.pt')
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        
        self.get_logger().info(f"🚨 [SRD] 모델 로딩 중: {model_path}")
        self.model = YOLO(model_path)
        
        self.publisher_ = self.create_publisher(String, '/srd/severity_data', 10)
        
        self.EMOTION_BUFFER_SIZE = 5
        self.GRIMACE_THRESHOLD = 0.6
        self.font = self._load_font()

        self.patient_history = defaultdict(lambda: {
            'prev_gray_roi': None,
            'motion_buffer': deque(maxlen=30),
            'depth_buffer': deque(maxlen=30),
            'first_seen': time.time(),
            'last_face_eval_time': 0,
            'emotion_buffer': deque(maxlen=self.EMOTION_BUFFER_SIZE),
            'stable_grimacing': False,
            'current_emotion': 'neutral',
            'face_analyzing': False
        })
        
        self.mock_depth_mode = True 
        self.skeleton_links = [
            (5, 6), (5, 11), (6, 12), (11, 12),
            (5, 7), (7, 9), (6, 8), (8, 10),
            (11, 13), (13, 15), (12, 14), (14, 16)
        ]

    def _load_font(self):
        for path in ["malgun.ttf", "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"]:
            try:
                return ImageFont.truetype(path, 20)
            except:
                continue
        return None

    def draw_korean_text(self, img, text, pos, color):
        if self.font is None:
            cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            return img
        img_pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(img_pil)
        draw.text(pos, text, font=self.font, fill=(color[2], color[1], color[0]))
        return cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)

    def draw_skeleton(self, img, keypoints, color):
        for start_idx, end_idx in self.skeleton_links:
            pt1, pt2 = keypoints[start_idx], keypoints[end_idx]
            if all(v > 0 for v in pt1) and all(v > 0 for v in pt2):
                cv2.line(img, (int(pt1[0]), int(pt1[1])), (int(pt2[0]), int(pt2[1])), color, 2)
        for i, pt in enumerate(keypoints):
            if pt[0] > 0:
                cv2.circle(img, (int(pt[0]), int(pt[1])), 3 if i < 5 else 5, (255, 255, 255), -1)

    def analyze_face_async(self, track_id, face_roi):
        patient = self.patient_history[track_id]
        try:
            if face_roi.shape[0] < 30: return
            res = DeepFace.analyze(face_roi, actions=['emotion'], enforce_detection=False, silent=True)
            emotion = res[0]['dominant_emotion'] if isinstance(res, list) else res['dominant_emotion']
            patient['current_emotion'] = emotion
            patient['emotion_buffer'].append(emotion)
            grimace_list = [e in ['sad', 'angry', 'fear', 'disgust'] for e in patient['emotion_buffer']]
            patient['stable_grimacing'] = (sum(grimace_list) / len(grimace_list)) >= self.GRIMACE_THRESHOLD
        except: pass
        finally:
            patient['face_analyzing'] = False
            patient['last_face_eval_time'] = time.time()

    def process_and_publish(self, frame):
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = self.model.track(frame, persist=True, conf=0.5, verbose=False)
        annotated_frame = frame.copy()
        
        if results[0].boxes is not None and results[0].boxes.id is not None:
            boxes = results[0].boxes.xyxy.cpu().numpy()
            track_ids = results[0].boxes.id.int().cpu().numpy()
            keypoints_data = results[0].keypoints.xy.cpu().numpy()

            for box, track_id, kp in zip(boxes, track_ids, keypoints_data):
                patient = self.patient_history[track_id]
                x1, y1, x2, y2 = map(int, box)
                x1, y1, x2, y2 = max(0, x1), max(0, y1), min(frame.shape[1], x2), min(frame.shape[0], y2)

                nose_y, hip_y = kp[0][1], (kp[11][1] + kp[12][1]) / 2 if kp[11][0] > 0 else 0
                is_abnormal_pose = (nose_y > hip_y - 50) if nose_y > 0 and hip_y > 0 else False
                
                motion_score = 0
                if patient['prev_gray_roi'] is not None:
                    try:
                        diff = cv2.absdiff(cv2.resize(patient['prev_gray_roi'], (x2-x1, y2-y1)), gray_frame[y1:y2, x1:x2])
                        motion_score = np.sum(diff) / (diff.size)
                    except: pass
                patient['prev_gray_roi'] = gray_frame[y1:y2, x1:x2]
                patient['motion_buffer'].append(motion_score)
                avg_motion = float(np.mean(patient['motion_buffer']))

                if not patient['face_analyzing'] and (time.time() - patient['last_face_eval_time'] > 0.8):
                    face_kps = [p for p in kp[0:5] if p[0] > 0]
                    if len(face_kps) >= 3:
                        fx1, fy1 = max(0, int(min(p[0] for p in face_kps))-20), max(0, int(min(p[1] for p in face_kps))-30)
                        fx2, fy2 = min(frame.shape[1], int(max(p[0] for p in face_kps))+20), min(frame.shape[0], int(max(p[1] for p in face_kps))+20)
                        patient['face_analyzing'] = True
                        threading.Thread(target=self.analyze_face_async, args=(track_id, frame[fy1:fy2, fx1:fx2].copy()), daemon=True).start()

                severity = "NORMAL"
                status_text = "양호 (의식 있음)"
                color = (0, 255, 0)

                if (time.time() - patient['first_seen']) > 1.5:
                    if is_abnormal_pose and avg_motion < 2.0 and patient['stable_grimacing']:
                        severity, status_text, color = "CRITICAL", "위급 (의식불명/극심고통)", (0, 0, 255)
                    elif is_abnormal_pose and avg_motion < 2.0:
                        severity, status_text, color = "WARNING", "경고 (거동불가/관찰필요)", (0, 165, 255)
                    elif is_abnormal_pose or patient['stable_grimacing']:
                        severity, status_text, color = "CAUTION", "주의 (부상/고통호소)", (0, 255, 255)

                data_payload = {
                    "track_id": int(track_id),
                    "severity": severity,
                    "status_msg": status_text,
                    "emotion": patient['current_emotion'],
                    "motion_score": avg_motion,
                    "is_lying": bool(is_abnormal_pose)
                }
                self.publisher_.publish(String(data=json.dumps(data_payload)))

                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                self.draw_skeleton(annotated_frame, kp, color)
                annotated_frame = self.draw_korean_text(annotated_frame, status_text, (x1, y1 - 35), color)

        return annotated_frame

def main(args=None):
    rclpy.init(args=args)
    node = SrdAnalyzerNode()
    cap = cv2.VideoCapture(0)
    try:
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret: break
            frame = cv2.flip(frame, 1)
            display_frame = node.process_and_publish(frame)
            cv2.imshow("SRD Analyzer Node", display_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
            rclpy.spin_once(node, timeout_sec=0)
    finally:
        cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()