import cv2
import time
import os
import threading
import numpy as np
from ultralytics import YOLO

# TensorFlow GPU 충돌 방지 및 필요한 만큼만 메모리 사용 설정
os.environ['TF_FORCE_GPU_ALLOW_GROWTH'] = 'true'
os.environ['CUDA_VISIBLE_DEVICES'] = '-1' # 안정적인 테스트를 위해 CPU 모드 권장

from deepface import DeepFace

# 백그라운드 쓰레드와 메인 화면이 공유할 전역 변수
current_emotion = "Searching..."
is_analyzing = False # 현재 AI가 분석 중인지 확인하는 플래그

def analyze_emotion_thread(face_crop):
    """백그라운드에서 표정을 분석하는 함수 (화면 끊김 방지)"""
    global current_emotion, is_analyzing
    
    try:
        # 이미 잘라낸 얼굴 영역을 넘기므로 enforce_detection=False 설정
        result = DeepFace.analyze(
            face_crop, 
            actions=['emotion'], 
            enforce_detection=False, 
            silent=True
        )
        
        if isinstance(result, list):
            result = result[0]
        
        current_emotion = result['dominant_emotion']
            
    except Exception as e:
        pass # 분석 실패 시 이전 결과 유지
    finally:
        is_analyzing = False

def main():
    global current_emotion, is_analyzing
    
    # 웹캠 켜기
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("❌ 웹캠을 찾을 수 없습니다.")
        return

    # [최신화] YOLO11-Pose 모델 로드
    # yolo11n-pose.pt는 이전 버전보다 속도와 정확도가 향상된 최신 모델입니다.
    print("🚨 YOLO11-Pose 모델 로딩 중...")
    yolo_model = YOLO("yolo11n-pose.pt")

    print("✅ YOLO11 + DeepFace 통합 테스트 시작 (종료: 'q' 키)")
    
    last_check_time = 0
    face_rect = None # (x1, y1, x2, y2)

    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        frame = cv2.flip(frame, 1) # 거울 모드
        h_orig, w_orig, _ = frame.shape
        current_time = time.time()

        # 1. YOLO11-Pose를 이용한 요구조자 및 얼굴 위치 추적
        results = yolo_model.track(frame, persist=True, verbose=False, conf=0.5)

        if results[0].keypoints is not None and len(results[0].keypoints.xy) > 0:
            # 여러 명 중 첫 번째 객체의 키포인트 사용
            kp = results[0].keypoints.xy[0].cpu().numpy() 
            
            # 관절 번호: 0(코), 1(왼쪽 눈), 2(오른쪽 눈), 3(왼쪽 귀), 4(오른쪽 귀)
            face_points = kp[0:5]
            valid_points = face_points[np.all(face_points > 0, axis=1)]
            
            if len(valid_points) >= 3:
                # 얼굴 관절점들을 감싸는 바운딩 박스 계산
                fx1, fy1 = np.min(valid_points, axis=0)
                fx2, fy2 = np.max(valid_points, axis=0)
                
                # YOLO11은 탐지가 더 날카로우므로 마진을 충분히 주어 얼굴 전체가 포함되게 함
                margin_w = (fx2 - fx1) * 0.7
                margin_h = (fy2 - fy1) * 1.2
                
                x1 = int(max(0, fx1 - margin_w))
                y1 = int(max(0, fy1 - margin_h))
                x2 = int(min(w_orig, fx2 + margin_w))
                y2 = int(min(h_orig, fy2 + margin_h))
                
                face_rect = (x1, y1, x2, y2)

                # 2. AI 표정 분석 (0.5초 주기)
                if current_time - last_check_time > 0.5 and not is_analyzing:
                    last_check_time = current_time
                    is_analyzing = True
                    
                    face_crop = frame[y1:y2, x1:x2]
                    if face_crop.size > 0:
                        thread = threading.Thread(target=analyze_emotion_thread, args=(face_crop,))
                        thread.daemon = True 
                        thread.start()
                    else:
                        is_analyzing = False
            else:
                face_rect = None
        else:
            face_rect = None
            current_emotion = "Searching..."

        # 3. 시각화
        if face_rect is not None:
            x1, y1, x2, y2 = face_rect
            
            # 감정에 따른 색상 변경
            color = (0, 255, 0) # Green (Normal)
            if current_emotion in ['sad', 'angry', 'fear', 'disgust']:
                color = (0, 0, 255) # Red (Danger/Pain)
            
            # 박스와 텍스트 그리기
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"Emotion: {current_emotion.upper()}", (x1, y1 - 15), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            
            # 관절 포인트 시각화
            for p in face_points:
                if p[0] > 0:
                    cv2.circle(frame, (int(p[0]), int(p[1])), 3, (255, 255, 255), -1)

        # 상태 오버레이
        cv2.putText(frame, "YOLO11 POSE + DEEPFACE ACTIVE", (20, 40), 
                    cv2.FONT_HERSHEY_DUPLEX, 0.7, (255, 255, 255), 1)

        if is_analyzing:
            cv2.putText(frame, "AI Analyzing...", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow("SRD YOLO11 + DeepFace Test", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()