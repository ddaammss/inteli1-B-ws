"""
SRD Severity Analyzer 모듈

이 모듈은 YOLOv8-Pose 모델을 사용하여 실시간으로 사람의 자세, 움직임, 호흡을 분석하고,
위급도를 판별하는 SeverityAnalyzer 클래스를 제공합니다.
주로 구조용 로봇이나 응급 상황 감지에 사용됩니다.
"""

import cv2
import numpy as np
import time
from collections import defaultdict, deque
from ultralytics import YOLO
from PIL import ImageFont, ImageDraw, Image  # 한글 출력을 위한 라이브러리 추가

class SeverityAnalyzer:
    """
    SeverityAnalyzer 클래스

    YOLOv8-Pose 모델을 기반으로 사람의 관절(Keypoints) 정보를 추출하고, 
    이를 시각적 분석 알고리즘(자세, 움직임, 가상 깊이 기반 호흡 등)과 결합하여 
    현재 사람의 위급 상태(Severity)를 실시간으로 판별합니다.
    """

    def __init__(self):
        """
        SeverityAnalyzer 초기화 메서드

        1. YOLO11-Pose 모델 로드 (yolo11n-pose.pt)
        2. UI 출력을 위한 한글 폰트(나눔고딕 등) 설정
        3. 개별 객체(환자) 추적을 위한 히스토리 저장소 초기화
        4. 시각화를 위한 관절 연결 구조 정의
        """
        # YOLO11-Pose 모델 로드: 최신 YOLO11 모델을 사용하여 더 정확한 포즈 추정 수행
        print("🚨 [SRD] 구조용 YOLO11-Pose 모델 로딩 중...")
        self.model = YOLO("yolo11n-pose.pt")
        
        # 한글 폰트 설정: OpenCV는 기본적으로 한글 출력을 지원하지 않으므로 PIL 라이브러리 사용
        try:
            # 윈도우 환경 대응
            self.font = ImageFont.truetype("malgun.ttf", 20)
        except:
            try:
                # 리눅스(Ubuntu) 환경 대응 (NanumGothic 설치 가정)
                self.font = ImageFont.truetype("/usr/share/fonts/truetype/nanum/NanumGothic.ttf", 20)
            except:
                print("⚠️ 한글 폰트를 찾을 수 없습니다. 영문으로 표시될 수 있습니다.")
                self.font = None

        # 환자 히스토리 초기화: 트랙 ID(Track ID)를 키로 사용하여 시계열 데이터 관리
        # - prev_gray_roi: 이전 프레임의 흑백 관심 영역 (움직임 계산용)
        # - motion_buffer: 최근 움직임 점수 저장 (노이즈 방지용 큐)
        # - depth_buffer: 최근 가슴 부위 깊이 값 저장 (호흡 분석용)
        self.patient_history = defaultdict(lambda: {
            'prev_gray_roi': None,
            'motion_buffer': deque(maxlen=30),
            'depth_buffer': deque(maxlen=30),
            'last_seen': time.time(),
            'first_seen': time.time()
        })
        
        # 실제 깊이 카메라가 없을 경우를 대비한 시뮬레이션 모드
        self.mock_depth_mode = True  
        
        # 스켈레톤 연결 구조 (COCO Keypoints 기준 인덱스)
        # 0: 코, 5/6: 어깨, 7/8: 팔꿈치, 9/10: 손목, 11/12: 골반, 13/14: 무릎, 15/16: 발목
        self.skeleton_links = [
            (5, 6), (5, 11), (6, 12), (11, 12),  # 상체(어깨-골반) 및 골반 라인
            (5, 7), (7, 9), (6, 8), (8, 10),     # 왼쪽/오른쪽 팔
            (11, 13), (13, 15), (12, 14), (14, 16)  # 왼쪽/오른쪽 다리
        ]

    def draw_korean_text(self, img, text, pos, color):
        """
        OpenCV 이미지에 한글 텍스트를 그리는 헬퍼 함수

        PIL 라이브러리를 사용하여 한글 폰트를 지원합니다.
        폰트가 없을 경우 OpenCV 기본 폰트로 대체합니다.

        Args:
            img (np.ndarray): OpenCV 이미지
            text (str): 그릴 텍스트
            pos (tuple): 텍스트 위치 (x, y)
            color (tuple): BGR 색상 (B, G, R)

        Returns:
            np.ndarray: 텍스트가 그려진 이미지
        """
        if self.font is None:
            cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            return img
        
        img_pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(img_pil)
        draw.text(pos, text, font=self.font, fill=(color[2], color[1], color[0]))
        return cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)

    def draw_skeleton(self, img, keypoints, color):
        """
        추출된 관절 포인트와 뼈대를 이미지에 시각화하는 함수

        스켈레톤 연결선과 관절 포인트를 그려 사람의 자세를 표시합니다.

        Args:
            img (np.ndarray): 그릴 이미지
            keypoints (np.ndarray): 관절 포인트 좌표 (17개 포인트)
            color (tuple): BGR 색상
        """
        # 1. 뼈대 연결선 그리기
        for start_idx, end_idx in self.skeleton_links:
            pt1 = keypoints[start_idx]
            pt2 = keypoints[end_idx]
            if pt1[0] > 0 and pt1[1] > 0 and pt2[0] > 0 and pt2[1] > 0:
                cv2.line(img, (int(pt1[0]), int(pt1[1])), (int(pt2[0]), int(pt2[1])), color, 2)
        
        # 2. 관절 포인트(도트) 그리기
        for i, pt in enumerate(keypoints):
            if pt[0] > 0 and pt[1] > 0:
                # 얼굴 부위(0~4)는 작게, 몸통 관절은 조금 더 크게 표시
                radius = 3 if i < 5 else 5
                cv2.circle(img, (int(pt[0]), int(pt[1])), radius, (255, 255, 255), -1)  # 흰색 테두리
                cv2.circle(img, (int(pt[0]), int(pt[1])), radius - 1, color, -1)

    def analyze_frame(self, frame):
        """
        입력 프레임에 대해 전체 분석 프로세스를 실행합니다.

        1. YOLO Pose 모델 추적: 객체 감지 및 스켈레톤 추출
        2. 개별 객체별 상태 분석 루프 실행
        3. 비정상 자세 판별 (Horizontal Pose)
        4. 프레임간 차분(Pixel Difference) 기반 움직임 분석
        5. 가상 깊이 값의 변화량 분석을 통한 호흡 검증
        6. 최종 상태 종합 및 UI 시각화
        """
        # 1. 전처리 및 추론
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # persist=True: 이전 프레임의 추적 정보를 유지함 (ID 유지 필수)
        results = self.model.track(frame, persist=True, conf=0.5, verbose=False)
        annotated_frame = frame.copy()
        
        # 실제 센서가 없는 경우 가상의 깊이 맵 생성 (평균 1500mm, 노이즈 10mm)
        depth_frame = np.random.normal(1500, 10, gray_frame.shape) if self.mock_depth_mode else None

        # 결과 처리
        if results[0].boxes is not None and results[0].boxes.id is not None:
            boxes = results[0].boxes.xyxy.cpu().numpy()  # 바운딩 박스 좌표
            track_ids = results[0].boxes.id.int().cpu().numpy()  # 고유 트랙 ID
            keypoints_data = results[0].keypoints.xy.cpu().numpy()  # 17개 관절 좌표

            # 감지된 사람별 반복 처리
            for box, track_id, kp in zip(boxes, track_ids, keypoints_data):
                patient = self.patient_history[track_id]
                patient['last_seen'] = time.time()
                x1, y1, x2, y2 = map(int, box)
                
                # ROI(관심 영역) 범위가 이미지 내부인지 확인
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(frame.shape[1], x2), min(frame.shape[0], y2)
                if x2 <= x1 or y2 <= y1: continue

                # [Module 1] 자세 분석: 수평 자세(Lying Down) 감지
                # 코(0)의 Y좌표와 골반(11, 12)의 평균 Y좌표를 비교
                # 코가 골반 높이와 비슷하거나 아래에 있으면 누워 있는 상태로 판단
                nose_y = kp[0][1]
                hip_y = (kp[11][1] + kp[12][1]) / 2 if (kp[11][1] > 0 and kp[12][1] > 0) else 0
                is_abnormal_pose = (nose_y > hip_y - 50) if (nose_y > 0 and hip_y > 0) else False

                # [Module 2] 움직임 분석: 픽셀 기반 변화량
                # 현재 ROI와 이전 프레임의 ROI를 비교하여 픽셀 밝기 값의 평균 변화량 계산
                curr_roi = gray_frame[y1:y2, x1:x2]
                motion_score = 0
                if patient['prev_gray_roi'] is not None:
                    try:
                        # 크기가 달라졌을 수 있으므로 리사이즈 후 차분 계산
                        prev_roi = cv2.resize(patient['prev_gray_roi'], (x2-x1, y2-y1))
                        diff = cv2.absdiff(prev_roi, curr_roi)
                        motion_score = np.sum(diff) / (diff.shape[0] * diff.shape[1])
                    except: pass
                
                patient['prev_gray_roi'] = curr_roi
                patient['motion_buffer'].append(motion_score)
                avg_motion = np.mean(patient['motion_buffer'])
                # 움직임 임계값: 2.0 미만이면 거의 정지 상태로 간주
                is_motionless = avg_motion < 2.0

                # [Module 3] 호흡 검증: 가슴 부위 미세 움직임 분석
                # 가슴(어깨 관절 5, 6 사이 중앙) 좌표 추출
                chest_y = int((kp[5][1] + kp[6][1]) / 2) if (kp[5][1] > 0 and kp[6][1] > 0) else int((y1+y2)/2)
                chest_x = int((x1+x2)/2)
                if 0 < chest_y < frame.shape[0] and 0 < chest_x < frame.shape[1]:
                    # 가슴 부위 주변 5x5 영역의 평균 깊이값 추출
                    depth_val = np.mean(depth_frame[max(0, chest_y-2):min(frame.shape[0], chest_y+3), 
                                                    max(0, chest_x-2):min(frame.shape[1], chest_x+3)])
                    patient['depth_buffer'].append(depth_val)
                
                # 깊이 값의 표준편차(Standard Deviation) 계산
                # 호흡을 하면 가슴 높이가 변하므로 표준편차가 커짐
                depth_std = np.std(patient['depth_buffer']) if len(patient['depth_buffer']) > 10 else 10.0
                is_not_breathing = depth_std < 1.5

                # [Module 4] 최종 위급도 판별 (4단계 Decision Tree Logic)
                # 초기 상태: 양호 (초록색)
                color = (0, 255, 0)  
                status_text = "양호 (의식 있음)"
                
                # 최소 관찰 시간(1.5초)이 지난 후부터 판정 시작
                time_observed = time.time() - patient['first_seen']
                if time_observed > 1.5:
                    if is_abnormal_pose and is_motionless and is_not_breathing:
                        # 🔴 위급: 쓰러짐 + 무반응 + 무호흡 (심정지 의심, 즉각 구조)
                        color = (0, 0, 255)  
                        status_text = "위급 (의식 불명/무호흡)"
                    elif is_abnormal_pose and is_motionless:
                        # 🟠 경고: 쓰러짐 + 무반응 + 호흡 유지 (중증 부상, 의식 희미)
                        color = (0, 165, 255) # Orange (BGR)
                        status_text = "경고 (거동 불가/호흡 중)"
                    elif is_abnormal_pose or is_motionless:
                        # 🟡 주의: 쓰러짐(움직임 관찰됨) 또는 장시간 정지(자세는 정상)
                        color = (0, 255, 255)  
                        status_text = "주의 (부상 의심/경과 관찰)"
                else:
                    status_text = "상태 분석 중..."
                    color = (255, 255, 255)

                # UI 시각화 - 바운딩 박스 및 스켈레톤
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                self.draw_skeleton(annotated_frame, kp, color) 
                
                # 환자 머리 위에 상태 텍스트 출력
                annotated_frame = self.draw_korean_text(annotated_frame, status_text, (x1, y1 - 30), color)

                # 하단에 디버깅 정보 (평균 움직임량, 깊이 변화량) 표시
                debug_info = f"M: {avg_motion:.1f} | D: {depth_std:.1f}"
                cv2.putText(annotated_frame, debug_info, (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        return annotated_frame

def main():
    """
    메인 실행 루프

    1. SeverityAnalyzer 인스턴스 생성
    2. 로컬 카메라(Index 0) 연결
    3. 실시간 프레임 획득 및 분석 결과 출력
    4. 프레임 리사이징을 통한 처리 속도 최적화
    5. 'q' 키 입력 시 안전하게 리소스 해제 후 종료
    """
    analyzer = SeverityAnalyzer()
    
    # 카메라 캡처 객체 생성
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ 카메라를 열 수 없습니다.")
        return

    print("🚀 SRD 위급도 분석기 시작 (종료하려면 'q'를 누르세요)")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ 프레임을 가져올 수 없습니다.")
            break
            
        # 가로 해상도 800 기준으로 비율 맞춰 리사이징 (연산 속도 향상)
        width = 800
        height = int(frame.shape[0] * (width / frame.shape[1]))
        frame = cv2.resize(frame, (width, height))
        
        # 분석 실행
        result_frame = analyzer.analyze_frame(frame)
        
        # 결과 화면 표시
        cv2.imshow("SRD Severity Analyzer (Pose Visualization)", result_frame)
        
        # 1ms 대기하며 키 입력 확인
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 리소스 해제
    cap.release()
    cv2.destroyAllWindows()
    print("👋 분석기를 종료합니다.")

if __name__ == "__main__":
    main()