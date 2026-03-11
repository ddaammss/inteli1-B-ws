import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import json
import threading

import whisper
import speech_recognition as sr
import torch
import numpy as np
from gtts import gTTS
import pygame
import os
import re
import time

class RescueDialogueNode(Node):
    def __init__(self):
        super().__init__('rescue_dialogue_node')
        
        # 기존 음성 기록 퍼블리셔 (네임스페이스 통일)
        self.publisher_ = self.create_publisher(String, '/robot6/victim_voice_reply', 10)
        
        # 행동 제어용 'keep_going' 토픽
        self.keep_going_pub = self.create_publisher(String, '/robot6/keep_going', 10)

        # Control 노드 연동용 토픽
        self.tts_req_sub = self.create_subscription(String, '/robot6/tts/request', self.tts_request_callback, 10)
        self.tts_done_pub = self.create_publisher(Bool, '/robot6/tts/done', 10)

        # 실행 상태 잠금용 플래그
        self.is_running = False

        
        self.get_logger().info("AI 시스템 로딩 중...")
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        # 최종 확정된 small 모델 적용
        self.model = whisper.load_model("small").to(self.device)
        self.r = sr.Recognizer()
        self.mic = sr.Microphone()
        
        with self.mic as source:
            self.r.adjust_for_ambient_noise(source, duration=1)
            
        self.get_logger().info("시스템 준비 완료. 시나리오를 시작합니다.")

    def speak(self, text):
        print(f"\n[구급 로봇 🔊]: '{text}'")
        try:
            tts = gTTS(text=text, lang='ko')
            tts.save("temp_voice.mp3")
            pygame.mixer.init()
            pygame.mixer.music.load("temp_voice.mp3")
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy(): time.sleep(0.1)
            pygame.mixer.quit()
            os.remove("temp_voice.mp3")
        except: pass

    def play_siren(self):
        siren_path = "/home/jaylee/Downloads/siren.mp3"
        if not os.path.exists(siren_path):
            print(f"\n[경고] 사이렌 파일을 찾을 수 없습니다: {siren_path}")
            return
            
        print(f"\n🚨 [비상] 응답 없음! 사이렌을 울립니다. 🚨")
        try:
            pygame.mixer.init()
            pygame.mixer.music.load(siren_path)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy(): time.sleep(0.1)
            pygame.mixer.quit()
        except Exception as e:
            print(f"[오류] 사이렌 재생 중 문제 발생: {e}")

    def listen(self):
        print("🎤 듣는 중...")
        try:
            with self.mic as source:
                audio = self.r.listen(source, timeout=7)
            raw_data = audio.get_raw_data(convert_rate=16000, convert_width=2)
            wav_data = np.frombuffer(raw_data, np.int16).flatten().astype(np.float32) / 32768.0
            result = self.model.transcribe(wav_data, language="ko")
            text = result['text'].strip()
            if not text or re.search('[a-zA-Z]', text): return ""
            return text
        except: return None

    def run_scenario(self):
        self.speak("괜찮으세요?")
        reply_1 = self.listen()
        
        if reply_1:
            print(f"👤 1차 응답: {reply_1}")
            self.speak("어디가 안 좋으신가요?")
            reply_2 = self.listen()
            final_msg = f"1차 응답: {reply_1} / 2차 응답: {reply_2 if reply_2 else '없음'}"
            
            # 🌟 2차 응답이 정상적으로 들어왔을 때 keep_going 토픽 발행
            if reply_2:
                keep_msg = String()
                keep_msg.data = "keep_going"
                self.keep_going_pub.publish(keep_msg)
            else:
                final_msg = "긴급!!!!!!!!!!!!!!!!!!!"
                self.play_siren()
        else:
            final_msg = "긴급!!!!!!!!!!!!!!!!!"
            self.play_siren()

        self.speak("물과 구급상자가 있고 곧 구조대가 올 것입니다.")

        msg = String()
        msg.data = final_msg
        self.publisher_.publish(msg)
        
        # 🌟 관제 노드에 임무 종료(TTS Done) 보고
        done_msg = Bool()
        done_msg.data = True
        self.tts_done_pub.publish(done_msg)
        print("✅ STT 시나리오 종료 및 관제 노드에 보고 완료.")
        
        self.is_running = False

    def tts_request_callback(self, msg: String):
        """ Control 노드로부터 '/robot6/tts/request' 신호가 오면 실행되는 콜백 """
        if self.is_running:
            self.get_logger().warn("이미 대화 시나리오가 실행 중입니다! 요청을 무시합니다.")
            return
            
        self.get_logger().info(f"관제 노드로부터 대화 요청 수신됨! 데이터: {msg.data}")
        self.is_running = True
        
        # ROS2 콜백 블로킹을 막기 위해 새 스레드에서 시나리오 실행
        thread = threading.Thread(target=self.run_scenario)
        thread.start()

        

def main(args=None):
    rclpy.init(args=args)
    node = RescueDialogueNode()
    
    try:
        # node.run_scenario() <- 시작하자마자 혼자 실행되던 부분 삭제
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()