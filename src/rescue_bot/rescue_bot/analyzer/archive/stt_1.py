import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

import whisper
import speech_recognition as sr
import torch
import numpy as np
from gtts import gTTS
import pygame
import os
import re
import time

'''
================================================================================
[Rescue Dialogue Node]
ROS 2 환경에서 구조 대상자와 상호작용하기 위한 음성 인식/합성 노드.
Mic 입력으로 피해자의 상황을 파악하고, TTS와 사이렌으로 응답 및 구조를 진행합니다.
================================================================================
'''
class RescueDialogueNode(Node):
    
    def __init__(self):
        '''
        [노드 초기화 및 AI 모델 로딩]
        노드의 이름('rescue_dialogue_node')을 설정하고 필요한 통신 인프라와 AI 모델을 준비합니다.
        
        통신 설정:
        - victim_voice_reply: 최종 판단 결과를 퍼블리시합니다. (큐 사이즈 10)
        - done: 대화 시나리오가 성공적으로 끝났음(또는 NORMAL 상태)을 알리는 플래그를 퍼블리시합니다.
        
        AI 모델 설정:
        - PyTorch를 활용해 시스템에 GPU(CUDA)가 사용 가능한지 확인하고, Whisper 모델을 할당합니다.
        - 음성 인식기(Recognizer)와 마이크를 초기화한 뒤, 1초간 주변 소음을 측정하여 
          adjust_for_ambient_noise()로 배경 노이즈를 보정합니다.
          
        주의: 무거운 Whisper 모델 로딩이 완료된 이후에만 'request' 토픽 Subscriber를 생성합니다.
        '''
        super().__init__('rescue_dialogue_node')

        self.publisher_ = self.create_publisher(String, 'victim_voice_reply', 10)
        self.done_pub = self.create_publisher(Bool, 'done', 10)

        self.get_logger().info("AI 시스템(Whisper) 로딩 중... 잠시만 기다려주세요.")
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = whisper.load_model("small").to(self.device)
        self.r = sr.Recognizer()
        self.mic = sr.Microphone()

        with self.mic as source:
            self.r.adjust_for_ambient_noise(source, duration=1)

        self.get_logger().info("시스템 준비 완료!")
        
        self.request_sub = self.create_subscription(String, 'request', self.trigger_callback, 10)
        self.get_logger().info("request 토픽 대기 중...")

    def speak(self, text):
        '''
        [음성 합성 및 출력 함수 (TTS)]
        입력받은 텍스트를 Google TTS(gTTS)를 사용하여 한국어 음성(.mp3)으로 변환하고 스피커로 출력합니다.
        재생이 완료될 때까지 코드 실행을 블로킹하여 하울링을 방지합니다.
        '''
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
        '''
        [긴급 사이렌 재생 함수]
        요구조자가 응답하지 않는 긴급 상황일 경우 실행되어 사이렌 소리를 재생합니다.
        '''
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
        '''
        [음성 인식 함수 (STT)]
        마이크를 통해 최대 7초간 음성을 수집하고, Whisper 모델을 이용해 텍스트로 변환합니다.
        노이즈로 의심되는 영어 단어나 빈 데이터는 걸러냅니다.
        '''
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

    def trigger_callback(self, msg):
        '''
        [시나리오 트리거 콜백]
        '/request' 토픽으로 들어오는 상태 메시지에 따라 동작을 분기합니다.
        - ANALYZING: 상태 분석 중이므로 무시합니다.
        - NORMAL: 환자가 안전한 상태이므로 시나리오 없이 바로 done(True)을 발행합니다.
        - CAUTION / WARNING / CRITICAL: 위험 상태이므로 대화 시나리오(run_scenario)를 가동합니다.
        '''
        status = msg.data.strip().upper() # 대소문자 구분을 없애고 공백 제거
        self.get_logger().info(f"[request 수신] 상태: '{status}'")

        if status == 'ANALYZING':
            self.get_logger().info("  -> 상태 분석 중... (무시)")
            return
            
        elif status == 'NORMAL':
            self.get_logger().info("  -> NORMAL 상태: 대화 생략, done 토픽 발행")
            done_msg = Bool()
            done_msg.data = True
            self.done_pub.publish(done_msg)
            
        elif status in ['CAUTION', 'WARNING', 'CRITICAL']:
            self.get_logger().info(f"  -> {status} 상태: 구조 대화 시나리오 시작!")
            self.run_scenario()
            
        else:
            self.get_logger().warn(f"  -> 알 수 없는 상태 메시지입니다: '{status}'")

    def run_scenario(self):
        '''
        [메인 대화 시나리오 로직]
        요구조자의 상태를 파악하기 위한 2단계 질문-답변(Q&A) 트리입니다.
        2차 응답까지 완료되면 'done' 토픽을 발행하고, 응답이 없으면 사이렌을 울립니다.
        최종 상태는 'victim_voice_reply' 토픽으로 퍼블리시됩니다.
        '''
        self.speak("괜찮으세요?")
        reply_1 = self.listen()
        
        if reply_1:
            print(f"👤 1차 응답: {reply_1}")
            self.speak("어디가 안 좋으신가요?")
            reply_2 = self.listen()
            final_msg = f"1차 응답: {reply_1} / 2차 응답: {reply_2 if reply_2 else '없음'}"
            
            if reply_2:
                # 2차 응답까지 무사히 받았을 때 done 토픽 발행
                self.get_logger().info("  -> 2차 응답 완료: done 토픽 발행")
                done_msg = Bool()
                done_msg.data = True
                self.done_pub.publish(done_msg)
            else:
                final_msg = "긴급!!!!!!!!!!!!!!!!!!!"
                self.play_siren()
        else:
            final_msg = "긴급!!!!!!!!!!!!!!!!!"
            self.play_siren()

        self.speak("물과 구급상자가 있고 곧 구조대가 올 것입니다.")

        # 1차, 2차 응답 내용 또는 긴급 상황 메시지를 퍼블리시
        msg = String()
        msg.data = final_msg
        self.publisher_.publish(msg)
        self.get_logger().info("시나리오 완료. 다음 request 대기 중...")


def main(args=None):
    '''
    [ROS 2 노드 메인 실행부]
    '''
    rclpy.init(args=args)
    node = RescueDialogueNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()