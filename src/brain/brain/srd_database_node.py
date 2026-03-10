import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sqlite3
import json
import os

class SrdDatabaseNode(Node):
    """
    /srd/severity_data 토픽을 구독하여 
    상태가 변경되었을 때만 SQLite3 DB에 저장하는 노드
    """
    def __init__(self):
        super().__init__('srd_database_node')
        
        # 데이터베이스 파일 경로
        self.db_path = 'srd_mission_records.db'
        self.init_db()
        
        # 상태 추적용 메모리 (중복 저장 방지)
        self.last_victim_states = {}
        
        # 구독자 설정
        self.subscription = self.create_subscription(
            String,
            '/srd/severity_data',
            self.listener_callback,
            10
        )
        
        self.get_logger().info(f"📁 [SRD] 데이터베이스 노드 가동 중: {self.db_path}")

    def init_db(self):
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS severity_logs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                track_id INTEGER,
                severity TEXT,
                status_msg TEXT,
                emotion TEXT,
                motion_score REAL,
                is_lying INTEGER
            )
        ''')
        conn.commit()
        conn.close()

    def has_state_changed(self, track_id, current_data):
        if track_id not in self.last_victim_states:
            return True
        last = self.last_victim_states[track_id]
        changed = (
            last['severity'] != current_data['severity'] or
            last['emotion'] != current_data['emotion'] or
            last['is_lying'] != current_data['is_lying']
        )
        return changed

    def listener_callback(self, msg):
        try:
            data = json.loads(msg.data)
            track_id = data['track_id']
            
            if self.has_state_changed(track_id, data):
                conn = sqlite3.connect(self.db_path)
                cursor = conn.cursor()
                cursor.execute('''
                    INSERT INTO severity_logs 
                    (track_id, severity, status_msg, emotion, motion_score, is_lying)
                    VALUES (?, ?, ?, ?, ?, ?)
                ''', (
                    track_id,
                    data['severity'],
                    data['status_msg'],
                    data['emotion'],
                    data['motion_score'],
                    1 if data['is_lying'] else 0
                ))
                conn.commit()
                conn.close()
                
                self.last_victim_states[track_id] = {
                    'severity': data['severity'],
                    'emotion': data['emotion'],
                    'is_lying': data['is_lying']
                }
                self.get_logger().info(f"💾 [DB SAVED] ID {track_id} 상태 변경 저장")
        except Exception as e:
            self.get_logger().error(f"DB 저장 오류: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SrdDatabaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()