'''
파일명: rescue_ui.py
날짜: 2026-03-11
버전: v0.600

버전 변경사항
- v0.600: victim_pose_stamped 히스토리 저장 API를 추가하고, History 패널에 요구조자 좌표 기록을 포함할 수 있도록 확장함
- v0.500: UI용 API(/api/system_status, /api/robot_state, /api/alerts, /api/map_summary)를 추가하고, 향후 ROS/Gazebo 실제 연동 전환이 쉽도록 데이터 구성 함수를 분리함
- v0.400: rescue_ui를 메인 Flask 서버로 고정하고, ROS 웹 연동 설정(rosbridge/map/alert/stream)을 템플릿 컨텍스트로 분리해 welcome 템플릿 기준으로 통합함
- v0.300: UI/day5/login 라우트는 유지하면서, SQLite 연동 준비를 위한 서비스 함수 분리 및 설정 상수를 추가함
- v0.200: 로그인 페이지, 세션 인증, 로그아웃, 인증 보호 데코레이터를 추가해 대시보드 접근을 로그인 기반으로 변경함
- v0.110: day5 형태에 맞게 Flask 서버를 render_template 기반으로 리팩토링하고 대시보드 UI를 templates로 분리함
- v0.020: 앱 팩토리(create_app), 설정 상수, health check 라우트를 추가해 실행 구조를 정리함
- v0.001: resetMapView 호출 대비 템플릿 연동과 기본 실행부를 안정화함
'''

import hmac
import os
import sqlite3
from functools import wraps
from typing import Any, Dict, Optional

from flask import Flask, flash, redirect, render_template, request, session, url_for

APP_HOST = os.getenv('SRD_FLASK_HOST', '0.0.0.0')
APP_PORT = int(os.getenv('SRD_FLASK_PORT', '5000'))
APP_DEBUG = os.getenv('SRD_FLASK_DEBUG', 'false').lower() == 'true'
APP_SECRET_KEY = os.getenv('SRD_SECRET_KEY', 'srd-day5-dev-key')
DASHBOARD_VERSION = 'v3.2'
LOGIN_USERNAME = os.getenv('SRD_LOGIN_USERNAME', 'admin')
LOGIN_PASSWORD = os.getenv('SRD_LOGIN_PASSWORD', '1234')
DEFAULT_DASHBOARD_TITLE = 'SRD 통합 관제 시스템'
SQLITE_DB_PATH = os.getenv('SRD_SQLITE_PATH', 'srd_mission_records.db')
ROSBRIDGE_WS_DEFAULT = os.getenv('SRD_ROSBRIDGE_WS_URL', 'ws://{host}:9090')
ROS_MAP_TOPIC = os.getenv('SRD_ROS_MAP_TOPIC', '/robot5/map')
ROS_ALERT_TOPIC = os.getenv('SRD_ROS_ALERT_TOPIC', '/srd/severity_data')
ROS_POSE_TOPIC = os.getenv('SRD_ROS_POSE_TOPIC', '/robot6/amcl_pose')
ROS_PLAN_TOPIC = os.getenv('SRD_ROS_PLAN_TOPIC', '/robot6/plan')
MJPEG_ENDPOINT = os.getenv('SRD_MJPEG_ENDPOINT', 'http://{host}:8080/stream')
# CAM 01 임시 기본 토픽 (원복: /robot6/image_result/compressed -> /robot6/image_result)
MJPEG_TOPIC = os.getenv('SRD_MJPEG_TOPIC', '/robot6/image_result')
MJPEG_WIDTH = int(os.getenv('SRD_MJPEG_WIDTH', '640'))
MJPEG_HEIGHT = int(os.getenv('SRD_MJPEG_HEIGHT', '480'))
ALERTS_QUERY_LIMIT = int(os.getenv('SRD_ALERTS_QUERY_LIMIT', '50'))


def get_login_credentials() -> Dict[str, str]:
    return {
        'username': LOGIN_USERNAME,
        'password': LOGIN_PASSWORD,
    }


def validate_login(username: str, password: str) -> bool:
    credentials = get_login_credentials()
    is_valid_user = hmac.compare_digest(username, credentials['username'])
    is_valid_password = hmac.compare_digest(password, credentials['password'])
    return is_valid_user and is_valid_password


def build_base_template_context() -> Dict[str, Any]:
    return {
        'dashboard_title': DEFAULT_DASHBOARD_TITLE,
        'dashboard_version': DASHBOARD_VERSION,
    }


def build_dashboard_context(username: Optional[str]) -> Dict[str, Any]:
    context = build_base_template_context()
    context['username'] = username or 'operator'
    context['rosbridge_ws_default'] = ROSBRIDGE_WS_DEFAULT
    context['ros_map_topic'] = ROS_MAP_TOPIC
    context['ros_alert_topic'] = ROS_ALERT_TOPIC
    context['ros_pose_topic'] = ROS_POSE_TOPIC
    context['ros_plan_topic'] = ROS_PLAN_TOPIC
    context['mjpeg_endpoint'] = MJPEG_ENDPOINT
    context['mjpeg_topic'] = MJPEG_TOPIC
    context['mjpeg_width'] = MJPEG_WIDTH
    context['mjpeg_height'] = MJPEG_HEIGHT
    return context


def get_sqlite_config() -> Dict[str, str]:
    """
    SQLite 연동 준비용 설정 제공 함수.
    실제 DB 연결 로직은 추후 이 함수/전용 모듈을 통해 확장한다.
    """
    return {
        'db_path': SQLITE_DB_PATH,
    }


def get_sqlite_connection(read_only: bool = True):
    """
    SQLite 연결 생성.
    - read_only=True: 읽기 전용 (mode=ro)
    - read_only=False: 쓰기 가능
    - 우선순위 1: SRD_SQLITE_PATH
    - 우선순위 2: 현재 작업 경로의 srd_mission_records.db
    - 우선순위 3: 패키지 내 database/data/srd_mission_records.db
    """
    candidate_paths = []

    configured_path = SQLITE_DB_PATH.strip()
    if configured_path:
        if os.path.isabs(configured_path):
            candidate_paths.append(configured_path)
        else:
            candidate_paths.append(os.path.abspath(configured_path))

    package_db_path = os.path.abspath(
        os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            '..',
            'database',
            'data',
            'srd_mission_records.db',
        )
    )
    if package_db_path not in candidate_paths:
        candidate_paths.append(package_db_path)

    for db_path in candidate_paths:
        if os.path.exists(db_path):
            uri = f'file:{db_path}?mode=ro' if read_only else f'file:{db_path}'
            return sqlite3.connect(uri, uri=True, check_same_thread=False)

    return None


def build_empty_alerts_data() -> Dict[str, Any]:
    return {
        'count': 0,
        'items': [],
        'source': 'unavailable',
    }


def build_health_payload() -> Dict[str, Any]:
    sqlite_config = get_sqlite_config()
    sqlite_conn = get_sqlite_connection()
    sqlite_ready = sqlite_conn is not None
    if sqlite_conn is not None:
        sqlite_conn.close()
    return {
        'status': 'ok',
        'service': 'srd-flask-server',
        'version': DASHBOARD_VERSION,
        'login_enabled': True,
        'sqlite_ready': sqlite_ready,
        'sqlite_db_path': sqlite_config['db_path'],
    }


def get_system_status_data() -> Dict[str, Any]:
    health = build_health_payload()
    return {
        'system': health,
        'ros': {
            'bridge_ws': ROSBRIDGE_WS_DEFAULT,
            'map_topic': ROS_MAP_TOPIC,
            'alert_topic': ROS_ALERT_TOPIC,
            'connected': None,
        },
        'stream': {
            'endpoint_template': MJPEG_ENDPOINT,
            'topic': MJPEG_TOPIC,
            'width': MJPEG_WIDTH,
            'height': MJPEG_HEIGHT,
            'connected': None,
        },
    }


def get_robot_state_data() -> Dict[str, Any]:
    return {
        'robot_id': None,
        'mode': 'unavailable',
        'connected': None,
        'battery_percent': None,
        'position': {
            'x': None,
            'y': None,
            'yaw': None,
        },
        'source': 'unavailable',
    }


def get_alerts_data() -> Dict[str, Any]:
    conn = None
    try:
        conn = get_sqlite_connection()
        if conn is None:
            return build_empty_alerts_data()

        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()

        cursor.execute('SELECT COUNT(*) AS total_count FROM severity_logs')
        total_count_row = cursor.fetchone()
        total_count = int(total_count_row['total_count']) if total_count_row else 0

        cursor.execute(
            '''
            SELECT
                id,
                timestamp,
                track_id,
                severity,
                status_msg,
                emotion,
                motion_score,
                is_lying
            FROM severity_logs
            ORDER BY id DESC
            LIMIT ?
            ''',
            (ALERTS_QUERY_LIMIT,),
        )

        items = []
        for row in cursor.fetchall():
            items.append(
                {
                    'id': row['id'],
                    'timestamp': row['timestamp'],
                    'track_id': row['track_id'],
                    'severity': row['severity'],
                    'status_msg': row['status_msg'],
                    'emotion': row['emotion'],
                    'motion_score': row['motion_score'],
                    'is_lying': bool(row['is_lying']) if row['is_lying'] is not None else None,
                }
            )

        return {
            'count': total_count,
            'items': items,
            'source': 'sqlite',
        }
    except Exception:
        return build_empty_alerts_data()
    finally:
        if conn is not None:
            conn.close()


def get_map_summary_data() -> Dict[str, Any]:
    return {
        'topic': ROS_MAP_TOPIC,
        'available': None,
        'resolution': None,
        'width': None,
        'height': None,
        'source': 'unavailable',
    }


def is_authenticated() -> bool:
    return bool(session.get('is_authenticated'))


def login_required(view_func):
    @wraps(view_func)
    def wrapped_view(*args, **kwargs):
        if not is_authenticated():
            flash('먼저 로그인해 주세요.', 'warning')
            return redirect(url_for('login'))
        return view_func(*args, **kwargs)

    return wrapped_view


def create_app() -> Flask:
    base_dir = os.path.dirname(os.path.abspath(__file__))
    app = Flask(
        __name__,
        template_folder=os.path.join(base_dir, 'templates'),
    )
    app.config['SECRET_KEY'] = APP_SECRET_KEY

    @app.route('/')
    def home():
        if is_authenticated():
            return redirect(url_for('dashboard'))
        return redirect(url_for('login'))

    @app.route('/login', methods=['GET', 'POST'])
    def login():
        if is_authenticated():
            return redirect(url_for('dashboard'))

        if request.method == 'POST':
            username = request.form.get('username', '').strip()
            password = request.form.get('password', '')

            if validate_login(username, password):
                session.clear()
                session['is_authenticated'] = True
                session['username'] = username
                flash('로그인되었습니다.', 'success')
                return redirect(url_for('dashboard'))

            flash('아이디 또는 비밀번호가 올바르지 않습니다.', 'danger')
            return redirect(url_for('login'))

        return render_template(
            'login_center_srd.html',
            **build_base_template_context(),
        )

    @app.route('/dashboard')
    @login_required
    def dashboard():
        return render_template(
            'welcome_center_srd.html',
            **build_dashboard_context(session.get('username')),
        )

    @app.route('/logout')
    def logout():
        session.clear()
        flash('로그아웃되었습니다.', 'info')
        return redirect(url_for('login'))

    @app.route('/health')
    def health():
        return build_health_payload()

    @app.route('/api/system_status')
    @login_required
    def api_system_status():
        return get_system_status_data()

    @app.route('/api/robot_state')
    @login_required
    def api_robot_state():
        return get_robot_state_data()

    @app.route('/api/alerts')
    @login_required
    def api_alerts():
        return get_alerts_data()

    @app.route('/api/map_summary')
    @login_required
    def api_map_summary():
        return get_map_summary_data()

    @app.route('/api/record_tts', methods=['POST'])
    @login_required
    def api_record_tts():
        data = request.json
        # severity_logs 테이블 구조를 활용하여 TTS 기록 저장
        conn = get_sqlite_connection(read_only=False)
        if conn is None:
            return {'status': 'error', 'message': 'DB connection failed'}, 500

        try:
            cursor = conn.cursor()
            cursor.execute(
                '''
                INSERT INTO severity_logs (track_id, severity, status_msg, timestamp)
                VALUES (?, ?, ?, datetime('now', 'localtime'))
                ''',
                (f"TTS_{data.get('type')}", 'INFO', data.get('message')),
            )
            conn.commit()
            return {'status': 'success'}
        except Exception as e:
            return {'status': 'error', 'message': str(e)}, 500
        finally:
            conn.close()

    @app.route('/api/record_victim_pose', methods=['POST'])
    @login_required
    def api_record_victim_pose():
        data = request.json or {}
        conn = get_sqlite_connection(read_only=False)
        if conn is None:
            return {'status': 'error', 'message': 'DB connection failed'}, 500

        x = data.get('x')
        y = data.get('y')
        z = data.get('z')
        status_msg = f"Victim pose received: X={x}, Y={y}, Z={z}"

        try:
            cursor = conn.cursor()
            cursor.execute(
                '''
                INSERT INTO severity_logs (track_id, severity, status_msg, timestamp)
                VALUES (?, ?, ?, datetime('now', 'localtime'))
                ''',
                ('VICTIM_POSE', 'INFO', status_msg),
            )
            conn.commit()
            return {'status': 'success'}
        except Exception as e:
            return {'status': 'error', 'message': str(e)}, 500
        finally:
            conn.close()

    return app


app = create_app()


def main():
    app.run(host=APP_HOST, port=APP_PORT, debug=APP_DEBUG, use_reloader=False)


if __name__ == '__main__':
    main()
