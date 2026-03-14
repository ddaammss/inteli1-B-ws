# SRD Flask Server Refactor

리팩토링 목표는 기존 1파일 Flask 대시보드를 day5 스타일 구조로 정리하면서, 기존 SRD UI는 유지하는 것입니다.

## 현재 구조

```text
web/
├── README.md
├── __init__.py
├── rescue_ui.py
└── templates/
    ├── login_center_srd.html
    └── welcome_center_srd.html
```

## 핵심 변경점

- `render_template_string()` 기반 1파일 구조를 `templates/` 기반 구조로 분리
- `create_app()` 패턴으로 앱 구성 정리
- 로그인 페이지 추가
- 세션 인증 후 `/dashboard` 접근 가능하도록 변경
- `/logout` 추가
- 기존 관제 대시보드 UI 유지
- 헤더에 로그인 사용자 표시
- `/health` 유지

## 실행 방법

### 1. ROS2 명령어로 실행 (권장)
```bash
ros2 run rescue_bot rescue_ui
```

### 2. Python 직접 실행
```bash
python3 rescue_ui.py
```

## 기본 로그인 정보

```text
ID: admin
PW: 1234
```

운영 환경에서는 아래 환경변수로 바꾸는 것을 권장합니다.

```bash
export SRD_SECRET_KEY='change-this-secret'
export SRD_LOGIN_USERNAME='your_id'
export SRD_LOGIN_PASSWORD='your_password'
```

## 라우트

- `/` : 로그인 여부에 따라 `/login` 또는 `/dashboard`로 이동
- `/login` : 로그인 페이지
- `/dashboard` : 로그인 후 접근 가능한 관제 대시보드
- `/logout` : 로그아웃
- `/health` : 상태 확인

## 주의사항

현재 로그인은 세션 기반의 단순 인증입니다.
데모나 내부 테스트에는 적합하지만, 실제 운영 환경에서는 비밀번호 해시 저장, 사용자 DB, CSRF 대응, HTTPS 적용까지 같이 보는 편이 안전합니다.
