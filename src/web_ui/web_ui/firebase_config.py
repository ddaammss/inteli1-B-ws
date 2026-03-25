import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
import os

#SERVICE_ACCOUNT_KEY_PATH = os.path.expanduser("~/rescuebot/rokey-1f8f3-firebase-adminsdk-fbsvc-2cb7f18dea.json")
SERVICE_ACCOUNT_KEY_PATH = os.path.expanduser("~/rescuebot_ws/rokey-1f8f3-firebase-adminsdk-fbsvc-1843bb3b10.json")

DATABASE_URL = "https://rokey-1f8f3-default-rtdb.asia-southeast1.firebasedatabase.app"

_initialized = False


def init_firebase(logger=None):
    """Firebase 초기화 (한 번만 실행됨)"""
    global _initialized

    if _initialized:
        if logger:
            logger.info("Firebase 이미 초기화됨")
        return

    try:
        cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
        firebase_admin.initialize_app(cred, {
            'databaseURL': DATABASE_URL
        })
        _initialized = True
        if logger:
            logger.info("Firebase 초기화 완료")
    except ValueError:
        _initialized = True
        if logger:
            logger.info("Firebase 이미 초기화됨")


def get_reference(path):
    """Firebase Database Reference 반환"""
    return db.reference(path)
