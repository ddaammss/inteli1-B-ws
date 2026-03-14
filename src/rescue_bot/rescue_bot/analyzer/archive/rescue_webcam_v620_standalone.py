# v0.620 Standalone Webcam Test
"""
Rescue Bot Webcam Standalone Test Script (v0.620)
==============================================
이 주석은 ROS2 의존성 없이 "웹캠"으로 실시간 분석 로직(v0.620)을 테스트하기 위한 파일이다.
기존의 archive/webcam_legacy/srd_webcam_analyzer.py 구조에 최신 v0.620 엔진을 이식했다.

사용법:
python3 rescue_webcam_v620_standalone.py
"""

import cv2
import numpy as np
import math
import time
import json
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
from ultralytics import YOLO

@dataclass
class AnalyzerConfig:
    model_path: str = "yolo11n-pose.pt"
    det_conf: float = 0.35
    kp_conf_th: float = 0.45
    kp_margin_px: int = 2

    low_conf_min_kps: int = 3
    upper_body_min_kps: int = 3
    full_body_extra_lower_kps: int = 1

    leaning_shoulder_tilt_deg: float = 25.0
    collapsed_shoulder_tilt_deg: float = 45.0
    leaning_head_drop_ratio: float = 0.55
    collapsed_head_drop_ratio: float = 0.78
    collapsed_torso_angle_deg: float = 60.0
    lying_torso_angle_deg: float = 72.0
    lying_aspect_ratio: float = 2.00
    upper_body_min_shoulder_span_ratio: float = 0.25

    motion_window: int = 12
    motion_active_smooth: float = 0.020
    motion_active_upper: float = 0.025
    motion_local_only_upper: float = 0.015
    motion_local_only_core: float = 0.010
    motion_low: float = 0.008

    analyzing_sec: float = 1.5
    caution_sec: float = 4.5
    warning_sec: float = 5.5
    critical_sec: float = 7.0

    show_debug: bool = True
    draw_skeleton: bool = True
    draw_box: bool = True

class PoseEmergencyEngine:
    UPPER_IDS = [0, 5, 6, 7, 8, 9, 10]
    LOWER_IDS = [11, 12, 13, 14, 15, 16]
    CORE_IDS = [5, 6, 11, 12]
    UPPER_LINKS = [(5, 6), (5, 7), (7, 9), (6, 8), (8, 10)]
    FULL_LINKS = UPPER_LINKS + [(5, 11), (6, 12), (11, 12), (11, 13), (13, 15), (12, 14), (14, 16)]
    COLORS = {
        "ANALYZING": (255, 255, 255), "NORMAL": (0, 200, 0), "CAUTION": (0, 220, 255),
        "WARNING": (0, 140, 255), "CRITICAL": (0, 0, 255)
    }
    EMERGENCY_PRIORITY = {"CRITICAL": 4, "WARNING": 3, "CAUTION": 2, "NORMAL": 1, "ANALYZING": 0}

    def __init__(self, config: Optional[AnalyzerConfig] = None):
        self.cfg = config or AnalyzerConfig()
        self.model = YOLO(self.cfg.model_path)
        self.history: Dict[int, dict] = {}

    def reset(self): self.history.clear()

    @staticmethod
    def _safe_mean(points):
        valid = [p for p in points if p is not None]
        return np.mean(valid, axis=0) if valid else None

    @staticmethod
    def _pt_to_list(point):
        if point is None: return None
        return [int(round(float(point[0]))), int(round(float(point[1])))]

    def _extract_rep_points(self, keypoints, kp_conf, box, visibility, shape):
        nose = self._get_point(keypoints, kp_conf, 0, shape)
        leye = self._get_point(keypoints, kp_conf, 1, shape)
        reye = self._get_point(keypoints, kp_conf, 2, shape)
        lear = self._get_point(keypoints, kp_conf, 3, shape)
        rear = self._get_point(keypoints, kp_conf, 4, shape)
        ls = self._get_point(keypoints, kp_conf, 5, shape)
        rs = self._get_point(keypoints, kp_conf, 6, shape)
        lh = self._get_point(keypoints, kp_conf, 11, shape) if visibility == "FULL_BODY" else None
        rh = self._get_point(keypoints, kp_conf, 12, shape) if visibility == "FULL_BODY" else None
        shoulder_center = self._safe_mean([ls, rs])
        hip_center = self._safe_mean([lh, rh])
        face_anchor = self._safe_mean([nose, leye, reye, lear, rear])
        rep_point, rep_method = None, "NONE"
        if visibility == "FULL_BODY" and shoulder_center is not None and hip_center is not None:
            rep_point, rep_method = self._safe_mean([shoulder_center, hip_center]), "SHOULDER_HIP_MID"
        elif visibility == "UPPER_BODY" and shoulder_center is not None:
            rep_point, rep_method = shoulder_center, "SHOULDER_CENTER"
        elif face_anchor is not None:
            rep_point, rep_method = face_anchor, "FACE_ANCHOR"
        else:
            x1, y1, x2, y2 = box.astype(float)
            rep_point, rep_method = np.array([(x1 + x2) * 0.5, (y1 + y2) * 0.5], dtype=np.float32), "BBOX_CENTER"
        return rep_point, shoulder_center, hip_center, face_anchor, rep_method

    def _is_valid_kp(self, point, conf, w, h):
        x, y = float(point[0]), float(point[1])
        m = self.cfg.kp_margin_px
        return m <= x < (w - m) and m <= y < (h - m) and float(conf) >= self.cfg.kp_conf_th

    def _valid_indices(self, keypoints, kp_conf, ids, shape):
        h, w = shape[:2]
        return [i for i in ids if self._is_valid_kp(keypoints[i], kp_conf[i], w, h)]

    def _get_point(self, keypoints, kp_conf, idx, shape):
        h, w = shape[:2]
        return keypoints[idx] if self._is_valid_kp(keypoints[idx], kp_conf[idx], w, h) else None

    @staticmethod
    def _angle_deg(a, b):
        dx, dy = float(b[0] - a[0]), float(b[1] - a[1])
        angle = abs(math.degrees(math.atan2(dy, dx)))
        return 180.0 - angle if angle > 90.0 else angle

    def _new_track_state(self):
        now = time.time()
        return {
            "first_seen": now, "prev_kps": None, "prev_conf": None,
            "motion_buf": deque(maxlen=self.cfg.motion_window),
            "last_signature": None, "state_since": now
        }

    def _classify_visibility(self, keypoints, kp_conf, shape):
        uv = self._valid_indices(keypoints, kp_conf, self.UPPER_IDS, shape)
        lv = self._valid_indices(keypoints, kp_conf, self.LOWER_IDS, shape)
        if len(uv) + len(lv) < self.cfg.low_conf_min_kps: return "LOW_CONF"
        if len(uv) >= self.cfg.upper_body_min_kps and (11 in lv or 12 in lv) and \
           len([i for i in lv if i not in (11, 12)]) >= self.cfg.full_body_extra_lower_kps: return "FULL_BODY"
        return "UPPER_BODY" if len(uv) >= self.cfg.upper_body_min_kps else "PARTIAL"

    def _classify_posture(self, keypoints, kp_conf, box, visibility, shape):
        x1, y1, x2, y2 = box.astype(float)
        bw, bh = max(x2 - x1, 1.0), max(y2 - y1, 1.0)
        aspect = bw / bh
        nose, le, re, la, ra = [self._get_point(keypoints, kp_conf, i, shape) for i in range(5)]
        ls, rs = self._get_point(keypoints, kp_conf, 5, shape), self._get_point(keypoints, kp_conf, 6, shape)
        lh = self._get_point(keypoints, kp_conf, 11, shape) if visibility == "FULL_BODY" else None
        rh = self._get_point(keypoints, kp_conf, 12, shape) if visibility == "FULL_BODY" else None
        sc, hc, fa = self._safe_mean([ls, rs]), self._safe_mean([lh, rh]), self._safe_mean([nose, le, re, la, ra])
        tilt = self._angle_deg(ls, rs) if ls is not None and rs is not None else 0.0
        span = abs(float(rs[0] - ls[0])) if ls is not None and rs is not None else 0.0
        if span < bw * self.cfg.upper_body_min_shoulder_span_ratio: tilt = 0.0
        hds = 0.0
        if fa is not None and sc is not None and span > 1.0:
            hds = 1.0 - min(max(float(sc[1] - fa[1]), 0.0) / span, 1.0)
        torso = abs(90.0 - self._angle_deg(sc, hc)) if sc is not None and hc is not None else 0.0
        if visibility == "FULL_BODY":
            if aspect >= self.cfg.lying_aspect_ratio or torso >= self.cfg.lying_torso_angle_deg: return "LYING", tilt, hds, torso
            if torso >= self.cfg.collapsed_torso_angle_deg or tilt >= self.cfg.collapsed_shoulder_tilt_deg or hds >= self.cfg.collapsed_head_drop_ratio:
                return "COLLAPSED", tilt, hds, torso
            if tilt >= self.cfg.leaning_shoulder_tilt_deg or hds >= self.cfg.leaning_head_drop_ratio: return "LEANING", tilt, hds, torso
            return "NORMAL", tilt, hds, torso
        if visibility == "UPPER_BODY":
            if tilt >= self.cfg.collapsed_shoulder_tilt_deg or hds >= self.cfg.collapsed_head_drop_ratio: return "COLLAPSED", tilt, hds, torso
            if tilt >= self.cfg.leaning_shoulder_tilt_deg or hds >= self.cfg.leaning_head_drop_ratio: return "LEANING", tilt, hds, torso
            return "NORMAL", tilt, hds, torso
        return "UNKNOWN", tilt, hds, torso

    def _motion_value(self, track_id, keypoints, kp_conf, box, shape):
        hist = self.history.setdefault(track_id, self._new_track_state())
        pk, pc = hist["prev_kps"], hist["prev_conf"]
        hist["prev_kps"], hist["prev_conf"] = keypoints.copy(), kp_conf.copy()
        bh = max(float(box[3] - box[1]), 1.0)
        h, w = shape[:2]
        if pk is None: return 0.0, 0.0, 0.0
        def d(ids):
            v = [float(np.linalg.norm(keypoints[i] - pk[i]) / bh) for i in ids if self._is_valid_kp(keypoints[i], kp_conf[i], w, h) and self._is_valid_kp(pk[i], pc[i], w, h)]
            return float(np.mean(v)) if v else 0.0
        u, c = d(self.UPPER_IDS), d(self.CORE_IDS)
        hist["motion_buf"].append(max(u, c))
        return float(np.mean(hist["motion_buf"])), u, c

    def _classify_motion(self, smooth, upper, core):
        if upper >= self.cfg.motion_local_only_upper and core <= self.cfg.motion_local_only_core: return "LOCAL_ONLY"
        if smooth >= self.cfg.motion_active_smooth or upper >= self.cfg.motion_active_upper: return "ACTIVE"
        return "LOW" if smooth >= self.cfg.motion_low else "NONE"

    @staticmethod
    def _possible_trapped(vis, pos, mot):
        return (vis == "PARTIAL" and mot in ("LOCAL_ONLY", "NONE")) or (vis == "UPPER_BODY" and pos == "COLLAPSED" and mot == "NONE")

    def _state_duration(self, track_id, signature):
        h = self.history[track_id]
        n = time.time()
        if h["last_signature"] != signature: h["last_signature"], h["state_since"] = signature, n
        return n - h["first_seen"], n - h["state_since"]

    def _decide(self, vis, pos, mot, trapped, seen, state):
        if seen < self.cfg.analyzing_sec: return "ANALYZING"
        if vis == "FULL_BODY":
            if pos in ("LYING", "COLLAPSED") and mot == "NONE" and state >= self.cfg.critical_sec: return "CRITICAL"
            if pos in ("LYING", "COLLAPSED") and mot in ("LOW", "NONE") and state >= self.cfg.warning_sec: return "WARNING"
            if (pos == "LEANING" and mot in ("LOW", "NONE") and state >= self.cfg.caution_sec) or \
               (pos == "NORMAL" and mot == "NONE" and state >= self.cfg.warning_sec) or \
               (mot == "LOW" and state >= self.cfg.caution_sec): return "CAUTION"
            return "NORMAL"
        if vis == "UPPER_BODY":
            if pos == "COLLAPSED" and mot == "NONE" and state >= self.cfg.warning_sec: return "WARNING"
            return "CAUTION" if mot in ("LOW", "NONE") and state >= self.cfg.caution_sec else "NORMAL"
        if vis == "PARTIAL":
            if trapped and mot == "NONE" and state >= self.cfg.critical_sec: return "WARNING"
            return "CAUTION" if mot in ("LOW", "NONE") and state >= self.cfg.caution_sec else "NORMAL"
        return "CAUTION"

    def _draw_skeleton(self, frame, kps, kc, vis, color):
        if not self.cfg.draw_skeleton: return
        h, w = frame.shape[:2]
        lks = self.FULL_LINKS if vis == "FULL_BODY" else self.UPPER_LINKS
        ids = set(self.UPPER_IDS + self.LOWER_IDS) if vis == "FULL_BODY" else set(self.UPPER_IDS)
        for a, b in lks:
            if self._is_valid_kp(kps[a], kc[a], w, h) and self._is_valid_kp(kps[b], kc[b], w, h):
                cv2.line(frame, tuple(kps[a].astype(int)), tuple(kps[b].astype(int)), color, 2)
        for i, p in enumerate(kps):
            if i in ids and self._is_valid_kp(p, kc[i], w, h):
                cv2.circle(frame, tuple(p.astype(int)), 4, (255, 255, 255), -1)
                cv2.circle(frame, tuple(p.astype(int)), 3, color, -1)

    def _pack_result(self, **kwargs): return kwargs

    def analyze_frame_with_results(self, frame):
        annotated = frame.copy()
        yolo = self.model.track(frame, persist=True, verbose=False, conf=self.cfg.det_conf)
        if not yolo or yolo[0].boxes is None: return annotated, []
        boxes, kps_xy = yolo[0].boxes.xyxy.cpu().numpy(), yolo[0].keypoints.xy.cpu().numpy()
        kps_cf = yolo[0].keypoints.conf.cpu().numpy() if yolo[0].keypoints.conf is not None else np.ones((len(kps_xy), 17))
        ids = yolo[0].boxes.id.int().cpu().tolist() if yolo[0].boxes.id is not None else list(range(len(boxes)))
        res = []
        for tid, box, kps, kcf in zip(ids, boxes, kps_xy, kps_cf):
            x1, y1, x2, y2 = box.astype(int)
            x1, y1, x2, y2 = max(0, x1), max(0, y1), min(frame.shape[1]-1, x2), min(frame.shape[0]-1, y2)
            cbox = np.array([x1, y1, x2, y2])
            vis = self._classify_visibility(kps, kcf, frame.shape)
            pos, tilt, hds, torso = self._classify_posture(kps, kcf, cbox, vis, frame.shape)
            sm, up, co = self._motion_value(tid, kps, kcf, cbox, frame.shape)
            mot = self._classify_motion(sm, up, co)
            trp = self._possible_trapped(vis, pos, mot)
            sn, st = self._state_duration(tid, f"{vis}|{pos}|{mot}|{trp}")
            lvl = self._decide(vis, pos, mot, trp, sn, st)
            rpt, sc, hc, fa, rm = self._extract_rep_points(kps, kcf, cbox, vis, frame.shape)
            clr = self.COLORS[lvl]
            self._draw_skeleton(annotated, kps, kcf, vis, clr)
            if rpt is not None: cv2.drawMarker(annotated, tuple(rpt.astype(int)), clr, cv2.MARKER_CROSS, 10, 2)
            if self.cfg.draw_box: cv2.rectangle(annotated, (x1, y1), (x2, y2), clr, 2)
            ty = max(20, y1 - 10)
            cv2.putText(annotated, f"ID {tid} | {lvl}", (x1, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.6, clr, 2)
            if self.cfg.show_debug:
                cv2.putText(annotated, f"{vis}|{pos}|{mot}", (x1, min(frame.shape[0]-25, y2+20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, clr, 1)
            res.append({"track_id": tid, "level": lvl, "visibility": vis, "posture": pos, "motion": mot})
        return annotated, res

def main():
    print("Testing Webcam with v0.620 Standalone...")
    cfg = AnalyzerConfig()
    engine = PoseEmergencyEngine(cfg)
    
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Webcam 0 failed, trying index 1...")
        cap = cv2.VideoCapture(1)
    
    if not cap.isOpened():
        print("No webcam found.")
        return

    print("Started. Press 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret: break
        
        annotated, results = engine.analyze_frame_with_results(frame)
        cv2.imshow("Rescue Bot Standalone Test (v0.620)", annotated)
        
        if results:
            print(f"Detections: {results}")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
